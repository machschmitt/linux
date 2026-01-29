// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Analog Devices, Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/offload/consumer.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#define AD4134_NUM_DOUT_LINES			1

#define AD4134_ODR_HIGH_TIME_NS			125
#define AD4134_DCLK_RISING_OFFSET_NS		8
#define AD4134_ODR_PERIOD_NS			250
#define AD4134_RESET_TIME_US			(10 * USEC_PER_SEC)

#define AD4134_MIN_ODR_FREQ_HZ			10
#define AD4134_MAX_ODR_FREQ_HZ			(1496 * HZ_PER_KHZ)

#define AD4134_REG_READ_MASK			BIT(7)
#define AD4134_SPI_MAX_XFER_LEN			3

#define AD4134_EXT_CLOCK_MHZ			(48 * HZ_PER_MHZ)

#define AD4134_NUM_CHANNELS			4
#define AD4134_CHAN_PRECISION_BITS		24

#define AD4134_IFACE_CONFIG_A_REG		0x00
#define AD4134_IFACE_CONFIG_B_REG		0x01
#define AD4134_IFACE_CONFIG_B_SINGLE_INSTR	BIT(7)

#define AD4134_DEVICE_CONFIG_REG		0x02
#define AD4134_DEVICE_CONFIG_POWER_MODE_MASK	BIT(0)
#define AD4134_POWER_MODE_HIGH_PERF		0x1

#define AD4134_SILICON_REV_REG			0x07
#define AD4134_SCRATCH_PAD_REG			0x0A
#define AD4134_STREAM_MODE_REG			0x0E
#define AD4134_SDO_PIN_SRC_SEL_REG		0x10
#define AD4134_SDO_PIN_SRC_SEL_SDO_SEL_MASK	BIT(2)

#define AD4134_DATA_PACKET_CONFIG_REG		0x11
#define AD4134_DATA_PACKET_CONFIG_FRAME_MASK	GENMASK(5, 4)
#define AD4134_DATA_PACKET_24BIT_FRAME		0x2

#define AD4134_DIG_IF_CFG_REG			0x12
#define AD4134_DIF_IF_CFG_FORMAT_MASK		GENMASK(1, 0)
#define AD4134_DATA_FORMAT_SINGLE_CH_MODE	0x0

#define AD4134_PW_DOWN_CTRL_REG			0x13
#define AD4134_DEVICE_STATUS_REG		0x15
#define AD4134_ODR_VAL_INT_LSB_REG		0x16
#define AD4134_CH3_OFFSET_MSB_REG		0x3E
#define AD4134_AIN_OR_ERROR_REG			0x48

/*
 * AD4134 register map ends at address 0x48 and there is no register for
 * retrieving ADC sample data. Though, to make use of Linux regmap API both
 * for register access and sample read, we define one virtual register for each
 * ADC channel. AD4134_CH_VREG(x) maps a channel number to it's virtual register
 * address while AD4134_VREG_CH(x) tells which channel given the address.
 */
#define AD4134_CH_VREG(x)			((x) + 0x50)
#define AD4134_VREG_CH(x)			((x) - 0x50)

#define AD4134_SPI_CRC_POLYNOM			0x07
#define AD4134_SPI_CRC_INIT_VALUE		0xA5
static unsigned char ad4134_spi_crc_table[CRC8_TABLE_SIZE];

#define AD4134_CHANNEL(_index) {						\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = AD4134_CHAN_PRECISION_BITS,				\
		.storagebits = 32,						\
		.shift = 32 - AD4134_CHAN_PRECISION_BITS,			\
		.endianness = IIO_BE,						\
	},									\
}

static const struct iio_chan_spec ad4134_chan_set[] = {
	AD4134_CHANNEL(0),
	AD4134_CHANNEL(1),
	AD4134_CHANNEL(2),
	AD4134_CHANNEL(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

/*
 * Hardcode 32-bit storagebits since the current available HDL only supports
 * that.
 */
#define AD4134_OFFLOAD_CHANNEL(_index) {					\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |			\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.storagebits = 32,						\
		.realbits = AD4134_CHAN_PRECISION_BITS,				\
		.endianness = IIO_CPU,						\
	},									\
}

/*
 * Offloaded SPI transfers can't support software timestamp so no additional
 * timestamp channel is added.
 */
static const struct iio_chan_spec ad4134_offload_chan_set[] = {
	AD4134_OFFLOAD_CHANNEL(0),
	AD4134_OFFLOAD_CHANNEL(1),
	AD4134_OFFLOAD_CHANNEL(2),
	AD4134_OFFLOAD_CHANNEL(3),
};

/* The chip can only convert and output all 4 channels on each sample request */
static const unsigned long ad4134_offload_scan_masks[] = {
	GENMASK(3, 0),
	0
};

static const struct spi_offload_config ad4134_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

struct ad4134_state {
	struct spi_device *spi;
	struct regmap *regmap;
	unsigned long sys_clk_hz;
	struct gpio_desc *odr_gpio;
	int refin_mv;
	struct spi_transfer xfers[AD4134_NUM_CHANNELS];
	struct spi_message msg;
	struct spi_offload *offload;
	struct spi_offload_trigger *offload_trigger;
	struct spi_offload_trigger_config offload_trigger_config;
	struct pwm_device *odr_trigger;
	struct pwm_waveform odr_wf;
	unsigned int odr_hz;
	/*
	 * DMA (thus cache coherency maintenance) requires the transfer buffers
	 * to live in their own cache lines.
	 *
	 * Make the buffer large enough for AD4134_NUM_CHANNELS 32-bit samples
	 * and one 64-bit aligned 64-bit timestamp.
	 */
	IIO_DECLARE_DMA_BUFFER_WITH_TS(u8, scan_data, AD4134_NUM_CHANNELS * sizeof(u32));
	/* Register access buffers */
	u8 rx_buf[AD4134_SPI_MAX_XFER_LEN];
	u8 tx_buf[AD4134_SPI_MAX_XFER_LEN];
};

static const struct regmap_range ad4134_regmap_rd_range[] = {
	regmap_reg_range(AD4134_IFACE_CONFIG_A_REG, AD4134_SILICON_REV_REG),
	regmap_reg_range(AD4134_SCRATCH_PAD_REG, AD4134_PW_DOWN_CTRL_REG),
	regmap_reg_range(AD4134_DEVICE_STATUS_REG, AD4134_AIN_OR_ERROR_REG),
	regmap_reg_range(AD4134_CH_VREG(0), AD4134_CH_VREG(AD4134_NUM_CHANNELS)),
};

static const struct regmap_range ad4134_regmap_wr_range[] = {
	regmap_reg_range(AD4134_IFACE_CONFIG_A_REG, AD4134_DEVICE_CONFIG_REG),
	regmap_reg_range(AD4134_SCRATCH_PAD_REG, AD4134_SCRATCH_PAD_REG),
	regmap_reg_range(AD4134_STREAM_MODE_REG, AD4134_PW_DOWN_CTRL_REG),
	regmap_reg_range(AD4134_ODR_VAL_INT_LSB_REG, AD4134_CH3_OFFSET_MSB_REG),
};

static const struct regmap_access_table ad4134_regmap_rd_table = {
	.yes_ranges = ad4134_regmap_rd_range,
	.n_yes_ranges = ARRAY_SIZE(ad4134_regmap_rd_range),
};

static const struct regmap_access_table ad4134_regmap_wr_table = {
	.yes_ranges = ad4134_regmap_wr_range,
	.n_yes_ranges = ARRAY_SIZE(ad4134_regmap_wr_range),
};

static int ad4134_calc_spi_crc(u8 inst, u8 data)
{
	u8 buf[] = { inst, data };

	return crc8(ad4134_spi_crc_table, buf, ARRAY_SIZE(buf),
		    AD4134_SPI_CRC_INIT_VALUE);
}

static void ad4134_prepare_spi_tx_buf(u8 inst, u8 data, u8 *buf)
{
	buf[0] = inst;
	buf[1] = data;
	buf[2] = ad4134_calc_spi_crc(inst, data);
}

static int ad4134_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad4134_state *st = context;
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
		.len = AD4134_SPI_MAX_XFER_LEN,
	};
	int ret;

	ad4134_prepare_spi_tx_buf(reg, val, st->tx_buf);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	if (st->rx_buf[2] != st->tx_buf[2])
		dev_dbg(&st->spi->dev, "reg write CRC check failed\n");

	return 0;
}

static int ad4134_data_read(struct ad4134_state *st, unsigned int reg,
			    unsigned int *val)
{
	unsigned int i;
	int ret;

	/*
	 * To be able to read data from all 4 channels through a single line, we
	 * set DOUTx output format to 0 in the digital interface config register
	 * (0x12). With that, data from all four channels is serialized and
	 * output on DOUT0. During the probe, we also set SDO_PIN_SRC_SEL in
	 * DEVICE_CONFIG_1 register to duplicate DOUT0 on the SDO pin. Combined,
	 * those configurations enable ADC data read through a conventional SPI
	 * interface. Now we read data from all channels but keep only the bits
	 * from the requested one.
	 */
	for (i = 0; i < ARRAY_SIZE(ad4134_chan_set); i++) {
		ret = spi_write_then_read(st->spi, NULL, 0, st->rx_buf,
					  BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS));
		if (ret)
			return ret;

		/*
		 * AD4134 has a built-in feature that flags when data transfers
		 * don't run enough clock cycles to read the entire data frame.
		 * Clock out data from all channels to avoid that.
		 */
		if (i == AD4134_VREG_CH(reg))
			*val = get_unaligned_be24(st->rx_buf);
	}

	return 0;
}

static int ad4134_register_read(struct ad4134_state *st, unsigned int reg,
				unsigned int *val)
{
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
		.len = AD4134_SPI_MAX_XFER_LEN,
	};
	unsigned int inst;
	int ret;

	inst = AD4134_REG_READ_MASK | reg;
	ad4134_prepare_spi_tx_buf(inst, 0, st->tx_buf);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	*val = st->rx_buf[1];

	/* Check CRC */
	if (st->rx_buf[2] != st->tx_buf[2])
		dev_dbg(&st->spi->dev, "reg read CRC check failed\n");

	return 0;
}

static int ad4134_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad4134_state *st = context;

	if (reg >= AD4134_CH_VREG(0))
		return ad4134_data_read(st, reg, val);

	return ad4134_register_read(st, reg, val);
}

static const struct regmap_config ad4134_regmap_config = {
	.reg_read = ad4134_reg_read,
	.reg_write = ad4134_reg_write,
	.rd_table = &ad4134_regmap_rd_table,
	.wr_table = &ad4134_regmap_wr_table,
	.max_register = AD4134_CH_VREG(ARRAY_SIZE(ad4134_chan_set)),
};

static int ad4134_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;

		gpiod_set_value_cansleep(st->odr_gpio, 1);
		/*
		 * For slave mode gated DCLK (data sheet page 11), the minimum
		 * ODR high time is 3 * tDIGCLK. The internal digital clock
		 * period is tDIGCLK = 1/fDIGCLK = 2/fSYSCLK.
		 * The System clock frequency (fSYSCLK) is typically 48 MHz.
		 * Thus, ODR high time = 3 * (2 / (48 * HZ_PER_MHZ))
		 * ODR high time = 0.000000125 s = 125 ns
		 * 1 micro second should be more than enough. Not worth it
		 * tweaking for shorter dealy since this is not a fast data path.
		 */
		fsleep(1);
		gpiod_set_value_cansleep(st->odr_gpio, 0);
		ret = regmap_read(st->regmap, AD4134_CH_VREG(chan->channel), val);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->refin_mv;
		*val2 = AD4134_CHAN_PRECISION_BITS - 1;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad4134_debugfs_reg_access(struct iio_dev *indio_dev,
				     unsigned int reg, unsigned int writeval,
				     unsigned int *readval)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4134_min_io_mode_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	st->odr_gpio = devm_gpiod_get(dev, "odr", GPIOD_OUT_LOW);
	if (IS_ERR(st->odr_gpio))
		return dev_err_probe(dev, PTR_ERR(st->odr_gpio),
				     "failed to get ODR GPIO\n");

	ret = regmap_update_bits(st->regmap, AD4134_DIG_IF_CFG_REG,
				 AD4134_DIF_IF_CFG_FORMAT_MASK,
				 FIELD_PREP(AD4134_DIF_IF_CFG_FORMAT_MASK,
					    AD4134_DATA_FORMAT_SINGLE_CH_MODE));
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set single channel mode\n");

	ret = regmap_set_bits(st->regmap, AD4134_SDO_PIN_SRC_SEL_REG,
			      AD4134_SDO_PIN_SRC_SEL_SDO_SEL_MASK);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set SDO source selection\n");

	return regmap_set_bits(st->regmap, AD4134_IFACE_CONFIG_B_REG,
			       AD4134_IFACE_CONFIG_B_SINGLE_INSTR);
}

static const struct iio_info ad4134_info = {
	.read_raw = ad4134_read_raw,
	.debugfs_reg_access = ad4134_debugfs_reg_access,
};

static int ad4134_update_conversion_rate(struct ad4134_state *st,
					 unsigned int freq_hz)
{
	struct spi_offload_trigger_config *config = &st->offload_trigger_config;
	struct pwm_waveform odr_wf = { };
	u64 target;
	u64 offload_period_ns;
	u64 offload_offset_ns;
	u64 odr_high_time_ns;
	unsigned int odr_hz;
	int ret;

	/*
	 * Every ODR pulse causes each of the 4 ADCs within the AD4134 chip to
	 * take a sample simultaneously. The peripheral then outputs the data
	 * from all those channels over one, two, or four data output lanes. If
	 * the controller can fetch data from multiple lanes, the throughput is
	 * increased proportionally to the number of data lanes in use.
	 * Conversely, when multiple data lanes are enabled, the requested
	 * sampling frequency can be reached with slower ODR frequencies.
	 */
	odr_hz = freq_hz / AD4134_NUM_DOUT_LINES;
	if (odr_hz < AD4134_MIN_ODR_FREQ_HZ || odr_hz > AD4134_MAX_ODR_FREQ_HZ)
		return -EINVAL;

	odr_high_time_ns = div64_ul(6ULL * NANO, st->sys_clk_hz);
	dev_info(&st->spi->dev, "%s: odr_high_time_ns: %llu\n",
		 __func__, odr_high_time_ns);

	odr_wf.period_length_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, odr_hz);
	/*
	 * For an arbitrary system clock (fSYSCLK), we have a minimum ODR high
	 * time of 6/fSYSCLK derived from the device clock and data interface
	 * timing (with Gated DCLK) specifications. Set the PWM duty cycle to
	 * keep ODR up for at least that long. If the rounded PWM's value is
	 * less than the minimum required, increase the target value by 10 and
	 * attempt to round the waveform again, until the minimum is reached.
	 */
	do {
		odr_wf.duty_length_ns = target;
		ret = pwm_round_waveform_might_sleep(st->odr_trigger, &odr_wf);
		if (ret)
			return ret;
		target += 10;
	} while (odr_wf.duty_length_ns < odr_high_time_ns);

	if (!in_range(odr_wf.period_length_ns, 2 * odr_high_time_ns, INT_MAX))
		return -EINVAL;


	/*
	 * The controller fetches one sample per active lane each time the
	 * offload is triggered. If multiple data lanes are enabled, the offload
	 * trigger frequency can be proportionally slower.
	 */
	offload_period_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC,
					      odr_hz * AD4134_NUM_DOUT_LINES);

	config->periodic.frequency_hz = DIV_ROUND_UP_ULL(NSEC_PER_SEC,
							 offload_period_ns);

	/*
	 * For gated DCLK, the minimum required time between ODR rising edge
	 * and DCLK rising edge is the sum of ODR high time and ODR falling
	 * edge to DCLK rising edge time. Delay the begin of data sampling for
	 * at least that amount of time.
	 */
	offload_offset_ns = odr_high_time_ns + AD4134_DCLK_RISING_OFFSET_NS;
	do {
		config->periodic.offset_ns = offload_offset_ns;
		ret = spi_offload_trigger_validate(st->offload_trigger, config);
		if (ret)
			return ret;

		offload_offset_ns += 10;
	} while (config->periodic.offset_ns < odr_high_time_ns +
					      AD4134_DCLK_RISING_OFFSET_NS);

	dev_info(&st->spi->dev, "%s: config->periodic.frequency_hz: %llu\n",
		 __func__, config->periodic.frequency_hz);
	dev_info(&st->spi->dev, "%s: config->periodic.offset_ns: %llu\n",
		 __func__, config->periodic.offset_ns);
	dev_info(&st->spi->dev, "%s: odr_wf.period_length_ns: %llu\n",
		 __func__, odr_wf.period_length_ns);
	dev_info(&st->spi->dev, "%s: odr_wf.duty_length_ns: %llu\n",
		 __func__, odr_wf.duty_length_ns);
	dev_info(&st->spi->dev, "%s: odr_wf.duty_offset_ns: %llu\n",
		 __func__, odr_wf.duty_offset_ns);

	st->odr_wf = odr_wf;
	st->odr_hz = odr_hz;

	return 0;
}

static ssize_t sampling_frequency_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct ad4134_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%u\n", st->odr_hz * AD4134_NUM_DOUT_LINES);
}

static ssize_t sampling_frequency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		goto out_store;

	ret = ad4134_update_conversion_rate(st, val);

out_store:
	iio_device_release_direct(indio_dev);
	return ret ?: len;
}

static IIO_DEVICE_ATTR_RW(sampling_frequency, 0);

static ssize_t sampling_frequency_available_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	return sysfs_emit(buf, "[%u %u %lu]\n",
			  AD4134_MIN_ODR_FREQ_HZ * AD4134_NUM_DOUT_LINES, 1,
			  AD4134_MAX_ODR_FREQ_HZ * AD4134_NUM_DOUT_LINES);
}

static IIO_DEVICE_ATTR_RO(sampling_frequency_available, 0);

static struct attribute *ad4134_offload_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad4134_offload_attribute_group = {
	.attrs = ad4134_offload_attributes,
};

static const struct iio_info ad4134_offload_info = {
	.attrs = &ad4134_offload_attribute_group,
	.read_raw = ad4134_read_raw,
	.debugfs_reg_access = ad4134_debugfs_reg_access,
};

static void ad4134_prepare_offload_msg(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int i;

	for (i = 0; i < AD4134_NUM_CHANNELS; i++) {
		st->xfers[i].rx_buf = NULL;
		st->xfers[i].tx_buf = NULL;
		st->xfers[i].bits_per_word = AD4134_CHAN_PRECISION_BITS;
		st->xfers[i].len = spi_bpw_to_bytes(AD4134_CHAN_PRECISION_BITS);
		st->xfers[i].offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
	}

	spi_message_init_with_transfers(&st->msg, st->xfers, AD4134_NUM_CHANNELS);
}

static int ad4134_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ad4134_prepare_offload_msg(indio_dev);
	st->msg.offload = st->offload;
	ret = spi_optimize_message(st->spi, &st->msg);
	if (ret)
		return ret;

	ret = pwm_set_waveform_might_sleep(st->odr_trigger, &st->odr_wf, false);
	if (ret)
		goto out_unoptimize;

	ret = spi_offload_trigger_enable(st->offload, st->offload_trigger,
					 &st->offload_trigger_config);
	if (ret)
		goto out_pwm_disable;

	return 0;

out_pwm_disable:
	pwm_disable(st->odr_trigger);
out_unoptimize:
	spi_unoptimize_message(&st->msg);

	return 0;
}

static int ad4134_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	spi_offload_trigger_disable(st->offload, st->offload_trigger);

	pwm_disable(st->odr_trigger);

	spi_unoptimize_message(&st->msg);

	return 0;
}

static const struct iio_buffer_setup_ops ad4134_offload_buffer_setup_ops = {
	.postenable = &ad4134_offload_buffer_postenable,
	.predisable = &ad4134_offload_buffer_predisable,
};

static int ad4134_spi_offload_setup(struct iio_dev *indio_dev,
				    struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct dma_chan *rx_dma;

	indio_dev->setup_ops = &ad4134_offload_buffer_setup_ops;

	st->offload_trigger = devm_spi_offload_trigger_get(dev, st->offload,
							   SPI_OFFLOAD_TRIGGER_PERIODIC);
	if (IS_ERR(st->offload_trigger))
		return dev_err_probe(dev, PTR_ERR(st->offload_trigger),
				     "failed to get offload trigger\n");

	st->offload_trigger_config.type = SPI_OFFLOAD_TRIGGER_PERIODIC;

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(dev, st->offload);
	if (IS_ERR(rx_dma))
		return dev_err_probe(dev, PTR_ERR(rx_dma),
				     "failed to get offload RX DMA\n");

	return devm_iio_dmaengine_buffer_setup_with_handle(dev, indio_dev, rx_dma,
							   IIO_BUFFER_DIRECTION_IN);
}

static int ad4134_pwm_get(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;

	st->odr_trigger = devm_pwm_get(dev, NULL);
	if (IS_ERR(st->odr_trigger))
		return dev_err_probe(dev, PTR_ERR(st->odr_trigger),
				     "failed to get ODR PWM\n");

	pwm_disable(st->odr_trigger);

	return 0;
}

static void ad4134_init_scan_msg(struct ad4134_state *st)
{
	st->xfers[0].rx_buf = &st->scan_data[0];
	st->xfers[0].len = BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS);
	st->xfers[1].rx_buf = &st->scan_data[4];
	st->xfers[1].len = BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS);
	st->xfers[2].rx_buf = &st->scan_data[8];
	st->xfers[2].len = BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS);
	st->xfers[3].rx_buf = &st->scan_data[12];
	st->xfers[3].len = BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS);

	spi_message_init_with_transfers(&st->msg, st->xfers, AD4134_NUM_CHANNELS);
}

static int ad4134_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ret = spi_optimize_message(st->spi, &st->msg);
	if (ret) {
		dev_err(&st->spi->dev, "failed to prepare msg, err: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ad4134_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	spi_unoptimize_message(&st->msg);
	return 0;
}

static const struct iio_buffer_setup_ops ad4134_buffer_setup_ops = {
	.postenable = ad4134_buffer_postenable,
	.predisable = ad4134_buffer_predisable,
};

static irqreturn_t ad4134_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int storagebytes = spi_bpw_to_bytes(AD4134_CHAN_PRECISION_BITS);
	u8 bounce_buffer[AD4134_NUM_CHANNELS * storagebytes];
	unsigned int chan_index;
	unsigned int i = 0;
	int ret;

	/*
	 * One ODR pulse causes each of the 4 ADCs within the AD4134 chip to
	 * take a sample simultaneously. The peripheral then outputs the data
	 * from those over one, two, or four data output lanes.
	 */
	gpiod_set_value_cansleep(st->odr_gpio, 1);
	fsleep(1);
	gpiod_set_value_cansleep(st->odr_gpio, 0);
	ret = spi_sync(st->spi, &st->msg);
	if (ret)
		goto err_out;

	/*
	 * The scan_data buffer contains data from all 4 channels. Copy to
	 * buffers only the data from the enabled channels.
	 */
	iio_for_each_active_channel(indio_dev, chan_index) {
		memcpy(&bounce_buffer[i],
		       &st->scan_data[chan_index * storagebytes], storagebytes);
		i = i + storagebytes;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, bounce_buffer, pf->timestamp);
err_out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static const char * const ad4143_required_regulators[] = {
	"avdd5", "dvdd5", "iovdd",
};

static const char * const ad4143_optional_regulators[] = {
	"avdd1v8", "dvdd1v8", "clkvdd",
};

static int ad4134_regulator_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ad4143_required_regulators),
					     ad4143_required_regulators);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable power supplies\n");

	/* Required regulator that we need to read the voltage */
	ret = devm_regulator_get_enable_read_voltage(dev, "refin");
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get REFIN voltage.\n");

	st->refin_mv = ret / (MICRO / MILLI);

	ret = devm_regulator_get_enable_optional(dev, "ldoin");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to enable ldoin supply\n");

	/* If ldoin was provided, then use the use the internal LDO regulators */
	if (ret == 0)
		return 0;

	/*
	 * If ldoin is not provided, then avdd1v8, dvdd1v8, and clkvdd are
	 * required.
	 */
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ad4143_optional_regulators),
					     ad4143_optional_regulators);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable 1V8 power supplies\n");

	return 0;
}

static int ad4134_clock_select(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct clk *xtal_clk, *clkin_clk;

	/*
	 * AD4134 requires one external clock source and only one external clock
	 * source can be provided at a time. Try to get a crystal provided clock.
	 * If that fails, try to get a CMOS clock.
	 */
	xtal_clk = devm_clk_get_optional_enabled(dev, "xtal");
	if (!xtal_clk)
		xtal_clk = devm_clk_get_optional_enabled(dev, "xtal");
	if (IS_ERR(xtal_clk))
		return dev_err_probe(dev, PTR_ERR(xtal_clk),
				     "failed to get xtal\n");

	clkin_clk = devm_clk_get_optional_enabled(dev, "clkin");
	if (!clkin_clk)
		clkin_clk = devm_clk_get_optional_enabled(dev, "clkin");
	if (IS_ERR(clkin_clk))
		return dev_err_probe(dev, PTR_ERR(clkin_clk),
				     "failed to get clkin\n");

	st->sys_clk_hz = clk_get_rate(xtal_clk) | clk_get_rate(clkin_clk);
	dev_info(dev, "%s: st->sys_clk_hz: %lu\n", __func__, st->sys_clk_hz);

	return 0;
}

static int ad4134_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct reset_control *rst;
	struct iio_dev *indio_dev;
	struct ad4134_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->name = "ad4134";
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad4134_regulator_setup(st);
	if (ret)
		return ret;

	ret = ad4134_clock_select(st);
	if (ret)
		return ret;

	rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "failed to get and deassert reset\n");

	crc8_populate_msb(ad4134_spi_crc_table, AD4134_SPI_CRC_POLYNOM);

	st->regmap = devm_regmap_init(dev, NULL, st, &ad4134_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to initialize regmap");

	ret = ad4134_min_io_mode_setup(st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to setup minimum I/O mode\n");

	/* Bump precision to 24-bit */
	ret = regmap_update_bits(st->regmap, AD4134_DATA_PACKET_CONFIG_REG,
				 AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
				 FIELD_PREP(AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
					    AD4134_DATA_PACKET_24BIT_FRAME));
	if (ret)
		return ret;

	/* Set high performance power mode */
	ret = regmap_update_bits(st->regmap, AD4134_DEVICE_CONFIG_REG,
				 AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
				 FIELD_PREP(AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
					    AD4134_POWER_MODE_HIGH_PERF));
	if (ret)
		return ret;

	st->offload = devm_spi_offload_get(dev, spi, &ad4134_offload_config);
	ret = PTR_ERR_OR_ZERO(st->offload);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to get offload\n");

	/* Fall back to low speed usage when no SPI offload is available. */
	if (ret == -ENODEV) {
		indio_dev->info = &ad4134_info;
		indio_dev->channels = ad4134_chan_set;
		indio_dev->num_channels = ARRAY_SIZE(ad4134_chan_set);
		ad4134_init_scan_msg(st);

		ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
						      iio_pollfunc_store_time,
						      &ad4134_trigger_handler,
						      &ad4134_buffer_setup_ops);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to setup triggered buffer\n");
	} else {
		indio_dev->info = &ad4134_offload_info;
		indio_dev->channels = ad4134_offload_chan_set;
		indio_dev->num_channels = ARRAY_SIZE(ad4134_offload_chan_set);
		indio_dev->available_scan_masks = ad4134_offload_scan_masks;

		ret = ad4134_spi_offload_setup(indio_dev, st);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to setup SPI offload\n");

		ret = ad4134_pwm_get(st);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "failed to get PWM: %d\n", ret);

		/*
		 * Start with a slower sampling rate so there is some room for
		 * adjusting the sampling frequency without hitting the maximum
		 * conversion rate.
		 */
		st->odr_hz = AD4134_MAX_ODR_FREQ_HZ >> 4;
		ret = ad4134_update_conversion_rate(st, st->odr_hz);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "failed to set offload samp freq\n");
	}

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id ad4134_id[] = {
	{ "ad4134" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad4134_id);

static const struct of_device_id ad4134_of_match[] = {
	{ .compatible = "adi,ad4134" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_of_match);

static struct spi_driver ad4134_driver = {
	.driver = {
		.name = "ad4134",
		.of_match_table = ad4134_of_match,
	},
	.probe = ad4134_probe,
	.id_table = ad4134_id,
};
module_spi_driver(ad4134_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4134 SPI driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_AD4134");
MODULE_IMPORT_NS("IIO_DMAENGINE_BUFFER");
