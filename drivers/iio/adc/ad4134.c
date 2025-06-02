// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 * Author: Ioan-daniel Pop <Pop.Ioan-daniel@analog.com>
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/crc8.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/units.h>
#include <linux/err.h>

#include <linux/iio/iio.h>

#define AD4134_DEVICE_CONFIG_REG		0x02
#define AD4134_DEVICE_CONFIG_POWER_MODE_MASK	BIT(0)
#define AD4134_POWER_MODE_HIGH_PERF		0b1

#define AD4134_SDO_PIN_SRC_SEL_REG		0x10
#define AD4134_SDO_PIN_SRC_SEL_SDO_SEL_MASK	BIT(2)

#define AD4134_DATA_PACKET_CONFIG_REG		0x11
#define AD4134_DATA_PACKET_CONFIG_FRAME_MASK	GENMASK(5, 4)
#define AD4134_DATA_PACKET_16BIT_FRAME		0x0
#define AD4134_DATA_PACKET_16BIT_CRC6_FRAME	0x1
#define AD4134_DATA_PACKET_24BIT_FRAME		0x2
#define AD4134_DATA_PACKET_24BIT_CRC6_FRAME	0x3

#define AD4134_DIG_IF_CFG_REG			0x12
#define AD4134_DIF_IF_CFG_FORMAT_MASK		GENMASK(1, 0)
#define AD4134_DATA_FORMAT_SINGLE_CH_MODE	0b00
#define AD4134_DATA_FORMAT_QUAD_CH_PARALLEL	0b10

#define AD4134_RESET_TIME_US			10000000

#define AD4134_SPI_MAX_XFER_LEN			3

#define AD4134_EXT_CLOCK_MHZ			(48 * MEGA)

#define AD4134_SPI_CRC_POLYNOM			0x07
#define AD4134_SPI_CRC_INIT_VALUE		0xA5
DECLARE_CRC8_TABLE(ad4134_spi_crc_table);

static const char * const ad4143_regulator_names[] = {
	"avdd5", "dvdd5", "iovdd", "refin",	/* Required supplies */
	"avdd1v8", "dvdd1v8", "clkvdd",		/* Required if ldoin not provided */
	"ldoin",
};

static const char *const ad4134_clk_sel[] = {
	"xtal1-xtal2", "clkin"
};

/* maps adi,adc-frame property value to enum */
static const char * const ad4134_frame_config[] = {
	[AD4134_DATA_PACKET_16BIT_FRAME] = "16-bit",
	[AD4134_DATA_PACKET_16BIT_CRC6_FRAME] = "16-bit+CRC",
	[AD4134_DATA_PACKET_24BIT_FRAME] = "24-bit",
	[AD4134_DATA_PACKET_24BIT_CRC6_FRAME] = "24-bit+CRC",
};

#define AD4134_CHANNEL(_index, _realbits, _storebits) {				\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |		\
				    BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = (_realbits),					\
		.storagebits = (_storebits),					\
		.shift = ((_storebits) - (_realbits))				\
	},									\
}

#define AD4134_CHAN_SET(_realbits, _storebits) {				\
	AD4134_CHANNEL(0, _realbits, _storebits),				\
	AD4134_CHANNEL(1, _realbits, _storebits),				\
	AD4134_CHANNEL(2, _realbits, _storebits),				\
	AD4134_CHANNEL(3, _realbits, _storebits),				\
}

static const struct iio_chan_spec ad4134_16_chan_set[] = AD4134_CHAN_SET(16, 16);
static const struct iio_chan_spec ad4134_16CRC_chan_set[] = AD4134_CHAN_SET(16, 24);
static const struct iio_chan_spec ad4134_24_chan_set[] = AD4134_CHAN_SET(24, 24);
static const struct iio_chan_spec ad4134_24CRC_chan_set[] = AD4134_CHAN_SET(24, 32);

static const unsigned long ad4134_channel_masks[] = {
	GENMASK(ARRAY_SIZE(ad4134_16_chan_set) - 1, 0),
	0,
};

struct ad4134_chip_info {
	const char *name;
};

static const struct ad4134_chip_info ad4134_chip_info = {
	.name = "ad4134",
};

static const struct ad4134_chip_info ad7134_chip_info = {
	.name = "ad7134",
};

struct ad4134_state {
	struct regmap			*regmap;
	struct spi_device		*spi;

	/*
	 * Synchronize access to members the of driver state, and ensure
	 * atomicity of consecutive regmap operations.
	 */
	struct mutex			lock;

	struct spi_message		buf_read_msg;
	struct spi_transfer		buf_read_xfer;

	unsigned long			sys_clk_rate;
	int				refin_mv;
	int				output_frame;
	struct gpio_desc *odr_gpio;
	u8 reg_tx_buf[AD4134_SPI_MAX_XFER_LEN];
	u8 reg_rx_buf[AD4134_SPI_MAX_XFER_LEN];
};

static int ad4134_calc_spi_crc(u8 *pdata)
{
	return crc8(ad4134_spi_crc_table, pdata, 2, AD4134_SPI_CRC_INIT_VALUE);
}

static void ad4134_format_reg_write(u8 reg, u8 val, u8 *buf)
{
	u8 inst_buf[2] = {reg, val};

	buf[0] = reg;
	buf[1] = val;
	buf[3] = ad4134_calc_spi_crc(inst_buf);
}

static int ad4134_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad4134_state *st = context;

	ad4134_format_reg_write(reg, val, st->reg_tx_buf);

	return spi_write(st->spi, st->reg_tx_buf, 3);
}

static int ad4134_crc_check(struct ad4134_state *st, u8 *buf)
{
	u8 inst_buf[2] = {buf[0], buf[1]};
	u8 expected_crc = ad4134_calc_spi_crc(inst_buf);

	if (buf[2] != expected_crc) {
		dev_err(&st->spi->dev, "Bad CRC %02x for 0x%02X%02X\n",
			expected_crc, buf[1], buf[0]);
		return -EINVAL;
	}

	return 0;
}

static int ad4134_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad4134_state *st = context;
	struct spi_transfer reg_read_xfer[] = {
		{
			.tx_buf = st->reg_tx_buf,
			.rx_buf = st->reg_rx_buf,
			.len = 3,
		},
	};
	int ret;

	ad4134_format_reg_write(reg, *val, st->reg_tx_buf);

	ret = spi_sync_transfer(st->spi, reg_read_xfer, ARRAY_SIZE(reg_read_xfer));
	if (ret)
		return ret;

	ret = ad4134_crc_check(st, st->reg_rx_buf);
	if (ret)
		return ret;

	*val = st->reg_rx_buf[1];

	return 0;
}

static const struct regmap_config ad4134_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_read = ad4134_reg_read,
	.reg_write = ad4134_reg_write,
};

static int ad4134_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		*val = st->refin_mv;
		*val2 = chan->scan_type.realbits - 1;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad4134_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info ad4134_info = {
	.read_raw = ad4134_read_raw,
	.debugfs_reg_access = ad4134_reg_access,
};

static int ad4134_clock_select(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct clk *sys_clk;
	int ret;

	ret = device_property_match_property_string(dev, "clock-names",
						    ad4134_clk_sel,
						    ARRAY_SIZE(ad4134_clk_sel));
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to find external clock\n");

	sys_clk = devm_clk_get_enabled(dev, ad4134_clk_sel[ret]);
	if (IS_ERR(sys_clk))
		return dev_err_probe(dev, PTR_ERR(sys_clk),
				     "Failed to get %s external clock\n",
				     ad4134_clk_sel[ret]);


	st->sys_clk_rate = clk_get_rate(sys_clk);
	if (st->sys_clk_rate != AD4134_EXT_CLOCK_MHZ)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid external clock frequency %lu\n",
				     st->sys_clk_rate);

	return 0;
}

static int ad4134_min_io_mode_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	st->odr_gpio = devm_gpiod_get(dev, "odr", GPIOD_OUT_HIGH);
	if (IS_ERR(st->odr_gpio))
		return dev_err_probe(dev, PTR_ERR(st->odr_gpio),
				     "Failed to get ODR GPIO\n");

	ret = regmap_update_bits(st->regmap, AD4134_DIG_IF_CFG_REG,
				 AD4134_DIF_IF_CFG_FORMAT_MASK,
				 FIELD_PREP(AD4134_DIF_IF_CFG_FORMAT_MASK,
					    AD4134_DATA_FORMAT_SINGLE_CH_MODE));
	if (ret)
		return ret;

	return regmap_set_bits(st->regmap, AD4134_SDO_PIN_SRC_SEL_REG,
			       AD4134_SDO_PIN_SRC_SEL_SDO_SEL_MASK);
}

static int ad4134_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *reset_gpio;
	int ret;

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Failed to find reset GPIO\n");

	if (reset_gpio) {
		fsleep(AD4134_RESET_TIME_US);
		gpiod_set_value_cansleep(reset_gpio, 0);
	}

	ret = ad4134_min_io_mode_setup(st);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4134_DATA_PACKET_CONFIG_REG,
				 AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
				 FIELD_PREP(AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
					    st->output_frame));
	if (ret)
		return ret;

	 ret = regmap_update_bits(st->regmap, AD4134_DEVICE_CONFIG_REG,
				  AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
				  FIELD_PREP(AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
					     AD4134_POWER_MODE_HIGH_PERF));
	if (ret)
		return ret;

	return 0;
}

static int ad4134_probe(struct spi_device *spi)
{
	const struct ad4134_chip_info *chip;
	bool use_internal_ldo_retulator;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4134_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->spi = spi;

	chip = spi_get_device_match_data(spi);
	if (!chip)
		return -EINVAL;

	indio_dev->name = chip->name;

	/* Required regulators */
	ret = devm_regulator_bulk_get_enable(dev, 3, ad4143_regulator_names);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable power supplies\n");

	/* Required regulator that we need to read the voltage */
	ret = devm_regulator_get_enable_read_voltage(dev, "refin");
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get REFIN voltage.\n");

	st->refin_mv = ret / 1000;

	/*
	 * If ldoin is not provided, then avdd1v8, dvdd1v8, and clkvdd are
	 * required.
	 */
	ret = devm_regulator_get_enable_optional(dev, "ldoin");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to enable ldoin supply\n");

	use_internal_ldo_retulator = ret == 0;

	if (!use_internal_ldo_retulator) {
		ret = devm_regulator_get_enable(dev, "avdd1v8");
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to enable avdd1v8 supply\n");

		ret = devm_regulator_get_enable(dev, "dvdd1v8");
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to enable dvdd1v8 supply\n");

		ret = devm_regulator_get_enable(dev, "clkvdd");
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to enable clkvdd supply\n");
	}

	ret = ad4134_clock_select(st);
	if (ret)
		return ret;

	st->output_frame = AD4134_DATA_PACKET_24BIT_FRAME;
	ret = device_property_match_property_string(dev, "adi,adc-frame",
						    ad4134_frame_config,
						    ARRAY_SIZE(ad4134_frame_config));
	if (ret < 0)
		dev_warn(dev, "Failed to get adi,adc-frame property: %d\n", ret);
	else
		st->output_frame = ret;

	switch (st->output_frame) {
	case AD4134_DATA_PACKET_16BIT_FRAME:
		indio_dev->channels = ad4134_16_chan_set;
		indio_dev->num_channels = ARRAY_SIZE(ad4134_16_chan_set);
		break;
	case AD4134_DATA_PACKET_16BIT_CRC6_FRAME:
		indio_dev->channels = ad4134_16CRC_chan_set;
		indio_dev->num_channels = ARRAY_SIZE(ad4134_16CRC_chan_set);
		break;
	case AD4134_DATA_PACKET_24BIT_FRAME:
		indio_dev->channels = ad4134_24_chan_set;
		indio_dev->num_channels = ARRAY_SIZE(ad4134_24_chan_set);
		break;
	case AD4134_DATA_PACKET_24BIT_CRC6_FRAME:
		indio_dev->channels = ad4134_24CRC_chan_set;
		indio_dev->num_channels = ARRAY_SIZE(ad4134_24CRC_chan_set);
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "Failed to config ADC frame\n");
	}

	st->regmap = devm_regmap_init_spi(spi, &ad4134_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	indio_dev->available_scan_masks = ad4134_channel_masks;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4134_info;

	crc8_populate_msb(ad4134_spi_crc_table, AD4134_SPI_CRC_POLYNOM);

	ret = ad4134_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id ad4134_id[] = {
	{ "ad4134", (kernel_ulong_t)&ad4134_chip_info },
	{ "ad7134", (kernel_ulong_t)&ad7134_chip_info },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad4134_id);

static const struct of_device_id ad4134_of_match[] = {
	{ .compatible = "adi,ad4134", .data = &ad4134_chip_info },
	{ .compatible = "adi,ad7134", .data = &ad7134_chip_info },
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

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_AUTHOR("Ioan-daniel Pop <Pop.Ioan-daniel@analog.com>");
MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4134 SPI driver");
MODULE_LICENSE("GPL");
