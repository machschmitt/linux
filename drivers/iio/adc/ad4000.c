// SPDX-License-Identifier: GPL-2.0+
/*
 * AD4000 SPI ADC driver
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <asm/unaligned.h>
#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/units.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define AD400X_READ_COMMAND	0x54
#define AD400X_WRITE_COMMAND	0x14

#define AD4000_CONFIG_REG_MSK	0xFF

/* AD4000 Configuration Register programmable bits */
#define AD4000_STATUS		BIT(4) /* Status bits output */
#define AD4000_SPAN_COMP	BIT(3) /* Input span compression  */
#define AD4000_HIGHZ		BIT(2) /* High impedance mode  */
#define AD4000_TURBO		BIT(1) /* Turbo mode */

#define AD4000_16BIT_MSK	GENMASK(31, 16)
#define AD4000_18BIT_MSK	GENMASK(31, 14)
#define AD4000_20BIT_MSK	GENMASK(31, 12)

#define AD4000_CHANNEL(_sign, _real_bits)				\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				      BIT(IIO_CHAN_INFO_SCALE),		\
		.scan_type = {						\
			.sign = _sign,					\
			.realbits = _real_bits,				\
			.storagebits = 32,				\
			.endianness = IIO_BE,				\
		},							\
	}								\

enum ad4000_ids {
	ID_AD4000,
	ID_AD4001,
	ID_AD4002,
	ID_AD4003,
	ID_AD4004,
	ID_AD4005,
	ID_AD4006,
	ID_AD4007,
	ID_AD4008,
	ID_AD4010,
	ID_AD4011,
	ID_AD4020,
	ID_AD4021,
	ID_AD4022,
	ID_ADAQ4003,
};

struct ad4000_chip_info {
	const char *dev_name;
	struct iio_chan_spec chan_spec;
};

static const struct ad4000_chip_info ad4000_chips[] = {
	[ID_AD4000] = {
		.dev_name = "ad4000",
		.chan_spec = AD4000_CHANNEL('u', 16),
	},
	[ID_AD4001] = {
		.dev_name = "ad4001",
		.chan_spec = AD4000_CHANNEL('s', 16),
	},
	[ID_AD4002] = {
		.dev_name = "ad4002",
		.chan_spec = AD4000_CHANNEL('u', 18),
	},
	[ID_AD4003] = {
		.dev_name = "ad4003",
		.chan_spec = AD4000_CHANNEL('s', 18),
	},
	[ID_AD4004] = {
		.dev_name = "ad4004",
		.chan_spec = AD4000_CHANNEL('u', 16),
	},
	[ID_AD4005] = {
		.dev_name = "ad4005",
		.chan_spec = AD4000_CHANNEL('s', 16),
	},
	[ID_AD4006] = {
		.dev_name = "ad4006",
		.chan_spec = AD4000_CHANNEL('u', 18),
	},
	[ID_AD4007] = {
		.dev_name = "ad4007",
		.chan_spec = AD4000_CHANNEL('s', 18),
	},
	[ID_AD4008] = {
		.dev_name = "ad4008",
		.chan_spec = AD4000_CHANNEL('u', 16),
	},
	[ID_AD4010] = {
		.dev_name = "ad4010",
		.chan_spec = AD4000_CHANNEL('u', 18),
	},
	[ID_AD4011] = {
		.dev_name = "ad4011",
		.chan_spec = AD4000_CHANNEL('s', 18),
	},
	[ID_AD4020] = {
		.dev_name = "ad4020",
		.chan_spec = AD4000_CHANNEL('s', 20),
	},
	[ID_AD4021] = {
		.dev_name = "ad4021",
		.chan_spec = AD4000_CHANNEL('s', 20),
	},
	[ID_AD4022] = {
		.dev_name = "ad4022",
		.chan_spec = AD4000_CHANNEL('s', 20),
	},
	[ID_ADAQ4003] = {
		.dev_name = "adaq4003",
		.chan_spec = AD4000_CHANNEL('s', 18),
	},
};

enum ad4000_gains {
	AD4000_0454_GAIN = 0,
	AD4000_0909_GAIN = 1,
	AD4000_1_GAIN = 2,
	AD4000_1900_GAIN = 3,
	AD4000_GAIN_LEN
};

/*
 * Gains stored and computed as fractions to avoid introducing rounding erros.
 */
static const int ad4000_gains_frac[AD4000_GAIN_LEN][2] = {
	[AD4000_0454_GAIN] = { 227, 500 },
	[AD4000_0909_GAIN] = { 909, 1000 },
	[AD4000_1_GAIN] = { 1, 1 },
	[AD4000_1900_GAIN] = { 19, 10 },
};

struct ad4000_state {
	struct spi_device *spi;
	struct gpio_desc *cnv_gpio;
	int vref;
	bool status_bits;
	bool span_comp;
	bool turbo_mode;
	bool high_z_mode;

	enum ad4000_gains pin_gain;
	int scale_tbl[AD4000_GAIN_LEN][2];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		struct {
			u8 sample_buf[4];
			s64 timestamp;
		} scan;
		u8 d8[2];
	} data __aligned(IIO_DMA_MINALIGN);
};

static void ad4000_fill_scale_tbl(struct ad4000_state *st, int scale_bits)
{
	int val, val2, tmp0, tmp1, i;
	u64 tmp2;

	val2 = scale_bits;
	for (i = 0; i < AD4000_GAIN_LEN; i++) {
		val = st->vref / 1000;
		/* Multiply by MILLI here to avoid losing precision */
		val = mult_frac(val, ad4000_gains_frac[i][1] * MILLI,
				ad4000_gains_frac[i][0]);
		/* Would multiply by NANO here but we already multiplied by MILLI */
		tmp2 = shift_right((u64)val * MICRO, val2);
		tmp0 = (int)div_s64_rem(tmp2, NANO, &tmp1);
		st->scale_tbl[i][0] = tmp0; /* Integer part */
		st->scale_tbl[i][1] = abs(tmp1); /* Fractional part */
	}
}

static int ad4000_write_reg(struct ad4000_state *st, uint8_t val)
{
	struct spi_transfer t = {
		.tx_buf	= st->data.d8,
		.len = 2,
	};
	struct spi_message m;
	int ret;

	put_unaligned_be16(AD400X_WRITE_COMMAND << BITS_PER_BYTE | val, st->data.d8);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	if (st->cnv_gpio)
		gpiod_set_value_cansleep(st->cnv_gpio, GPIOD_OUT_HIGH);

	ret = spi_sync(st->spi, &m);
	if (ret < 0)
		return ret;

	if (st->cnv_gpio)
		gpiod_set_value_cansleep(st->cnv_gpio, GPIOD_OUT_LOW);

	return 0;
}

static int ad4000_read_reg(struct ad4000_state *st, unsigned int *val)
{
	struct spi_transfer t = {0};
	struct spi_message m;
	int ret;

	st->data.d8[0] = AD400X_READ_COMMAND;

	t.rx_buf = st->data.d8;
	t.tx_buf = st->data.d8;
	t.len = 2;

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->cnv_gpio)
		gpiod_set_value_cansleep(st->cnv_gpio, GPIOD_OUT_HIGH);

	ret = spi_sync(st->spi, &m);
	if (ret < 0)
		return ret;

	if (st->cnv_gpio)
		gpiod_set_value_cansleep(st->cnv_gpio, GPIOD_OUT_LOW);

	*val = FIELD_GET(AD4000_CONFIG_REG_MSK, get_unaligned_be16(st->data.d8));

	return ret;
}

static int ad4000_read_sample(struct ad4000_state *st, uint32_t *val)
{
	struct spi_transfer t = {0};
	struct spi_message m;
	int ret;

	t.rx_buf = &st->data.scan.sample_buf;
	t.len = 4;
	t.delay.value = 60;
	t.delay.unit = SPI_DELAY_UNIT_NSECS;

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->cnv_gpio)
		gpiod_set_value_cansleep(st->cnv_gpio, GPIOD_OUT_HIGH);

	ret = spi_sync(st->spi, &m);
	if (ret < 0)
		return ret;

	if (st->cnv_gpio)
		gpiod_set_value_cansleep(st->cnv_gpio, GPIOD_OUT_LOW);

	*val = get_unaligned_be32(&st->data.scan.sample_buf);

	return 0;
}

static int ad4000_single_conversion(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan, int *val)
{
	struct ad4000_state *st = iio_priv(indio_dev);
	u32 sample;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = ad4000_read_sample(st, &sample);

	iio_device_release_direct_mode(indio_dev);

	if (ret)
		return ret;

	switch (chan->scan_type.realbits) {
	case 16:
		sample = FIELD_GET(AD4000_16BIT_MSK, sample);
		break;
	case 18:
		sample = FIELD_GET(AD4000_18BIT_MSK, sample);
		break;
	case 20:
		sample = FIELD_GET(AD4000_20BIT_MSK, sample);
		break;
	default:
		return -EINVAL;
	}

	if (chan->scan_type.sign == 's')
		*val = sign_extend32(sample, chan->scan_type.realbits - 1);

	return IIO_VAL_INT;
}

static int ad4000_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	struct ad4000_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad4000_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		*val = st->scale_tbl[st->pin_gain][0];
		*val2 = st->scale_tbl[st->pin_gain][1];
		if (st->span_comp)
			*val2 = DIV_ROUND_CLOSEST(*val2 * 4, 5);
		return IIO_VAL_INT_PLUS_NANO;
	default:
		break;
	}

	return -EINVAL;
}

static ssize_t ad4000_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4000_state *st = iio_priv(indio_dev);

	switch ((u32)this_attr->address) {
	case AD4000_STATUS:
		return sysfs_emit(buf, "%d\n", st->status_bits);
	case AD4000_SPAN_COMP:
		return sysfs_emit(buf, "%d\n", st->span_comp);
	case AD4000_HIGHZ:
		return sysfs_emit(buf, "%d\n", st->high_z_mode);
	case AD4000_TURBO:
		return sysfs_emit(buf, "%d\n", st->turbo_mode);
	default:
		return -EINVAL;
	}
}

static ssize_t ad4000_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4000_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	unsigned int reg_val;
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret < 0)
		return ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = ad4000_read_reg(st, &reg_val);
	if (ret < 0)
		goto err_release;

	switch ((u32)this_attr->address) {
	case AD4000_STATUS:
		reg_val &= ~AD4000_STATUS;
		reg_val |= FIELD_PREP(AD4000_STATUS, val);
		ret = ad4000_write_reg(st, reg_val);
		if (ret < 0)
			goto err_release;

		st->status_bits = val;
		break;
	case AD4000_SPAN_COMP:
		reg_val &= ~AD4000_SPAN_COMP;
		reg_val |= FIELD_PREP(AD4000_SPAN_COMP, val);
		ret = ad4000_write_reg(st, reg_val);
		if (ret < 0)
			goto err_release;

		st->span_comp = val;
		break;
	case AD4000_HIGHZ:
		reg_val &= ~AD4000_HIGHZ;
		reg_val |= FIELD_PREP(AD4000_HIGHZ, val);
		ret = ad4000_write_reg(st, reg_val);
		if (ret < 0)
			goto err_release;

		st->high_z_mode = val;
		break;
	case AD4000_TURBO:
		reg_val &= ~AD4000_TURBO;
		reg_val |= FIELD_PREP(AD4000_TURBO, val);
		ret = ad4000_write_reg(st, reg_val);
		if (ret < 0)
			goto err_release;

		st->turbo_mode = val;
		break;
	default:
		ret = -EINVAL;
		goto err_release;
	}

err_release:
	iio_device_release_direct_mode(indio_dev);
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(status_bits_en, 0644, ad4000_show, ad4000_store,
		       AD4000_STATUS);

static IIO_DEVICE_ATTR(span_compression_en, 0644, ad4000_show, ad4000_store,
		       AD4000_SPAN_COMP);

static IIO_DEVICE_ATTR(high_impedance_en, 0644, ad4000_show, ad4000_store,
		       AD4000_HIGHZ);

static IIO_DEVICE_ATTR(turbo_en, 0644, ad4000_show, ad4000_store,
		       AD4000_TURBO);

static struct attribute *ad4000_attributes[] = {
	&iio_dev_attr_status_bits_en.dev_attr.attr,
	&iio_dev_attr_span_compression_en.dev_attr.attr,
	&iio_dev_attr_high_impedance_en.dev_attr.attr,
	&iio_dev_attr_turbo_en.dev_attr.attr,
	NULL
};

static const struct attribute_group ad4000_attribute_group = {
	.attrs = ad4000_attributes,
};

static irqreturn_t ad4000_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4000_state *st = iio_priv(indio_dev);
	struct spi_transfer t = {0};
	struct spi_message m;
	int ret;

	t.rx_buf = &st->data.scan.sample_buf;
	t.len = 4;
	t.delay.value = 60;
	t.delay.unit = SPI_DELAY_UNIT_NSECS;

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->cnv_gpio)
		gpiod_set_value(st->cnv_gpio, GPIOD_OUT_HIGH);

	ret = spi_sync(st->spi, &m);
	if (ret < 0)
		goto err_out;

	if (st->cnv_gpio)
		gpiod_set_value(st->cnv_gpio, GPIOD_OUT_LOW);

	iio_push_to_buffers_with_timestamp(indio_dev, &st->data.scan,
					   iio_get_time_ns(indio_dev));

err_out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static const struct iio_info ad4000_info = {
	.read_raw = &ad4000_read_raw,
	.attrs = &ad4000_attribute_group,
};

static void ad4000_regulator_disable(void *reg)
{
	regulator_disable(reg);
}

static int ad4000_probe(struct spi_device *spi)
{
	const struct ad4000_chip_info *chip;
	struct regulator *vref_reg;
	struct iio_dev *indio_dev;
	struct ad4000_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	chip = (const struct ad4000_chip_info *)device_get_match_data(&spi->dev);
	if (!chip)
		return -EINVAL;

	st = iio_priv(indio_dev);
	st->spi = spi;

	vref_reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(vref_reg))
		return dev_err_probe(&spi->dev, PTR_ERR(vref_reg),
				     "Failed to get vref regulator\n");

	ret = regulator_enable(vref_reg);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to enable voltage regulator\n");

	ret = devm_add_action_or_reset(&spi->dev, ad4000_regulator_disable, vref_reg);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to add regulator disable action\n");

	st->vref = regulator_get_voltage(vref_reg);
	if (st->vref < 0)
		return dev_err_probe(&spi->dev, st->vref, "Failed to get vref\n");

	st->cnv_gpio = devm_gpiod_get_optional(&spi->dev, "cnv", GPIOD_OUT_HIGH);
	if (IS_ERR(st->cnv_gpio)) {
		if (PTR_ERR(st->cnv_gpio) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_dbg(&spi->dev, "Failed to get CNV GPIO: %ld\n",
			PTR_ERR(st->cnv_gpio));
		st->cnv_gpio =  NULL;
	}

	indio_dev->name = chip->dev_name;
	indio_dev->info = &ad4000_info;
	indio_dev->channels = &chip->chan_spec;
	indio_dev->num_channels = 1;

	if (device_property_present(&spi->dev, "adi,gain-milli")) {
		u32 val;

		ret = device_property_read_u32(&spi->dev, "adi,gain-milli", &val);
		if (ret)
			return ret;

		switch (val) {
		case 454:
			st->pin_gain = AD4000_0454_GAIN;
			break;
		case 909:
			st->pin_gain = AD4000_0909_GAIN;
			break;
		case 1000:
			st->pin_gain = AD4000_1_GAIN;
			break;
		case 1900:
			st->pin_gain = AD4000_1900_GAIN;
			break;
		default:
			return dev_err_probe(&spi->dev, -EINVAL,
					     "Invalid firmware provided gain\n");
		}
	} else {
		st->pin_gain = AD4000_1_GAIN;
	}

	/*
	 * ADCs that output twos complement code have one less bit to express
	 * voltage magnitude.
	 */
	if (chip->chan_spec.scan_type.sign == 's')
		ad4000_fill_scale_tbl(st, chip->chan_spec.scan_type.realbits - 1);
	else
		ad4000_fill_scale_tbl(st, chip->chan_spec.scan_type.realbits);

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &ad4000_trigger_handler, NULL);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad4000_id[] = {
	{ "ad4000", (kernel_ulong_t)&ad4000_chips[ID_AD4000] },
	{ "ad4001", (kernel_ulong_t)&ad4000_chips[ID_AD4001] },
	{ "ad4002", (kernel_ulong_t)&ad4000_chips[ID_AD4002] },
	{ "ad4003", (kernel_ulong_t)&ad4000_chips[ID_AD4003] },
	{ "ad4004", (kernel_ulong_t)&ad4000_chips[ID_AD4004] },
	{ "ad4005", (kernel_ulong_t)&ad4000_chips[ID_AD4005] },
	{ "ad4006", (kernel_ulong_t)&ad4000_chips[ID_AD4006] },
	{ "ad4007", (kernel_ulong_t)&ad4000_chips[ID_AD4007] },
	{ "ad4008", (kernel_ulong_t)&ad4000_chips[ID_AD4008] },
	{ "ad4010", (kernel_ulong_t)&ad4000_chips[ID_AD4010] },
	{ "ad4011", (kernel_ulong_t)&ad4000_chips[ID_AD4011] },
	{ "ad4020", (kernel_ulong_t)&ad4000_chips[ID_AD4020] },
	{ "ad4021", (kernel_ulong_t)&ad4000_chips[ID_AD4021] },
	{ "ad4022", (kernel_ulong_t)&ad4000_chips[ID_AD4022] },
	{ "adaq4003", (kernel_ulong_t)&ad4000_chips[ID_ADAQ4003] },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad4000_id);

static const struct of_device_id ad4000_of_match[] = {
	{ .compatible = "adi,ad4000",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4000] },
	{ .compatible = "adi,ad4001",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4001] },
	{ .compatible = "adi,ad4002",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4002] },
	{ .compatible = "adi,ad4003",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4003] },
	{ .compatible = "adi,ad4004",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4004] },
	{ .compatible = "adi,ad4005",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4005] },
	{ .compatible = "adi,ad4006",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4006] },
	{ .compatible = "adi,ad4007",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4007] },
	{ .compatible = "adi,ad4008",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4008] },
	{ .compatible = "adi,ad4010",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4010] },
	{ .compatible = "adi,ad4011",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4011] },
	{ .compatible = "adi,ad4020",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4020] },
	{ .compatible = "adi,ad4021",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4021] },
	{ .compatible = "adi,ad4022",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_AD4022] },
	{ .compatible = "adi,adaq4003",
	  .data = (struct ad4000_chip_info *)&ad4000_chips[ID_ADAQ4003] },
	{ }
};
MODULE_DEVICE_TABLE(of, ad4000_of_match);

static struct spi_driver ad4000_driver = {
	.driver = {
		.name   = "ad4000",
		.of_match_table = ad4000_of_match,
	},
	.probe          = ad4000_probe,
	.id_table       = ad4000_id,
};
module_spi_driver(ad4000_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4000 ADC driver");
MODULE_LICENSE("GPL");
