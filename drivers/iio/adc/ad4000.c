// SPDX-License-Identifier: GPL-2.0+
/*
 * AD4000 SPI ADC driver
 *
 * Copyright 2018 Analog Devices Inc.
 */
#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
//#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
//#include <linux/spi/spi-engine.h>
#include <linux/sysfs.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#define AD400X_READ_COMMAND	0x54
#define AD400X_WRITE_COMMAND	0x14
#define AD400X_RESERVED_MSK	0xE0

#define AD400X_TURBO_MODE(x)	FIELD_PREP(BIT_MASK(1), x)
#define AD400X_HIGH_Z_MODE(x)	FIELD_PREP(BIT_MASK(2), x)

#define AD400X_CHANNEL(real_bits)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE) |			\
			BIT(IIO_CHAN_INFO_OFFSET),			\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = real_bits,				\
			.storagebits = 32,				\
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

enum ad4000_input_type {
	SINGLE_ENDED,
	DIFFERENTIAL,
};

struct ad4000_chip_info {
	struct iio_chan_spec chan_spec;
	int max_rate;
	enum ad4000_input_type input_type;
};

static const struct ad4000_chip_info ad4000_chips[] = {
	[ID_AD4000] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 2000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4001] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 2000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4002] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4003] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4004] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 1000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4005] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 1000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4006] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 1000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4007] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 1000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4008] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  =  500000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4010] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  =  500000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4011] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  =  500000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4020] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  = 1800000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4021] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  = 1000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4022] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  =  500000,
		.input_type = DIFFERENTIAL,
	},
	[ID_ADAQ4003] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
		.input_type = DIFFERENTIAL,
	},
};

struct ad4000_state {
	struct spi_device *spi;
	struct regulator *vref;
	/* protect device accesses */
	struct mutex lock;

	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;

	const struct ad4000_chip_info *chip;
	struct gpio_desc *cnv_gpio;
	bool turbo_mode;
	bool high_z_mode;

	unsigned int num_bits;
	int read_offset;

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

static int ad4000_write_reg(struct ad4000_state *st, uint8_t val)
{
	struct spi_transfer t = {
		.tx_buf	= st->data.d8,
		.len = 2,
	};
	struct spi_message m;

	put_unaligned_be16(AD400X_WRITE_COMMAND << BITS_PER_BYTE | val, st->data.d8);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(st->spi, &m);
}

static int ad4000_read_reg(struct ad4000_state *st, unsigned int *val)
{
	struct spi_message m;
	struct spi_transfer t = {0};
	int ret;

	st->data.d8[0] = AD400X_READ_COMMAND;

	t.rx_buf = st->data.d8;
	t.tx_buf = st->data.d8;
	t.len = 2;

	spi_message_init_with_transfers(&m, &t, 1);

	ret = spi_sync(st->spi, &m);
	if (ret < 0)
		return ret;

	*val = get_unaligned_be16(st->data.d8);

	return ret;
}

static int ad4000_read_sample(struct ad4000_state *st, uint32_t *val)
{
	struct spi_message m;
	struct spi_transfer t = {0};
	int ret;

	t.rx_buf = &st->data.scan.sample_buf;
	if (st->num_bits <= 24) // TODO if(num_bits + status <= 24)
		t.len = 3;
	else
		t.len = 4;
	t.delay.value = 60;
	t.delay.unit = SPI_DELAY_UNIT_NSECS;

	spi_message_init_with_transfers(&m, &t, 1);

	gpiod_set_value_cansleep(st->cnv_gpio, 1);

	ret = spi_sync(st->spi, &m);
	if (ret < 0)
		return ret;

	gpiod_set_value_cansleep(st->cnv_gpio, 0);

	if (st->num_bits <= 24) // TODO if(num_bits + status <= 24)
		*val = get_unaligned_be24(&st->data.scan.sample_buf);
	else
		*val = get_unaligned_be32(&st->data.scan.sample_buf);

	return 0;
}

static int ad4000_set_mode(struct ad4000_state *st)
{
	uint8_t mode;
	int ret;

	mode = AD400X_TURBO_MODE(st->turbo_mode) |
		AD400X_HIGH_Z_MODE(st->high_z_mode);

	ret = ad4000_write_reg(st, mode);

	return ret;
}

static int ad4000_single_conversion(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val)
{
	struct ad4000_state *st = iio_priv(indio_dev);
	unsigned int sample, raw_sample;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = ad4000_read_sample(st, &raw_sample);

	iio_device_release_direct_mode(indio_dev);

	if (ret)
		return ret;

	sample = raw_sample >> chan->scan_type.shift;
	if (st->chip->input_type == DIFFERENTIAL)
		*val = sign_extend32(sample, st->num_bits - 1);

	return IIO_VAL_INT;
}

static int ad4000_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ad4000_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad4000_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		*val = st->read_offset;
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int ad4000_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad4000_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	if (readval)
		ret = ad4000_read_reg(st, readval);
	else
		ret = ad4000_write_reg(st, writeval);

	mutex_unlock(&st->lock);

	return ret;
}

static int ad4000_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad4000_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_OFFSET:
		st->read_offset = val;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad4000_info = {
	.read_raw = &ad4000_read_raw,
	.write_raw = &ad4000_write_raw,
	.debugfs_reg_access = &ad4000_reg_access,
};

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


static void ad4000_regulator_disable(void *reg)
{
	regulator_disable(reg);
}

static int ad4000_probe(struct spi_device *spi)
{
	const struct ad4000_chip_info *chip;
	struct ad4000_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	chip = (const struct ad4000_chip_info *)device_get_match_data(&spi->dev);
	if (!chip)
		return -EINVAL;

	st = iio_priv(indio_dev);
	st->chip = chip;
	st->spi = spi;
	mutex_init(&st->lock);

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad4000_regulator_disable, st->vref);
	if (ret)
		return ret;

	st->cnv_gpio = devm_gpiod_get_optional(&st->spi->dev, "cnv",
					       GPIOD_OUT_LOW);
	if (IS_ERR(st->cnv_gpio))
		return dev_err_probe(&st->spi->dev, PTR_ERR(st->cnv_gpio),
				     "Error on requesting cnv GPIO\n");

	dev_info(&st->spi->dev, "st->cnv_gpio: %p", st->cnv_gpio);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->info = &ad4000_info;
	indio_dev->channels = &st->chip->chan_spec;
	indio_dev->num_channels = 1;

	st->num_bits = indio_dev->channels->scan_type.realbits;

	/* Set turbo mode */
	st->turbo_mode = true;
	ret = ad4000_set_mode(st);
	if (ret < 0)
		return ret;


	return devm_iio_device_register(&spi->dev, indio_dev);
}

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
