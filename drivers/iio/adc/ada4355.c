// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices ADA4355 SPI ADC driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/iio/backend.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADA4355_CHIP_CONFIG_REG			0x00
#define ADA4355_CHIP_ID_REG			0x01
#define ADA4355_CHIP_ID				0x8B

#define ADA4355_DEVICE_INDEX_REG		0x05
#define ADA4355_TRANSFER_REG			0xFF
#define ADA4355_TRANSFER_OVERRIDE		BIT(0)

#define ADA4355_POWER_MODES_REG			0x08
#define ADA4355_PW_MODE_DIGITAL_RESET		GENMASK(1, 0)
#define ADA4355_PW_MODE_NORMAL			0xFC

#define ADA4355_TEST_MODE_REG			0x0D
#define ADA4355_TEST_MODE_OFF			0x00
#define ADA4355_TEST_MODE_USER_INPUT		0x48

#define ADA4355_OUTPUT_MODE_REG			0x14
#define ADA4355_OUTPUT_MODE_TWOSCOMP		BIT(0)

#define ADA4355_USER_PATT1_LSB_REG		0x19
#define ADA4355_USER_PATT1_MSB_REG		0x1A
#define ADA4355_USER_PATT2_LSB_REG		0x1B
#define ADA4355_USER_PATT2_MSB_REG		0x1C
#define ADA4355_SERIAL_OUT_DATA_CNTRL_REG	0x21
#define ADA4355_DDR_TWO_LANE_BITWISE		0x20

#define ADA4355_SERIAL_CHANNEL_STATUS_REG	0x22
#define ADA4355_RESOLUTION_SAMPLE_RATE_REG	0x100
#define ADA4355_125_RATE			0x06

//HDL address: 0x32
#define ADA4355_ENABLE_ERROR_MASK		0x00C8

static const int ada4355_scale_table[][2] = {
	{2000, 0}, /* 2V differential range (±1V) for 1V reference ADC */
};

struct ada4355_chip_info {
	const char *name;
	unsigned int id;
	unsigned int max_samp_rate_hz;
};

static const struct ada4355_chip_info ada4355_chip_info = {
	.name = "ADA4355",
	.id = ADA4355_CHIP_ID,
	.max_samp_rate_hz = 125000000UL,
};

struct ada4355_state {
	struct iio_backend *back;
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clk;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex		lock;
	const struct ada4355_chip_info *chip_info;
	unsigned int		num_lanes;
};

static const struct regmap_config ada4355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static struct ada4355_state *ada4355_get_data(struct iio_dev *indio_dev)
{
	struct ada4355_state *st = iio_priv(indio_dev);


	return st;
};

static int ada4355_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				      unsigned int writeval, unsigned int *readval)
{
	struct ada4355_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ada4355_get_scale(struct ada4355_state *st, int *val, int *val2)
{
	unsigned int tmp;

	tmp = (ada4355_scale_table[0][0] * 1000000ULL) >> 14;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ada4355_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long m)
{
	struct ada4355_state *st = ada4355_get_data(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		return ada4355_get_scale(st, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(st->clk);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ada4355_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return -EINVAL;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return -EINVAL;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ada4355_update_scan_mode(struct iio_dev *indio_dev,
				    const unsigned long *scan_mask)
{
	struct ada4355_state *st = iio_priv(indio_dev);
	unsigned int c;
	int ret;

	for (c = 0; c < iio_get_masklength(indio_dev); c++) {
		if (test_bit(c, scan_mask))
			ret = iio_backend_chan_enable(st->back, c);
		else
			ret = iio_backend_chan_disable(st->back, c);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct iio_info ada4355_iio_info = {
	.read_raw = ada4355_read_raw,
	.update_scan_mode = ada4355_update_scan_mode,
	.debugfs_reg_access = ada4355_debugfs_reg_access,
};

#define ADA4355_CHAN(_chan, _si, _bits, _sign, _shift)			\
{									\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _chan,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _si,						\
	.scan_type = {							\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = _shift,					\
	},								\
}

static const struct iio_chan_spec ada4355_channels[] = {
	ADA4355_CHAN(0, 0, 14, 's', 2),
	ADA4355_CHAN(1, 1, 14, 's', 2),
};

static int ada4355_post_setup(struct iio_dev *indio_dev)
{
	struct ada4355_state *st = ada4355_get_data(indio_dev);
	u8 pn_status[3][32];
	int opt_delay, c, s;
	int ret;
	unsigned int reg_cntrl;
	unsigned int i;
	unsigned int j;
	unsigned int delay;
	unsigned int val;

	// Set the numbers of lanes
	ret = iio_backend_num_lanes_set(st->back, st->num_lanes);
	if (ret)
		return ret;

	// enable the sync
	ret = iio_backend_interface_data_align(st->back, 1000);

	ret = iio_backend_chan_enable(st->back, 0);
	ret = iio_backend_chan_enable(st->back, 1);

	ret = iio_backend_frame_setup(st->back, st->num_lanes);
	if (ret)
		return ret;

	ret = iio_backend_chan_disable(st->back, 0);
	ret = iio_backend_chan_disable(st->back, 1);

	ret = regmap_write(st->regmap, ADA4355_TEST_MODE_REG, ADA4355_TEST_MODE_OFF);
	if (ret)
		return ret;

	return 0;
}

static int ada4355_setup(struct ada4355_state *st)
{
	unsigned int reg, id;
	int ret;

	struct gpio_desc *gpio_vld_en;

	ret = regmap_write(st->regmap,  ADA4355_POWER_MODES_REG,
			   ADA4355_PW_MODE_DIGITAL_RESET);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADA4355_POWER_MODES_REG, &reg);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap,  ADA4355_POWER_MODES_REG,
			  (reg & ADA4355_PW_MODE_NORMAL));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_CHIP_CONFIG_REG, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_DEVICE_INDEX_REG, 0x02);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_SERIAL_CHANNEL_STATUS_REG, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_DEVICE_INDEX_REG, 0x31);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADA4355_CHIP_ID_REG, &id);
	if (ret)
		return ret;

	if (id != ADA4355_CHIP_ID) {
		dev_err(&st->spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		return -EINVAL;
	}

	ret = regmap_read(st->regmap, ADA4355_SERIAL_OUT_DATA_CNTRL_REG, &reg);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_SERIAL_OUT_DATA_CNTRL_REG,
		(reg & ADA4355_DDR_TWO_LANE_BITWISE) | ADA4355_DDR_TWO_LANE_BITWISE);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADA4355_TRANSFER_REG, &reg);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_TRANSFER_REG, (reg | ADA4355_TRANSFER_OVERRIDE));
	if (ret)
		return ret;

	// Set User input mode
	ret = regmap_write(st->regmap, ADA4355_TEST_MODE_REG, ADA4355_TEST_MODE_USER_INPUT);
	if (ret)
		return ret;

	// Write test_pattern = 0xFFFC;
	ret = regmap_write(st->regmap, ADA4355_USER_PATT1_MSB_REG, 0xFF);
	if (ret)
		return ret;
	ret = regmap_write(st->regmap, ADA4355_USER_PATT1_LSB_REG, 0xFC);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_USER_PATT2_MSB_REG, 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_USER_PATT2_LSB_REG, 0xFC);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADA4355_TEST_MODE_REG, &reg);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_OUTPUT_MODE_REG, ADA4355_OUTPUT_MODE_TWOSCOMP);

	ret = regmap_read(st->regmap, ADA4355_OUTPUT_MODE_REG, &reg);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_RESOLUTION_SAMPLE_RATE_REG, ADA4355_125_RATE);
		return ret;

	gpio_vld_en = devm_gpiod_get_optional(&st->spi->dev, "vld-en", GPIOD_OUT_LOW);
	if (IS_ERR(gpio_vld_en))
		return dev_err_probe(&st->spi->dev, PTR_ERR(gpio_vld_en),
				     "Failed to find vld-en-gpios \n");

	gpiod_set_value_cansleep(gpio_vld_en, 1);
}

static int ada4355_properties_parse(struct ada4355_state *st)
{
	struct spi_device *spi = st->spi;
	unsigned int val;
	int ret;

	ret = of_property_read_u32(spi->dev.of_node, "num_lanes", &val);
	if (!ret)
		st->num_lanes = val;
	else
		st->num_lanes = 1;

	return 0;
}

static int ada4355_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct ada4355_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return -EINVAL;

	regmap = devm_regmap_init_spi(spi, &ada4355_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st->regmap = regmap;
	st->spi = spi;

	ret = devm_mutex_init(&spi->dev, &st->lock);
	if (ret)
		return ret;


	ret = ada4355_properties_parse(st);
		if (ret)
			return ret;

	st->clk = devm_clk_get_enabled(&spi->dev, NULL);
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	ret = ada4355_setup(st);
	if (ret < 0)
		return ret;

	st->back = devm_iio_backend_get(&spi->dev, NULL);
	if (IS_ERR(st->back))
		return dev_err_probe(&spi->dev, PTR_ERR(st->back),
				     "failed to get IIO backend\n");

	ret = devm_iio_backend_request_buffer(&spi->dev, st->back, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_enable(&spi->dev, st->back);
	if (ret)
		return ret;

	ret = ada4355_post_setup(indio_dev);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret, "failed post_setup");

	indio_dev->name = st->chip_info->name;
	indio_dev->channels = ada4355_channels;
	indio_dev->num_channels = ARRAY_SIZE(ada4355_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ada4355_iio_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ada4355_id[] = {
	{ .name = "ada4355", .driver_data = (kernel_ulong_t)&ada4355_chip_info },
	{ .name = "ada4356", .driver_data = (kernel_ulong_t)&ada4355_chip_info },
	{ }
};

MODULE_DEVICE_TABLE(spi, ada4355_id);

static const struct of_device_id ada4355_of_match[] = {
	{ .compatible = "adi,ada4355", .data = &ada4355_chip_info },
	{ .compatible = "adi,ada4356", .data = &ada4355_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ada4355_of_match);

static struct spi_driver ada4355_driver = {
	.driver = {
		.name = "ada4355",
		.of_match_table = ada4355_of_match,
	},
	.probe = ada4355_probe,
	.id_table = ada4355_id,
};
module_spi_driver(ada4355_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_AUTHOR("Pop Ioan Daniel <pop.ioan-daniel@analog.com>");
MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADA4355");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_BACKEND");
