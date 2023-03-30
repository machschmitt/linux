// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7091R8 12-bit SAR ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "ad7091r-base.h"

#define AD7091R_SPI_CHIP_INFO(n) {					\
	.type =	AD7091R##n,						\
	.channels = ad7091r##n##_channels,				\
	.num_channels = ARRAY_SIZE(ad7091r##n##_channels),		\
	.vref_mV = 2500,						\
}

static const struct iio_chan_spec ad7091r2_channels[] = {
	AD7091R_CHANNEL(0, 12, NULL, 0),
	AD7091R_CHANNEL(1, 12, NULL, 0),
};

static const struct iio_chan_spec ad7091r4_channels[] = {
	AD7091R_CHANNEL(0, 12, NULL, 0),
	AD7091R_CHANNEL(1, 12, NULL, 0),
	AD7091R_CHANNEL(2, 12, NULL, 0),
	AD7091R_CHANNEL(3, 12, NULL, 0),
};

static const struct iio_chan_spec ad7091r8_channels[] = {
	AD7091R_CHANNEL(0, 12, NULL, 0),
	AD7091R_CHANNEL(1, 12, NULL, 0),
	AD7091R_CHANNEL(2, 12, NULL, 0),
	AD7091R_CHANNEL(3, 12, NULL, 0),
	AD7091R_CHANNEL(4, 12, NULL, 0),
	AD7091R_CHANNEL(5, 12, NULL, 0),
	AD7091R_CHANNEL(6, 12, NULL, 0),
	AD7091R_CHANNEL(7, 12, NULL, 0),
};

static const struct regmap_range ad7091r2_readable_ranges[] = {
	regmap_reg_range(AD7091R_REG_RESULT,
			 AD7091R_REG_CH_HYSTERESIS(AD7091R2_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r4_readable_ranges[] = {
	regmap_reg_range(AD7091R_REG_RESULT,
			 AD7091R_REG_CH_HYSTERESIS(AD7091R4_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r8_readable_ranges[] = {
	regmap_reg_range(AD7091R_REG_RESULT,
			 AD7091R_REG_CH_HYSTERESIS(AD7091R8_NUM_CHANNELS)),
};

static const struct regmap_access_table ad7091r2_readable_regs_table = {
	.yes_ranges = ad7091r2_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r2_readable_ranges),
};

static const struct regmap_access_table ad7091r4_readable_regs_table = {
	.yes_ranges = ad7091r4_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r4_readable_ranges),
};

static const struct regmap_access_table ad7091r8_readable_regs_table = {
	.yes_ranges = ad7091r8_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r8_readable_ranges),
};

static const struct regmap_range ad7091r2_writable_ranges[] = {
	regmap_reg_range(AD7091R_REG_CHANNEL, AD7091R_REG_CONF),
	regmap_reg_range(AD7091R_REG_CH_LOW_LIMIT(0),
			 AD7091R_REG_CH_HYSTERESIS(AD7091R2_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r4_writable_ranges[] = {
	regmap_reg_range(AD7091R_REG_CHANNEL, AD7091R_REG_CONF),
	regmap_reg_range(AD7091R_REG_CH_LOW_LIMIT(0),
			 AD7091R_REG_CH_HYSTERESIS(AD7091R4_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r8_writable_ranges[] = {
	regmap_reg_range(AD7091R_REG_CHANNEL, AD7091R_REG_CONF),
	regmap_reg_range(AD7091R_REG_CH_LOW_LIMIT(0),
			 AD7091R_REG_CH_HYSTERESIS(AD7091R8_NUM_CHANNELS)),
};

static const struct regmap_access_table ad7091r2_writable_regs_table = {
	.yes_ranges = ad7091r2_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r2_writable_ranges),
};

static const struct regmap_access_table ad7091r4_writable_regs_table = {
	.yes_ranges = ad7091r4_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r4_writable_ranges),
};

static const struct regmap_access_table ad7091r8_writable_regs_table = {
	.yes_ranges = ad7091r8_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r8_writable_ranges),
};

static const struct ad7091r_chip_info ad7091r_spi_chip_info[] = {
	[AD7091R2] = AD7091R_SPI_CHIP_INFO(2),
	[AD7091R4] = AD7091R_SPI_CHIP_INFO(4),
	[AD7091R8] = AD7091R_SPI_CHIP_INFO(8),
};

static const struct regmap_config ad7091r_spi_regmap_config[] = {
	[AD7091R2] = {
		.reg_bits = 8,
		.val_bits = 16,
		.rd_table = &ad7091r2_readable_regs_table,
		.wr_table = &ad7091r2_writable_regs_table,
		.max_register = AD7091R_REG_CH_HYSTERESIS(2),
	},
	[AD7091R4] = {
		.reg_bits = 8,
		.val_bits = 16,
		.rd_table = &ad7091r4_readable_regs_table,
		.wr_table = &ad7091r4_writable_regs_table,
		.max_register = AD7091R_REG_CH_HYSTERESIS(4),
	},
	[AD7091R8] = {
		.reg_bits = 8,
		.val_bits = 16,
		.rd_table = &ad7091r8_readable_regs_table,
		.wr_table = &ad7091r8_writable_regs_table,
		.max_register = AD7091R_REG_CH_HYSTERESIS(8),
	},
};

static int ad7091r8_spi_probe(struct spi_device *spi)
{
	const struct ad7091r_chip_info *chip_info;
	struct ad7091r_state *st;
	struct iio_dev *iio_dev;
	struct regmap *map;

	chip_info = device_get_match_data(&spi->dev);
	if (!chip_info)
		chip_info = (const struct ad7091r_chip_info *)spi_get_device_id(spi)->driver_data;

	iio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!iio_dev)
		return -ENOMEM;

	st = iio_priv(iio_dev);
	st->dev = &spi->dev;

	map = devm_regmap_init_spi(spi,
				   &ad7091r_spi_regmap_config[chip_info->type]);

	if (IS_ERR(map))
		return PTR_ERR(map);

	return ad7091r_probe(iio_dev, spi_get_device_id(spi)->name, chip_info,
			     map, 0);
}

static const struct of_device_id ad7091r8_of_match[] = {
	{ .compatible = "adi,ad7091r2", .data = &ad7091r_spi_chip_info[AD7091R2] },
	{ .compatible = "adi,ad7091r4", .data = &ad7091r_spi_chip_info[AD7091R4] },
	{ .compatible = "adi,ad7091r8", .data = &ad7091r_spi_chip_info[AD7091R8] },
	{ },
};
MODULE_DEVICE_TABLE(of, ad7091r8_of_match);

static const struct spi_device_id ad7091r8_spi_id[] = {
	{ "adi,ad7091r2", (kernel_ulong_t)&ad7091r_spi_chip_info[AD7091R2] },
	{ "adi,ad7091r4", (kernel_ulong_t)&ad7091r_spi_chip_info[AD7091R4] },
	{ "adi,ad7091r8", (kernel_ulong_t)&ad7091r_spi_chip_info[AD7091R8] },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad7091r8_spi_id);

static struct spi_driver ad7091r8_driver = {
	.driver = {
		.name = "ad7091r8",
		.of_match_table = ad7091r8_of_match,
	},
	.probe = ad7091r8_spi_probe,
	.id_table = ad7091r8_spi_id,
};
module_spi_driver(ad7091r8_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7091R8 ADC driver");
MODULE_LICENSE("GPL v2");
