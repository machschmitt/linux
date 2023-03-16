// SPDX-License-Identifier: GPL-2.0+
/*
 * LTC6946 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/gcd.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#define LTC6946_RD_LB_REG		0x04
#define LTC6946_RD_HB_REG		0x03
#define LTC6946_RD_HB_MSK		GENMASK(1, 0)
#define LTC6946_RD(x)			FIELD_PREP(LTC6946_RD_HB_MSK, (x))

#define LTC6946_ND_LB_REG		0x06
#define LTC6946_ND_HB_REG		0x05

#define LTC6946_OD_REG			0x08
#define LTC6946_OD_REG_MSK		GENMASK(2, 0)
#define LTC6946_OD(x)			FIELD_PREP(LTC6946_OD_REG_MSK, (x))

#define LTC6946_RD_MAX			1023

#define LTC6946_OD_MIN			1
#define LTC6946_OD_MAX			6

#define LTC6946_FREF_MAX		250000000
#define LTC6946_FREF_MIN		10000000

#define LTC6946_FVCO_MAX		3740000000
#define LTC6946_FRF_MAX			LTC6946_FVCO_MAX
#define LTC6946_FRF_MIN			373333333

enum supported_parts {
	LTC6946,
};

struct ltc6946 {
    struct regmap *regmap;
    struct clk_hw clk_hw;
	struct clk *fref_clk;
	unsigned long fref;
	unsigned int r_div;
	unsigned int n_div;
	unsigned int o_div;
};

static const struct iio_chan_spec ltc6946_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),
	},
};

static const struct regmap_config ltc6946_regmap_config = {
	.reg_bits = 7,
	.pad_bits = 1,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
	.max_register = 0x0B,
};

static int ltc6946_setup(struct iio_dev *indio_dev)
{
	struct ltc6946 *dev = iio_priv(indio_dev);

	regmap_write(dev->regmap, 0x01,	0x00);
	regmap_write(dev->regmap, 0x02,	0x0A);

	/* ref = 250 MHz, vco = 3750 MHz, rf = 3750 MHz*/
	regmap_write(dev->regmap, 0x03,	0x00);
	regmap_write(dev->regmap, 0x04,	0x19);
	regmap_write(dev->regmap, 0x05,	0x01);
	regmap_write(dev->regmap, 0x06,	0x77);

	/* Enable ALC Monitor for Status Flags Only, Start VCO Calibration */
	regmap_write(dev->regmap, 0x07,	0x63);

	// Output Divider Value = 1
	regmap_write(dev->regmap, 0x08,	0x81);
	regmap_write(dev->regmap, 0x09,	0xF0);
	regmap_write(dev->regmap, 0x0A,	0x00);

	// Power down PLL and just use the VCO portion to lock the LTC6952
	regmap_write(dev->regmap, 0x02,	0x08);

	return 0;
}

static int ltc6946_reg_access(struct iio_dev *indio_dev,
	unsigned int reg,
	unsigned int tx_val,
	unsigned int *rx_val)
{
	struct ltc6946 *dev = iio_priv(indio_dev);

	if (rx_val)
		regmap_read(dev->regmap, reg, rx_val);
	else
		regmap_write(dev->regmap, reg, tx_val);

	return 0;
}

static unsigned long ltc6946_calc_dividers(struct ltc6946 *dev,
					   unsigned long rate)
{
	unsigned long goal_rate, rate_fref_gcd, rd_times_od;
	int i;

	/* Limit the rate to the frequency range allowed by hardware */
	goal_rate = clamp_t(unsigned long, rate, LTC6946_FRF_MIN, LTC6946_FRF_MAX);

	/*
	 * Calculate the dividers to scale Fref to a value close to goal_rate.
	 * If we're lucky, goal_rate and Fref will have a common divisor.
	 * If so, we might be able to output the exact goal_rate frequency by
	 * finding a common divisor and setting the appropriate ND, RD, and OD.
	 * First we find the Greatest Common Divisor (GCD) between Fref and
	 * goal_rate.
	 */
	rate_fref_gcd = gcd(goal_rate, dev->fref);
	/*
	 * We want to find RD and OD such that RD * OD = rate_fref_gcd.
	 * However, maximum RD * OD is 6138, so we look for the greatest common
	 * factor (divisor) of 6138 and rate_fref_gcd.
	 */
	rd_times_od = gcd(LTC6946_RD_MAX * LTC6946_OD_MAX, rate_fref_gcd);
	/* We take the greatest possible OD value that is a factor of rd_times_od */
	for (i = LTC6946_OD_MAX; i > 0; i--) {
		if (rd_times_od % i == 0) {
			dev->o_div = i;
			break;
		}
	}
	dev->r_div = rd_times_od / dev->o_div;
	/* Now, ND is given by: goal_rate / (Fref / (RD * OD)) */
	dev->n_div = goal_rate / (dev->fref / rd_times_od);

	/* Output for debug/test */
	pr_info("ltc6946: rate %lu\n", rate);
	pr_info("ltc6946: goal_rate %lu\n", goal_rate);
	pr_info("ltc6946: Fref %lu\n", dev->fref);
	pr_info("ltc6946: rate_fref_gcd %lu\n", rate_fref_gcd);
	pr_info("ltc6946: rd_times_od %lu\n", rd_times_od);

	pr_info("ltc6946: o_div %u\n", dev->o_div);
	pr_info("ltc6946: r_div %u\n", dev->r_div);
	pr_info("ltc6946: n_div %u\n", dev->n_div);

	pr_info("ltc6946: Fvco %lu\n", (dev->fref * dev->n_div) / dev->r_div);
	pr_info("ltc6946: Frf %lu\n", ((dev->fref * dev->n_div) / dev->r_div) / dev->o_div);
	pr_info("==================\n\n");

	return ((dev->fref * dev->n_div) / dev->r_div) / dev->o_div;
}

static unsigned long ltc6946_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	struct ltc6946 *dev = container_of(clk_hw, struct ltc6946, clk_hw);
	unsigned int n_div_reg, r_div_reg, o_div_reg, aux;
	unsigned long n_div, r_div, o_div;
	int ret;

	ret = regmap_read(dev->regmap, LTC6946_ND_LB_REG, &n_div_reg);
	if (ret < 0)
		return ret;

	ret = regmap_read(dev->regmap, LTC6946_ND_HB_REG, &aux);
	if (ret < 0)
		return ret;

	n_div = aux << 8 | n_div_reg;

	ret = regmap_read(dev->regmap, LTC6946_RD_LB_REG, &r_div_reg);
	if (ret < 0)
		return ret;

	ret = regmap_read(dev->regmap, LTC6946_RD_HB_REG, &aux);
	if (ret < 0)
		return ret;

	r_div = FIELD_GET(LTC6946_RD_HB_MSK, aux) << 8 | r_div_reg;

	ret = regmap_read(dev->regmap, LTC6946_OD_REG, &o_div_reg);
	if (ret < 0)
		return ret;

	o_div = FIELD_GET(LTC6946_OD_REG_MSK, o_div_reg);

	return ((dev->fref * n_div) / r_div) / o_div;
}

static long ltc6946_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	struct ltc6946 *dev = container_of(clk_hw, struct ltc6946, clk_hw);

	return ltc6946_calc_dividers(dev, rate);
}

static int ltc6946_set_rate(struct clk_hw *clk_hw, unsigned long rate,
	unsigned long parent_rate)
{
	struct ltc6946 *dev = container_of(clk_hw, struct ltc6946, clk_hw);
	unsigned int r_div_reg, o_div_reg;
	int ret;

	ret = regmap_write(dev->regmap, LTC6946_ND_LB_REG, dev->n_div);
	if (ret < 0)
		return ret;

	ret = regmap_write(dev->regmap, LTC6946_ND_HB_REG, dev->n_div >> 8);
	if (ret < 0)
		return ret;

	ret = regmap_read(dev->regmap, LTC6946_OD_REG, &o_div_reg);
	if (ret < 0)
		return ret;

	o_div_reg &= ~LTC6946_OD_REG_MSK;
	o_div_reg |= LTC6946_OD(dev->o_div);
	ret = regmap_write(dev->regmap, LTC6946_OD_REG, o_div_reg);
	if (ret < 0)
		return ret;

	ret = regmap_write(dev->regmap, LTC6946_RD_LB_REG, dev->r_div);
	if (ret < 0)
		return ret;

	ret = regmap_read(dev->regmap, LTC6946_RD_HB_REG, &r_div_reg);
	if (ret < 0)
		return ret;

	r_div_reg &= ~LTC6946_RD_HB_MSK;
	r_div_reg |= LTC6946_RD(dev->r_div >> 8);
	ret = regmap_write(dev->regmap, LTC6946_RD_HB_REG, r_div_reg);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct clk_ops ltc6946_clk_ops = {
	.recalc_rate = ltc6946_recalc_rate,
	.round_rate = ltc6946_round_rate,
	.set_rate = ltc6946_set_rate,
};

static void ltc6946_clk_disable(void *data)
{
	struct ltc6946 *dev = data;

	clk_disable_unprepare(dev->fref_clk);
}

static int ltc6946_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct ltc6946 *dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		dev_info(&indio_dev->dev, "%s write_raw\n", indio_dev->name);
		ltc6946_calc_dividers(dev, val);
		ltc6946_set_rate(&dev->clk_hw, val, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct iio_info ltc6946_info = {
	.write_raw = &ltc6946_write_raw,
	.debugfs_reg_access = &ltc6946_reg_access,
};

static int ltc6946_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct clk_init_data init;
	struct regmap *regmap;
	struct ltc6946 *dev;
	struct clk *fref_clk;
	struct clk *clk;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &ltc6946_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	spi_set_drvdata(spi, indio_dev);

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ltc6946_info;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->channels = ltc6946_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltc6946_channels);

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	ltc6946_setup(indio_dev);

	fref_clk = devm_clk_get(&spi->dev, "fref");

	if (IS_ERR(fref_clk))
		return PTR_ERR(fref_clk);

	ret = clk_prepare_enable(fref_clk);
	if (ret < 0)
		return ret;

	dev->fref = clk_get_rate(fref_clk);

	ret = devm_add_action_or_reset(&spi->dev, ltc6946_clk_disable, dev);
	if (ret)
		return ret;

	dev->fref_clk = fref_clk;
	if (dev->fref < LTC6946_FREF_MIN || dev->fref > LTC6946_FREF_MAX)
		return -EINVAL;

	init.name = spi->dev.of_node->name;
	init.ops = &ltc6946_clk_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;

	dev->clk_hw.init = &init;

	clk = devm_clk_register(&spi->dev, &dev->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	dev_info(&spi->dev, "%s probed\n", indio_dev->name);

	return of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk);
}

static int ltc6946_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ltc6946_id[] = {
	{ "ltc6946", LTC6946 },
	{}
};
MODULE_DEVICE_TABLE(spi, ltc6946_id);

static const struct of_device_id ltc6946_of_match[] = {
	{ .compatible = "adi,ltc6946" },
	{},
};
MODULE_DEVICE_TABLE(of, ltc6946_of_match);

static struct spi_driver ltc6946_driver = {
	.driver = {
			.name = "ltc6946",
			.of_match_table = of_match_ptr(ltc6946_of_match),
		},
	.probe = ltc6946_probe,
	.remove = ltc6946_remove,
	.id_table = ltc6946_id,
};
module_spi_driver(ltc6946_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC6946");
MODULE_LICENSE("GPL v2");