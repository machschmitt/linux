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

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

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

struct ada4355_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clk;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex		lock;
	unsigned int		num_lanes;
};

static const struct regmap_config ada4355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static struct ada4355_state *ada4355_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	conv = iio_device_get_drvdata(indio_dev);

	return conv->phy;
};

static int ada4355_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct ada4355_state *st = ada4355_get_data(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ada4355_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	unsigned int tmp;

	tmp = (conv->chip_info->scale_table[0][0] * 1000000ULL) >>
	       conv->chip_info->channel[0].scan_type.realbits;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ada4355_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ada4355_state *st = ada4355_get_data(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		return ada4355_get_scale(conv, val, val2);
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

#define ADA4355_CHAN(_chan, _si, _bits, _sign, _shift) {	 \
	.type = IIO_VOLTAGE,					 \
	.indexed = 1,						 \
	.channel = _chan,					 \
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),		 \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = _si,					 \
	.scan_type = {						 \
		.sign = _sign,					 \
		.realbits = _bits,				 \
		.storagebits = 16,				 \
		.shift = _shift,				 \
		},						 \
	}

static const struct axiadc_chip_info ada4355_chip_info = {
	.name = "ADA4355",
	.id = ADA4355_CHIP_ID,
	.max_rate = 125000000UL,
	.scale_table = ada4355_scale_table,
	.num_scales = ARRAY_SIZE(ada4355_scale_table),
	.num_channels = 1,
	.channel[0] = ADA4355_CHAN(0, 0, 14, 's', 2),
};

int find_opt(u8 *field, u32 size, u32 *ret_start)
{
	int i, cnt = 0, max_cnt = 0, start, max_start = 0;

	for (i = 0, start = -1; i < size; i++) {
		if (field[i] == 0) {
			if (start == -1)
				start = i;
			cnt++;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}
				start = -1;
				cnt = 0;
		}
	}

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}

	*ret_start = max_start;

	return max_cnt;
}

static int ada4355_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct ada4355_state *st = ada4355_get_data(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	u8 pn_status[3][32];
	int opt_delay, c, s;
	int ret;
	unsigned int reg_cntrl;
	unsigned int i;
	unsigned int j;
	unsigned int delay;
	unsigned int val;

	// Set the numbers of lanes
	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);

	// enable the sync
	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
	reg_cntrl |= ADI_SYNC;
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);
	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);

	axiadc_write(axi_adc_st, ADI_REG_CHAN_CNTRL(0), ADI_ENABLE);

	// frame calibration
	axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, (1 << 2));
	for (delay = 0; delay < 32; delay++) {
		val = axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0));
		axiadc_write(axi_adc_st, ADI_REG_CHAN_STATUS(0), val);
		axiadc_write(axi_adc_st, 0x808, delay);
		mdelay(1);
		if (axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0)) & ADI_PN_ERR)
			pn_status[0][delay] = 1;
		else
			pn_status[0][delay] = 0;
	}

	dev_info(&conv->spi->dev, "digital interface frame tuning:\n");

	pr_cont("  ");
	for (i = 0; i < 31; i++)
		pr_cont("%02d:", i);
	pr_cont("31\n");

	pr_info("%x:", 0);
	for (j = 0; j < 32; j++) {
		if (pn_status[0][j])
			pr_cont(" # ");
		else
			pr_cont(" o ");
	}
	pr_cont("\n");

	c = find_opt(&pn_status[0][0], 32, &s);
	opt_delay = s + c / 2;
	axiadc_write(axi_adc_st, 0x808, opt_delay);
	dev_info(&conv->spi->dev, "frame lane : selected delay: %d\n", opt_delay);

	axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, 0);

	// data calibration
	for (i = 0; i < (st->num_lanes); i++) {
		axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, (1 << i));

		for (delay = 0; delay < 32; delay++) {
			val = axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0));
			axiadc_write(axi_adc_st, ADI_REG_CHAN_STATUS(0), val);
			axiadc_write(axi_adc_st, 0x800 + (i * 4), delay);
			mdelay(1);
			if (axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0)) & ADI_PN_ERR)
				pn_status[i][delay] = 1;
			else
				pn_status[i][delay] = 0;
		}
		axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, 0);
	}

	dev_info(&conv->spi->dev, "digital interface lanes tuning:\n");

	pr_cont("  ");
	for (i = 0; i < 31; i++)
		pr_cont("%02d:", i);
	pr_cont("31\n");

	for (i = 0; i < (st->num_lanes); i++) {
		pr_info("%x:", i);
		for (j = 0; j < 32; j++) {
			if (pn_status[i][j])
				pr_cont(" # ");
			else
				pr_cont(" o ");
		}
		pr_cont("\n");
	}

	for (i = 0; i < (st->num_lanes); i++) {
		c = find_opt(&pn_status[i][0], 32, &s);
		opt_delay = s + c / 2;
		axiadc_write(axi_adc_st, 0x800 + (i * 4), opt_delay);
		dev_info(&conv->spi->dev, "lane %d: selected delay: %d\n",
			i, opt_delay);
	}

	axiadc_write(axi_adc_st, ADI_REG_CHAN_CNTRL(0), 0);

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

	ret = regmap_write(st->regmap, ADA4355_TRANSFER_REG, (reg | ADA4355_OVERRIDE));
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

	gpio_vld_en = devm_gpiod_get_optional(&st->spi->dev, "gpio-vld-en", GPIOD_OUT_LOW);
		if (IS_ERR(gpio_vld_en))
			return dev_err_probe(&st->spi->dev, PTR_ERR(gpio_vld_en),
					     "Failed to find gpio_vld_en \n");

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
	struct axiadc_converter *conv;
	struct ada4355_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &ada4355_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st = iio_priv(indio_dev);
	st->regmap = regmap;
	st->spi = spi;

	ret = devm_mutex_init(&st->lock);
	if (ret)
		return ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
		if (!conv)
			return -ENOMEM;
	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	ret = ada4355_properties_parse(st);
		if (ret)
			return ret;

	st->clk = devm_clk_get_enabled(&spi->dev, NULL);
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	ret = ada4355_setup(st);
	if (ret < 0)
		return ret;

	conv->spi = st->spi;
	conv->clk = st->clk;
	conv->chip_info = &ada4355_chip_info;
	conv->reg_access = ada4355_reg_access;
	conv->read_raw = ada4355_read_raw;
	conv->write_raw = ada4355_write_raw;
	conv->post_setup = ada4355_post_setup;
	conv->phy = st;

	return 0;
}

static const struct spi_device_id ada4355_id[] = {
	{ .name = "ada4355" },
	{ .name = "ada4356" },
	{ }
};

MODULE_DEVICE_TABLE(spi, ada4355_id);

static const struct of_device_id ada4355_of_match[] = {
	{ .compatible = "adi,ada4355" },
	{ .compatible = "adi,ada4356" },
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
