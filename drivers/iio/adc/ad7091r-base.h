/* SPDX-License-Identifier: GPL-2.0 */
/*
 * AD7091RX Analog to Digital converter driver
 *
 * Copyright 2014-2019 Analog Devices Inc.
 */

#ifndef __DRIVERS_IIO_ADC_AD7091R_BASE_H__
#define __DRIVERS_IIO_ADC_AD7091R_BASE_H__

#define AD7091R_REG_RESULT  0
#define AD7091R_REG_CHANNEL 1
#define AD7091R_REG_CONF    2
#define AD7091R_REG_ALERT   3

#define AD7091R_REG_CH_LOW_LIMIT(ch) ((ch) * 3 + 4)
#define AD7091R_REG_CH_HIGH_LIMIT(ch) ((ch) * 3 + 5)
#define AD7091R_REG_CH_HYSTERESIS(ch) ((ch) * 3 + 6)

#define AD7091R2_NUM_CHANNELS				2
#define AD7091R4_NUM_CHANNELS				4
#define AD7091R8_NUM_CHANNELS				8

#define AD7091R2_REG_CHANNEL_MSK	GENMASK(AD7091R2_NUM_CHANNELS, 0)
#define AD7091R4_REG_CHANNEL_MSK	GENMASK(AD7091R4_NUM_CHANNELS, 0)
#define AD7091R8_REG_CHANNEL_MSK	GENMASK(AD7091R8_NUM_CHANNELS, 0)

#define AD7091R_CHANNEL(idx, bits, ev, num_ev) {			\
	.type = IIO_VOLTAGE,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.indexed = 1,							\
	.channel = idx,							\
	.event_spec = ev,						\
	.num_event_specs = num_ev,					\
	.scan_type.storagebits = 16,					\
	.scan_type.realbits = bits,					\
}

#include <linux/gpio/consumer.h>

struct device;

enum ad7091r_mode {
	AD7091R_MODE_SAMPLE,
	AD7091R_MODE_COMMAND,
	AD7091R_MODE_AUTOCYCLE,
};

struct ad7091r_state {
	struct device *dev;
	struct regmap *map;
	struct gpio_desc *convst_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator *vref;
	const struct ad7091r_chip_info *chip_info;
	enum ad7091r_mode mode;
	struct mutex lock; /*lock to prevent concurent reads */
};

enum ad7091r_device_type {
	AD7091R2,
	AD7091R4,
	AD7091R5,
	AD7091R8,
};

struct ad7091r_chip_info {
	const char *name;
	enum ad7091r_device_type type;
	unsigned int num_channels;
	const struct iio_chan_spec *channels;
	unsigned int vref_mV;
};

static const struct iio_event_spec ad7091r_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_HYSTERESIS),
	},
};

extern const struct regmap_config ad7091r_regmap_config;

int ad7091r_probe(struct iio_dev *iio_dev, const char *name,
		  const struct ad7091r_chip_info *chip_info,
		  struct regmap *map, int irq);

#endif /* __DRIVERS_IIO_ADC_AD7091R_BASE_H__ */
