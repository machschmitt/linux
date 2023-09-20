// SPDX-License-Identifier: GPL-2.0


#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/regulator/consumer.h>
#include <linux/util_macros.h>


#include <linux/bitfield.h>
#include <linux/iio/sysfs.h>

/* HARDWARE_GAIN */
#define ADAQ4224_GAIN_MAX		6670000000

#define AD4630_OUT_DATA_MODE_MSK	GENMASK(2, 0)

#define ADAQ4224_NUM_CHANS		2

/*
 * Gains computed as fractions of 1000 so they can be expressed by integers.
 */
static const int ad4630_gains[4] = {
	330, 560, 2220, 6670
};

static struct adaq4224_chip_info {
	const unsigned long *available_masks;
	const struct ad4630_out_mode *modes;
	const char *name;
	unsigned long out_modes_mask;
	int min_offset;
	int max_offset;
	u16 base_word_len;
	u8 grade;
	u8 n_channels;
	bool has_pgia;
};

static struct adaq4224_state {
	struct platform_device *pdev;
	struct iio_channel *iio_chans;
	int num_chans;
	struct iio_chan_spec_ext_info *ext_info;
	const struct adaq4224_chip_info *chip;
	struct gpio_descs *pgia_gpios;
	int vref;
	int pgia_idx;
	unsigned int out_data;
};

static ssize_t in_voltage_scale_available_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adaq4224_state *st = iio_priv(indio_dev);
	int i, len = 0;
	int val, val2;
	int tmp0, tmp1;
	s64 tmp2;

	val2 = st->chip->base_word_len;

	for (i = 0; i < ARRAY_SIZE(ad4630_gains); i++) {
		val = (st->vref * 2) / 1000;
		if (st->chip->has_pgia)
			val = val * ad4630_gains[i] / 1000;

		tmp2 = shift_right((s64)val * 1000000000LL, val2);
		tmp0 = (int)div_s64_rem(tmp2, 1000000000LL, &tmp1);

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"%d.%09u ", tmp0, abs(tmp1));
	}
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEVICE_ATTR_RO(in_voltage_scale_available, 0);

static struct attribute *ad4630_attributes[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad4630_attribute_group = {
	.attrs = ad4630_attributes,
};

static int adaq4224_write_raw_get_fmt(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int ad4630_calc_pgia_gain(int gain_int, int gain_fract, int vref,
				 int precision)
{
	int gain_idx;
	u64 gain;

	gain = gain_int * 1000000000 + gain_fract;

	if (gain < 0 || gain > ADAQ4224_GAIN_MAX)
		return -EINVAL;

	gain = DIV_ROUND_CLOSEST_ULL(gain << precision, 1000000000);
	gain = DIV_ROUND_CLOSEST_ULL(gain, (vref / 1000000) * 2);
	gain_idx = find_closest(gain, ad4630_gains, ARRAY_SIZE(ad4630_gains));

	return gain_idx;
}

static int ad4630_set_pgia_gain(struct iio_dev *indio_dev, int gain_idx)
{
	struct adaq4224_state *st = iio_priv(indio_dev);
	int ret, n_mux_pins = 2;

	unsigned long *values = bitmap_alloc(n_mux_pins, GFP_KERNEL);
	if(!values)
		return -ENOMEM;

	/* Set appropriate status for A0, A1 pins according to requested gain */
	switch (gain_idx)
	{
	case 0:
		bitmap_zero(values, n_mux_pins); /* Clear MUX_A0 and MUX_A1 pins */
		break;
	case 1:
		bitmap_set(values, 0, 1);        /* Set MUX_A0 pin */
		bitmap_clear(values, 1, 1);      /* Clear MUX_A1 pin */
		break;
	case 2:
		bitmap_clear(values, 0, 1);      /* Clear MUX_A0 pin */
		bitmap_set(values, 1, 1);        /* Set MUX_A1 pin */
		break;
	case 3:
		bitmap_fill(values, n_mux_pins); /* Set MUX_A0 and MUX_A1 pins */
		break;
	default:
		return -EINVAL;
	}

	ret = gpiod_set_array_value_cansleep(n_mux_pins,
					     st->pgia_gpios->desc,
					     st->pgia_gpios->info, values);

	bitmap_free(values);

	if (ret)
		dev_info(&st->pdev->dev, "Error on gpiod set: %d\n", ret);

	if (!ret)
		st->pgia_idx = gain_idx;

	return ret;
}
static int adaq4224_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long mask)
{
	struct adaq4224_state *st = iio_priv(indio_dev);
	int gain_idx;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		gain_idx = ad4630_calc_pgia_gain(val, val2, st->vref,
						 chan->scan_type.realbits);
		return ad4630_set_pgia_gain(indio_dev, gain_idx);
	default:
		if (chan->type == IIO_TEMP)
			return iio_write_channel_attribute(st->iio_chans + 1, val,
							   val2, mask);
		else
			return iio_write_channel_attribute(st->iio_chans + chan->channel,
							   val, val2, mask);
	}
}

//iio_buffer_channel_enable()
//adc->indio_dev->buffer

static int adaq4224_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct adaq4224_state *st = iio_priv(indio_dev);
	//int ret;


	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*val = (st->vref * 2) / 1000;
		if (st->chip->has_pgia)
			*val = *val * ad4630_gains[st->pgia_idx] / 1000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		if (chan->type == IIO_TEMP)
			return iio_read_channel_attribute(st->iio_chans + 1,
							  val, val2, mask);
		else
			return iio_read_channel_attribute(st->iio_chans + chan->channel,
							  val, val2, mask);
	}
}

static const char *const ad4630_average_modes[] = {
	"0", "2", "4", "8", "16", "32",	"64", "128", "256", "512", "1024",
	"2048", "4096", "8192", "16384", "32768", "65536"
};

ssize_t adaq4224_ext_info_write(struct iio_dev *indio_dev, uintptr_t priv,
		 struct iio_chan_spec const *chan, const char *buf,
		 size_t len)
{
	struct adaq4224_state *st = iio_priv(indio_dev);
	const char *name = (const char *)priv;

	return iio_write_channel_ext_info(st->iio_chans + chan->channel, name,
					  buf, len);
}

ssize_t adaq4224_ext_info_read(struct iio_dev *indio_dev, uintptr_t priv,
			struct iio_chan_spec const *chan, char *buf)
{
	struct adaq4224_state *st = iio_priv(indio_dev);
	const char *name = (const char *)priv;

	return iio_read_channel_ext_info(st->iio_chans + chan->channel,
					 name,
					 buf);
}

static const struct iio_enum ad4630_avg_frame_len_enum = {
	.items = ad4630_average_modes,
	.num_items = ARRAY_SIZE(ad4630_average_modes),
};

static const struct iio_chan_spec_ext_info ad4630_ext_info[] = {
	{
		.name = "sample_averaging",
		.shared = IIO_SHARED_BY_TYPE,
		.read = &adaq4224_ext_info_read,
		.write = &adaq4224_ext_info_write,
		.private = (uintptr_t)("sample_averaging"),
	},
	{
		.name = "sample_averaging_available",
		.shared = IIO_SHARED_BY_TYPE,
		.read = &adaq4224_ext_info_read,
		.write = &adaq4224_ext_info_write,
		//.private = &ad4630_avg_frame_len_enum,
		.private = (uintptr_t)("sample_averaging_available"),
	},
	{}
};

#define AD4630_CHAN(_idx, _storage, _real, _shift, _info) {		\
	.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			BIT(IIO_CHAN_INFO_CALIBBIAS),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.info_mask_shared_by_type =  BIT(IIO_CHAN_INFO_SCALE),		\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _idx,						\
	.scan_index = _idx,						\
	.ext_info = _info,						\
	.scan_type = {							\
		.sign = 's',						\
		.storagebits = _storage,				\
		.realbits = _real,					\
		.shift = _shift,					\
	},								\
}

#define MAX31827_CHAN {							\
	.type = IIO_TEMP,						\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW) |		\
				   BIT(IIO_CHAN_INFO_SAMP_FREQ) |	\
				   BIT(IIO_CHAN_INFO_ENABLE) |		\
				   BIT(IIO_CHAN_INFO_SCALE),		\
	.output = 0,							\
}

#define AD4630_MAX_CHANNEL_NR		3

enum {
	AD4630_16_DIFF = 0x00,
	AD4630_24_DIFF = 0x00,
	AD4630_16_DIFF_8_COM = 0x01,
	AD4630_24_DIFF_8_COM = 0x02,
	AD4630_30_AVERAGED_DIFF = 0x03,
	AD4630_32_PATTERN = 0x04
};

struct ad4630_out_mode {
	const struct iio_chan_spec channels[AD4630_MAX_CHANNEL_NR];
	u32 data_width;
};

static const struct ad4630_out_mode ad4030_24_modes[] = {
	[AD4630_24_DIFF] = {
		.channels = {
			AD4630_CHAN(0, 64, 24, 0, NULL),
			MAX31827_CHAN,
		},
		.data_width = 24,
	},
	[AD4630_16_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, 64, 16, 8, NULL),
			MAX31827_CHAN,
		},
		.data_width = 24,
	},
	[AD4630_24_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, 64, 24, 8, NULL),
			MAX31827_CHAN,
		},
		.data_width = 32,
	},
	[AD4630_30_AVERAGED_DIFF] = {
		.channels = {
			AD4630_CHAN(0, 64, 30, 2, ad4630_ext_info),
			MAX31827_CHAN,
		},
		.data_width = 32,
	}
};


static const unsigned long ad4030_channel_masks[] = {
	BIT(0),
	0,
};

static const struct adaq4224_chip_info chip_info = {
	.available_masks = ad4030_channel_masks,
	.modes = ad4030_24_modes,
	.out_modes_mask = GENMASK(3, 0),
	.name = "adaq4224",
	.grade = 0x10,   /* !FIXME: To be updated with the value from datasheet */
	.min_offset = (int)BIT(23) * -1,
	.max_offset = BIT(23) - 1,
	.base_word_len = 24,
	.has_pgia = true,
	.n_channels = 1,
};

//Have to get the channel spec structs from the ADC and temp drivers.

static const struct iio_info adaq4224_info = {
	.read_raw = &adaq4224_read_raw,
	.write_raw = &adaq4224_write_raw,
	.write_raw_get_fmt = &adaq4224_write_raw_get_fmt,
	//.debugfs_reg_access = &ad4630_reg_access,
	.attrs = &ad4630_attribute_group,
};


unsigned int iio_get_iio_channel_count(struct iio_channel *chans)
{
	struct iio_channel *chan;
	unsigned int i = 0;

	if (!chans)
		return i;

	for (chan = chans; chan != NULL; chan++)
		++i;

	return i;
}

static int adaq_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_channel *iio_chans;
	struct iio_dev *indio_dev;
	struct adaq4224_state *st;
	struct regulator *ref;
	int num_chans = 0;
	int sizeof_chans;
	int sizeof_ext_info = 0;
	int sizeof_priv;
	int i, j, k;
	int ret;


	iio_chans = devm_iio_channel_get_all(dev);
	if (IS_ERR(iio_chans))
		return dev_err_probe(dev, PTR_ERR(iio_chans),
				     "failed to get source channels\n");

	num_chans = iio_get_iio_channel_count(iio_chans);
	if (num_chans) {
		sizeof_chans = num_chans + 1; /* one extra entry for the sentinel */
		sizeof_chans *= sizeof(*st->iio_chans);
	}

	for (i = 0; i < num_chans; i++)
		sizeof_ext_info += iio_get_channel_ext_info_count(iio_chans + i);

	if (sizeof_ext_info) {
		sizeof_ext_info += 1; /* one extra entry for the sentinel */
		sizeof_ext_info *= sizeof(*st->ext_info);
	}

	sizeof_priv = sizeof(*st) + sizeof_chans + sizeof_ext_info;

	indio_dev = devm_iio_device_alloc(dev, sizeof_priv);
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->pdev = pdev;
	st->iio_chans = iio_chans;
	st->num_chans = num_chans;

	st->chip = device_get_match_data(&pdev->dev);
	if (!st->chip)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "Could not find chip info data\n");

	//get the same vref as the ADC device
	ref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(ref)) {
		if (PTR_ERR(ref) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vref regulator");

		/* if not using optional REF, the internal REFIN must be used */
		ref = devm_regulator_get(dev, "vrefin");
		if (IS_ERR(ref))
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vrefin regulator");
	}
	st->vref = regulator_get_voltage(ref);


	//get same out_data so can pick the correct channel list latter
	ret = device_property_read_u32(&pdev->dev, "adi,out-data-mode", &st->out_data);
	if (!ret) {
		if (st->out_data > AD4630_30_AVERAGED_DIFF ||
		    !test_bit(st->out_data, &st->chip->out_modes_mask))
			return dev_err_probe(dev, -EINVAL, "Invalid out data mode(%u)\n",
					     st->out_data);
	}

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = st->chip->name;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->channels = st->chip->modes[st->out_data].channels;
	indio_dev->num_channels = num_chans;
	indio_dev->info = &adaq4224_info;
	indio_dev->available_scan_masks = st->chip->available_masks;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}



static const struct of_device_id adaq_adc_of_match[] = {
	{ .compatible = "adi,adaq_adc", .data = &chip_info },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adaq_adc_of_match);

static struct platform_driver adaq_adc_driver = {
	.probe = adaq_adc_probe,
	.driver = {
		.name = "adi,adaq_adc",
		.of_match_table = adaq_adc_of_match,
	},
};
module_platform_driver(adaq_adc_driver);

MODULE_DESCRIPTION("Analog Devices ADAQ4224 platform driver");
MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_LICENSE("GPL v2");
