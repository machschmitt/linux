// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices ADA4355 device driver
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

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

#define ADA4355_TEST_MODE_REG			0x0D
#define ADA4355_TEST_MODE_MASK			GENMASK(3, 0)
/* Reset the pattern generator whenever a test mode is selected. */
#define ADA4355_TEST_MODE_RESET_PN		BIT(6)
#define ADA4355_TEST_MODE_OFF			0x0
#define ADA4355_TEST_MODE_ALT_CHECKERBOARD	0x4
#define ADA4355_TEST_MODE_USER			0x8

#define ADA4355_OUTPUT_MODE_REG			0x14
#define ADA4355_OUTPUT_MODE_TWOSCOMP		BIT(0)

#define ADA4355_USER_PATT1_LSB_REG		0x19
#define ADA4355_USER_PATT1_MSB_REG		0x1A
#define ADA4355_USER_PATT2_LSB_REG		0x1B
#define ADA4355_USER_PATT2_MSB_REG		0x1C
#define ADA4355_SERIAL_OUT_DATA_CNTRL_REG	0x21
#define ADA4355_SERIAL_OUT_LANE_MODE_MASK	GENMASK(6, 4)
#define ADA4355_DDR_ONE_LANE_WORDWISE		0x4
#define ADA4355_DDR_TWO_LANE_BITWISE		0x2

#define ADA4355_SERIAL_CHANNEL_STATUS_REG	0x22
#define ADA4355_RESOLUTION_SAMPLE_RATE_REG	0x100
#define ADA4355_125_RATE			0x06

/*
 * Pattern the digital interface is calibrated against. The pattern match
 * block of the HDL core expects this exact value.
 */
#define ADA4355_CALIB_PATTERN			0xFFFC

#define ADA4355_MAX_TEST_POINTS			32
/*
 * Number of data lanes of the LVDS interface. The frame signal gets its own
 * I/O delay line, right after the data ones.
 */
#define ADA4355_MAX_DATA_LANES			2
#define ADA4355_FRAME_LANE			ADA4355_MAX_DATA_LANES
/* One set of test points per data lane, plus one for the frame signal. */
#define ADA4355_MAX_CALIB_MAP_SIZE \
	(ADA4355_MAX_TEST_POINTS * (ADA4355_MAX_DATA_LANES + 1))

#define ADA4355_SYNC_TIMEOUT_US			1000
/* Time the duty cycle stabilizer loop needs to relock after a rate change. */
#define ADA4355_DCS_RELOCK_US			5

#define ADA4355_MAX_SAMP_RATE_SPS		(125 * MEGA)

/* ADC differential input span, ±1 V with the 1 V internal reference. */
#define ADA4355_ADC_SPAN_MV			2000
/* ADC differential input with no input current flowing. */
#define ADA4355_ADC_BIAS_MV			825

#define ADA4355_NUM_GSEL_GPIOS			2

/* Transimpedance gain in ohms and the matching GSEL1 and GSEL0 setting. */
static const unsigned int ada4355_scale_table[][2] = {
	{ 2000, 0x1 },
	{ 20000, 0x2 },
	{ 200000, 0x0 },
};

static const char * const ada4355_test_modes[] = {
	[ADA4355_TEST_MODE_OFF] = "off",
	[ADA4355_TEST_MODE_ALT_CHECKERBOARD] = "checkerboard",
	[ADA4355_TEST_MODE_USER] = "user",
};

struct ada4355_chip_info {
	const char *name;
	unsigned int id;
	const unsigned int (*scale_table)[2];
	int num_scales;
	unsigned long test_mask;
	unsigned int test_mask_len;
};

struct ada4355_chan_test_mode {
	struct ada4355_state *st;
	unsigned int idx;
	u8 mode;
};

struct ada4355_state {
	const struct ada4355_chip_info *info;
	struct iio_backend *back;
	struct spi_device *spi;
	struct regmap *regmap;
	struct clk *clk;
	const struct iio_chan_spec *chan;
	/* used for debugfs */
	struct ada4355_chan_test_mode *chan_test;
	/* GSEL0 and GSEL1, in that order. NULL if the board drives them. */
	struct gpio_descs *gsel;
	int scales[ARRAY_SIZE(ada4355_scale_table)][2];
	unsigned int scale_idx;
	/*
	 * One section of @info->test_points bits per calibrated I/O delay
	 * line: one per data lane and one for the frame signal. A set bit
	 * marks an invalid test point.
	 */
	DECLARE_BITMAP(calib_map, ADA4355_MAX_CALIB_MAP_SIZE);
	/* number of bits of the map */
	unsigned int calib_map_size;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex lock;
};

static const struct regmap_config ada4355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int ada4355_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				      unsigned int writeval, unsigned int *readval)
{
	struct ada4355_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

/*
 * One ADC code in milliamps of input current: the ADC LSB in millivolts over
 * the transimpedance in ohms. The TIA output falls as the input current rises,
 * hence the negative sign.
 */
static void __ada4355_get_scale(struct ada4355_state *st, int index,
				int *val, int *val2)
{
	const struct ada4355_chip_info *info = st->info;
	u64 tmp;

	tmp = div_u64((u64)ADA4355_ADC_SPAN_MV * NANO,
		      info->scale_table[index][0]) >> st->chan->scan_type.realbits;
	*val = 0;
	*val2 = -(int)tmp;
}

static int ada4355_get_scale(struct ada4355_state *st, int *val, int *val2)
{
	*val = st->scales[st->scale_idx][0];
	*val2 = st->scales[st->scale_idx][1];

	return IIO_VAL_INT_PLUS_NANO;
}

static int ada4355_set_scale(struct ada4355_state *st, int val, int val2)
{
	const struct ada4355_chip_info *info = st->info;
	unsigned long gsel;
	unsigned int i;
	int ret;

	guard(mutex)(&st->lock);

	if (!st->gsel)
		return -EOPNOTSUPP;

	for (i = 0; i < info->num_scales; i++) {
		if (st->scales[i][0] != val || st->scales[i][1] != val2)
			continue;

		gsel = info->scale_table[i][1];

		ret = gpiod_multi_set_value_cansleep(st->gsel, &gsel);
		if (ret)
			return ret;

		st->scale_idx = i;
		return 0;
	}

	return -EINVAL;
}

/*
 * The TIA biases the ADC input at ADA4355_ADC_BIAS_MV with no input current,
 * so that is the code that has to read as zero milliamps.
 */
static int ada4355_get_offset(struct iio_chan_spec const *chan, int *val, int *val2)
{
	s64 tmp;

	tmp = div_s64(((s64)ADA4355_ADC_BIAS_MV * MICRO) << chan->scan_type.realbits,
		      ADA4355_ADC_SPAN_MV);
	*val = div_s64_rem(-tmp, MICRO, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

static int ada4355_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long m)
{
	struct ada4355_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		return ada4355_get_scale(st, val, val2);
	case IIO_CHAN_INFO_OFFSET:
		return ada4355_get_offset(chan, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(st->clk);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ada4355_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	struct ada4355_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*vals = (const int *)st->scales;
		*type = IIO_VAL_INT_PLUS_NANO;
		/*
		 * Values are stored in a 2D matrix. Without the GSEL lines the
		 * gain is fixed, so only the assumed one is available.
		 */
		*length = (st->gsel ? st->info->num_scales : 1) * 2;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec ada4355_channel = {
	.type = IIO_CURRENT,
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE) |
			      BIT(IIO_CHAN_INFO_OFFSET) |
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_SCALE),
	.scan_type = {
		.format = IIO_SCAN_FORMAT_SIGNED_INT,
		.realbits = 14,
		.storagebits = 16,
		.shift = 2,
	},
};

static const struct ada4355_chip_info ada4355_chip_info = {
	.name = "ada4355",
	.id = ADA4355_CHIP_ID,
	.scale_table = ada4355_scale_table,
	.num_scales = ARRAY_SIZE(ada4355_scale_table),
	.test_mask = BIT(ADA4355_TEST_MODE_OFF) |
		BIT(ADA4355_TEST_MODE_ALT_CHECKERBOARD) |
		BIT(ADA4355_TEST_MODE_USER),
	.test_mask_len = ADA4355_TEST_MODE_USER + 1,
};

static const struct ada4355_chip_info ada4356_chip_info = {
	.name = "ada4356",
	.id = ADA4355_CHIP_ID,
	.scale_table = ada4355_scale_table,
	.num_scales = ARRAY_SIZE(ada4355_scale_table),
	.test_mask = BIT(ADA4355_TEST_MODE_OFF) |
		BIT(ADA4355_TEST_MODE_ALT_CHECKERBOARD) |
		BIT(ADA4355_TEST_MODE_USER),
	.test_mask_len = ADA4355_TEST_MODE_USER + 1,
};

static int ada4355_testmode_set(struct ada4355_state *st, unsigned int test_mode)
{
	unsigned int val = 0;

	if (test_mode != ADA4355_TEST_MODE_OFF)
		val = ADA4355_TEST_MODE_RESET_PN |
			FIELD_PREP(ADA4355_TEST_MODE_MASK, test_mode);

	return regmap_write(st->regmap, ADA4355_TEST_MODE_REG, val);
}

/*
 * The idea is pretty simple. Find the max number of successful points in a row
 * and get the one in the middle.
 */
static unsigned int ada4355_find_optimal_point(const unsigned long *calib_map,
					       unsigned int start,
					       unsigned int nbits,
					       unsigned int *val)
{
	unsigned int bit = start, end, start_cnt, cnt = 0;

	for_each_clear_bitrange_from(bit, end, calib_map, nbits + start) {
		if (end - bit > cnt) {
			cnt = end - bit;
			start_cnt = bit;
		}
	}

	if (cnt)
		*val = start_cnt + cnt / 2;

	return cnt;
}

static int ada4355_calibrate_prepare(struct ada4355_state *st)
{
	struct iio_backend_data_fmt data = {
		.enable = false,
	};
	int ret;

	/*
	 * The channel spec already accounts for the 14-bit sample being left
	 * aligned in a 16-bit word, so the backend format conversion stays off
	 * both during and after the calibration.
	 */
	ret = iio_backend_data_format_set(st->back, 0, &data);
	if (ret)
		return ret;

	ret = ada4355_testmode_set(st, ADA4355_TEST_MODE_USER);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_USER_PATT1_MSB_REG,
			   ADA4355_CALIB_PATTERN >> 8);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_USER_PATT1_LSB_REG,
			   ADA4355_CALIB_PATTERN & 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_USER_PATT2_MSB_REG,
			   ADA4355_CALIB_PATTERN >> 8);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_USER_PATT2_LSB_REG,
			   ADA4355_CALIB_PATTERN & 0xFF);
	if (ret)
		return ret;

	return iio_backend_chan_enable(st->back, 0);
}

static int ada4355_calibrate_stop(struct ada4355_state *st)
{
	int ret;

	ret = iio_backend_calibration_source_set(st->back,
						 IIO_BACKEND_CALIB_SOURCE_NONE,
						 0);
	if (ret)
		return ret;

	ret = iio_backend_chan_disable(st->back, 0);
	if (ret)
		return ret;

	return ada4355_testmode_set(st, ADA4355_TEST_MODE_OFF);
}

/*
 * Sweep the I/O delay of one delay line while only the error source fed by
 * that line is reported, and apply the delay in the middle of the widest run
 * of valid test points. @offset is where this line's test points start in
 * @st->calib_map.
 */
static int ada4355_calibrate_lane(struct ada4355_state *st,
				  enum iio_backend_calib_source source,
				  unsigned int lane, unsigned int offset)
{
	struct device *dev = &st->spi->dev;
	unsigned int point, val, cnt;
	bool stat;
	int ret;

	ret = iio_backend_calibration_source_set(st->back, source, lane);
	if (ret)
		return ret;

	for (point = 0; point < ADA4355_MAX_TEST_POINTS; point++) {
		ret = iio_backend_iodelay_set(st->back, lane, point);
		if (ret)
			return ret;

		ret = iio_backend_chan_status(st->back, 0, &stat);
		if (ret)
			return ret;

		if (stat) {
			dev_dbg(dev, "Invalid point(%u) for lane %u\n", point,
				lane);
			continue;
		}

		__clear_bit(offset + point, st->calib_map);
	}

	cnt = ada4355_find_optimal_point(st->calib_map, offset, ADA4355_MAX_TEST_POINTS,
					 &val);
	if (!cnt) {
		dev_err(dev, "No valid delay found for lane %u\n", lane);
		return -EIO;
	}

	/* "normalize" the selected value */
	val -= offset;
	dev_dbg(dev, "lane %u: selected delay: %u\n", lane, val);

	return iio_backend_iodelay_set(st->back, lane, val);
}

static int ada4355_calibrate(struct ada4355_state *st)
{
	unsigned int lane;
	int ret;

	/* all points invalid */
	bitmap_fill(st->calib_map, st->calib_map_size);

	ret = ada4355_calibrate_prepare(st);
	if (ret)
		return ret;

	/*
	 * The frame signal has to be aligned first: the pattern match block
	 * only reports meaningful per-lane errors once the word boundary is
	 * known.
	 */
	ret = ada4355_calibrate_lane(st, IIO_BACKEND_CALIB_SOURCE_FRAME,
				     ADA4355_FRAME_LANE,
				     ADA4355_MAX_DATA_LANES * ADA4355_MAX_TEST_POINTS);
	if (ret)
		return ret;

	for (lane = 0; lane < ADA4355_MAX_DATA_LANES; lane++) {
		ret = ada4355_calibrate_lane(st,
					     IIO_BACKEND_CALIB_SOURCE_DATA_LANE,
					     lane,
					     lane * ADA4355_MAX_TEST_POINTS);
		if (ret)
			return ret;
	}

	return ada4355_calibrate_stop(st);
}

/*
 * The internal state machine needs a stable clock to bring the ADC up. If the
 * clock was absent or unstable by then, the ADC starts in an unknown state and
 * only a digital reset recovers it. The same holds for clock instability during
 * normal operation, a rate change included. Unlike the soft reset of
 * ADA4355_CHIP_CONFIG_REG, this one leaves the user registers untouched.
 */
static int ada4355_digital_reset(struct ada4355_state *st)
{
	unsigned int reg;
	int ret;

	ret = regmap_write(st->regmap, ADA4355_POWER_MODES_REG,
			   ADA4355_PW_MODE_DIGITAL_RESET);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADA4355_POWER_MODES_REG, &reg);
	if (ret)
		return ret;

	return regmap_clear_bits(st->regmap, ADA4355_POWER_MODES_REG,
				 ADA4355_PW_MODE_DIGITAL_RESET);
}

static int ada4355_update_clock(struct ada4355_state *st, long r_clk)
{
	unsigned long sample_rate;
	int ret;

	/* Avoid unnecessary calibration if the clock won't change. */
	sample_rate = clk_get_rate(st->clk);
	if (sample_rate == r_clk)
		return 0;

	guard(mutex)(&st->lock);

	ret = clk_set_rate(st->clk, r_clk);
	if (ret)
		return ret;

	fsleep(ADA4355_DCS_RELOCK_US);

	/*
	 * Changing the rate under the running state machine counts as clock
	 * instability, so put the digital side back into a known state.
	 */
	ret = ada4355_digital_reset(st);
	if (ret)
		return ret;

	if (iio_backend_has_caps(st->back, IIO_BACKEND_CAP_CALIBRATION))
		return ada4355_calibrate(st);

	return 0;
}

static int ada4355_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ada4355_state *st = iio_priv(indio_dev);
	long r_clk;

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		r_clk = clk_round_rate(st->clk, val);
		if (!in_range(r_clk, 1, ADA4355_MAX_SAMP_RATE_SPS))
			return -ERANGE;

		return ada4355_update_clock(st, r_clk);
	case IIO_CHAN_INFO_SCALE:
		return ada4355_set_scale(st, val, val2);
	default:
		return -EINVAL;
	}
}

static int ada4355_write_raw_get_fmt(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ada4355_iio_info = {
	.read_raw = ada4355_read_raw,
	.read_avail = ada4355_read_avail,
	.write_raw = ada4355_write_raw,
	.write_raw_get_fmt = ada4355_write_raw_get_fmt,
	.debugfs_reg_access = ada4355_debugfs_reg_access,
};

static int ada4355_setup(struct ada4355_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *gpio_vld_en;
	struct gpio_desc *gpio_lfp;
	unsigned int id;
	int ret;

	/*
	 * Boards powering the ADC from the on chip LDO have no 1.8 V rail until
	 * VLDEN is asserted, so this has to come before any register access. No
	 * LDO start-up time is given, hence the conservative settling delay.
	 */
	gpio_vld_en = devm_gpiod_get(dev, "adi,vld-en", GPIOD_OUT_LOW);
	if (IS_ERR(gpio_vld_en))
		return dev_err_probe(dev, PTR_ERR(gpio_vld_en),
				     "Failed to find adi,vld-en-gpios\n");

	if (gpio_vld_en) {
		gpiod_set_value_cansleep(gpio_vld_en, 1);
		fsleep(1000);
	}

	gpio_lfp = devm_gpiod_get(dev, "adi,lfp", GPIOD_OUT_LOW);
	if (IS_ERR(gpio_lfp))
		return dev_err_probe(dev, PTR_ERR(gpio_lfp),
				     "Failed to find adi,lfp-gpios\n");

	ret = ada4355_digital_reset(st);
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

	if (id != st->info->id)
		dev_warn(dev, "Mismatch CHIP_ID, got 0x%X, expected 0x%X\n",
			 id, st->info->id);

	ret = regmap_update_bits(st->regmap, ADA4355_SERIAL_OUT_DATA_CNTRL_REG,
				 ADA4355_SERIAL_OUT_LANE_MODE_MASK,
				 FIELD_PREP(ADA4355_SERIAL_OUT_LANE_MODE_MASK,
					    ADA4355_DDR_TWO_LANE_BITWISE));
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, ADA4355_TRANSFER_REG,
			      ADA4355_TRANSFER_OVERRIDE);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_OUTPUT_MODE_REG,
			   ADA4355_OUTPUT_MODE_TWOSCOMP);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADA4355_RESOLUTION_SAMPLE_RATE_REG,
			   ADA4355_125_RATE);
	if (ret)
		return ret;

	return 0;
}

static int ada4355_gain_setup(struct ada4355_state *st)
{
	const struct ada4355_chip_info *info = st->info;
	struct device *dev = &st->spi->dev;
	unsigned long gsel;
	unsigned int i;

	for (i = 0; i < info->num_scales; i++)
		__ada4355_get_scale(st, i, &st->scales[i][0], &st->scales[i][1]);

	st->scale_idx = 0;

	/*
	 * Start with the lowest gain, which gives the widest input current range.
	 */
	st->gsel = devm_gpiod_get_array(dev, "adi,gsel", GPIOD_OUT_LOW);
	if (IS_ERR(st->gsel))
		return dev_err_probe(dev, PTR_ERR(st->gsel),
				     "Failed to get adi,gsel-gpios\n");

	if (st->gsel->ndescs != ADA4355_NUM_GSEL_GPIOS)
		return dev_err_probe(dev, -EINVAL,
				     "Expected %u adi,gsel-gpios, got %u\n",
				     ADA4355_NUM_GSEL_GPIOS, st->gsel->ndescs);

	gsel = info->scale_table[st->scale_idx][1];

	return gpiod_multi_set_value_cansleep(st->gsel, &gsel);
}

static int ada4355_interface_setup(struct ada4355_state *st)
{
	int ret;

	ret = iio_backend_num_lanes_set(st->back, ADA4355_MAX_DATA_LANES);
	if (ret)
		return ret;

	return iio_backend_interface_data_align(st->back,
						ADA4355_SYNC_TIMEOUT_US);
}

static int ada4355_test_mode_available_show(struct seq_file *s, void *ignored)
{
	struct ada4355_state *st = s->private;
	unsigned int bit;

	for_each_set_bit(bit, &st->info->test_mask, st->info->test_mask_len)
		seq_printf(s, "%s\n", ada4355_test_modes[bit]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ada4355_test_mode_available);

static ssize_t ada4355_chan_test_mode_read(struct file *file,
					   char __user *userbuf, size_t count,
					   loff_t *ppos)
{
	struct ada4355_chan_test_mode *chan = file->private_data;
	struct ada4355_state *st = chan->st;
	char buf[128] = {0};
	size_t len;
	int ret;

	if (chan->mode == ADA4355_TEST_MODE_OFF) {
		len = scnprintf(buf, sizeof(buf), "No test Running...\n");
	} else {
		len = scnprintf(buf, sizeof(buf), "Running \"%s\" Test:\n\t",
				ada4355_test_modes[chan->mode]);

		if (iio_backend_has_caps(st->back, IIO_BACKEND_CAP_CALIBRATION)) {
			ret = iio_backend_debugfs_print_chan_status(st->back,
								    chan->idx,
								    buf + len,
								    sizeof(buf) - len);
			if (ret < 0)
				return ret;
			len += ret;
		}
	}

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t ada4355_chan_test_mode_write(struct file *file,
					    const char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct ada4355_chan_test_mode *chan = file->private_data;
	struct ada4355_state *st = chan->st;
	char test_mode[32] = {0};
	unsigned int mode;
	int ret;

	ret = simple_write_to_buffer(test_mode, sizeof(test_mode) - 1, ppos,
				     userbuf, count);
	if (ret < 0)
		return ret;

	for_each_set_bit(mode, &st->info->test_mask, st->info->test_mask_len) {
		if (sysfs_streq(test_mode, ada4355_test_modes[mode]))
			break;
	}

	if (mode == st->info->test_mask_len)
		return -EINVAL;

	guard(mutex)(&st->lock);

	ret = ada4355_testmode_set(st, mode);
	if (ret)
		return ret;

	chan->mode = mode;

	return count;
}

static const struct file_operations ada4355_chan_test_mode_fops = {
	.open = simple_open,
	.read = ada4355_chan_test_mode_read,
	.write = ada4355_chan_test_mode_write,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static ssize_t ada4355_dump_calib_table(struct file *file,
					char __user *userbuf,
					size_t count, loff_t *ppos)
{
	struct ada4355_state *st = file->private_data;
	/* one newline per delay line and +1 for the string termination */
	unsigned char map[ADA4355_MAX_CALIB_MAP_SIZE +
			  ADA4355_MAX_DATA_LANES + 2];
	unsigned int bit;
	ssize_t len = 0;

	guard(mutex)(&st->lock);
	if (*ppos)
		goto out_read;

	for (bit = 0; bit < st->calib_map_size; bit++) {
		if (bit && bit % ADA4355_MAX_TEST_POINTS == 0)
			len += scnprintf(map + len, sizeof(map) - len, "\n");

		len += scnprintf(map + len, sizeof(map) - len, "%c",
				 test_bit(bit, st->calib_map) ? 'x' : 'o');
	}

	len += scnprintf(map + len, sizeof(map) - len, "\n");
out_read:
	return simple_read_from_buffer(userbuf, count, ppos, map, len);
}

static const struct file_operations ada4355_calib_table_fops = {
	.open = simple_open,
	.read = ada4355_dump_calib_table,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static void ada4355_debugfs_init(struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	struct ada4355_state *st = iio_priv(indio_dev);
	char attr_name[32];
	unsigned int chan;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	st->chan_test = devm_kcalloc(&st->spi->dev, st->info->num_channels,
				     sizeof(*st->chan_test), GFP_KERNEL);
	if (!st->chan_test)
		return;

	if (iio_backend_has_caps(st->back, IIO_BACKEND_CAP_CALIBRATION))
		debugfs_create_file("calibration_table_dump", 0400, d, st,
				    &ada4355_calib_table_fops);

	for (chan = 0; chan < st->info->num_channels; chan++) {
		snprintf(attr_name, sizeof(attr_name), "in_voltage%u_test_mode",
			 chan);
		st->chan_test[chan].idx = chan;
		st->chan_test[chan].st = st;
		debugfs_create_file(attr_name, 0600, d, &st->chan_test[chan],
				    &ada4355_chan_test_mode_fops);
	}

	debugfs_create_file("in_voltage_test_mode_available", 0400, d, st,
			    &ada4355_test_mode_available_fops);

	iio_backend_debugfs_add(st->back, indio_dev);
}

static int ada4355_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ada4355_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -ENODEV;

	st->regmap = devm_regmap_init_spi(spi, &ada4355_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	/* One set of test points per data lane, plus one for the frame. */
	st->calib_map_size = (ADA4355_MAX_DATA_LANES + 1) * ADA4355_MAX_TEST_POINTS;

	st->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	indio_dev->name = st->info->name;
	indio_dev->channels = &ada4355_channel;
	indio_dev->num_channels = 1;
	indio_dev->info = &ada4355_iio_info;
	st->chan = &ada4355_channel;

	ret = ada4355_setup(st);
	if (ret)
		return ret;

	ret = ada4355_gain_setup(st);
	if (ret)
		return ret;

	st->back = devm_iio_backend_get(dev, NULL);
	if (IS_ERR(st->back))
		return dev_err_probe(dev, PTR_ERR(st->back),
				     "failed to get IIO backend\n");

	if (iio_backend_has_caps(st->back, IIO_BACKEND_CAP_BUFFER)) {
		ret = devm_iio_backend_request_buffer(dev, st->back, indio_dev);
		if (ret)
			return ret;
	}

	if (iio_backend_has_caps(st->back, IIO_BACKEND_CAP_ENABLE)) {
		ret = devm_iio_backend_enable(dev, st->back);
		if (ret)
			return ret;
	}

	ret = ada4355_interface_setup(st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to setup the digital interface\n");

	if (iio_backend_has_caps(st->back, IIO_BACKEND_CAP_CALIBRATION)) {
		ret = ada4355_calibrate(st);
		if (ret)
			return ret;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	ada4355_debugfs_init(indio_dev);

	return 0;
}

static const struct spi_device_id ada4355_id[] = {
	{ .name = "ada4355", .driver_data = (kernel_ulong_t)&ada4355_chip_info },
	{ .name = "ada4356", .driver_data = (kernel_ulong_t)&ada4356_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ada4355_id);

static const struct of_device_id ada4355_of_match[] = {
	{ .compatible = "adi,ada4355", .data = &ada4355_chip_info },
	{ .compatible = "adi,ada4356", .data = &ada4356_chip_info },
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
MODULE_DESCRIPTION("Analog Devices ADA4355 device driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_BACKEND");
