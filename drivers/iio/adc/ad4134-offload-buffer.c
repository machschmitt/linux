// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Analog Devices, Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/offload/consumer.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#include "ad4134.h"

/*
 * Hardcoded 32-bit storagebits because the currently available HDL only
 * supports that.
 */
#define AD4134_OFFLOAD_CHANNEL(_index) {					\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.storagebits = 32,						\
		.realbits = AD4134_CHAN_PRECISION_BITS,				\
		.endianness = IIO_CPU,						\
	},									\
}

/*
 * It's not possible for software to record when offloaded SPI transfers run so
 * no additional timestamp channel is added.
 */
static const struct iio_chan_spec ad4134_offload_chan_set[] = {
	AD4134_OFFLOAD_CHANNEL(0),
	AD4134_OFFLOAD_CHANNEL(1),
	AD4134_OFFLOAD_CHANNEL(2),
	AD4134_OFFLOAD_CHANNEL(3),
};

/* The chip converts and outputs all 4 channels on each sample request */
static const unsigned long ad4134_offload_scan_masks[] = {
	GENMASK(3, 0),
	0
};

static const struct spi_offload_config ad4134_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

static int ad4134_update_conversion_rate(struct ad4134_state *st,
					 unsigned int freq_hz)
{
	struct spi_offload_trigger_config *config = &st->offload_trigger_config;
	struct pwm_waveform odr_wf = { };
	u64 offload_period_ns;
	u64 offload_offset_ns;
	u64 odr_high_time_ns;
	unsigned int odr_hz;
	u64 target = 0;
	int ret;

	/*
	 * Every ODR pulse causes each of the 4 ADCs within the AD4134 chip to
	 * take a sample simultaneously. The peripheral then outputs the data
	 * from all those channels over one, two, or four data output lanes. If
	 * the controller can fetch data from multiple lanes, the throughput is
	 * increased proportionally to the number of data lanes in use.
	 * Conversely, when multiple data lanes are enabled, the requested
	 * sampling frequency can be reached with slower ODR frequencies.
	 */
	odr_hz = freq_hz / AD4134_NUM_DOUT_LINES;
	if (odr_hz < AD4134_MIN_ODR_FREQ_HZ || odr_hz > AD4134_MAX_ODR_FREQ_HZ)
		return -EINVAL;

	odr_wf.period_length_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, odr_hz);
	/*
	 * For an arbitrary system clock (fSYSCLK), we have a minimum ODR high
	 * time of 6/fSYSCLK derived from the device clock and data interface
	 * timing (with Gated DCLK) specifications. Set the PWM duty cycle to
	 * keep ODR up for at least that long. If the rounded PWM's value is
	 * less than the minimum required, increase the target value by 10 and
	 * attempt to round the waveform again, until the minimum is reached.
	 */
	odr_high_time_ns = div64_ul(6ULL * NANO, st->sys_clk_hz);
	do {
		odr_wf.duty_length_ns = target;
		ret = pwm_round_waveform_might_sleep(st->odr_trigger, &odr_wf);
		if (ret)
			return ret;
		target += 10;
	} while (odr_wf.duty_length_ns < odr_high_time_ns);

	if (!in_range(odr_wf.period_length_ns, 2 * odr_high_time_ns, UINT_MAX))
		return -EINVAL;

	/*
	 * The controller fetches one sample per active lane each time the
	 * offload module is triggered. If multiple data lanes are enabled, the
	 * offload trigger frequency can be proportionally slower.
	 */
	offload_period_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC,
					      odr_hz * AD4134_NUM_DOUT_LINES);

	config->periodic.frequency_hz = DIV_ROUND_UP_ULL(NSEC_PER_SEC,
							 offload_period_ns);

	/*
	 * For gated DCLK, the minimum required time between ODR rising edge
	 * and DCLK rising edge is the sum of ODR high time and ODR falling
	 * edge to DCLK rising edge time. Delay the offload trigger for at least
	 * that amount of time so the ADC sample data will be available when the
	 * SPI transfer begin.
	 */
	offload_offset_ns = odr_high_time_ns + AD4134_DCLK_RISING_OFFSET_NS;
	do {
		config->periodic.offset_ns = offload_offset_ns;
		ret = spi_offload_trigger_validate(st->offload_trigger, config);
		if (ret)
			return ret;

		offload_offset_ns += 10;
	} while (config->periodic.offset_ns < odr_high_time_ns +
					      AD4134_DCLK_RISING_OFFSET_NS);

	st->odr_wf = odr_wf;
	st->odr_hz = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC, odr_wf.period_length_ns);

	return 0;
}

static ssize_t sampling_frequency_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct ad4134_state *st = iio_priv(dev_to_iio_dev(dev));

	/*
	 * If the controller can fetch data from multiple lanes, the throughput
	 * is increased proportionally to the number of data lanes in use.
	 */
	return sysfs_emit(buf, "%u\n", st->odr_hz * AD4134_NUM_DOUT_LINES);
}

static ssize_t sampling_frequency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		goto out_store;

	ret = ad4134_update_conversion_rate(st, val);

out_store:
	iio_device_release_direct(indio_dev);
	return ret ?: len;
}

static IIO_DEVICE_ATTR_RW(sampling_frequency, 0);

static ssize_t sampling_frequency_available_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	return sysfs_emit(buf, "[%u %u %lu]\n",
			  AD4134_MIN_ODR_FREQ_HZ * AD4134_NUM_DOUT_LINES, 1,
			  AD4134_MAX_ODR_FREQ_HZ * AD4134_NUM_DOUT_LINES);
}

static IIO_DEVICE_ATTR_RO(sampling_frequency_available, 0);

static struct attribute *ad4134_offload_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

const struct attribute_group ad4134_offload_attribute_group = {
	.attrs = ad4134_offload_attributes,
};
EXPORT_SYMBOL_NS_GPL(ad4134_offload_attribute_group, "IIO_AD4134");

static void ad4134_prepare_offload_msg(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int i;

	for (i = 0; i < AD4134_NUM_CHANNELS; i++) {
		st->xfers[i].bits_per_word = AD4134_CHAN_PRECISION_BITS;
		st->xfers[i].len = spi_bpw_to_bytes(AD4134_CHAN_PRECISION_BITS);
		st->xfers[i].offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
	}

	spi_message_init_with_transfers(&st->msg, st->xfers, AD4134_NUM_CHANNELS);
}

static int ad4134_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ad4134_prepare_offload_msg(indio_dev);
	st->msg.offload = st->offload;
	ret = spi_optimize_message(st->spi, &st->msg);
	if (ret)
		return ret;

	ret = pwm_set_waveform_might_sleep(st->odr_trigger, &st->odr_wf, false);
	if (ret)
		goto out_unoptimize;

	ret = spi_offload_trigger_enable(st->offload, st->offload_trigger,
					 &st->offload_trigger_config);
	if (ret)
		goto out_pwm_disable;

	return 0;

out_pwm_disable:
	pwm_disable(st->odr_trigger);
out_unoptimize:
	spi_unoptimize_message(&st->msg);

	return 0;
}

static int ad4134_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	spi_offload_trigger_disable(st->offload, st->offload_trigger);

	pwm_disable(st->odr_trigger);

	spi_unoptimize_message(&st->msg);

	return 0;
}

static const struct iio_buffer_setup_ops ad4134_offload_buffer_setup_ops = {
	.postenable = &ad4134_offload_buffer_postenable,
	.predisable = &ad4134_offload_buffer_predisable,
};

static int ad4134_spi_offload_setup(struct iio_dev *indio_dev,
				    struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct dma_chan *rx_dma;

	indio_dev->setup_ops = &ad4134_offload_buffer_setup_ops;

	st->offload_trigger = devm_spi_offload_trigger_get(dev, st->offload,
							   SPI_OFFLOAD_TRIGGER_PERIODIC);
	if (IS_ERR(st->offload_trigger))
		return dev_err_probe(dev, PTR_ERR(st->offload_trigger),
				     "failed to get offload trigger\n");

	st->offload_trigger_config.type = SPI_OFFLOAD_TRIGGER_PERIODIC;

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(dev, st->offload);
	if (IS_ERR(rx_dma))
		return dev_err_probe(dev, PTR_ERR(rx_dma),
				     "failed to get offload RX DMA\n");

	return devm_iio_dmaengine_buffer_setup_with_handle(dev, indio_dev, rx_dma,
							   IIO_BUFFER_DIRECTION_IN);
}

static int ad4134_pwm_get(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;

	st->odr_trigger = devm_pwm_get(dev, NULL);
	if (IS_ERR(st->odr_trigger))
		return dev_err_probe(dev, PTR_ERR(st->odr_trigger),
				     "failed to get ODR PWM\n");

	pwm_disable(st->odr_trigger);

	return 0;
}

int ad4134_offload_buffer_setup(struct iio_dev *indio_dev, struct spi_device *spi)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	struct device *dev = &spi->dev;
	int ret;

	st->offload = devm_spi_offload_get(dev, spi, &ad4134_offload_config);
	ret = PTR_ERR_OR_ZERO(st->offload);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to get offload\n");

	indio_dev->channels = ad4134_offload_chan_set;
	indio_dev->num_channels = ARRAY_SIZE(ad4134_offload_chan_set);
	indio_dev->available_scan_masks = ad4134_offload_scan_masks;

	ret = ad4134_spi_offload_setup(indio_dev, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to setup SPI offload\n");

	ret = ad4134_pwm_get(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get PWM: %d\n", ret);

	/*
	 * Start with a slower sampling rate so there is some room for
	 * adjusting the sampling frequency without hitting the maximum
	 * conversion rate.
	 */
	st->odr_hz = AD4134_MAX_ODR_FREQ_HZ >> 4;
	ret = ad4134_update_conversion_rate(st, st->odr_hz);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set offload samp freq\n");

	return 0;
}
EXPORT_SYMBOL_NS_GPL(ad4134_offload_buffer_setup, "IIO_AD4134");
