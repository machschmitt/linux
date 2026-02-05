/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices AD4134 and similar ADCs common definitions and properties
 * Copyright (C) 2026 Analog Devices, Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 */

#ifndef __DRIVERS_IIO_ADC_AD4134_H__
#define __DRIVERS_IIO_ADC_AD4134_H__

#include <linux/iio/iio.h>
#ifdef CONFIG_AD4134_OFFLOAD_BUFFER
#include <linux/pwm.h>
#endif
#include <linux/spi/spi.h>
#ifdef CONFIG_AD4134_OFFLOAD_BUFFER
#include <linux/spi/offload/consumer.h>
#include <linux/spi/offload/types.h>
#endif
#include <linux/types.h>
#include <linux/units.h>

#define AD4134_NUM_DOUT_LINES			1

#define AD4134_DCLK_RISING_OFFSET_NS		8
#define AD4134_MIN_ODR_FREQ_HZ			10
#define AD4134_MAX_ODR_FREQ_HZ			(1496 * HZ_PER_KHZ)

#define AD4134_SPI_MAX_XFER_LEN			3
#define AD4134_NUM_CHANNELS			4
#define AD4134_CHAN_PRECISION_BITS		24

struct ad4134_state {
	struct spi_device *spi;
	struct regmap *regmap;
	unsigned long sys_clk_hz;
	struct gpio_desc *odr_gpio;
	struct spi_transfer xfers[AD4134_NUM_CHANNELS];
	struct spi_message msg;
#ifdef CONFIG_AD4134_OFFLOAD_BUFFER
	struct spi_offload *offload;
	struct spi_offload_trigger *offload_trigger;
	struct spi_offload_trigger_config offload_trigger_config;
	struct pwm_device *odr_trigger;
	struct pwm_waveform odr_wf;
	unsigned int odr_hz;
#endif
	int refin_mv;
	/*
	 * DMA (thus cache coherency maintenance) requires the transfer buffers
	 * to live in their own cache lines.
	 *
	 * Make the buffer large enough for AD4134_NUM_CHANNELS 32-bit samples
	 * and one 64-bit aligned 64-bit timestamp.
	 */
	IIO_DECLARE_DMA_BUFFER_WITH_TS(u8, scan_data, AD4134_NUM_CHANNELS * sizeof(u32));
	/* Register access buffers */
	u8 rx_buf[AD4134_SPI_MAX_XFER_LEN];
	u8 tx_buf[AD4134_SPI_MAX_XFER_LEN];
};

#ifdef CONFIG_AD4134_OFFLOAD_BUFFER
extern const struct attribute_group ad4134_offload_attribute_group;
#endif

#ifdef CONFIG_AD4134_OFFLOAD_BUFFER
int ad4134_offload_buffer_setup(struct iio_dev *indio_dev, struct spi_device *spi);
#else
static inline int ad4134_offload_buffer_setup(struct iio_dev *indio_dev,
					      struct spi_device *spi)
{
	might_sleep();
	return -ENODEV;
}
#endif

#endif /* __DRIVERS_IIO_ADC_AD4134_COMMON_H__ */
