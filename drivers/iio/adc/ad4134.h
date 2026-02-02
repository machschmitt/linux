/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices AD4134 and similar ADCs common definitions and properties
 * Copyright (C) 2026 Analog Devices, Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 */

#ifndef __DRIVERS_IIO_ADC_AD4134_H__
#define __DRIVERS_IIO_ADC_AD4134_H__

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/compiler_attributes.h>
#include <linux/iio/iio.h>
#include <linux/units.h>
#include <linux/types.h>

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

#endif /* __DRIVERS_IIO_ADC_AD4134_COMMON_H__ */
