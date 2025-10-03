// SPDX-License-Identifier: GPL-2.0-only
/*
 * gpio-fixed.c
 *
 * Copyright 2025 Analog Devices Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 *
 * This is useful for devices that require information about the state of input
 * pins. For example, a sensor may have input pins for configuring properties
 * relevant for data acquisition such as sampling rate, oversampling ratio,
 * and signal gain. Those pins may be connected to GPIOs in some setups, or
 * hardwired set to some logic level in others. The gpio-fixed driver enables
 * client devices to use the GPIO API for the second cenario as well. All
 * configuration requests to the GPIO output pin are rejected but users are
 * still allowed to read the current GPIO state.
 */




#if defined(CONFIG_OF)
static const struct fixed_dev_type fixed_voltage_data = {
	.has_enable_clock = false,
};

static const struct fixed_dev_type fixed_clkenable_data = {
	.has_enable_clock = true,
};

static const struct fixed_dev_type fixed_domain_data = {
	.has_performance_state = true,
};

static const struct of_device_id fixed_of_match[] = {
	{
		.compatible = "fixed-gpios",
		.data = &fixed_voltage_data,
	},
	{
	},
};
MODULE_DEVICE_TABLE(of, fixed_of_match);
#endif


