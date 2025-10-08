// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2025 Analog Devices Inc.
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

#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>

#include "gpiolib-fixed.h"


static int gpio_fixed_proxy_probe(struct auxiliary_device *adev,
				   const struct auxiliary_device_id *id)
{
	struct gpio_fixed_proxy_data *proxy;
	struct gpio_fixed_desc *shared_desc;
	struct device *dev = &adev->dev;
	struct gpio_chip *gc;

	//shared_desc = devm_gpiod_fixed_get(dev);
	//if (IS_ERR(shared_desc))
	//	return PTR_ERR(shared_desc);

	//proxy = devm_kzalloc(dev, sizeof(*proxy), GFP_KERNEL);
	//if (!proxy)
	//	return -ENOMEM;

	//proxy->shared_desc = shared_desc;
	//proxy->dev = dev;

	//gc = &proxy->gc;
	//gc->base = -1;
	//gc->ngpio = 1;
	//gc->label = dev_name(dev);
	//gc->parent = dev;
	//gc->owner = THIS_MODULE;
	//gc->can_sleep = shared_desc->can_sleep;

	//gc->request = gpio_shared_proxy_request;
	//gc->free = gpio_shared_proxy_free;
	//gc->set_config = gpio_shared_proxy_set_config;
	//gc->direction_input = gpio_shared_proxy_direction_input;
	//gc->direction_output = gpio_shared_proxy_direction_output;
	//if (gc->can_sleep) {
	//	gc->set = gpio_shared_proxy_set_cansleep;
	//	gc->get = gpio_shared_proxy_get_cansleep;
	//} else {
	//	gc->set = gpio_shared_proxy_set;
	//	gc->get = gpio_shared_proxy_get;
	//}
	//gc->get_direction = gpio_shared_proxy_get_direction;
	//gc->to_irq = gpio_shared_proxy_to_irq;

	//return devm_gpiochip_add_data(dev, &proxy->gc, proxy);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id fixed_of_match[] = {
	{ .compatible = "fixed-gpios" },
	{ },
};
MODULE_DEVICE_TABLE(of, fixed_of_match);
#endif

static const struct auxiliary_device_id gpio_fixed_proxy_id_table[] = {
	{ .name = "gpiolib_shared.proxy" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, gpio_fixed_proxy_id_table);

static struct auxiliary_driver gpio_fixed_proxy_driver = {
	.driver = {
		.name = "gpio-shared-proxy",
	},
	.probe = gpio_fixed_proxy_probe,
	.id_table = gpio_fixed_proxy_id_table,
};
module_auxiliary_driver(gpio_fixed_proxy_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Fixed-state GPIO driver.");
MODULE_LICENSE("GPL");
