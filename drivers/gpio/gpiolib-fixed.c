// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2025 Analog Devices Inc.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/idr.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "gpiolib.h"
#include "gpiolib-shared.h"


/* Represents a single GPIO pin. */
struct gpio_fixed_entry {
	struct list_head list;
	/* Firmware node associated with the GPIO controller. */
	struct fwnode_handle *fwnode;
	/* Hardware offset of the GPIO within its chip. */
	unsigned int offset;
	/* Index in the property value array. */
	size_t index;
	struct gpio_fixed_desc *shared_desc;
	struct kref ref;
	struct list_head refs;
};


int gpio_device_setup_fixed(struct gpio_device *gdev)
{

	return 0;
}

void gpio_device_teardown_fixed(struct gpio_device *gdev)
{

}

int gpio_fixed_add_proxy_lookup(struct device *consumer, unsigned long lflags)
{

	return 0;
}

