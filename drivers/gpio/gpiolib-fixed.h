/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __LINUX_GPIO_FIXED_H
#define __LINUX_GPIO_FIXED_H

#include <linux/cleanup.h>
#include <linux/lockdep.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

struct gpio_device;
struct gpio_desc;
struct device;

#if IS_ENABLED(CONFIG_GPIO_FIXED)

int gpio_device_setup_fixed(struct gpio_device *gdev);
void gpio_device_teardown_fixed(struct gpio_device *gdev);
int gpio_fixed_add_proxy_lookup(struct device *consumer, unsigned long lflags);

#else

static inline int gpio_device_setup_fixed(struct gpio_device *gdev)
{
	return 0;
}

static inline void gpio_device_teardown_fixed(struct gpio_device *gdev) { }

static inline int gpio_fixed_add_proxy_lookup(struct device *consumer,
					       unsigned long lflags)
{
	return 0;
}

#endif /* CONFIG_GPIO_FIXED */

struct gpio_fixed_desc {
	struct gpio_desc *desc;
	bool can_sleep;
	unsigned long cfg;
	unsigned int usecnt;
	unsigned int highcnt;
	union {
		struct mutex mutex;
		spinlock_t spinlock;
	};
};

struct gpio_fixed_desc *devm_gpiod_fixed_get(struct device *dev);

DEFINE_LOCK_GUARD_1(gpio_fixed_desc_lock, struct gpio_fixed_desc,
	if (_T->lock->can_sleep)
		mutex_lock(&_T->lock->mutex);
	else
		spin_lock_irqsave(&_T->lock->spinlock, _T->flags),
	if (_T->lock->can_sleep)
		mutex_unlock(&_T->lock->mutex);
	else
		spin_unlock_irqrestore(&_T->lock->spinlock, _T->flags),
	unsigned long flags)

static inline void gpio_fixed_lockdep_assert(struct gpio_fixed_desc *shared_desc)
{
	if (shared_desc->can_sleep)
		lockdep_assert_held(&shared_desc->mutex);
	else
		lockdep_assert_held(&shared_desc->spinlock);
}

#endif /* __LINUX_GPIO_FIXED_H */
