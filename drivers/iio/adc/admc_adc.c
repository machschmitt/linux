/*
 * Analog Devices MC-ADC Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

/**
 * Note:
 * This driver is an old copy from the cf_axi_adc/axi-adc driver.
 * And some things were common with that driver. The cf_axi_adc/axi-adc
 * driver is a more complete implementation, while this one is just caring
 * about Motor Control.
 * The code duplication [here] is intentional, as we try to cleanup the
 * AXI ADC and decouple it from this driver.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/dma-direction.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/hw-consumer.h>

#include <linux/fpga/adi-axi-common.h>

/* ADC Common */
#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			(1 << 0)

#define ADI_REG_STATUS			0x005C
#define ADI_REG_DMA_STATUS		0x0088

/* ADC Channel */
#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
#define ADI_ENABLE			(1 << 0)

#define ID_AD_MC_ADC   1
#define ID_AD_MC_ADC_RAMP 1
#define ID_AD_MC_ADC_SHA3 2

struct axiadc_chip_info {
	char				*name;
	unsigned			num_channels;
	const unsigned long		*scan_masks;
	unsigned int			max_rate;
	struct iio_chan_spec		channel[3];
};

//struct admc_debugfs_entry {
//	struct axiadc_state *st;
//	const char *propname;
//	void *out_value;
//	u32 val;
//	u8 size;
//	u8 cmd;
//};

enum ad_sha3_buffer {
	AD_SHA3_RAMP,
	AD_SHA3_DATA,
};

static const char * const sha_dma_names[] = {
	[AD_SHA3_RAMP] = "ramp-dma",
	[AD_SHA3_DATA] = "sha3-dma",
};

static const char * const reset_gpio_names[] = {
	[AD_SHA3_RAMP] = "ramp-reset",
	[AD_SHA3_DATA] = "sha-reset",
};

static const char * const device_names[] = {
	[AD_SHA3_RAMP] = "ramp-reader",
	[AD_SHA3_DATA] = "sha3-reader",
};

struct axiadc_state {
        struct iio_info			iio_info;
        /* protect against device accesses */
        struct mutex			lock;
        void __iomem			*regs;
        unsigned int			pcore_version;
        //struct admc_debugfs_entry debugfs_entry;
        //u64 buf[8] __aligned(IIO_DMA_MINALIGN);
	struct iio_dev *ch_indio_dev[ARRAY_SIZE(device_names)];
	struct iio_hw_consumer *ramp_hw_cons;
	struct iio_channel *ramp_channel;
	struct gpio_desc *ramp_reset_gpio;
	struct gpio_desc *sha3_reset_gpio;
};

struct sha3_chan {
	struct axiadc_state *axiadc_st;
	int type;
	struct gpio_desc *reset_gpio;
};

//static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
//{
//	iowrite32(val, st->regs + reg);
//}
//
//static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
//{
//	return ioread32(st->regs + reg);
//}

static int axiadc_reg_access(struct iio_dev *indio_dev,
			     unsigned reg, unsigned writeval,
			     unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	//struct sha3_chan *ch = iio_priv(indio_dev);

        //mutex_lock(&st->lock);
        //if (readval == NULL)
        //	axiadc_write(st, reg & 0xFFFF, writeval);
        //else
        //	*readval = axiadc_read(st, reg & 0xFFFF);
        //mutex_unlock(&st->lock);
        dev_info(indio_dev->dev.parent, "%s\n", __func__);
        // static int iio_buffer_mmap(struct file *filep, struct vm_area_struct *vma) ??

        return 0;
}

static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	//struct sha3_chan *ch = iio_priv(indio_dev);
	//unsigned i, ctrl;

	//for (i = 0; i < indio_dev->masklength; i++) {
	//	ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

	//	if (test_bit(i, scan_mask))
	//		ctrl |= ADI_ENABLE;
	//	else
	//		ctrl &= ~ADI_ENABLE;

	//	axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	//}
	dev_info(indio_dev->dev.parent, "%s\n", __func__);
	//axiadc_read();
	//readel();
	//iio_push_to_buffers(indio_dev, &st->scan);

	return 0;
}

//static int admc_hwfifo_flush_to_buffer(struct iio_dev *indio_dev, unsigned count)
//{
//	struct iio_buffer *iio_buf = indio_dev->buffer;
//	size_t data_available;
//
//	dev_info(indio_dev->dev.parent, "%s\n", __func__);
//
//	data_available = iio_dma_buffer_usage(iio_buf);
//	dev_info(indio_dev->dev.parent, "%s data_available %d\n", __func__, data_available);
//
////int iio_dma_buffer_read(struct iio_buffer *buffer, size_t n,
////			char __user *user_buffer)
//
//	return 0;
//}

static const struct iio_info axiadc_info = {
	.debugfs_reg_access = &axiadc_reg_access,
	.update_scan_mode = &axiadc_update_scan_mode,
	//.hwfifo_flush_to_buffer = &admc_hwfifo_flush_to_buffer,
};

//static ssize_t admc_debugfs_read(struct file *file, char __user *userbuf,
//			      size_t count, loff_t *ppos)
//{
//	struct admc_debugfs_entry *entry = file->private_data;
//	struct axiadc_state *st = entry->st;
//	char buf[700];
//	u32 val = 0;
//	ssize_t len = 0;
//	int ret;
//
//	//TODO read from DMA at put data into val
//	//struct iio_dev *indio_dev = dev_to_iio_dev();
//	//struct iio_buffer *iio_buf = indio_dev->buffer;
//	len = snprintf(buf, sizeof(buf), "%u\n", val);
//	//return iio_buf->access->read(buffer, count, userbuf);
//
//	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
//
//	//return iio_buffer_read(file, userbuf, count, ppos);
//	//return iio_buffer_read_wrapper(file, userbuf, count, ppos);
//}

//static const struct file_operations admc_debugfs_reg_fops = {
//	.open = simple_open,
//	.read = admc_debugfs_read,
//};

//static void admc_add_debugfs_entry(struct axiadc_state *st,
//	const char *propname)
//{
//	st->debugfs_entry.st = st;
//	st->debugfs_entry.propname = propname;
//}

//static int admc_register_debugfs(struct iio_dev *indio_dev)
//{
//	struct axiadc_state *st = iio_priv(indio_dev);
//	struct dentry *d;
//	int i;
//
//	if (!iio_get_debugfs_dentry(indio_dev))
//		return -ENODEV;
//
//	admc_add_debugfs_entry(st, "sha3-read");
//	d = debugfs_create_file(
//		"sha3-read", 0644,
//		iio_get_debugfs_dentry(indio_dev),
//		&st->debugfs_entry,
//		&admc_debugfs_reg_fops);
//
//	return 0;
//}

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)		  \
	{ .type = IIO_VOLTAGE,					  \
	  .indexed = 1,						 \
	  .channel = _chan,					 \
	  .scan_index = _si,						\
	  .scan_type = {				\
		.sign = _sign,				\
		.realbits = _bits,			\
		.storagebits = _bits,			\
		.shift = 0,				\
	  },						\
	}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[AD_SHA3_RAMP] = {
		.name = "AD-MC-ADC-RAMP",
		.max_rate = 1000000UL,
		.num_channels = 1,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 64, 'u'),
		},
	},
	[AD_SHA3_DATA] = {
		.name = "AD-MC-ADC-SHA3",
		.max_rate = 1000000UL,
		.num_channels = 1,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 512, 'u'),
		},
	},
};

static int admc_buffer_preenable(struct iio_dev *indio_dev)
{

	struct axiadc_state *st = iio_priv(indio_dev);

	//struct sha3_chan *ch = iio_priv(indio_dev);
	//struct axiadc_state *st = ch->axiadc_st;
	//struct sha3_chan *ramp_ch = iio_priv(st->ch_indio_dev[AD_SHA3_RAMP]);
	//struct sha3_chan *sha3_ch = iio_priv(st->ch_indio_dev[AD_SHA3_DATA]);
	//struct sha3_chan *other_ch;
	//enum ad_sha3_buffer other;
	struct iio_buffer *ramp_buf = st->ramp_channel->indio_dev->buffer;
	int ret;

	dev_info(indio_dev->dev.parent, "%s\n", __func__);
	dev_info(indio_dev->dev.parent, "%s current mode %d\n", __func__,
			iio_device_get_current_mode(indio_dev));

	if (strcmp(indio_dev->name, "ramp-reader") == 0)
		return 0;

	//if (iio_buffer_enabled(st->ch_indio_dev[AD_SHA3_RAMP]) !=
	//    iio_buffer_enabled(st->ch_indio_dev[AD_SHA3_DATA])) {
	//	//gpiod_set_value_cansleep(ch->reset_gpio, 1);
	//	dev_info(indio_dev->dev.parent, "%s activate reset GPIOs\n", __func__);
	//	//return -EBUSY;

	//	other_ch = iio_priv(st->ch_indio_dev[other]);
	//	//reset sha3
	//	//reset ramp
	//	//"unreset" sha3
	//	//"unreset" ramp

	//	other = ch->type == AD_SHA3_RAMP ? AD_SHA3_RAMP : AD_SHA3_DATA;
	//	other_ch = iio_priv(st->ch_indio_dev[other]);
	//	//gpiod_set_value(ch->reset_gpio, 1);
	//	//gpiod_set_value(other_ch->reset_gpio, 1);
	//	gpiod_set_value_cansleep(sha3_ch->reset_gpio, 1);
	//	gpiod_set_value_cansleep(ramp_ch->reset_gpio, 1);
	//	fsleep(2 * MICRO);

	//	dev_info(indio_dev->dev.parent, "%s deactivate reset GPIOs\n", __func__);
	//	//gpiod_set_value(ch->reset_gpio, 0);
	//	//gpiod_set_value(other_ch->reset_gpio, 0);
	//	gpiod_set_value_cansleep(sha3_ch->reset_gpio, 0);
	//	gpiod_set_value_cansleep(ramp_ch->reset_gpio, 0);
	//	fsleep(2 * MICRO);
	//}


	//reset sha3
	//reset ramp
	//"unreset" sha3
	//"unreset" ramp
	gpiod_set_value_cansleep(st->sha3_reset_gpio, 1);
	gpiod_set_value_cansleep(st->ramp_reset_gpio, 1);
	fsleep(2 * MICRO);
	dev_info(indio_dev->dev.parent, "%s deactivate reset GPIOs\n", __func__);
	gpiod_set_value_cansleep(st->sha3_reset_gpio, 0);
	gpiod_set_value_cansleep(st->ramp_reset_gpio, 0);
	fsleep(2 * MICRO);


	//ret = iio_hw_consumer_enable(st->ramp_hw_cons);
	//if (ret) {
	//	dev_err(indio_dev->dev.parent,
	//		"iio_hw_consumer_enable failed %d\n", ret);
	//	return ret;
	//}

	iio_buffer_channel_enable(st->ramp_channel->indio_dev->buffer,
				st->ramp_channel);


	if (!iio_buffer_enabled(st->ramp_channel->indio_dev)) {
		// Force enable ramp buffer
		ret = ramp_buf->access->enable(ramp_buf,
						st->ramp_channel->indio_dev);
		if (ret) {
			dev_err(indio_dev->dev.parent, "err on enable: %d\n", ret);
			return ret;
		}
	}

	//hw_cons_buf = list_first_entry(st->ramp_hw_cons->buffers);
	//iio_buffer_channel_enable(hw_cons_buf,
	//		&st->ramp_hw_cons->channels[0]);
	//ret = iio_device_claim_buffer_mode(st->ch_indio_dev[other]);
	//if (ret)
	//	return -EBUSY;


	return 0;
}

static int admc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct sha3_chan *ch = iio_priv(indio_dev);
	struct axiadc_state *st = ch->axiadc_st;
	struct sha3_chan *other_ch;
	enum ad_sha3_buffer other;

	//unsigned int readval;
	//char __user *user_buffer;
	//int ret;

	dev_info(indio_dev->dev.parent, "%s\n", __func__);
	dev_info(indio_dev->dev.parent, "%s current mode %d\n", __func__,
			iio_device_get_current_mode(indio_dev));

	if (strcmp(indio_dev->name, "ramp-reader") == 0)
		return 0;

	//dev_info(indio_dev->dev.parent, "%s deactivate reset GPIOs\n", __func__);
	//other = ch->type == AD_SHA3_RAMP ? AD_SHA3_RAMP : AD_SHA3_DATA;
	//other_ch = iio_priv(st->ch_indio_dev[other]);
	//gpiod_set_value_cansleep(ch->reset_gpio, 0);
	//gpiod_set_value_cansleep(other_ch->reset_gpio, 0);

	//*readval = axiadc_read(st, reg & 0xFFFF);

	//ret = iio_dma_buffer_read(indio_dev->buffer, 64,
	//		char __user *user_buffer)
	//if (ret) {
	//	pr_info("admc_adc: %s\n", __func__);
	//}

	//data_available = iio_dma_buffer_usage(iio_buffer);
	//ret = memory_read_from_buffer(buf, count, &off, tab, len);
	//buffer->access->read(buffer, size_t n, char __user *buf);
	// Silly test
	//st->buf[0] = 7;
	//st->buf[1] = 6;
	//st->buf[2] = 5;
	//st->buf[3] = 4;
	//st->buf[4] = 3;
	//st->buf[5] = 2;
	//st->buf[6] = 1;
	//st->buf[7] = 0;
	//iio_push_to_buffers(indio_dev, st->buf);
	//or just
	//wake_up_interruptible_poll(&buffer->pollq, EPOLLIN | EPOLLRDNORM);
	return 0;
}

static int admc_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	//struct iio_buffer *buffer = indio_dev->buffer;
	//struct sha3_chan *ch = iio_priv(indio_dev);
	//struct axiadc_state *st = ch->axiadc_st;
	//enum ad_sha3_buffer other;
	struct iio_buffer *ramp_buf = st->ramp_channel->indio_dev->buffer;
	int ret;

	dev_info(indio_dev->dev.parent, "%s\n", __func__);

	dev_info(indio_dev->dev.parent, "%s current mode %d\n", __func__,
			iio_device_get_current_mode(indio_dev));
	//gpiod_set_value_cansleep(ch->reset_gpio, 1);

	if (strcmp(indio_dev->name, "ramp-reader") == 0)
		return 0;

	//other = ch->type == AD_SHA3_RAMP ? AD_SHA3_RAMP : AD_SHA3_DATA;
	//iio_device_release_buffer_mode(st->ch_indio_dev[other]);
	//iio_hw_consumer_disable(st->ramp_hw_cons);


	ret = ramp_buf->access->disable(ramp_buf,
					st->ramp_channel->indio_dev);
	if (ret) {
		dev_err(indio_dev->dev.parent, "err on disable: %d\n", ret);
		return ret;
	}

	iio_buffer_channel_disable(st->ramp_channel->indio_dev->buffer,
				st->ramp_channel);

	return 0;
}

static const struct iio_buffer_setup_ops admc_buffer_setup_ops = {
	.preenable = &admc_buffer_preenable,
	.postenable = &admc_buffer_postenable,
	.postdisable = &admc_buffer_postdisable,
};

//inpired on drivers/iio/frequency/m2k-dac.c
//static int ad_sha3_generator(struct platform_device *pdev,
//			     struct axiadc_state *st,
//			     enum ad_sha3_buffer b_type)
//{
//	struct iio_dev *indio_dev;
//	struct sha3_chan *ch;
//	int ret;
//
//	dev_info(&pdev->dev, "ADI AIM probing %s\n", sha_dma_names[b_type]);
//
//	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*ch));
//	if (!indio_dev)
//		return -ENOMEM;
//
//	ch = iio_priv(indio_dev);
//	ch->type = b_type;
//	ch->axiadc_st = st;
//
//	indio_dev->dev.parent = &pdev->dev;
//	indio_dev->name = device_names[b_type];
//	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
//	indio_dev->setup_ops = &admc_buffer_setup_ops;
//	indio_dev->info = &axiadc_info;
//	indio_dev->channels = axiadc_chip_info_tbl[b_type].channel;
//	indio_dev->num_channels = axiadc_chip_info_tbl[b_type].num_channels;
//
//	ret = devm_iio_dmaengine_buffer_setup(&pdev->dev, indio_dev,
//					      sha_dma_names[b_type]);
//	if (ret < 0)
//		return ret;
//
//	st->ch_indio_dev[b_type] = indio_dev;
//
//	ch->reset_gpio = devm_gpiod_get_optional(&pdev->dev, reset_gpio_names[b_type],
//						 GPIOD_OUT_LOW);
//	if (IS_ERR(ch->reset_gpio)) {
//		//return PTR_ERR(ch->reset_gpio);
//		dev_err_probe(&pdev->dev, PTR_ERR(ch->reset_gpio),
//			      "failed to get %s reset\n",
//			      reset_gpio_names[b_type]);
//	}
//
//	return 0;
//}

static int axiadc_probe(struct platform_device *pdev)
{
	const struct axiadc_chip_info *chip_info;
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	int ret;

	dev_info(&pdev->dev, "Probing xlnx,axi-ad-mc-adc-1.00.a\n");

	//st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	//if (!st)
	//	return -ENOMEM;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	//mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//st->regs = devm_ioremap_resource(&pdev->dev, mem);
	//if (IS_ERR(st->regs))
	//	return PTR_ERR(st->regs);

	if (strcmp(pdev->dev.of_node->name, "sha3-reader") == 0) {
		dev_info(&pdev->dev, "sha3-reader");
		//st->regs = 0x64a00000;
		chip_info = &axiadc_chip_info_tbl[AD_SHA3_DATA];

		//st->ramp_reset_gpio = devm_gpiod_get_optional(&pdev->dev,
		//					reset_gpio_names[AD_SHA3_RAMP],
		//					GPIOD_OUT_LOW);
		//if (IS_ERR(st->ramp_reset_gpio))
		//	//return PTR_ERR(ch->reset_gpio);
		//	dev_err_probe(&pdev->dev, PTR_ERR(st->ramp_reset_gpio),
		//		      "failed to get %s reset\n",
		//		      reset_gpio_names[AD_SHA3_RAMP]);

		//st->sha3_reset_gpio = devm_gpiod_get_optional(&pdev->dev,
		//					reset_gpio_names[AD_SHA3_DATA],
		//					GPIOD_OUT_LOW);
		//if (IS_ERR(st->sha3_reset_gpio))
		//	//return PTR_ERR(ch->reset_gpio);
		//	dev_err_probe(&pdev->dev, PTR_ERR(st->sha3_reset_gpio),
		//		      "failed to get %s reset\n",
		//		      reset_gpio_names[AD_SHA3_DATA]);



		//st->ramp_channel = devm_iio_channel_get(&pdev->dev, "ramp-data");
		//if (IS_ERR(st->ramp_channel)) {
		//	dev_err_probe(&pdev->dev, PTR_ERR(st->ramp_channel),
		//			"devm_iio_channel_get failed\n");
		//}

		//st->ramp_hw_cons = devm_iio_hw_consumer_alloc(&pdev->dev);
		//if (IS_ERR(st->ramp_hw_cons))
		//	dev_err_probe(&pdev->dev, PTR_ERR(st->ramp_hw_cons),
		//			"devm_iio_hw_consumer_alloc failed\n");
	} else {
		dev_info(&pdev->dev, "ramp-reader");
		//st->regs = 0x65a00000;
		chip_info = &axiadc_chip_info_tbl[AD_SHA3_RAMP];
	}

	platform_set_drvdata(pdev, indio_dev);

	/* Reset all HDL Cores */
	//axiadc_write(st, ADI_REG_RSTN, 0);
	//axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	//st->pcore_version = axiadc_read(st, ADI_AXI_REG_VERSION);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = chip_info->channel;
	indio_dev->num_channels = chip_info->num_channels;

	st->iio_info = axiadc_info;
	indio_dev->info = &st->iio_info;

	ret = devm_iio_dmaengine_buffer_setup(&pdev->dev, indio_dev, "ad-mc-adc-dma");
	if (ret < 0)
		return ret;

	//indio_dev->setup_ops = &admc_buffer_setup_ops;

	//ret = ad_sha3_generator(pdev, st, AD_SHA3_RAMP);
	//if (ret)
	//	dev_err_probe(&pdev->dev, ret, "AD_SHA3_RAMP failed\n");

	//ret = ad_sha3_generator(pdev, st, AD_SHA3_DATA);
	//if (ret)
	//	dev_err_probe(&pdev->dev, ret, "AD_SHA3_DATA failed\n");

	//TODO
	//phy->pdata->sync_gpio = devm_gpiod_get_optional(&spi->dev, "sha-mux",
	//	GPIOD_OUT_LOW);
	//if (IS_ERR(phy->pdata->sync_gpio))
	//	return PTR_ERR(phy->pdata->sync_gpio);


	//mutex_init(&st->lock);

	//dev_info(&pdev->dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p, probed ADC %s as %s\n",
	//	 st->pcore_version,
	//	 (unsigned long long)mem->start, st->regs, chip_info->name,
	//	 axiadc_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER");


	//ret = admc_register_debugfs(indio_dev);
	//if (ret < 0)
	//	dev_warn(&pdev->dev, "%s: failed to register debugfs", __func__);

	//ret = devm_iio_device_register(&pdev->dev, st->ch_indio_dev[AD_SHA3_RAMP]);
	//if (ret)
	//	return ret;

	//ret = devm_iio_device_register(&pdev->dev, st->ch_indio_dev[AD_SHA3_DATA]);
	//if (ret)
	//	return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "ADI AIM probed ADC\n");

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] = {
	{ .compatible = "xlnx,axi-ad-mc-adc-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

static struct platform_driver axiadc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe	  = axiadc_probe,
};

module_platform_driver(axiadc_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
