/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/uaccess.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include "adi-axi-dds.h"

/* FIXME
 * Fix buffer setup or remove this.
 */
//static int dds_buffer_submit_block(struct iio_dma_buffer_queue *queue,
//	struct iio_dma_buffer_block *block)
//{
//	struct cf_axi_dds_state *st = iio_priv(queue->driver_data);
//	bool enable_fifo = false;
//	bool oneshot = true;
//
//	if (block->block.bytes_used) {
//		if (cf_axi_dds_dma_fifo_en(st)) {
//			enable_fifo = true;
//
//			if (block->block.flags & IIO_BUFFER_BLOCK_FLAG_CYCLIC) {
//				block->block.flags &= ~IIO_BUFFER_BLOCK_FLAG_CYCLIC;
//				oneshot = false;
//			}
//
//			cf_axi_dds_pl_ddr_fifo_ctrl_oneshot(st, oneshot);
//		}
//
//		cf_axi_dds_pl_ddr_fifo_ctrl(st, enable_fifo);
//	}
//
//	/* FIXME
//	 * Fix buffer setup or remove this.
//	 */
//	return 0;
//	//return iio_dmaengine_buffer_submit_block(queue, block);
//}

static int dds_buffer_state_set(struct iio_dev *indio_dev, bool state)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	int ret;

#if 0
	tmp_reg = dds_read(st, CF_AXI_DDS_DMA_STAT);
	if (tmp_reg & (CF_AXI_DDS_DMA_STAT_OVF | CF_AXI_DDS_DMA_STAT_UNF))
		dev_warn(indio_dev->dev.parent, "VDMA Status: %s %s\n",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "overflow" : "",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "underflow" : "");
#endif

	if (!state)
		return cf_axi_dds_datasel(st, -1, DATA_SEL_DDS);

	ret = regmap_write(st->regmap, ADI_REG_VDMA_STATUS,
			   ADI_VDMA_OVF | ADI_VDMA_UNF);
	if (ret)
		return ret;

	return cf_axi_dds_start_sync(st, 1);
}

static int dds_buffer_preenable(struct iio_dev *indio_dev)
{
	return dds_buffer_state_set(indio_dev, 1);
}

static int dds_buffer_postdisable(struct iio_dev *indio_dev)
{
	return dds_buffer_state_set(indio_dev, 0);
}

static const struct iio_buffer_setup_ops dds_buffer_setup_ops = {
	.preenable = &dds_buffer_preenable,
	.postdisable = &dds_buffer_postdisable,
};

/* FIXME
 * Fix buffer setup or remove this.
 */
//static const struct iio_dma_buffer_ops dds_buffer_dma_buffer_ops = {
//	.submit = dds_buffer_submit_block,
//	.abort = iio_dmaengine_buffer_abort,
//};

int cf_axi_dds_configure_buffer(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	int ret;

	/* FIXME
	 * The upstream API always use the default dma_buffer_ops which
	 * will not do the checks and preparations of dds_buffer_submit_block().
	 */
	//buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "tx",
	//					 &dds_buffer_dma_buffer_ops, indio_dev);
	//if (IS_ERR(buffer))
	//	return PTR_ERR(buffer);

	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
						 indio_dev, "tx");
	if (ret < 0)
		return ret;
	//buffer->direction = IIO_BUFFER_DIRECTION_OUT;
	indio_dev->buffer->direction = IIO_BUFFER_DIRECTION_OUT;
	//ret = iio_device_attach_buffer(indio_dev, buffer);
	//if (ret < 0)
	//	return ret;

	//indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &dds_buffer_setup_ops;

	return 0;
}
EXPORT_SYMBOL_GPL(cf_axi_dds_configure_buffer);

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
