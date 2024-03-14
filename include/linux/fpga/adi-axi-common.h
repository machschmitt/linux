/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices AXI common registers & definitions
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_ip
 * https://wiki.analog.com/resources/fpga/docs/hdl/regmap
 */

#ifndef ADI_AXI_COMMON_H_
#define ADI_AXI_COMMON_H_

#define ADI_AXI_REG_VERSION			0x0000
#define ADI_AXI_REG_ID				0x0004

#define ADI_AXI_PCORE_VER(major, minor, patch)	\
	(((major) << 16) | ((minor) << 8) | (patch))

#define ADI_AXI_PCORE_VER_MAJOR(version)	(((version) >> 16) & 0xff)
#define ADI_AXI_PCORE_VER_MINOR(version)	(((version) >> 8) & 0xff)
#define ADI_AXI_PCORE_VER_PATCH(version)	((version) & 0xff)

#define ADI_AXI_REG_CONFIG			0x000C
#define ADI_AXI_IQCORRECTION_DISABLE		(1 << 0)
#define ADI_AXI_DCFILTER_DISABLE		(1 << 1)
#define ADI_AXI_DATAFORMAT_DISABLE		(1 << 2)
#define ADI_AXI_USERPORTS_DISABLE		(1 << 3)
#define ADI_AXI_MODE_1R1T			(1 << 4)
#define ADI_AXI_DELAY_CONTROL_DISABLE		(1 << 5)
#define ADI_AXI_DDS_DISABLE			(1 << 6)
#define ADI_AXI_CMOS_OR_LVDS_N			(1 << 7)
#define ADI_AXI_PPS_RECEIVER_ENABLE		(1 << 8)
#define ADI_AXI_SCALECORRECTION_ONLY		(1 << 9)
#define ADI_AXI_XBAR_ENABLE			(1 << 10)
#define ADI_AXI_EXT_SYNC			(1 << 12)

#endif /* ADI_AXI_COMMON_H_ */
