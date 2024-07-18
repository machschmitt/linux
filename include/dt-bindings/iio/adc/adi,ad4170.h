/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */

#ifndef _DT_BINDINGS_IIO_ADC_AD4170_H_
#define _DT_BINDINGS_IIO_ADC_AD4170_H_

/*
 * ADC channel voltage reference selection.
 * Use for adi,reference-select.
 */
#define AD4170_REF_REFIN1		0
#define AD4170_REF_REFIN2		1
#define AD4170_REF_REFOUT		2
#define AD4170_REF_AVDD			3

/*
 * Input pin selection.
 * Use for diff-channels, single-channel, and common-mode-channel.
 */
#define AD4170_MAP_AIN0			0
#define AD4170_MAP_AIN1			1
#define AD4170_MAP_AIN2			2
#define AD4170_MAP_AIN3			3
#define AD4170_MAP_AIN4			4
#define AD4170_MAP_AIN5			5
#define AD4170_MAP_AIN6			6
#define AD4170_MAP_AIN7			7
#define AD4170_MAP_AIN8			8
#define AD4170_MAP_TEMP_SENSOR		17
#define AD4170_MAP_AVDD_AVSS_P		18
#define AD4170_MAP_AVDD_AVSS_N		18
#define AD4170_MAP_IOVDD_DGND_P		19
#define AD4170_MAP_IOVDD_DGND_N		19
#define AD4170_MAP_DAC_P		20
#define AD4170_MAP_DAC_N		20
#define AD4170_MAP_ALDO			21
#define AD4170_MAP_DLDO			22
#define AD4170_MAP_AVSS			23
#define AD4170_MAP_DGND			24
#define AD4170_MAP_REFIN1_P		25
#define AD4170_MAP_REFIN1_N		26
#define AD4170_MAP_REFIN2_P		27
#define AD4170_MAP_REFIN2_N		28
#define AD4170_MAP_REFOUT		29

/*
 * External circuit excitation pin selection.
 * Use for adi,excitation-pins.
 */
#define AD4170_IOUT_AIN0		0
#define AD4170_IOUT_AIN1		1
#define AD4170_IOUT_AIN2		2
#define AD4170_IOUT_AIN3		3
#define AD4170_IOUT_AIN4		4
#define AD4170_IOUT_AIN5		5
#define AD4170_IOUT_AIN6		6
#define AD4170_IOUT_AIN7		7
#define AD4170_IOUT_AIN8		8
#define AD4170_IOUT_GPIO0		17
#define AD4170_IOUT_GPIO1		18
#define AD4170_IOUT_GPIO2		19
#define AD4170_IOUT_GPIO3		20

#endif /* _DT_BINDINGS_IIO_ADC_AD4170_H_ */
