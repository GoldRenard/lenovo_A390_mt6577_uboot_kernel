/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for EPL6881 als/ps sensor chip.
 */
#ifndef __EPL6881_H__
#define __EPL6881_H__

#include <linux/ioctl.h>

extern int EPL6881_CMM_PPCOUNT_VALUE;
extern int ZOOM_TIME;

#define EPL6881_CMM_ENABLE 		0X07
#define EPL6881_CMM_ATIME 		0X81
#define EPL6881_CMM_PTIME 		0X82
#define EPL6881_CMM_WTIME 		0X83
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/

#define EPL6881_CMM_INT_LOW_THD_LOW   0X04
#define EPL6881_CMM_INT_LOW_THD_HIGH  0X05
#define EPL6881_CMM_INT_HIGH_THD_LOW  0X02
#define EPL6881_CMM_INT_HIGH_THD_HIGH 0X03

#define EPL6881_CMM_Persistence       0X8C
#define EPL6881_CMM_STATUS            0X93
#define TAOS_TRITON_CMD_REG           0X80
#define TAOS_TRITON_CMD_SPL_FN        0x60

#define EPL6881_CMM_CONFIG 		0X8D
#define EPL6881_CMM_PPCOUNT 		0X8E
#define EPL6881_CMM_CONTROL 		0X8F

#define EPL6881_CMM_PDATA_L 		0X98
#define EPL6881_CMM_PDATA_H 		0X99
#define EPL6881_CMM_C0DATA_L 	0X94
#define EPL6881_CMM_C0DATA_H 	0X95
#define EPL6881_CMM_C1DATA_L 	0X96
#define EPL6881_CMM_C1DATA_H 	0X97


#define EPL6881_SUCCESS						0
#define EPL6881_ERR_I2C						-1
#define EPL6881_ERR_STATUS					-3
#define EPL6881_ERR_SETUP_FAILURE				-4
#define EPL6881_ERR_GETGSENSORDATA			-5
#define EPL6881_ERR_IDENTIFICATION			-6

/*for epl6881 command */


/*for epl6881 command */
#define REG_0			0X00
#define REG_1			0X01
#define REG_2			0X02
#define REG_3			0X03
#define REG_4			0X04
#define REG_5			0X05
#define REG_6			0X06
#define REG_7			0X07
#define REG_8			0X08
#define REG_9			0X09
#define REG_10			0X0A
#define REG_11			0X0B
#define REG_12			0X0C
#define REG_13			0X0D
#define REG_14			0X0E
#define REG_15			0X0F
#define REG_16			0X10
#define REG_17			0X11
#define REG_18			0X12
#define REG_19			0X13
#define REG_20			0X14
#define REG_21			0X15

#define W_SINGLE_BYTE		0X00
#define W_TWO_BYTE		0X01
#define W_THREE_BYTE		0X02
#define W_FOUR_BYTE		0X03
#define W_FIVE_BYTE		0X04
#define W_SIX_BYTE		0X05
#define W_SEVEN_BYTE		0X06
#define W_EIGHT_BYTE		0X07

#define R_SINGLE_BYTE		0X00
#define R_TWO_BYTE		0X01
#define R_THREE_BYTE		0X02
#define R_FOUR_BYTE		0X03
#define R_FIVE_BYTE		0X04
#define R_SIX_BYTE		0X05
#define R_SEVEN_BYTE		0X06
#define R_EIGHT_BYTE		0X07

#define EPL_SENSING_1_TIME	(0 << 5) 
#define EPL_SENSING_2_TIME	(1 << 5)
#define EPL_SENSING_4_TIME	(2 << 5)
#define EPL_SENSING_8_TIME	(3 << 5)
#define EPL_SENSING_16_TIME	(4 << 5)
#define EPL_SENSING_32_TIME	(5 << 5)
#define EPL_SENSING_64_TIME	(6 << 5)
#define EPL_SENSING_128_TIME	(7 << 5)
#define EPL_C_SENSING_MODE	(0 << 4)
#define EPL_S_SENSING_MODE	(1 << 4)
#define EPL_ALS_MODE		(0 << 2)
#define EPL_PS_MODE		(1 << 2)
#define EPL_TEMP_MODE 		(2 << 2)
#define EPL_H_GAIN		(0)
#define EPL_M_GAIN		(1)
#define EPL_L_GAIN		(3)
#define EPL_AUTO_GAIN		(2)


#define EPL_8BIT_ADC		0
#define EPL_10BIT_ADC		1
#define EPL_12BIT_ADC		2
#define EPL_14BIT_ADC		3


#define EPL_C_RESET		0x00
#define EPL_C_START_RUN		0x04
#define EPL_C_P_UP		0x04
#define EPL_C_P_DOWN		0x06
#define EPL_DATA_LOCK		0x05
#define EPL_DATA_UNLOCK		0x04

#define EPL_INT_BINARY			0
#define EPL_INT_DISABLE			2
#define EPL_INT_ACTIVE_LOW		3
#define EPL_INT_FRAME_ENABLE		4

#define EPL_PST_1_TIME		(0 << 2)
#define EPL_PST_4_TIME		(1 << 2)
#define EPL_PST_8_TIME		(2 << 2)
#define EPL_PST_16_TIME		(3 << 2)


#endif


