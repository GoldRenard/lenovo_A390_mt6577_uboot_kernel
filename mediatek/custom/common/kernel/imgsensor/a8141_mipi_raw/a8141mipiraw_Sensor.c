/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   RAW
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Jackie Su (MTK02380)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 11 11 2011 koli.lin
 * [ALPS00086510] [Camera] preview?率?低
 * [Camera] Modify the exposure line for flicker enable.
 *
 * 11 11 2011 koli.lin
 * [ALPS00030473] [Camera]
 * [Camera] Modify the flicker frame rate control.
 *
 * 11 01 2011 koli.lin
 * [ALPS00030473] [Camera]
 * [Camera] Add the flicker flag disable control before enter the ZSD mode.
 *
 * 10 31 2011 koli.lin
 * [ALPS00081266] [Li Zhen]

P49?是存在此?象


[Wenjing]

Hi,Koli:

tester?有抓到??出??的?片，但是?描述看，??跟之前6573上何?提在video mode的一?issue相同，麻??忙check

thanks


李珍  61204

 * [Camera] 1. Modify the preview output speed 648Mbps/lane.
 *                2. Fix the flicker min limitation value. (max 1 fps)
 *
 * 10 17 2011 koli.lin
 * [ALPS00074853] [Camera]flicker is serious when preview with light object
 * [Camera] change the sensor binning mode to 2x from 4x to reduce the flicker serious..
 *
 * 10 07 2011 koli.lin
 * [ALPS00030473] [Camera]
 * [Camera] Add the reset video mode flag in the capture function for ZSD used..
 *
 * 10 07 2011 koli.lin
 * [ALPS00077581] [Need Patch][Sanity Fail] When entering to Camera at the second time, camera cannot work and will see black screen
 * [Camera] Rest the flicker flag value during the sensor initial.
 *
 * 06 13 2011 koli.lin
 * [ALPS00053429] [Need Patch] [Volunteer Patch]
 * [Camera] Modify the sensor color order for the CCT tool.
 *
 * 06 07 2011 koli.lin
 * [ALPS00050047] [Camera]AE flash when set EV as -2
 * [Camera] Rollback the preview resolution to 800x600.
 *
 * 06 07 2011 koli.lin
 * [ALPS00049935] [Camera] AE is not correct when switch between camera still mode and video mode
 * [Camera] Rollback the preview resolution to 800x600.
 *
 * 06 01 2011 koli.lin
 * [ALPS00051509] [Need Patch] [Volunteer Patch]
 * [Camera] Fix the mipi color order is not correctly for MT6573.
 *
 * 05 17 2011 koli.lin
 * [ALPS00048194] [Need Patch] [Volunteer Patch]
 * [Camera]. Chagne the preview size to 1600x1200 for A8141 sensor.
 *
 * 05 02 2011 koli.lin
 * [ALPS00040837] [Need check with submitter][WW FT][MT6573][Guangzhou]Exception ANR
 * [Camera] Fix the exposure time calculate mistake.
 *
 * 04 22 2011 koli.lin
 * [ALPS00042482] [Need Patch] [Volunteer Patch]
 * [Camera] Move the sensor senstivity control to AE for preview and capture mode.
 *
 * 04 11 2011 koli.lin
 * [ALPS00039429] [Need Patch] [Volunteer Patch]
 * [Camera] Add flicker frame rate and sensor test pattern feature id.
 *
 * 04 08 2011 koli.lin
 * [ALPS00037040] [Display]There is tearing on preview screen
 * [Camera] Disable the gain group setting and reduce the gain delay frame number.
 *
 * 04 06 2011 koli.lin
 * [ALPS00036187] [Camera]after do EV shot sometimes, part of the preview screen flashes
 * [Camera] Modify the capture delay frame to get better performance..
 *
 * 04 02 2011 koli.lin
 * [ALPS00036213] [Camera]change EV but it does not work
 * [Camera] 1.Add the CSI2 debug message for dump register function.
 *                2.Modify image start position to avoid the black line..
 *
 * 04 01 2011 koli.lin
 * [ALPS00143914] [Android Build Warning Issue] mediatek/custom/out/ztemt73v2/kernel/imgsensor/a8141mipiraw_Sensor.c
 * [Camera] 1.Clear the compile warning message.
 *                2. Add the sensor driver 2D/3D information.
 *                3.Add the auto flicker control and test pattern feature.
 *
 * 04 01 2011 koli.lin
 * [ALPS00037668] [MPEG4 recording]record high quality video with night mode, the frame rate is not 15fps
 * [Camera] Add the video mode frame rate control function.
 *
 * 04 01 2011 koli.lin
 * [ALPS00037670] [MPEG4 recording]the frame rate of fine quality video can not reach 30fps
 * [Camera]Modify the sensor preview output resolution and line time to fix frame rate at 30fps for video mode.
 *
 * 03 15 2011 koli.lin
 * [ALPS00034474] [Need Patch] [Volunteer Patch]
 * Move sensor driver current setting to isp of middleware.
 *
 * 03 10 2011 koli.lin
 * [ALPS00033930] [Need Patch] [Volunteer Patch]
 * Moving the CSI2 control flow from sensor driver to middleware.
 *
 * 03 02 2011 koli.lin
 * [ALPS00032905] [Need Patch] [Volunteer Patch]
 * Reset to default.
 *
 * 03 02 2011 koli.lin
 * [ALPS00032905] [Need Patch] [Volunteer Patch]
 * Provide the sensor width/height sampling ratio..
 *
 * 02 25 2011 koli.lin
 * [ALPS00032248] [Need Patch] [Volunteer Patch]
 * Close the csi2 before sensor driver close.
 *
 * 02 11 2011 koli.lin
 * [ALPS00030473] [Camera]
 * Add the csi2 control to a8141 sensor driver.
 *
 * 02 11 2011 koli.lin
 * [ALPS00030473] [Camera]
 * Change sensor driver preview size ratio to 4:3.
 *
 * 02 11 2011 koli.lin
 * [ALPS00030473] [Camera]
 * Modify the A8141 sensor driver for preview mode.
 *
 * 02 11 2011 koli.lin
 * [ALPS00030473] [Camera]
 * Create A8141 sensor driver to database.
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "a8141mipiraw_Sensor.h"
#include "a8141mipiraw_Camera_Sensor_para.h"
#include "a8141mipiraw_CameraCustomized.h"

static DEFINE_SPINLOCK(a8141mipi_drv_lock);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_bool  A8141MIPI_MPEG4_encode_mode = KAL_FALSE;
kal_bool A8141MIPI_Auto_Flicker_mode = KAL_FALSE;

kal_uint32 A8141MIPI_PV_vt_pclk = 1690; 
kal_uint32 A8141MIPI_CAP_vt_pclk = 1625; //1820; //1625; //182;

kal_uint16  A8141MIPI_sensor_gain = 0x0;

kal_uint16  A8141MIPI_sensor_gain_base = 0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 A8141MIPI_MAX_EXPOSURE_LINES = A8141MIPI_PV_PERIOD_LINE_NUMS;//650;
//kal_uint8  A8141MIPI_MIN_EXPOSURE_LINES = 1;
kal_uint32 A8141MIPI_isp_master_clock;
//kal_uint16 A8141MIPI_CURRENT_FRAME_LINES = A8141MIPI_PV_PERIOD_LINE_NUMS;//650;

static kal_uint16 A8141MIPI_dummy_pixels=0, A8141MIPI_dummy_lines=0;
kal_uint16 A8141MIPI_PV_dummy_pixels=0,A8141MIPI_PV_dummy_lines=0;

kal_uint8 A8141MIPI_sensor_write_I2C_address = A8141MIPI_WRITE_ID;
kal_uint8 A8141MIPI_sensor_read_I2C_address = A8141MIPI_READ_ID;

ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM COLOR_FORMAT=SENSOR_OUTPUT_FORMAT_RAW_R;

#define SENSORDB(fmt, arg...) printk( "[A8141MIPIRaw] "  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT

UINT8 A8141MIPIPixelClockDivider=0;


kal_uint16 A8141MIPI_pv_exposure_lines=0x100, A8141MIPI_g_iBackupExtraExp = 0, A8141MIPI_extra_exposure_lines = 0;

kal_uint16 A8141MIPI_sensor_id=0;

MSDK_SENSOR_CONFIG_STRUCT A8141MIPISensorConfigData;

kal_uint32 A8141MIPI_FAC_SENSOR_REG;
kal_uint16 A8141MIPI_sensor_flip_value; 

static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = ACDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT A8141MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT A8141MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

typedef enum
{
  A8141MIPI_2M,       //2M 1600x1200
  A8141MIPI_8M,           //8M 3280x2464
} A8141MIPI_RES_TYPE;
A8141MIPI_RES_TYPE A8141MIPI_g_RES=A8141MIPI_2M;

typedef enum
{
  A8141MIPI_MODE_PREVIEW,  //1M  	1280x960
  A8141MIPI_MODE_CAPTURE   //8M    3280x2464
} A8141MIPI_MODE;
A8141MIPI_MODE g_iA8141MIPI_Mode=A8141MIPI_MODE_PREVIEW;



kal_uint16 A8141MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8) ,(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 4,A8141MIPI_sensor_write_I2C_address);

}
kal_uint16 A8141MIPI_write_cmos_sensor8(kal_uint32 addr, kal_uint32 para)
{
    char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 3,A8141MIPI_sensor_write_I2C_address);

}

kal_uint16 A8141MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,A8141MIPI_sensor_write_I2C_address);
#ifdef A8141MIPI_DEBUG
	SENSORDB("A8141MIPI_read_cmos_sensor, addr:%x;get_byte:%x \n",addr,get_byte);
#endif		
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);;
}



#define Sleep(ms) mdelay(ms)

void A8141MIPI_write_shutter(kal_uint16 shutter)
{
    kal_uint16 iExp = shutter;

	if(A8141MIPI_Auto_Flicker_mode)
	{
		iExp=iExp+(iExp>>7);
	}
	
	A8141MIPI_write_cmos_sensor(0x3012, (iExp  & 0xFFFF));
	
	if(A8141MIPI_g_RES == A8141MIPI_2M && !A8141MIPI_MPEG4_encode_mode)
		{
	    if(shutter > A8141MIPI_PV_PERIOD_LINE_NUMS)
	    	{
			A8141MIPI_write_cmos_sensor(0x0340,shutter);  //in binning mode, frame length lines - shutter must >3
			#ifdef A8141MIPI_DEBUG
			SENSORDB("[A8141MIPI_write_shutter] framelength is :0x%x \n ", shutter+3);
   			#endif
			
	    	}
		else 
			{
			A8141MIPI_write_cmos_sensor(0x0340,A8141MIPI_PV_PERIOD_LINE_NUMS);  //in binning mode, frame length lines - shutter must >3
			#ifdef A8141MIPI_DEBUG
			SENSORDB("[A8141MIPI_write_shutter] framelength is :0x%x \n ", A8141MIPI_PV_PERIOD_LINE_NUMS);
			#endif
			}
		}
	

   
}   /* write_A8141MIPI_shutter */

static kal_uint16 A8141MIPIReg2Gain(const kal_uint8 iReg)
{
	kal_uint16 iGain;
	#ifdef A8141MIPI_DEBUG
	SENSORDB("[A8141MIPIReg2Gain] iReg is :%d \n", iReg);
    #endif
    // Range: 1x to 4x
    iGain=iReg & 0x007F;
    #ifdef A8141MIPI_DEBUG
	SENSORDB("[A8141MIPIReg2Gain] iGain is :%d \n", iGain);
	#endif
    return iGain;
}

static kal_uint16 A8141MIPIGain2Reg(const kal_uint16 iGain)
{
	kal_uint16 iReg,tempGain;
	
	//printk("[A8141MIPIGain2Reg info] final iGain is :%d \n", iGain);
	if(iGain<96)  //sensor work mix gain must not < 1.5X
	{
	  #ifdef A8141MIPI_DEBUG
      SENSORDB("[A8141MIPIGain2Reg Error] iGain is :%d \n, must >= 96(1.5X)", iGain);
	  #endif
	  tempGain = 96;
	  iReg = 0x1000 | (tempGain & 0x007F);
	  #ifdef A8141MIPI_DEBUG
	  SENSORDB("[A8141MIPIGain2Reg] iReg is :0x%x \n ", iReg);
	  #endif
	  return iReg;
	}
    if(iGain>1016)
	{
	  #ifdef A8141MIPI_DEBUG
      SENSORDB("[A8141MIPIGain2Reg Error] iGain is :%d \n, must <= 1016(15.875X)", iGain);
	  #endif
	  tempGain = 1016;
	  tempGain=iGain/8;
	  iReg = 0x1680 | (tempGain & 0x007F);
	  #ifdef A8141MIPI_DEBUG
	  SENSORDB("[A8141MIPIGain2Reg] iReg is :0x%x \n ", iReg);
	  #endif
	  return iReg;
	}
	tempGain=iGain;
	
	if (iGain >= 512)
		{
		tempGain=iGain/8;
	    iReg = 0x1680 | (tempGain & 0x007F);
		}
	else if (iGain >=256 && iGain<= 508) 
		{
		tempGain=iGain/4;
	    iReg = 0x1600 | (tempGain & 0x007F);
		}
	else if ( iGain >= 128 && iGain <= 254)	
		{
		tempGain=iGain/2;
	    iReg = 0x1400 | (tempGain & 0x007F);
		}
	else
		{
		iReg = 0x1000 | (tempGain & 0x007F);
		}
    #ifdef A8141MIPI_DEBUG
	SENSORDB("[A8141MIPIGain2Reg] iReg is :0x%x \n ", iReg);
	#endif
    return iReg;
}

/*************************************************************************
* FUNCTION
*    A8141MIPI_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void A8141MIPI_SetGain(UINT16 iGain)
{
	unsigned long flags;
    kal_uint16 iReg;
//	#ifdef A8141MIPI_DEBUG
	SENSORDB("[A8141MIPI_SetGain] iGain is :%d \n ",iGain);
//	#endif
	spin_lock_irqsave(&a8141mipi_drv_lock,flags);
    A8141MIPI_sensor_gain=iGain;
	spin_unlock_irqrestore(&a8141mipi_drv_lock,flags);
    iReg = A8141MIPIGain2Reg(iGain);
    //A8141MIPI_write_cmos_sensor(0x3056, (kal_uint16)iReg);  //
	//A8141MIPI_write_cmos_sensor(0x305C, (kal_uint16)iReg);
	//A8141MIPI_write_cmos_sensor(0x305A, (kal_uint16)iReg);
	//A8141MIPI_write_cmos_sensor(0x3058, (kal_uint16)iReg);
	A8141MIPI_write_cmos_sensor(0x305E, (kal_uint16)iReg);

}   /*  A8141MIPI_SetGain  */


/*************************************************************************
* FUNCTION
*    read_A8141MIPI_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_A8141MIPI_gain(void)
{
    return (kal_uint16)A8141MIPI_read_cmos_sensor(0x305E);
}  /* read_A8141MIPI_gain */

void write_A8141MIPI_gain(kal_uint16 gain)
{
    A8141MIPI_SetGain(gain);
}
void A8141MIPI_camera_para_to_sensor(void)
{
    kal_uint32    i;
	#ifdef A8141MIPI_DEBUG
	SENSORDB("A8141MIPI_camera_para_to_sensor\n");
	#endif
    for(i=0; 0xFFFFFFFF!=A8141MIPISensorReg[i].Addr; i++)
    {
        A8141MIPI_write_cmos_sensor(A8141MIPISensorReg[i].Addr, A8141MIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=A8141MIPISensorReg[i].Addr; i++)
    {
        A8141MIPI_write_cmos_sensor(A8141MIPISensorReg[i].Addr, A8141MIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        A8141MIPI_write_cmos_sensor(A8141MIPISensorCCT[i].Addr, A8141MIPISensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    A8141MIPI_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void A8141MIPI_sensor_to_camera_para(void)
{
    kal_uint32 i, temp_data;
	#ifdef A8141MIPI_DEBUG
	SENSORDB("A8141MIPI_sensor_to_camera_para\n"); 
	#endif
    for(i=0; 0xFFFFFFFF!=A8141MIPISensorReg[i].Addr; i++)
    {
        temp_data = A8141MIPI_read_cmos_sensor(A8141MIPISensorReg[i].Addr);
		spin_lock(&a8141mipi_drv_lock);
        A8141MIPISensorReg[i].Para = temp_data;
		spin_unlock(&a8141mipi_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=A8141MIPISensorReg[i].Addr; i++)
    {
    	temp_data = A8141MIPI_read_cmos_sensor(A8141MIPISensorReg[i].Addr);
		spin_lock(&a8141mipi_drv_lock);
        A8141MIPISensorReg[i].Para = temp_data;
		spin_unlock(&a8141mipi_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    A8141MIPI_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  A8141MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void A8141MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void A8141MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;
    
    switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Global");
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"GLOBAL_GAIN");
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }

            temp_para=A8141MIPISensorCCT[temp_addr].Para;

            temp_gain = A8141MIPIReg2Gain(temp_para);

            temp_gain=(temp_gain*1000)/BASEGAIN;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min=1536;
            info_ptr->Max=16256;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                    //temp_reg=A8141MIPISensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }
                
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=A8141MIPI_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}

//void A8141MIPI_set_isp_driving_current(kal_uint8 current)
//{

//}

kal_bool A8141MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16 temp_addr=0, temp_para=0;
   #ifdef A8141MIPI_DEBUG
   SENSORDB("A8141MIPI_set_sensor_item_info\n"); 
   #endif
   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }

            temp_para = A8141MIPIGain2Reg(ItemValue);
            spin_lock(&a8141mipi_drv_lock);
            A8141MIPISensorCCT[temp_addr].Para = temp_para;
			spin_unlock(&a8141mipi_drv_lock);
            A8141MIPI_write_cmos_sensor(A8141MIPISensorCCT[temp_addr].Addr,temp_para);
			
			spin_lock(&a8141mipi_drv_lock);
            A8141MIPI_sensor_gain_base=read_A8141MIPI_gain();
			spin_unlock(&a8141mipi_drv_lock);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
					spin_lock(&a8141mipi_drv_lock);
                    if(ItemValue==2)
                    {
                        A8141MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                        //A8141MIPI_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        A8141MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                        //A8141MIPI_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        A8141MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                        //A8141MIPI_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                        A8141MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                        //A8141MIPI_set_isp_driving_current(ISP_DRIVING_8MA);
                    }
					spin_unlock(&a8141mipi_drv_lock);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&a8141mipi_drv_lock);
                    A8141MIPI_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&a8141mipi_drv_lock);
                    break;
                case 1:
                    A8141MIPI_write_cmos_sensor(A8141MIPI_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

static void A8141MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{

}   /*  A8141MIPI_SetDummy */

static void A8141MIPI_Sensor_Init(void)
{
#ifdef A8141MIPI_DEBUG
	SENSORDB("A8141MIPI_Sensor_Init +\n"); 
#endif 

#if 1
    
	A8141MIPI_write_cmos_sensor8( 0x0103, 0x01 	);// SOFTWARE_RESET
	mdelay(300);

	//Stop Streaming
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0058); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0050); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor8( 0x0104, 0x01);// GROUPED_PARAMETER_HOLD
	                                           
	//2-lane MIPI Interface Configuration      
	A8141MIPI_write_cmos_sensor( 0x3064, 0x7800); 	// SMIA_TEST
	A8141MIPI_write_cmos_sensor( 0x31AE, 0x0202); 	// SERIAL_FORMAT
	A8141MIPI_write_cmos_sensor( 0x31B8, 0x0E3F); 	// MIPI_TIMING_2
	mdelay(5);                                    
#ifdef USING_A8140_setting
	//[Recommended Settings]
	A8141MIPI_write_cmos_sensor( 0x3044, 0x0590);
	A8141MIPI_write_cmos_sensor( 0x306E, 0xFC80);
	A8141MIPI_write_cmos_sensor( 0x30B2, 0xC000);
	A8141MIPI_write_cmos_sensor( 0x30D6, 0x0800);
	A8141MIPI_write_cmos_sensor( 0x316C, 0xB42F);
	A8141MIPI_write_cmos_sensor( 0x316E, 0x869A);
	A8141MIPI_write_cmos_sensor( 0x3170, 0x210E);
	A8141MIPI_write_cmos_sensor( 0x317A, 0x010E);
	//A8141MIPI_write_cmos_sensor( 0x31E0, 0x1FB9);
	A8141MIPI_write_cmos_sensor( 0x31E0, 0x07F9);
	A8141MIPI_write_cmos_sensor( 0x31E6, 0x07FC);
	A8141MIPI_write_cmos_sensor( 0x37C0, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C2, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C4, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C6, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E00, 0x0011);
	A8141MIPI_write_cmos_sensor( 0x3E02, 0x8801);
	A8141MIPI_write_cmos_sensor( 0x3E04, 0x2801); 
	A8141MIPI_write_cmos_sensor( 0x3E06, 0x8449);
	A8141MIPI_write_cmos_sensor( 0x3E08, 0x6841);
	A8141MIPI_write_cmos_sensor( 0x3E0A, 0x400C);
	A8141MIPI_write_cmos_sensor( 0x3E0C, 0x1001);
	A8141MIPI_write_cmos_sensor( 0x3E0E, 0x2603); 
	A8141MIPI_write_cmos_sensor( 0x3E10, 0x4B41);
	A8141MIPI_write_cmos_sensor( 0x3E12, 0x4B24); 
	A8141MIPI_write_cmos_sensor( 0x3E14, 0xA3CF);
	A8141MIPI_write_cmos_sensor( 0x3E16, 0x8802);
	A8141MIPI_write_cmos_sensor( 0x3E18, 0x84FF);
	A8141MIPI_write_cmos_sensor( 0x3E1A, 0x8601);
	A8141MIPI_write_cmos_sensor( 0x3E1C, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E1E, 0x840A);
	A8141MIPI_write_cmos_sensor( 0x3E20, 0xFF00);
	A8141MIPI_write_cmos_sensor( 0x3E22, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E24, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E26, 0x0088);
	A8141MIPI_write_cmos_sensor( 0x3E28, 0x2E8A);
	A8141MIPI_write_cmos_sensor( 0x3E30, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E32, 0x8801);
	A8141MIPI_write_cmos_sensor( 0x3E34, 0x4029); 
	A8141MIPI_write_cmos_sensor( 0x3E36, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E38, 0x8469);
	A8141MIPI_write_cmos_sensor( 0x3E3A, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E3C, 0x2801);
	A8141MIPI_write_cmos_sensor( 0x3E3E, 0x3E2A); 
	A8141MIPI_write_cmos_sensor( 0x3E40, 0x1C01);
	A8141MIPI_write_cmos_sensor( 0x3E42, 0xFF84); 
	A8141MIPI_write_cmos_sensor( 0x3E44, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E46, 0x0C01); 
	A8141MIPI_write_cmos_sensor( 0x3E48, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E4A, 0x00FF); 
	A8141MIPI_write_cmos_sensor( 0x3E4C, 0x8402);
	A8141MIPI_write_cmos_sensor( 0x3E4E, 0x8984); //0x00FF
	A8141MIPI_write_cmos_sensor( 0x3E50, 0x6628); 
	A8141MIPI_write_cmos_sensor( 0x3E52, 0x8340);
	A8141MIPI_write_cmos_sensor( 0x3E54, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E56, 0x4A42);
	A8141MIPI_write_cmos_sensor( 0x3E58, 0x2703); 
	A8141MIPI_write_cmos_sensor( 0x3E5A, 0x6752);
	A8141MIPI_write_cmos_sensor( 0x3E5C, 0x3F2A); 
	A8141MIPI_write_cmos_sensor( 0x3E5E, 0x846A);
	A8141MIPI_write_cmos_sensor( 0x3E60, 0x4C01);
	A8141MIPI_write_cmos_sensor( 0x3E62, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E66, 0x3901);
	A8141MIPI_write_cmos_sensor( 0x3E90, 0x2C01);	  
	A8141MIPI_write_cmos_sensor( 0x3E98, 0x2B02);	  
	A8141MIPI_write_cmos_sensor( 0x3E92, 0x2A04);	  
	A8141MIPI_write_cmos_sensor( 0x3E94, 0x2509);	   
	A8141MIPI_write_cmos_sensor( 0x3E96, 0x0000);	   
	A8141MIPI_write_cmos_sensor( 0x3E9A, 0x2905);	  
	A8141MIPI_write_cmos_sensor( 0x3E9C, 0x00FF);	  
	A8141MIPI_write_cmos_sensor( 0x3ECC, 0x00EB);
	A8141MIPI_write_cmos_sensor( 0x3ED0, 0x1E24);
	A8141MIPI_write_cmos_sensor( 0x3ED4, 0xAFC4);
	A8141MIPI_write_cmos_sensor( 0x3ED6, 0x909B);
	A8141MIPI_write_cmos_sensor( 0x3EE0, 0x2424);
	//A8141MIPI_write_cmos_sensor( 0x3EE2, 0x9797);  //mask this line by FAE's request, it will affect the current of AF VCM
	A8141MIPI_write_cmos_sensor( 0x3EE4, 0xC100);
	A8141MIPI_write_cmos_sensor( 0x3EE6, 0x0540);
	A8141MIPI_write_cmos_sensor( 0x3174, 0x8000);

	A8141MIPI_write_cmos_sensor( 0x3F02, 0x0030);
	A8141MIPI_write_cmos_sensor( 0x3F04, 0x0120);
	A8141MIPI_write_cmos_sensor( 0x3F06, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3F08, 0x0140);
	/*A8141MIPI_write_cmos_sensor( 0x3070, 0x0001);	  //test pattern
	A8141MIPI_write_cmos_sensor( 0x3072, 0x0200);
	A8141MIPI_write_cmos_sensor( 0x3074, 0x0200);
	A8141MIPI_write_cmos_sensor( 0x3076, 0x0200);
	A8141MIPI_write_cmos_sensor( 0x3078, 0x0200);*/


#else

	//[Recommended Settings]
	A8141MIPI_write_cmos_sensor( 0x3044, 0x0590);
	A8141MIPI_write_cmos_sensor( 0x306E, 0xFC80);
	A8141MIPI_write_cmos_sensor( 0x30B2, 0xC000);
	A8141MIPI_write_cmos_sensor( 0x30D6, 0x0800);
	A8141MIPI_write_cmos_sensor( 0x316C, 0xB42A); //0xB42F
	A8141MIPI_write_cmos_sensor( 0x316E, 0x869C);
	A8141MIPI_write_cmos_sensor( 0x3170, 0x210E);
	A8141MIPI_write_cmos_sensor( 0x317A, 0x010E);
	A8141MIPI_write_cmos_sensor( 0x31E0, 0x1FB9);
	A8141MIPI_write_cmos_sensor( 0x31E6, 0x07FC);
	A8141MIPI_write_cmos_sensor( 0x37C0, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C2, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C4, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C6, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E00, 0x0011);
	A8141MIPI_write_cmos_sensor( 0x3E02, 0x8801);
	A8141MIPI_write_cmos_sensor( 0x3E04, 0x2801); 
	A8141MIPI_write_cmos_sensor( 0x3E06, 0x8449);
	A8141MIPI_write_cmos_sensor( 0x3E08, 0x6841);
	A8141MIPI_write_cmos_sensor( 0x3E0A, 0x400C);
	A8141MIPI_write_cmos_sensor( 0x3E0C, 0x1001);
	A8141MIPI_write_cmos_sensor( 0x3E0E, 0x2603); 
	A8141MIPI_write_cmos_sensor( 0x3E10, 0x4B41);
	A8141MIPI_write_cmos_sensor( 0x3E12, 0x4B24); 
	A8141MIPI_write_cmos_sensor( 0x3E14, 0xA3CF); //0xA3CF
	A8141MIPI_write_cmos_sensor( 0x3E16, 0x8802);
	A8141MIPI_write_cmos_sensor( 0x3E18, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E1A, 0x8601);
	A8141MIPI_write_cmos_sensor( 0x3E1C, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E1E, 0x840A);
	A8141MIPI_write_cmos_sensor( 0x3E20, 0xFF00);
	A8141MIPI_write_cmos_sensor( 0x3E22, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E24, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E26, 0x0088);
	A8141MIPI_write_cmos_sensor( 0x3E28, 0x2E8A);
	A8141MIPI_write_cmos_sensor( 0x3E30, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E32, 0x00FF); //0x8801
	A8141MIPI_write_cmos_sensor( 0x3E34, 0x4029); 
	A8141MIPI_write_cmos_sensor( 0x3E36, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E38, 0x8469);
	A8141MIPI_write_cmos_sensor( 0x3E3A, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E3C, 0x2801);
	A8141MIPI_write_cmos_sensor( 0x3E3E, 0x3E2A); 
	A8141MIPI_write_cmos_sensor( 0x3E40, 0x1C01);
	A8141MIPI_write_cmos_sensor( 0x3E42, 0xFF84); 
	A8141MIPI_write_cmos_sensor( 0x3E44, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E46, 0x0C01); 
	A8141MIPI_write_cmos_sensor( 0x3E48, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E4A, 0x00FF); 
	A8141MIPI_write_cmos_sensor( 0x3E4C, 0x8402);
	A8141MIPI_write_cmos_sensor( 0x3E4E, 0x8984); //0x00FF
	A8141MIPI_write_cmos_sensor( 0x3E50, 0x6628); 
	A8141MIPI_write_cmos_sensor( 0x3E52, 0x8340);
	A8141MIPI_write_cmos_sensor( 0x3E54, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E56, 0x4A42);
	A8141MIPI_write_cmos_sensor( 0x3E58, 0x2703); 
	A8141MIPI_write_cmos_sensor( 0x3E5A, 0x6752);
	A8141MIPI_write_cmos_sensor( 0x3E5C, 0x3F2A); 
	A8141MIPI_write_cmos_sensor( 0x3E5E, 0x846A);
	A8141MIPI_write_cmos_sensor( 0x3E60, 0x4C01);
	A8141MIPI_write_cmos_sensor( 0x3E62, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E66, 0x3901);
	A8141MIPI_write_cmos_sensor( 0x3E90, 0x2C01);	
	A8141MIPI_write_cmos_sensor( 0x3E98, 0x2B02);	
	A8141MIPI_write_cmos_sensor( 0x3E92, 0x2A04);	
	A8141MIPI_write_cmos_sensor( 0x3E94, 0x2509);	
	A8141MIPI_write_cmos_sensor( 0x3E96, 0xF000); //0x0000	
	A8141MIPI_write_cmos_sensor( 0x3E9A, 0x2905);	 
	A8141MIPI_write_cmos_sensor( 0x3E9C, 0x00FF);	
	A8141MIPI_write_cmos_sensor( 0x3ECC, 0x00EB);
	A8141MIPI_write_cmos_sensor( 0x3ED0, 0x1E24);
	A8141MIPI_write_cmos_sensor( 0x3ED4, 0xFAA4); //0xAFC4
	A8141MIPI_write_cmos_sensor( 0x3ED6, 0x909B);
	A8141MIPI_write_cmos_sensor( 0x3EE0, 0x2424);
	//A8141MIPI_write_cmos_sensor( 0x3EE2, 0x9797);  //mask this line by FAE's request, it will affect the current of AF VCM
	A8141MIPI_write_cmos_sensor( 0x3EE4, 0xC100);
	A8141MIPI_write_cmos_sensor( 0x3EE6, 0x0540);
	A8141MIPI_write_cmos_sensor( 0x3174, 0x8000);
	//STATE= Minimum Gain, 1500	// gain * 1000
#endif
	//RAW10
	A8141MIPI_write_cmos_sensor( 0x0112, 0x0A0A); 	// CCP_DATA_FORMAT

	//PLL Configuration (Ext=26MHz, vt_pix_clk=169MHz, op_pix_clk=67.6MHz)
	A8141MIPI_write_cmos_sensor( 0x0300, 0x0004); 	// VT_PIX_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x0302, 0x0001); 	// VT_SYS_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x0304, 0x0002); 	// PRE_PLL_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x0306, 0x0034); 	// PLL_MULTIPLIER
	A8141MIPI_write_cmos_sensor( 0x0308, 0x000A); 	// OP_PIX_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x030A, 0x0001); 	// OP_SYS_CLK_DIV
	mdelay(1);

	//Output size (Pixel address must start with EVEN and end with ODD!)
	A8141MIPI_write_cmos_sensor( 0x0344, 0x0028); 	// X_ADDR_START
	A8141MIPI_write_cmos_sensor( 0x0348, 0x0CA9); 	// X_ADDR_END
	A8141MIPI_write_cmos_sensor( 0x0346, 0x0020); 	// Y_ADDR_START
	A8141MIPI_write_cmos_sensor( 0x034A, 0x0981); 	// Y_ADDR_END
	A8141MIPI_write_cmos_sensor( 0x034C, 0x0640); 	// X_OUTPUT_SIZE
	A8141MIPI_write_cmos_sensor( 0x034E, 0x04B0); 	// Y_OUTPUT_SIZE
	A8141MIPI_write_cmos_sensor( 0x3040, 0x00C1); 	// READ_MODE
	A8141MIPI_write_cmos_sensor( 0x3040, 0x00C3); 	// READ_MODE

	//"X-Bin2 Y-Bin2" and "X-Bin2Skip2 Y-Bin2Skip2" Optimization
	A8141MIPI_write_cmos_sensor( 0x306E, 0xFCB0); 	// DATAPATH_SELECT
	A8141MIPI_write_cmos_sensor( 0x3040, 0x10C3); 	// READ_MODE

	//Binning Configuration
	A8141MIPI_write_cmos_sensor( 0x3040, 0x10C3); 	// READ_MODE
	A8141MIPI_write_cmos_sensor( 0x3040, 0x14C3); 	// READ_MODE

	//"X-Bin2 Y-Skip2", "X-Bin2Skip2 Y-Bin2Skip2", "X-Bin2Skip2 Y-Skip4" Optimization
	A8141MIPI_write_cmos_sensor( 0x3040, 0x14C3); 	// READ_MODE
	A8141MIPI_write_cmos_sensor( 0x3178, 0x0000); 	// ANALOG_CONTROL5
	A8141MIPI_write_cmos_sensor( 0x3ED0, 0x1E24); 	// DAC_LD_4_5

	//Scale Configuration
	A8141MIPI_write_cmos_sensor( 0x0400, 0x0002); 	// SCALING_MODE
	A8141MIPI_write_cmos_sensor( 0x0404, 0x0010); 	// SCALE_M


	//Timing Configuration
	A8141MIPI_write_cmos_sensor( 0x0342, 0x1066); 	// LINE_LENGTH_PCK
	A8141MIPI_write_cmos_sensor( 0x0340, 0x0543); 	// FRAME_LENGTH_LINES
	A8141MIPI_write_cmos_sensor( 0x0202, 0x053F); 	// COARSE_INTEGRATION_TIME
	A8141MIPI_write_cmos_sensor( 0x3014, 0x0846); 	// FINE_INTEGRATION_TIME_
	A8141MIPI_write_cmos_sensor( 0x3010, 0x0130); 	// FINE_CORRECTION

	A8141MIPI_write_cmos_sensor( 0x30F0, 0x8000); 	// VCM_CONTROL 0x0000:disable 0x8000:enable

	//Start Streaming
	A8141MIPI_write_cmos_sensor( 0x301A, 0x8250); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x8650); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x8658); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor8( 0x0104, 0x00 	);// GROUPED_PARAMETER_HOLD
	A8141MIPI_write_cmos_sensor( 0x301A, 0x065C); 	// RESET_REGISTER
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_g_RES=A8141MIPI_2M;
	A8141MIPI_Auto_Flicker_mode=KAL_FALSE;
	spin_unlock(&a8141mipi_drv_lock);

#else
    A8141MIPI_write_cmos_sensor(0x0307, 0x1B);//INCK 24MHz
    A8141MIPI_write_cmos_sensor(0x302B, 0x4B);//PLL stablilized time 

    A8141MIPI_write_cmos_sensor(0x30E5, 0x04);//MIPI enable setting
    A8141MIPI_write_cmos_sensor(0x3300, 0x00);

    // Global setting
    A8141MIPI_write_cmos_sensor(0x0101, 0x03);
    A8141MIPI_write_cmos_sensor(0x0340, 0x02);
    A8141MIPI_write_cmos_sensor(0x0341, 0x75);
    A8141MIPI_write_cmos_sensor(0x0202, 0x02);    // exposure time
    A8141MIPI_write_cmos_sensor(0x0203, 0x68);
    A8141MIPI_write_cmos_sensor(0x0205, 224);
	
    A8141MIPI_write_cmos_sensor(0x300A, 0x80);
    A8141MIPI_write_cmos_sensor(0x3014, 0x08);
    A8141MIPI_write_cmos_sensor(0x3015, 0x37);
    A8141MIPI_write_cmos_sensor(0x3017, 0x60);
    A8141MIPI_write_cmos_sensor(0x301C, 0x01);
    A8141MIPI_write_cmos_sensor(0x3031, 0x28);
    A8141MIPI_write_cmos_sensor(0x3040, 0x00);
    A8141MIPI_write_cmos_sensor(0x3041, 0x60);
    A8141MIPI_write_cmos_sensor(0x3047, 0x10);
    A8141MIPI_write_cmos_sensor(0x3051, 0x24);
    A8141MIPI_write_cmos_sensor(0x3053, 0x34);
    A8141MIPI_write_cmos_sensor(0x3055, 0x3B);
    A8141MIPI_write_cmos_sensor(0x3057, 0xC0);
    A8141MIPI_write_cmos_sensor(0x3060, 0x30);
    A8141MIPI_write_cmos_sensor(0x3065, 0x00);
    A8141MIPI_write_cmos_sensor(0x30A1, 0x03);
    A8141MIPI_write_cmos_sensor(0x30A3, 0x01);
    A8141MIPI_write_cmos_sensor(0x30AA, 0x88);
    A8141MIPI_write_cmos_sensor(0x30AB, 0x1C);
    A8141MIPI_write_cmos_sensor(0x30B0, 0x32);
    A8141MIPI_write_cmos_sensor(0x30B2, 0x83);
    A8141MIPI_write_cmos_sensor(0x30D3, 0x04);
    A8141MIPI_write_cmos_sensor(0x310C, 0xE9);
    A8141MIPI_write_cmos_sensor(0x310D, 0x00);
    A8141MIPI_write_cmos_sensor(0x316B, 0x14);
    A8141MIPI_write_cmos_sensor(0x316D, 0x3B);
    A8141MIPI_write_cmos_sensor(0x31A4, 0xD8);
    A8141MIPI_write_cmos_sensor(0x31A6, 0x17);
    A8141MIPI_write_cmos_sensor(0x31AC, 0xCF);
    A8141MIPI_write_cmos_sensor(0x31AE, 0xF1);
    A8141MIPI_write_cmos_sensor(0x31B4, 0xD8);
    A8141MIPI_write_cmos_sensor(0x31B6, 0x17);
    A8141MIPI_write_cmos_sensor(0x3302, 0x0A);
    A8141MIPI_write_cmos_sensor(0x3303, 0x09);
    A8141MIPI_write_cmos_sensor(0x3304, 0x05);
    A8141MIPI_write_cmos_sensor(0x3305, 0x04);
    A8141MIPI_write_cmos_sensor(0x3306, 0x15);
    A8141MIPI_write_cmos_sensor(0x3307, 0x03);
    A8141MIPI_write_cmos_sensor(0x3308, 0x13);
    A8141MIPI_write_cmos_sensor(0x3309, 0x05);
    A8141MIPI_write_cmos_sensor(0x330A, 0x0B);
    A8141MIPI_write_cmos_sensor(0x330B, 0x04);
    A8141MIPI_write_cmos_sensor(0x330C, 0x0B);
    A8141MIPI_write_cmos_sensor(0x330D, 0x06);
    // The register only need to enable 1 time.
    A8141MIPI_write_cmos_sensor(0x0100, 0x01);   // Streaming 
    
    A8141MIPI_Auto_Flicker_mode = KAL_FALSE;     // reset the flicker status
    
#endif
    #ifdef A8141MIPI_DEBUG
    SENSORDB("[A8141MIPIRaw] Init Success \n");
	#endif
}   /*  A8141MIPI_Sensor_Init  */
void A8141MIPI_set_2M(void)
{	
	if(A8141MIPI_g_RES != A8141MIPI_2M)
		{
		#ifdef A8141MIPI_DEBUG
		SENSORDB("A8141MIPI_set_2M\n"); 
		#endif
	#if 0
	A8141MIPI_write_cmos_sensor( 0x0103, 0x01); 	// SOFTWARE_RESET
	mdelay(300);
		//Stop Streaming
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0058);	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0050);	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor8( 0x0104, 0x01); 	// GROUPED_PARAMETER_HOLD

	//2-lane MIPI Interface Configuration
	A8141MIPI_write_cmos_sensor( 0x3064, 0x7800); 	// SMIA_TEST
	A8141MIPI_write_cmos_sensor( 0x31AE, 0x0202); 	// SERIAL_FORMAT
	A8141MIPI_write_cmos_sensor( 0x31B8, 0x0E3F); 	// MIPI_TIMING_2
	mdelay(5);

	//[Recommended Settings]
	A8141MIPI_write_cmos_sensor( 0x3044, 0x0590);
	A8141MIPI_write_cmos_sensor( 0x306E, 0xFC80);
	A8141MIPI_write_cmos_sensor( 0x30B2, 0xC000);
	A8141MIPI_write_cmos_sensor( 0x30D6, 0x0800);
	A8141MIPI_write_cmos_sensor( 0x316C, 0xB42A); //0xB42F
	A8141MIPI_write_cmos_sensor( 0x316E, 0x869C);
	A8141MIPI_write_cmos_sensor( 0x3170, 0x210E);
	A8141MIPI_write_cmos_sensor( 0x317A, 0x010E);
	A8141MIPI_write_cmos_sensor( 0x31E0, 0x1FB9);
	A8141MIPI_write_cmos_sensor( 0x31E6, 0x07FC);
	A8141MIPI_write_cmos_sensor( 0x37C0, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C2, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C4, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C6, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E00, 0x0011);
	A8141MIPI_write_cmos_sensor( 0x3E02, 0x8801);
	A8141MIPI_write_cmos_sensor( 0x3E04, 0x2801); 
	A8141MIPI_write_cmos_sensor( 0x3E06, 0x8449);
	A8141MIPI_write_cmos_sensor( 0x3E08, 0x6841);
	A8141MIPI_write_cmos_sensor( 0x3E0A, 0x400C);
	A8141MIPI_write_cmos_sensor( 0x3E0C, 0x1001);
	A8141MIPI_write_cmos_sensor( 0x3E0E, 0x2603); 
	A8141MIPI_write_cmos_sensor( 0x3E10, 0x4B41);
	A8141MIPI_write_cmos_sensor( 0x3E12, 0x4B24); 
	A8141MIPI_write_cmos_sensor( 0x3E14, 0xA3CF); //0xA3CF
	A8141MIPI_write_cmos_sensor( 0x3E16, 0x8802);
	A8141MIPI_write_cmos_sensor( 0x3E18, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E1A, 0x8601);
	A8141MIPI_write_cmos_sensor( 0x3E1C, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E1E, 0x840A);
	A8141MIPI_write_cmos_sensor( 0x3E20, 0xFF00);
	A8141MIPI_write_cmos_sensor( 0x3E22, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E24, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E26, 0x0088);
	A8141MIPI_write_cmos_sensor( 0x3E28, 0x2E8A);
	A8141MIPI_write_cmos_sensor( 0x3E30, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E32, 0x00FF); //0x8801
	A8141MIPI_write_cmos_sensor( 0x3E34, 0x4029); 
	A8141MIPI_write_cmos_sensor( 0x3E36, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E38, 0x8469);
	A8141MIPI_write_cmos_sensor( 0x3E3A, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E3C, 0x2801);
	A8141MIPI_write_cmos_sensor( 0x3E3E, 0x3E2A); 
	A8141MIPI_write_cmos_sensor( 0x3E40, 0x1C01);
	A8141MIPI_write_cmos_sensor( 0x3E42, 0xFF84); 
	A8141MIPI_write_cmos_sensor( 0x3E44, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E46, 0x0C01); 
	A8141MIPI_write_cmos_sensor( 0x3E48, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E4A, 0x00FF); 
	A8141MIPI_write_cmos_sensor( 0x3E4C, 0x8402);
	A8141MIPI_write_cmos_sensor( 0x3E4E, 0x8984); //0x00FF
	A8141MIPI_write_cmos_sensor( 0x3E50, 0x6628); 
	A8141MIPI_write_cmos_sensor( 0x3E52, 0x8340);
	A8141MIPI_write_cmos_sensor( 0x3E54, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E56, 0x4A42);
	A8141MIPI_write_cmos_sensor( 0x3E58, 0x2703); 
	A8141MIPI_write_cmos_sensor( 0x3E5A, 0x6752);
	A8141MIPI_write_cmos_sensor( 0x3E5C, 0x3F2A); 
	A8141MIPI_write_cmos_sensor( 0x3E5E, 0x846A);
	A8141MIPI_write_cmos_sensor( 0x3E60, 0x4C01);
	A8141MIPI_write_cmos_sensor( 0x3E62, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E66, 0x3901);
	A8141MIPI_write_cmos_sensor( 0x3E90, 0x2C01);	
	A8141MIPI_write_cmos_sensor( 0x3E98, 0x2B02);	
	A8141MIPI_write_cmos_sensor( 0x3E92, 0x2A04);	
	A8141MIPI_write_cmos_sensor( 0x3E94, 0x2509);	
	A8141MIPI_write_cmos_sensor( 0x3E96, 0xF000); //0x0000	
	A8141MIPI_write_cmos_sensor( 0x3E9A, 0x2905);	 
	A8141MIPI_write_cmos_sensor( 0x3E9C, 0x00FF);	
	A8141MIPI_write_cmos_sensor( 0x3ECC, 0x00EB);
	A8141MIPI_write_cmos_sensor( 0x3ED0, 0x1E24);
	A8141MIPI_write_cmos_sensor( 0x3ED4, 0xFAA4); //0xAFC4
	A8141MIPI_write_cmos_sensor( 0x3ED6, 0x909B);
	A8141MIPI_write_cmos_sensor( 0x3EE0, 0x2424);
	//A8141MIPI_write_cmos_sensor( 0x3EE2, 0x9797);
	A8141MIPI_write_cmos_sensor( 0x3EE4, 0xC100);
	A8141MIPI_write_cmos_sensor( 0x3EE6, 0x0540);
	A8141MIPI_write_cmos_sensor( 0x3174, 0x8000);
	//STATE= Minimum Gain, 1500	// gain * 1000
#endif
  	//Stop Streaming
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0058); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0050); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor8( 0x0104, 0x01);// GROUPED_PARAMETER_HOLD
	//RAW10
	A8141MIPI_write_cmos_sensor( 0x0112, 0x0A0A); 	// CCP_DATA_FORMAT

	//PLL Configuration (Ext=26MHz, vt_pix_clk=169MHz, op_pix_clk=67.6MHz)
	A8141MIPI_write_cmos_sensor( 0x0300, 0x0004); 	// VT_PIX_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x0302, 0x0001); 	// VT_SYS_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x0304, 0x0002); 	// PRE_PLL_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x0306, 0x0034); 	// PLL_MULTIPLIER
	A8141MIPI_write_cmos_sensor( 0x0308, 0x000A); 	// OP_PIX_CLK_DIV
	A8141MIPI_write_cmos_sensor( 0x030A, 0x0001); 	// OP_SYS_CLK_DIV
	mdelay(1);

	//Output size (Pixel address must start with EVEN and end with ODD!)
	A8141MIPI_write_cmos_sensor( 0x0344, 0x0028); 	// X_ADDR_START
	A8141MIPI_write_cmos_sensor( 0x0348, 0x0CA9); 	// X_ADDR_END
	A8141MIPI_write_cmos_sensor( 0x0346, 0x0020); 	// Y_ADDR_START
	A8141MIPI_write_cmos_sensor( 0x034A, 0x0981); 	// Y_ADDR_END
	A8141MIPI_write_cmos_sensor( 0x034C, 0x0640); 	// X_OUTPUT_SIZE
	A8141MIPI_write_cmos_sensor( 0x034E, 0x04B0); 	// Y_OUTPUT_SIZE
	A8141MIPI_write_cmos_sensor( 0x3040, 0x00C1); 	// READ_MODE
	A8141MIPI_write_cmos_sensor( 0x3040, 0x00C3); 	// READ_MODE

	//"X-Bin2 Y-Bin2" and "X-Bin2Skip2 Y-Bin2Skip2" Optimization
	A8141MIPI_write_cmos_sensor( 0x306E, 0xFCB0); 	// DATAPATH_SELECT
	A8141MIPI_write_cmos_sensor( 0x3040, 0x10C3); 	// READ_MODE

	//Binning Configuration
	A8141MIPI_write_cmos_sensor( 0x3040, 0x10C3); 	// READ_MODE
	A8141MIPI_write_cmos_sensor( 0x3040, 0x14C3); 	// READ_MODE

	//"X-Bin2 Y-Skip2", "X-Bin2Skip2 Y-Bin2Skip2", "X-Bin2Skip2 Y-Skip4" Optimization
	A8141MIPI_write_cmos_sensor( 0x3040, 0x14C3); 	// READ_MODE
	A8141MIPI_write_cmos_sensor( 0x3178, 0x0000); 	// ANALOG_CONTROL5
	A8141MIPI_write_cmos_sensor( 0x3ED0, 0x1E24); 	// DAC_LD_4_5

	//Scale Configuration
	A8141MIPI_write_cmos_sensor( 0x0400, 0x0002); 	// SCALING_MODE
	A8141MIPI_write_cmos_sensor( 0x0404, 0x0010); 	// SCALE_M


	//Timing Configuration
	A8141MIPI_write_cmos_sensor( 0x0342, 0x1066); 	// LINE_LENGTH_PCK
	A8141MIPI_write_cmos_sensor( 0x0340, 0x0543); 	// FRAME_LENGTH_LINES
	A8141MIPI_write_cmos_sensor( 0x0202, 0x053F); 	// COARSE_INTEGRATION_TIME
	A8141MIPI_write_cmos_sensor( 0x3014, 0x0846); 	// FINE_INTEGRATION_TIME_
	A8141MIPI_write_cmos_sensor( 0x3010, 0x0130); 	// FINE_CORRECTION

	//Start Streaming
	A8141MIPI_write_cmos_sensor( 0x301A, 0x8250); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x8650); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x8658); 	// RESET_REGISTER
	A8141MIPI_write_cmos_sensor8( 0x0104, 0x00 	);// GROUPED_PARAMETER_HOLD
	A8141MIPI_write_cmos_sensor( 0x301A, 0x065C); 	// RESET_REGISTER
		}
}

void A8141MIPI_set_8M(void)
{	

	if(A8141MIPI_g_RES==A8141MIPI_8M) return;
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_g_RES=A8141MIPI_8M;
	spin_unlock(&a8141mipi_drv_lock);

		#ifdef A8141MIPI_DEBUG
		SENSORDB("A8141MIPI_set_8M\n"); 
		#endif

	//MIPI 2-lane FPGA
	//SERIAL_REG = 0xCA, 0x00, 0x8016, 8:16   // FPGA disabled
	//SERIAL_REG = 0xCA, 0x00, 0x0016, 8:16   // FPGA into MIPI dual lane mode

	//DEMO2X REV3/REV4.1 version require Hardware reset
	//STATE= Sensor Reset, 1
	//STATE= Sensor Reset, 0
	//DELAY=10
#if 0
	A8141MIPI_write_cmos_sensor( 0x0103, 0x01);          // SOFTWARE_RESET
	mdelay(300);

	//Stop Streaming
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0058);     // RESET_REGISTER
	A8141MIPI_write_cmos_sensor( 0x301A, 0x0050);     // RESET_REGISTER
	A8141MIPI_write_cmos_sensor8( 0x0104, 0x01  );        // GROUPED_PARAMETER_HOLD

	//2-lane MIPI Interface Configuration
	A8141MIPI_write_cmos_sensor( 0x3064, 0x7800);      // SMIA_TEST
	A8141MIPI_write_cmos_sensor( 0x31AE, 0x0202);     // SERIAL_FORMAT
	A8141MIPI_write_cmos_sensor( 0x31B8, 0x0E3F);     // MIPI_TIMING_2
	mdelay(5);

	//[Recommended Settings]
	A8141MIPI_write_cmos_sensor( 0x3044, 0x0590);
	A8141MIPI_write_cmos_sensor( 0x306E, 0xFC80);
	A8141MIPI_write_cmos_sensor( 0x30B2, 0xC000);
	A8141MIPI_write_cmos_sensor( 0x30D6, 0x0800);
	A8141MIPI_write_cmos_sensor( 0x316C, 0xB42A); //0xB42F
	A8141MIPI_write_cmos_sensor( 0x316E, 0x869C);
	A8141MIPI_write_cmos_sensor( 0x3170, 0x210E);
	A8141MIPI_write_cmos_sensor( 0x317A, 0x010E);
	A8141MIPI_write_cmos_sensor( 0x31E0, 0x1FB9);
	A8141MIPI_write_cmos_sensor( 0x31E6, 0x07FC);
	A8141MIPI_write_cmos_sensor( 0x37C0, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C2, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C4, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x37C6, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E00, 0x0011);
	A8141MIPI_write_cmos_sensor( 0x3E02, 0x8801);
	A8141MIPI_write_cmos_sensor( 0x3E04, 0x2801); 
	A8141MIPI_write_cmos_sensor( 0x3E06, 0x8449);
	A8141MIPI_write_cmos_sensor( 0x3E08, 0x6841);
	A8141MIPI_write_cmos_sensor( 0x3E0A, 0x400C);
	A8141MIPI_write_cmos_sensor( 0x3E0C, 0x1001);
	A8141MIPI_write_cmos_sensor( 0x3E0E, 0x2603); 
	A8141MIPI_write_cmos_sensor( 0x3E10, 0x4B41);
	A8141MIPI_write_cmos_sensor( 0x3E12, 0x4B24); 
	A8141MIPI_write_cmos_sensor( 0x3E14, 0xA3CF); //0xA3CF
	A8141MIPI_write_cmos_sensor( 0x3E16, 0x8802);
	A8141MIPI_write_cmos_sensor( 0x3E18, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E1A, 0x8601);
	A8141MIPI_write_cmos_sensor( 0x3E1C, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E1E, 0x840A);
	A8141MIPI_write_cmos_sensor( 0x3E20, 0xFF00);
	A8141MIPI_write_cmos_sensor( 0x3E22, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E24, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E26, 0x0088);
	A8141MIPI_write_cmos_sensor( 0x3E28, 0x2E8A);
	A8141MIPI_write_cmos_sensor( 0x3E30, 0x0000);
	A8141MIPI_write_cmos_sensor( 0x3E32, 0x00FF); //0x8801
	A8141MIPI_write_cmos_sensor( 0x3E34, 0x4029); 
	A8141MIPI_write_cmos_sensor( 0x3E36, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E38, 0x8469);
	A8141MIPI_write_cmos_sensor( 0x3E3A, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E3C, 0x2801);
	A8141MIPI_write_cmos_sensor( 0x3E3E, 0x3E2A); 
	A8141MIPI_write_cmos_sensor( 0x3E40, 0x1C01);
	A8141MIPI_write_cmos_sensor( 0x3E42, 0xFF84); 
	A8141MIPI_write_cmos_sensor( 0x3E44, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E46, 0x0C01); 
	A8141MIPI_write_cmos_sensor( 0x3E48, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E4A, 0x00FF); 
	A8141MIPI_write_cmos_sensor( 0x3E4C, 0x8402);
	A8141MIPI_write_cmos_sensor( 0x3E4E, 0x8984); //0x00FF
	A8141MIPI_write_cmos_sensor( 0x3E50, 0x6628); 
	A8141MIPI_write_cmos_sensor( 0x3E52, 0x8340);
	A8141MIPI_write_cmos_sensor( 0x3E54, 0x00FF);
	A8141MIPI_write_cmos_sensor( 0x3E56, 0x4A42);
	A8141MIPI_write_cmos_sensor( 0x3E58, 0x2703); 
	A8141MIPI_write_cmos_sensor( 0x3E5A, 0x6752);
	A8141MIPI_write_cmos_sensor( 0x3E5C, 0x3F2A); 
	A8141MIPI_write_cmos_sensor( 0x3E5E, 0x846A);
	A8141MIPI_write_cmos_sensor( 0x3E60, 0x4C01);
	A8141MIPI_write_cmos_sensor( 0x3E62, 0x8401);
	A8141MIPI_write_cmos_sensor( 0x3E66, 0x3901);
	A8141MIPI_write_cmos_sensor( 0x3E90, 0x2C01);     
	A8141MIPI_write_cmos_sensor( 0x3E98, 0x2B02);     
	A8141MIPI_write_cmos_sensor( 0x3E92, 0x2A04);     
	A8141MIPI_write_cmos_sensor( 0x3E94, 0x2509);      
	A8141MIPI_write_cmos_sensor( 0x3E96, 0xF000); //0x0000   
	A8141MIPI_write_cmos_sensor( 0x3E9A, 0x2905);     
	A8141MIPI_write_cmos_sensor( 0x3E9C, 0x00FF);     
	A8141MIPI_write_cmos_sensor( 0x3ECC, 0x00EB);
	A8141MIPI_write_cmos_sensor( 0x3ED0, 0x1E24);
	A8141MIPI_write_cmos_sensor( 0x3ED4, 0xFAA4); //0xAFC4
	A8141MIPI_write_cmos_sensor( 0x3ED6, 0x909B);
	A8141MIPI_write_cmos_sensor( 0x3EE0, 0x2424);
	//A8141MIPI_write_cmos_sensor( 0x3EE2, 0x9797);
	A8141MIPI_write_cmos_sensor( 0x3EE4, 0xC100);
	A8141MIPI_write_cmos_sensor( 0x3EE6, 0x0540);
	A8141MIPI_write_cmos_sensor( 0x3174, 0x8000);
	//STATE= Minimum Gain, 1500      // gain * 1000
#endif
  A8141MIPI_write_cmos_sensor8( 0x0104, 0x01);// GROUPED_PARAMETER_HOLD
	//RAW10
	A8141MIPI_write_cmos_sensor( 0x0112, 0x0A0A);   // CCP_DATA_FORMAT

	//PLL Configuration (Ext=26MHz, vt_pix_clk=162.5MHz, op_pix_clk=65MHz)
	A8141MIPI_write_cmos_sensor( 0x0300, 0x4 );//VT_PIX_CLK_DIV=4
	A8141MIPI_write_cmos_sensor( 0x0302, 0x1 );//VT_SYS_CLK_DIV=1
	A8141MIPI_write_cmos_sensor( 0x0304, 0x2 );//PRE_PLL_CLK_DIV=2 //Note: 26MHz/2=13MHz
	A8141MIPI_write_cmos_sensor( 0x0306, 0x32); //PLL_MULTIPLIER=50 //Note: Running at 650MHz
	A8141MIPI_write_cmos_sensor( 0x0308, 0xA );//OP_PIX_CLK_DIV=10
	A8141MIPI_write_cmos_sensor( 0x030A, 0x1 );//OP_SYS_CLK_DIV=1
	mdelay(1);

	//Output size (Pixel address must start with EVEN and end with ODD!)
	A8141MIPI_write_cmos_sensor(0x0344, 0xC  ); //X_ADDR_START 0
	A8141MIPI_write_cmos_sensor(0x0348, 0xCC7); //X_ADDR_END 3279
	A8141MIPI_write_cmos_sensor(0x0346, 0xc  ); //Y_ADDR_START 0
	A8141MIPI_write_cmos_sensor(0x034A, 0x997); //Y_ADDR_END 2463
	A8141MIPI_write_cmos_sensor(0x034C, 0xCD0); //X_OUTPUT_SIZE 3280
	A8141MIPI_write_cmos_sensor(0x034E, 0x9A0); //Y_OUTPUT_SIZE 2464
	A8141MIPI_write_cmos_sensor(0x3040, 0x01C0); //X_ODD_INC
	A8141MIPI_write_cmos_sensor(0x3040, 0x003F);  //Y_ODD_INC

	//"X-Bin2 Y-Bin2" and "X-Bin2Skip2 Y-Bin2Skip2" Optimization
	A8141MIPI_write_cmos_sensor(0x306E, 0xFC80);   //Resample_Binning 3: enable
	A8141MIPI_write_cmos_sensor(0x3040, 0x0041); //BinSum 1: Enable

	//Binning Configuration
	A8141MIPI_write_cmos_sensor(0x3040, 0x0041); //LOW_POWER
	A8141MIPI_write_cmos_sensor(0x3040, 0x0041); //XY_BIN_ENABLE

	//"X-Bin2 Y-Skip2", "X-Bin2Skip2 Y-Bin2Skip2", "X-Bin2Skip2 Y-Skip4" Optimization
	A8141MIPI_write_cmos_sensor(0x3040, 0x0041); //X_BIN_ENABLE
	//BITFIELD=0x3ED8, 0x0000, 0 //CASUM_Related
	//BITFIELD=0x3EE6, 0x0010, 0 //CASUM
	A8141MIPI_write_cmos_sensor(0x3178, 0x0000); //XSkip2Bin2YSkip4 4x_optimization
	A8141MIPI_write_cmos_sensor(0x3ED0, 0x1e24); //XSkip2Bin2YSkip4 4x_optimization

	//Scale Configuration
	A8141MIPI_write_cmos_sensor(0x0400, 0x0000); //SCALE_MODE: 2: ENABLE
	A8141MIPI_write_cmos_sensor(0x0404, 0x10  ); //SCALE_M = 16


	//Timing Configuration
	A8141MIPI_write_cmos_sensor(0x0342, 0x1272);//LINE_LENGTH_PCK 4722
	A8141MIPI_write_cmos_sensor(0x0340, 0xA2F );//FRAME_LENGTH_LINES 2607
	A8141MIPI_write_cmos_sensor(0x0202, 0xA2F );//COARSE_INTEGRATION_TIME 2607
	A8141MIPI_write_cmos_sensor(0x3014, 0x3F6 );//FINE_INTEGRATION_TIME 1014
	A8141MIPI_write_cmos_sensor(0x3010, 0x78  );//FINE_CORRECTION 120

	//Start Streaming
	//A8141MIPI_write_cmos_sensor( 0x301A, 0x8250);     // RESET_REGISTER
	//A8141MIPI_write_cmos_sensor( 0x301A, 0x8650);     // RESET_REGISTER
	//A8141MIPI_write_cmos_sensor( 0x301A, 0x8658);     // RESET_REGISTER
	A8141MIPI_write_cmos_sensor8( 0x0104, 0x00  );        // GROUPED_PARAMETER_HOLD
	//A8141MIPI_write_cmos_sensor( 0x301A, 0x065C);     // RESET_REGISTER

    SENSORDB("[A8141MIPIRaw] Set 8M End\n"); 
}



/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   A8141MIPIOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 A8141MIPIOpen(void)
{
    int  retry = 0; 
    kal_uint16 temp_data;
	temp_data = A8141MIPI_read_cmos_sensor(0x0000);
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_sensor_id = temp_data;
	spin_unlock(&a8141mipi_drv_lock);


    // check if sensor ID correct
    retry = 3; 
    do {
		A8141MIPI_write_cmos_sensor8( 0x0103, 0x01 	);// SOFTWARE_RESET  
		mdelay(100);
		spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_sensor_write_I2C_address=A8141MIPI_WRITE_ID;
		spin_unlock(&a8141mipi_drv_lock);
		
        temp_data = A8141MIPI_read_cmos_sensor(0x0000); 
		spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_sensor_id = temp_data; 
		spin_unlock(&a8141mipi_drv_lock);
        if (A8141MIPI_sensor_id == A8141MIPI_SENSOR_ID)
            break; 
		
        SENSORDB("write id: 0x6C, Read Sensor ID Fail = 0x%04x\n", A8141MIPI_sensor_id); 
		
        retry--; 
    } while (retry > 0);
	
    SENSORDB("write id: 0x6C,, Read Sensor ID = 0x%04x\n", A8141MIPI_sensor_id); 
	
    if (A8141MIPI_sensor_id != A8141MIPI_SENSOR_ID)
    	{
    	retry = 3; 
    do {
		
		spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_sensor_write_I2C_address=A8141MIPI_WRITE_ID_1;
		spin_unlock(&a8141mipi_drv_lock);
		
        temp_data = A8141MIPI_read_cmos_sensor(0x0000);  
		spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_sensor_id = temp_data;
		spin_unlock(&a8141mipi_drv_lock);
        if (A8141MIPI_sensor_id == A8141MIPI_SENSOR_ID)
            break; 
        SENSORDB("write id: 0x6E, Read Sensor ID Fail = 0x%04x\n", A8141MIPI_sensor_id); 
        retry--; 
    } while (retry > 0);
	    if (A8141MIPI_sensor_id != A8141MIPI_SENSOR_ID)
          return ERROR_SENSOR_CONNECT_FAIL;
    	}
	SENSORDB("write id:0x%02x, Read Sensor ID success = 0x%04x\n",A8141MIPI_sensor_write_I2C_address, A8141MIPI_sensor_id); 

    A8141MIPI_Sensor_Init();

    spin_lock(&a8141mipi_drv_lock);
    A8141MIPI_g_iBackupExtraExp = 0;
	spin_unlock(&a8141mipi_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   A8141MIPIGetSensorID
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 A8141MIPIGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
	kal_uint16 temp_data;
	temp_data = A8141MIPI_read_cmos_sensor(0x0000);
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_sensor_id = temp_data;
	spin_unlock(&a8141mipi_drv_lock);
	

	// check if sensor ID correct
	retry = 3; 
	do {
	A8141MIPI_write_cmos_sensor8( 0x0103, 0x01  );// SOFTWARE_RESET	
	mdelay(100);
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_sensor_write_I2C_address=A8141MIPI_WRITE_ID;
	spin_unlock(&a8141mipi_drv_lock);
	*sensorID = A8141MIPI_read_cmos_sensor(0x0000);		
	if (*sensorID == A8141MIPI_SENSOR_ID)
	break; 
	SENSORDB("write id: 0x6C, Read Sensor ID Fail = 0x%04x\n", *sensorID); 
	retry--; 
	} while (retry > 0);

	SENSORDB("write id: 0x6C, Read Sensor ID = 0x%04x\n", *sensorID); 

	if (*sensorID != A8141MIPI_SENSOR_ID)
	{
	retry = 3; 
	do {
	//A8141MIPI_write_cmos_sensor8( 0x0103, 0x01    );// SOFTWARE_RESET	
	//mdelay(100);
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_sensor_write_I2C_address=A8141MIPI_WRITE_ID_1;
	spin_unlock(&a8141mipi_drv_lock);
	*sensorID = A8141MIPI_read_cmos_sensor(0x0000);		
	if (*sensorID == A8141MIPI_SENSOR_ID)
	break; 
	SENSORDB("write id: 0x6E, Read Sensor ID Fail = 0x%04x\n", A8141MIPI_sensor_id); 
	retry--; 
	} while (retry > 0);
	}

    if (*sensorID != A8141MIPI_SENSOR_ID) {		
		SENSORDB("Read Sensor ID fail= 0x%04x\n", A8141MIPI_sensor_id);
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	SENSORDB("write id: 0x%02x, Read Sensor ID success = 0x%04x\n", A8141MIPI_sensor_write_I2C_address,A8141MIPI_sensor_id);
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   A8141MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of A8141MIPI to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void A8141MIPI_SetShutter(kal_uint16 iShutter)
{
	unsigned long flags;
	//if(iShutter == A8141MIPI_pv_exposure_lines) return;  //to sovle burst shot lightness is not same

    if (iShutter < 1)
        iShutter = 1; 
   
    
	SENSORDB("[A8141MIPI_SetShutter] iShutter is :%d \n ", iShutter);
	spin_lock_irqsave(&a8141mipi_drv_lock,flags);
    A8141MIPI_pv_exposure_lines = iShutter;
	spin_unlock_irqrestore(&a8141mipi_drv_lock,flags);
    A8141MIPI_write_shutter(iShutter);
}   /*  A8141MIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   A8141MIPI_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 A8141MIPI_read_shutter(void)
{
    return (UINT16)( (A8141MIPI_read_cmos_sensor(0x3012) ));
}

/*************************************************************************
* FUNCTION
*   A8141MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of A8141MIPI.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void A8141MIPI_NightMode(kal_bool bEnable)
{
	SENSORDB("A8141MIPI_NightMode = %d\n", bEnable); 

}/*	A8141MIPI_NightMode */



/*************************************************************************
* FUNCTION
*   A8141MIPIClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 A8141MIPIClose(void)
{
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(A8141MIPIhDrvI2C);
    //e_porting
    return ERROR_NONE;
}	/* A8141MIPIClose() */

void A8141MIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint16  iTemp; 
#if 1
    iTemp = A8141MIPI_read_cmos_sensor(0x3040) ;	//Clear the mirror and flip bits.
    SENSORDB("A8141MIPI before iTemp = %0x\n", iTemp);
	iTemp=iTemp & 0x3fff;
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            A8141MIPI_write_cmos_sensor(0x3040, iTemp |0x0000);	//Set normal
			spin_lock(&a8141mipi_drv_lock);
			COLOR_FORMAT=SENSOR_OUTPUT_FORMAT_RAW_B;
			spin_unlock(&a8141mipi_drv_lock);
            break;
        case IMAGE_V_MIRROR:
            A8141MIPI_write_cmos_sensor(0x3040, iTemp | 0x8000);	//Set flip
			spin_lock(&a8141mipi_drv_lock);
			COLOR_FORMAT=SENSOR_OUTPUT_FORMAT_RAW_Gr;
			spin_unlock(&a8141mipi_drv_lock);
            break;
        case IMAGE_H_MIRROR:
            A8141MIPI_write_cmos_sensor(0x3040, iTemp | 0x4000);	//Set mirror
			spin_lock(&a8141mipi_drv_lock);
			COLOR_FORMAT=SENSOR_OUTPUT_FORMAT_RAW_Gr;
			spin_unlock(&a8141mipi_drv_lock);
            break;
        case IMAGE_HV_MIRROR:
            A8141MIPI_write_cmos_sensor(0x3040, iTemp |0xc000);	//Set mirror and flip
			spin_lock(&a8141mipi_drv_lock);
			COLOR_FORMAT=SENSOR_OUTPUT_FORMAT_RAW_R;
			spin_unlock(&a8141mipi_drv_lock);
            break;
    }
   iTemp = A8141MIPI_read_cmos_sensor(0x3040) ;
   SENSORDB("A8141MIPI after iTemp = %0x\n", iTemp);
#endif
}


/*************************************************************************
* FUNCTION
*   A8141MIPIPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 A8141MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
    SENSORDB("A8141MIPIPreview\n");
	spin_lock(&a8141mipi_drv_lock);
	g_iA8141MIPI_Mode = A8141MIPI_MODE_PREVIEW;
	spin_unlock(&a8141mipi_drv_lock);
    

    //if(A8141MIPI_720P == A8141MIPI_g_RES)
    {
        A8141MIPI_set_2M();	
		spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_g_RES = A8141MIPI_2M;
		spin_unlock(&a8141mipi_drv_lock);
		
    }

    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_MPEG4_encode_mode = KAL_TRUE;
		spin_unlock(&a8141mipi_drv_lock);
		
        
    }
    else
    {
        spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_MPEG4_encode_mode = KAL_FALSE;
		spin_unlock(&a8141mipi_drv_lock);
    }

    iStartX += A8141MIPI_IMAGE_SENSOR_PV_STARTX;
    iStartY += A8141MIPI_IMAGE_SENSOR_PV_STARTY;
	sensor_config_data->SensorImageMirror=IMAGE_HV_MIRROR;
    A8141MIPISetFlipMirror(sensor_config_data->SensorImageMirror);
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_dummy_pixels = 0;
    A8141MIPI_dummy_lines = 0;
    A8141MIPI_PV_dummy_pixels = A8141MIPI_dummy_pixels;
    A8141MIPI_PV_dummy_lines = A8141MIPI_dummy_lines;
	spin_unlock(&a8141mipi_drv_lock);

    
	A8141MIPI_write_shutter(A8141MIPI_pv_exposure_lines);

    memcpy(&A8141MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));


    SENSORDB("Preview resolution:%d %d %d %d\n", image_window->GrabStartX, image_window->GrabStartY, image_window->ExposureWindowWidth, image_window->ExposureWindowHeight); 
    return ERROR_NONE;
}	/* A8141MIPIPreview() */

UINT32 A8141MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter=A8141MIPI_pv_exposure_lines;
	kal_uint16 iStartX = 0, iStartY = 0;
    SENSORDB("A8141MIPICapture\n");
	//if(A8141MIPI_g_RES==A8141MIPI_8M) return;
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_dummy_pixels= 0;
	A8141MIPI_dummy_lines = 0; 
	A8141MIPI_MPEG4_encode_mode == KAL_FALSE;
	A8141MIPI_Auto_Flicker_mode=KAL_FALSE;
	spin_unlock(&a8141mipi_drv_lock);
	
	A8141MIPI_set_8M();
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_g_RES = A8141MIPI_8M;
	spin_unlock(&a8141mipi_drv_lock);
	sensor_config_data->SensorImageMirror=IMAGE_HV_MIRROR;
	A8141MIPISetFlipMirror(sensor_config_data->SensorImageMirror); 
    shutter = ((UINT32)(shutter*A8141MIPI_PV_PERIOD_PIXEL_NUMS)+A8141MIPI_FULL_PERIOD_PIXEL_NUMS/2)/A8141MIPI_FULL_PERIOD_PIXEL_NUMS;
	shutter = ((UINT32)(shutter*A8141MIPI_CAP_vt_pclk*2)+A8141MIPI_PV_vt_pclk/2)/A8141MIPI_PV_vt_pclk;

	A8141MIPI_write_shutter(shutter);
	//SENSORDB("A8141MIPI capture read shutter 0x%x\n",A8141MIPI_read_shutter());
	//SENSORDB("A8141MIPI capture read gain 0x%x\n",read_A8141MIPI_gain());
	//A8141MIPI_write_cmos_sensor(0x3012, 13765);
	//A8141MIPI_SetGain(0x1060);

	iStartX = A8141MIPI_IMAGE_SENSOR_CAP_STARTX;
	iStartY = A8141MIPI_IMAGE_SENSOR_CAP_STARTY;

    memcpy(&A8141MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}	/* A8141MIPICapture() */

UINT32 A8141MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    pSensorResolution->SensorPreviewWidth	= A8141MIPI_REAL_PV_WIDTH-64;
    pSensorResolution->SensorPreviewHeight	= A8141MIPI_REAL_PV_HEIGHT-48;
    pSensorResolution->SensorFullWidth		= A8141MIPI_IMAGE_SENSOR_FULL_WIDTH -192;  //3264-192=3072
    pSensorResolution->SensorFullHeight		= A8141MIPI_IMAGE_SENSOR_FULL_HEIGHT-144;  //2448-144=2304

    return ERROR_NONE;
}   /* A8141MIPIGetResolution() */

UINT32 A8141MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch(ScenarioId){
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
				pSensorInfo->SensorPreviewResolutionX=A8141MIPI_IMAGE_SENSOR_FULL_WIDTH-192;
				pSensorInfo->SensorPreviewResolutionY=A8141MIPI_IMAGE_SENSOR_FULL_HEIGHT-144;
				pSensorInfo->SensorCameraPreviewFrameRate=13;
			break;

		default:
		        pSensorInfo->SensorPreviewResolutionX=A8141MIPI_REAL_PV_WIDTH-64;
		        pSensorInfo->SensorPreviewResolutionY=A8141MIPI_REAL_PV_HEIGHT-48;
				pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}
	pSensorInfo->SensorPreviewResolutionX=A8141MIPI_REAL_PV_WIDTH-64;
    pSensorInfo->SensorPreviewResolutionY=A8141MIPI_REAL_PV_HEIGHT-48;

    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=13;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensorOutputDataFormat=COLOR_FORMAT;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    pSensorInfo->SensorDriver3D = 0;   // the sensor driver is 2D
    
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_05M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=TRUE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_05M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_05M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=TRUE;

    pSensorInfo->CaptureDelayFrame = 1; 
    pSensorInfo->PreviewDelayFrame = 1; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;      
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 1;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = A8141MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = A8141MIPI_IMAGE_SENSOR_PV_STARTY;           		
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = A8141MIPI_IMAGE_SENSOR_CAP_STARTX;	//2*A8141MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = A8141MIPI_IMAGE_SENSOR_CAP_STARTY;	//2*A8141MIPI_IMAGE_SENSOR_PV_STARTY;          			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 1; 
            pSensorInfo->SensorGrabStartY = 1;             
            break;
    }

    memcpy(pSensorConfigData, &A8141MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* A8141MIPIGetInfo() */


UINT32 A8141MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	spin_lock(&a8141mipi_drv_lock);
		CurrentScenarioId = ScenarioId;
	spin_unlock(&a8141mipi_drv_lock);
     SENSORDB("CurrentScenarioId=%d\n",CurrentScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            A8141MIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            A8141MIPICapture(pImageWindow, pSensorConfigData);
            break;
            //s_porting add
            //s_porting add
            //s_porting add
        default:
            return ERROR_INVALID_SCENARIO_ID;
            //e_porting add
            //e_porting add
            //e_porting add
    }
    return TRUE;
} /* A8141MIPIControl() */

UINT32 A8141MIPISetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("[A8141MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
	if(u2FrameRate==0) return;
	spin_lock(&a8141mipi_drv_lock);
	A8141MIPI_MPEG4_encode_mode = KAL_TRUE; 
    A8141MIPI_MAX_EXPOSURE_LINES=(UINT32)(A8141MIPI_PV_vt_pclk*100000)/(A8141MIPI_PV_PERIOD_PIXEL_NUMS*u2FrameRate);
	spin_unlock(&a8141mipi_drv_lock);
   
	A8141MIPI_write_cmos_sensor(0x0340,A8141MIPI_MAX_EXPOSURE_LINES+4);  //in binning mode, frame length lines - shutter must >3
	SENSORDB("[A8141MIPI_write_shutter] framelength is :0x%x \n ", A8141MIPI_MAX_EXPOSURE_LINES+4);
	
    return TRUE;
}

UINT32 A8141MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
kal_uint32 pv_max_frame_rate_lines = A8141MIPI_MAX_EXPOSURE_LINES;

    SENSORDB("[A8141MIPISetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
    if(bEnable) {   // enable auto flicker   
	    spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_Auto_Flicker_mode = KAL_TRUE; 
		spin_unlock(&a8141mipi_drv_lock);
        
        if(A8141MIPI_MPEG4_encode_mode == KAL_TRUE) {    // in the video mode, reset the frame rate
            pv_max_frame_rate_lines = A8141MIPI_MAX_EXPOSURE_LINES + (A8141MIPI_MAX_EXPOSURE_LINES>>7);            
            A8141MIPI_write_cmos_sensor(0x0340,pv_max_frame_rate_lines);         	
        }
    } else {
    	spin_lock(&a8141mipi_drv_lock);
		A8141MIPI_Auto_Flicker_mode = KAL_FALSE; 
		spin_unlock(&a8141mipi_drv_lock);
        
        if(A8141MIPI_MPEG4_encode_mode == KAL_TRUE) {    // in the video mode, restore the frame rate
              A8141MIPI_write_cmos_sensor(0x0340,A8141MIPI_MAX_EXPOSURE_LINES+4);         	
        }
        SENSORDB("Disable Auto flicker\n");    
    }
    return TRUE;
}

UINT32 A8141MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[A8141MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    
    if(bEnable) {   // enable color bar   
        A8141MIPI_write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
        A8141MIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
        A8141MIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern 
    } else {
        A8141MIPI_write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    return TRUE;
}

UINT32 A8141MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=A8141MIPI_IMAGE_SENSOR_FULL_WIDTH -80;
            *pFeatureReturnPara16=A8141MIPI_IMAGE_SENSOR_FULL_HEIGHT -60;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
		            *pFeatureReturnPara16++=A8141MIPI_FULL_PERIOD_PIXEL_NUMS;  
		            *pFeatureReturnPara16=A8141MIPI_FULL_PERIOD_LINE_NUMS;	
		            SENSORDB("Sensor period:%d %d\n", A8141MIPI_FULL_PERIOD_PIXEL_NUMS, A8141MIPI_FULL_PERIOD_LINE_NUMS); 
		            *pFeatureParaLen=4;        				
        				break;
        			
        			default:	
		            *pFeatureReturnPara16++=A8141MIPI_PV_PERIOD_PIXEL_NUMS;  
		            *pFeatureReturnPara16=A8141MIPI_PV_PERIOD_LINE_NUMS;	
		            SENSORDB("Sensor period:%d %d\n", A8141MIPI_PV_PERIOD_PIXEL_NUMS,A8141MIPI_PV_PERIOD_LINE_NUMS); 
		            *pFeatureParaLen=4;
	            break;
          	}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
		            *pFeatureReturnPara32 = A8141MIPI_CAP_vt_pclk*100000; //
		            *pFeatureParaLen=4;		         	
		         		break;
		         		
		         		default:
		            *pFeatureReturnPara32 = A8141MIPI_PV_vt_pclk*100000; //
		            *pFeatureParaLen=4;
		            break;
		         }
		         break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            A8141MIPI_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            A8141MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            A8141MIPI_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&a8141mipi_drv_lock);
			A8141MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&a8141mipi_drv_lock);  
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            A8141MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = A8141MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
	            spin_lock(&a8141mipi_drv_lock);
				A8141MIPISensorCCT[i].Addr=*pFeatureData32++;
	            A8141MIPISensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&a8141mipi_drv_lock);     
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=A8141MIPISensorCCT[i].Addr;
                *pFeatureData32++=A8141MIPISensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
                spin_lock(&a8141mipi_drv_lock);
                A8141MIPISensorReg[i].Addr=*pFeatureData32++;
                A8141MIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&a8141mipi_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=A8141MIPISensorReg[i].Addr;
                *pFeatureData32++=A8141MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=A8141MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, A8141MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, A8141MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &A8141MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            A8141MIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            A8141MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=A8141MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            A8141MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            A8141MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            A8141MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=COLOR_FORMAT;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            A8141MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            A8141MIPIGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            A8141MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            //A8141MIPISetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* A8141MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncA8141MIPI=
{
    A8141MIPIOpen,
    A8141MIPIGetInfo,
    A8141MIPIGetResolution,
    A8141MIPIFeatureControl,
    A8141MIPIControl,
    A8141MIPIClose
};

UINT32 A8141_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncA8141MIPI;

    return ERROR_NONE;
}   /* SensorInit() */

