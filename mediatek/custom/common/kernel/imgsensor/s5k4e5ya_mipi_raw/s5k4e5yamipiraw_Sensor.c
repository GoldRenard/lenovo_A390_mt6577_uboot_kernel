/*******************************************************************************************/


/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k4e5yamipiraw_Sensor.h"
#include "s5k4e5yamipiraw_Camera_Sensor_para.h"
#include "s5k4e5yamipiraw_CameraCustomized.h"
#include <asm/system.h>
static DEFINE_SPINLOCK(s5k4e5_drv_lock);


#define S5K4E5YA_DEBUG

#ifdef S5K4E5YA_DEBUG
#define SENSORDB(fmt, arg...) printk( "[S5K4E5YARaw] "  fmt, ##arg)
#define LOG_TAG "[SENSOR_DRV]"
#define SENSOR_DEBUG(fmt,arg...) printk(LOG_TAG "in func %s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSORDB(x,...)
#define SENSOR_DEBUG(fmt,arg...)  
#endif

//If use ZSD, undefine S5K4E5YA_1M_SIZE_PREVIEW
#define S5K4E5YA_1M_SIZE_PREVIEW

#define mDELAY(ms)  mdelay(ms)

MSDK_SENSOR_CONFIG_STRUCT S5K4E5YASensorConfigData;

kal_uint32 S5K4E5YA_FAC_SENSOR_REG;

static kal_bool bSensorShutterLock=KAL_FALSE;

#define SENSOR_SHUTTER_LOCK(lock) do{bSensorShutterLock=lock;}while(0)
#define IS_SENSOR_SHUTTER_LOCKED() KAL_TRUE==bSensorShutterLock?1:0


#define S5K4E5YA_PV_PCLK 122400000
#define S5K4E5YA_ZSD_PCLK 129600000
#define S5K4E5YA_CP_PCLK S5K4E5YA_ZSD_PCLK


static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = ACDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT S5K4E5YASensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT S5K4E5YASensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static S5K4E5YA_PARA_STRUCT s5k4e5ya;
//static kal_uint32 u4PreviewShutter=0; //To resolve setting preview shutter at capture issue

//extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
//extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
kal_uint16 S5K4E5YA_Read_Gain(void);
UINT16 S5K4E5YA_read_shutter(void);
static UINT32 S5K4E5YASetMaxFrameRate(UINT16 u2FrameRate);




#if 1
kal_uint16 S5K4E5YA_write_cmos_sensor(kal_uint32 Addr, kal_uint32 Para)
{
	char puSendCmd[3] = {(char)(Addr >> 8) , (char)(Addr & 0xFF) ,(char)(Para & 0xFF)};
	iWriteRegI2C(puSendCmd , 3,S5K4E5YAMIPI_WRITE_ID);
}

kal_uint16 S5K4E5YA_read_cmos_sensor(kal_uint32 Addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(Addr >> 8) , (char)(Addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,S5K4E5YAMIPI_WRITE_ID);
	
    return get_byte;
}
#else
#define S5K4E5YA_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, S5K4E5YAMIPI_WRITE_ID)

kal_uint16 S5K4E5YA_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,S5K4E5YAMIPI_WRITE_ID);
    return get_byte;
}
#endif

#define Sleep(ms) mdelay(ms)

void S5K4E5YA_write_shutter(kal_uint16 shutter)
{
    kal_uint16 iExp = shutter;
    kal_uint16 S5K4E5YA_g_iExtra_ExpLines = 0 ;
	
	kal_uint16 max_shutter = 0;
	kal_uint16 extra_lines = 0;
	kal_uint16 line_length = 0;
	kal_uint16 frame_length = 0;
	unsigned long flags;

	// Max coarse integration time is Frame Length - 8
	// Min coarse integration time is 2.
	
	if ( SENSOR_MODE_PREVIEW == s5k4e5ya.sensorMode )  //(g_iS5K4E5YA_Mode == S5K4E5YA_MODE_PREVIEW)	//SXGA size output
	{
		max_shutter = S5K4E5YA_PV_PERIOD_LINE_NUMS + s5k4e5ya.DummyLines ; //992
	}
	else				//QSXGA size output
	{
		max_shutter = S5K4E5YA_FULL_PERIOD_LINE_NUMS; //+ s5k4e5ya.DummyLines ; //1972
	}
	
	if (shutter < 3)
		shutter = 3;

	if (shutter > max_shutter)
		extra_lines = shutter - max_shutter + 8;
	else
		extra_lines = 0;

	if ( SENSOR_MODE_PREVIEW == s5k4e5ya.sensorMode )	//SXGA size output
	{
		line_length = S5K4E5YA_PV_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels; 
		frame_length = S5K4E5YA_PV_PERIOD_LINE_NUMS+ s5k4e5ya.DummyLines + extra_lines ; 
	}
	else				//QSXGA size output
	{
		line_length = S5K4E5YA_FULL_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels; 
		frame_length = S5K4E5YA_FULL_PERIOD_LINE_NUMS + s5k4e5ya.DummyLines + extra_lines ; 
	}

	S5K4E5YA_write_cmos_sensor(0x0104, 0x01);	//Grouped parameter hold
	//Set total frame length
	S5K4E5YA_write_cmos_sensor(0x0340, (frame_length >> 8) & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0341, frame_length & 0xFF);
	spin_lock_irqsave(&s5k4e5_drv_lock,flags);
	s5k4e5ya.maxExposureLines = frame_length;
	s5k4e5ya.shutter=shutter;
	spin_unlock_irqrestore(&s5k4e5_drv_lock,flags);
	//Set total line length
	//S5K4E5YA_write_cmos_sensor(0x0342, (line_length >> 8) & 0xFF);
	//S5K4E5YA_write_cmos_sensor(0x0343, line_length & 0xFF);

	//Set shutter (Coarse integration time, uint: lines.)
	S5K4E5YA_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0203, shutter & 0xFF);
	
	S5K4E5YA_write_cmos_sensor(0x0104, 0x00);	//Grouped parameter release
	
	SENSOR_DEBUG("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d", shutter, extra_lines, line_length, frame_length);	
	if(s5k4e5ya.bAutoFlickerMode && (0==extra_lines))//change framerate from 30 to 29.6, 15 to 14.8
	{
		if(SENSOR_MODE_PREVIEW == s5k4e5ya.sensorMode) 
		{
			if(KAL_TRUE==s5k4e5ya.bIsVideoNightMode)
			{
				S5K4E5YASetMaxFrameRate(148);
			}
			else
			{
				S5K4E5YASetMaxFrameRate(296);
			}
		}
		else if(SENSOR_MODE_ZSD_PREVIEW == s5k4e5ya.sensorMode)			
		{
			#ifdef ENABLE_ZSD_FPS_24
			S5K4E5YASetMaxFrameRate(238);
			#else
			S5K4E5YASetMaxFrameRate(148);
			#endif
		}	
	}		
}   /* write_S5K4E5YA_shutter */

/*
static kal_uint16 S5K4E5YAReg2Gain(const kal_uint8 iReg)
{
    kal_uint8 iI;

    // Range: 1x to 8x
    for (iI = 0; iI < S5K4E5YA_MaxGainIndex; iI++) {
        if(iReg <= S5K4E5YA_sensorGainMapping[iI][1]){
            break;
        }
    }

    return S5K4E5YA_sensorGainMapping[iI][0];
}

static kal_uint8 S5K4E5YAGain2Reg(const kal_uint16 iGain)
{
    kal_uint8 iI;

    for (iI = 0; iI < (S5K4E5YA_MaxGainIndex-1); iI++) {
        if(iGain <= S5K4E5YA_sensorGainMapping[iI][0]){
            break;
        }
    }
   
    if(iGain != S5K4E5YA_sensorGainMapping[iI][0])
    {
         printk("[S5K4E5YAGain2Reg] Gain mapping don't correctly:%d %d \n", iGain, S5K4E5YA_sensorGainMapping[iI][0]);
    }
	
    return S5K4E5YA_sensorGainMapping[iI][1];
}
*/

void write_S5K4E5YA_gain(kal_uint16 gain)
{
	return;
}

/*************************************************************************
* FUNCTION
*    S5K4E5YA_SetGain
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
void S5K4E5YA_SetGain(UINT16 iGain)
{
	UINT16 gain = iGain;

	//Sensor base gain = 32;
	//ISP base gain =64;
    //Sensor just guarantees by 16 time, using the value under BASEGAIN is not allowed.
	//if(gain == 0)
	//	gain = s5k4e5ya.ispBaseGain ;
	if(gain < (S5K4E5YA_MIN_ANALOG_GAIN * s5k4e5ya.ispBaseGain))
	{
		gain = S5K4E5YA_MIN_ANALOG_GAIN * s5k4e5ya.ispBaseGain;
		SENSOR_DEBUG("[error] gain value is below 1*IspBaseGain!");
	}
	if(gain > (S5K4E5YA_MAX_ANALOG_GAIN * s5k4e5ya.ispBaseGain))
	{
		gain = S5K4E5YA_MAX_ANALOG_GAIN * s5k4e5ya.ispBaseGain;//Max up to 16X
		SENSOR_DEBUG("[error] gain value exceeds 16*IspBaseGain!");
	}
	//ASSERT((gain >= (S5K4E5YA_MIN_ANALOG_GAIN * s5k4e5ya.ispBaseGain)) && (gain <= (S5K4E5YA_MAX_ANALOG_GAIN * s5k4e5ya.ispBaseGain)));	
	// Analog gain = Analog_gain_code[15:0] / 32
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.sensorGlobalGain= (gain * s5k4e5ya.sensorBaseGain) / s5k4e5ya.ispBaseGain;
	spin_unlock(&s5k4e5_drv_lock);
	
	S5K4E5YA_write_cmos_sensor(0x0104, 0x01);	//Grouped parameter hold

	S5K4E5YA_write_cmos_sensor(0x0204, (s5k4e5ya.sensorGlobalGain & 0xFF00) >> 8); // ANALOG_GAIN_CTRLR
	S5K4E5YA_write_cmos_sensor(0x0205, s5k4e5ya.sensorGlobalGain & 0xFF);

	S5K4E5YA_write_cmos_sensor(0x0104, 0x00);	//Grouped parameter release
	//SENSOR_DEBUG("s5k4e5ya.sensorBaseGain value is %d",s5k4e5ya.sensorBaseGain);
	//SENSOR_DEBUG("s5k4e5ya.ispBaseGain value is %d",s5k4e5ya.ispBaseGain);
	SENSOR_DEBUG("ISP gain=%d, s5k4e5ya.sensorGlobalGain=%d\n", gain, s5k4e5ya.sensorGlobalGain);
	//SENSORDB("gain=%d, S5K4E5YA_sensor_global_gain=%d\n", gain, s5k4e5ya.sensorGlobalGain);
	//s5k4e5ya.sensorGlobalGain = S5K4E5YA_Read_Gain(); 
}   /*  S5K4E5YA_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    S5K4E5YA_Read_Gain
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
kal_uint16 S5K4E5YA_Read_Gain(void)
{
    kal_uint8  temp_reg;
	kal_uint16 sensor_gain;

	sensor_gain = (((S5K4E5YA_read_cmos_sensor(0x0204)<<8)&0xFF00) | (S5K4E5YA_read_cmos_sensor(0x0205)&0xFF)); // ANALOG_GAIN_CTRLR  

	SENSOR_DEBUG("read sensor gain value is %d",sensor_gain);
	
	return sensor_gain;
}  /* S5K4E5YA_Read_Gain */

/*
void write_S5K4E5YA_gain(kal_uint16 gain)
{
    
}
*/
void S5K4E5YA_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=S5K4E5YASensorReg[i].Addr; i++)
    {
        S5K4E5YA_write_cmos_sensor(S5K4E5YASensorReg[i].Addr, S5K4E5YASensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K4E5YASensorReg[i].Addr; i++)
    {
        S5K4E5YA_write_cmos_sensor(S5K4E5YASensorReg[i].Addr, S5K4E5YASensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        S5K4E5YA_write_cmos_sensor(S5K4E5YASensorCCT[i].Addr, S5K4E5YASensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    S5K4E5YA_sensor_to_camera_para
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
void S5K4E5YA_sensor_to_camera_para(void)
{
    kal_uint32    i;
	kal_uint32 uTemp=0;
    for(i=0; 0xFFFFFFFF!=S5K4E5YASensorReg[i].Addr; i++)
    {
		uTemp = S5K4E5YA_read_cmos_sensor(S5K4E5YASensorReg[i].Addr);
		spin_lock(&s5k4e5_drv_lock);
        S5K4E5YASensorReg[i].Para = uTemp;
		spin_unlock(&s5k4e5_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K4E5YASensorReg[i].Addr; i++)
    {
		uTemp = S5K4E5YA_read_cmos_sensor(S5K4E5YASensorReg[i].Addr);
		spin_lock(&s5k4e5_drv_lock);
        S5K4E5YASensorReg[i].Para = uTemp;
		spin_unlock(&s5k4e5_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    S5K4E5YA_get_sensor_group_count
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
kal_int32  S5K4E5YA_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void S5K4E5YA_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void S5K4E5YA_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;
    
    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= S5K4E5YASensorCCT[temp_addr].Para;
			temp_gain= (temp_para/s5k4e5ya.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= S5K4E5YA_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= S5K4E5YA_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
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
                    info_ptr->ItemValue=    111;  //MT9P017_MAX_EXPOSURE_LINES;
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



kal_bool S5K4E5YA_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
             temp_para=(temp_gain * s5k4e5ya.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }          
          else
			  ASSERT(0);
		  spin_lock(&s5k4e5_drv_lock);
          S5K4E5YASensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&s5k4e5_drv_lock);
          S5K4E5YA_write_cmos_sensor(S5K4E5YASensorCCT[temp_addr].Addr,temp_para);

           //s5k4e5ya.sensorGlobalGain= S5K4E5YA_Read_Gain();

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
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
					spin_lock(&s5k4e5_drv_lock);
                    S5K4E5YA_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&s5k4e5_drv_lock);
                    break;
                case 1:
                    S5K4E5YA_write_cmos_sensor(S5K4E5YA_FAC_SENSOR_REG,ItemValue);
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

//void S5K4E5YA_set_isp_driving_current(kal_uint8 current)
//{

//}

static void S5K4E5YA_SetDummy( const kal_uint16 iPixels, const kal_uint16 iLines )
{
	kal_uint16 line_length = 0;
	kal_uint16 frame_length = 0;
	
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.DummyPixels=0;
	s5k4e5ya.DummyLines=iLines;	
	spin_unlock(&s5k4e5_drv_lock);
	
	if ( SENSOR_MODE_PREVIEW == s5k4e5ya.sensorMode )	//SXGA size output
	{
		line_length = S5K4E5YA_PV_PERIOD_PIXEL_NUMS + 0; //s5k4e5 linelength is fixed to 0xC6F
		frame_length = S5K4E5YA_PV_PERIOD_LINE_NUMS + iLines;
	}
	else				//QSXGA size output
	{
		line_length = S5K4E5YA_FULL_PERIOD_PIXEL_NUMS + 0;	//s5k4e5 linelength is fixed to 0xC6F
		frame_length = S5K4E5YA_FULL_PERIOD_LINE_NUMS + iLines;
	}
	
	if(s5k4e5ya.maxExposureLines > frame_length )
		return;	
	
	ASSERT(line_length < S5K4E5YA_MAX_LINE_LENGTH);		//0xCCCC
	ASSERT(frame_length < S5K4E5YA_MAX_FRAME_LENGTH);	//0xFFFF
	
	S5K4E5YA_write_cmos_sensor(0x0104, 0x01);	//Grouped parameter hold

	//Set total frame length
	S5K4E5YA_write_cmos_sensor(0x0340, (frame_length >> 8) & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0341, frame_length & 0xFF);
	
	//s5k4e5ya.maxExposureLines = frame_length;
	//S5K4E5YA_MAX_EXPOSURE_LINES =frame_length;

	//Set total line length
	S5K4E5YA_write_cmos_sensor(0x0342, (line_length >> 8) & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0343, line_length & 0xFF);

	S5K4E5YA_write_cmos_sensor(0x0104, 0x00);	//Grouped parameter release
	//SENSOR_DEBUG("set frame_length=%d, line_length=%d",frame_length,line_length);
	
}   /*  S5K4E5YA_SetDummy */

void S5K4E5YAPreviewSetting(void)
{	
    SENSOR_DEBUG("enter!");
	
	//if(s5k4e5ya.sensorMode != SENSOR_MODE_INIT) 
	//{
	//	S5K4E5YA_write_cmos_sensor(0x0100, 0x00);
	//}
	
	S5K4E5YA_write_cmos_sensor(0x0100,0x00); // stream off
	S5K4E5YA_write_cmos_sensor(0x0103,0x01); // software reset

	//--> This registers are for FACTORY ONLY. If you change it without prior notification.
	// YOU are RESPONSIBLE for the FAILURE that will happen in the future.
	//+++++++++++++++++++++++++++++++//
	/////////////////////////////////////////////////////////////
	/////////Should reset global settings after software reset [begin]
	/////////////////////////////////////////////////////////////
	// Analog  setting
	//// CDS timing setting ...
	S5K4E5YA_write_cmos_sensor(0x3000, 0x05);
	S5K4E5YA_write_cmos_sensor(0x3001, 0x03);
	S5K4E5YA_write_cmos_sensor(0x3002, 0x08);
	S5K4E5YA_write_cmos_sensor(0x3003, 0x0A);
	S5K4E5YA_write_cmos_sensor(0x3004, 0x50);
	S5K4E5YA_write_cmos_sensor(0x3005, 0x0E);
	S5K4E5YA_write_cmos_sensor(0x3006, 0x5E);
	S5K4E5YA_write_cmos_sensor(0x3007, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3008, 0x78);
	S5K4E5YA_write_cmos_sensor(0x3009, 0x78);
	S5K4E5YA_write_cmos_sensor(0x300A, 0x50);
	S5K4E5YA_write_cmos_sensor(0x300B, 0x08);
	S5K4E5YA_write_cmos_sensor(0x300C, 0x14);
	S5K4E5YA_write_cmos_sensor(0x300D, 0x00);
	S5K4E5YA_write_cmos_sensor(0x300F, 0x40);
	S5K4E5YA_write_cmos_sensor(0x301B, 0x77);
		   
	//// CDS option setting ...
	S5K4E5YA_write_cmos_sensor(0x3010, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3011, 0x3A);
								  
	S5K4E5YA_write_cmos_sensor(0x3012, 0x30);
	S5K4E5YA_write_cmos_sensor(0x3013, 0xA0);
	S5K4E5YA_write_cmos_sensor(0x3014, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3015, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3016, 0x52);
	S5K4E5YA_write_cmos_sensor(0x3017, 0x94);
	S5K4E5YA_write_cmos_sensor(0x3018, 0x70);
								  
	S5K4E5YA_write_cmos_sensor(0x301D, 0xD4);
								  
	S5K4E5YA_write_cmos_sensor(0x3021, 0x02);
	S5K4E5YA_write_cmos_sensor(0x3022, 0x24);
	S5K4E5YA_write_cmos_sensor(0x3024, 0x40);
	S5K4E5YA_write_cmos_sensor(0x3027, 0x08);
									
	//// Pixel option setting ...		
	S5K4E5YA_write_cmos_sensor(0x301C, 0x06);
	S5K4E5YA_write_cmos_sensor(0x30D8, 0x3F);
									
	//+++++++++++++++++++++++++++++++// 
	// ADLC setting ... 				
	S5K4E5YA_write_cmos_sensor(0x3070, 0x5F);
	S5K4E5YA_write_cmos_sensor(0x3071, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3080, 0x04);
	S5K4E5YA_write_cmos_sensor(0x3081, 0x38);
	S5K4E5YA_write_cmos_sensor(0x302E, 0x0B);
	
// MIPI 1304 x 980 @ 30fps				
// MCLK 24Mhz, Vt_Pix_Clk 122Mhz,  612Mbps per lane				
				
//+++++++++++++++++++++++++++++++//				
// MIPI setting				
    S5K4E5YA_write_cmos_sensor(0x30BD, 0x00); //    S5K4E5YA_write_cmos_sensor(0xEL_CCP[0]				
    S5K4E5YA_write_cmos_sensor(0x3084, 0x15); //    S5K4E5YA_write_cmos_sensor(0xYNC Mode				
    S5K4E5YA_write_cmos_sensor(0x30BE, 0x1A); //M_PCLKDIV_AUTO[4], M_DIV_PCLK[3:0]				
    S5K4E5YA_write_cmos_sensor(0x30C1, 0x01); //pack video enable [0]				
    S5K4E5YA_write_cmos_sensor(0x30EE, 0x02); //DPHY enable [1]				
    S5K4E5YA_write_cmos_sensor(0x3111, 0x86); //Embedded data off [5]				
    S5K4E5YA_write_cmos_sensor(0x30E8, 0x0F); //Continuous mode 				
				                            
    S5K4E5YA_write_cmos_sensor(0x30E3, 0x38);	//According to MCLK			
    S5K4E5YA_write_cmos_sensor(0x30E4, 0x40);				
    S5K4E5YA_write_cmos_sensor(0x3113, 0x70);				
    S5K4E5YA_write_cmos_sensor(0x3114, 0x80);				
    S5K4E5YA_write_cmos_sensor(0x3115, 0x7B);				
    S5K4E5YA_write_cmos_sensor(0x3116, 0xC0);				
    S5K4E5YA_write_cmos_sensor(0x30EE, 0x12);
	/////////////////////////////////////////////////////////////
	/////////Should reset global settings after software reset [end]
	/////////////////////////////////////////////////////////////

	// PLL setting ...				          
    S5K4E5YA_write_cmos_sensor(0x0305, 0x04);				
    S5K4E5YA_write_cmos_sensor(0x0306, 0x00);				
    S5K4E5YA_write_cmos_sensor(0x0307, 0x66);		
    S5K4E5YA_write_cmos_sensor(0x30B5, 0x01);				
    S5K4E5YA_write_cmos_sensor(0x30E2, 0x02); //num lanes[1:0] = 2				
    S5K4E5YA_write_cmos_sensor(0x30F1, 0xA0); // DPHY Band Control 				
				                            
    S5K4E5YA_write_cmos_sensor(0x30BC, 0xA8); // [7]bit : DBLR enable, [6]~[3]bit : DBLR Div				
        // DBLR clock = Pll output/DBLR Div = 61.2Mhz				
				
    S5K4E5YA_write_cmos_sensor(0x30BF, 0xAB); //outif_enable[7], data_type[5:0](2Bh = bayer 10bit)				
    S5K4E5YA_write_cmos_sensor(0x30C0, 0xA0); //video_offset[7:4] 				
    S5K4E5YA_write_cmos_sensor(0x30C8, 0x06); //video_data_length 				
    S5K4E5YA_write_cmos_sensor(0x30C9, 0x5E);				
				                            
    S5K4E5YA_write_cmos_sensor(0x3112, 0x00); //gain option sel off				
    S5K4E5YA_write_cmos_sensor(0x3030, 0x07); //old shut mode				
				                            
//--> This register are for user.				
//+++++++++++++++++++++++++++++++//				
// Integration setting ... 				  
    S5K4E5YA_write_cmos_sensor(0x0200, 0x03); // fine integration time				
    S5K4E5YA_write_cmos_sensor(0x0201, 0x5C);	
	S5K4E5YA_write_cmos_sensor(0x0202, (s5k4e5ya.shutter >> 8) & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0203, s5k4e5ya.shutter & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0204, (s5k4e5ya.sensorGlobalGain & 0xFF00) >> 8); // ANALOG_GAIN_CTRLR
	S5K4E5YA_write_cmos_sensor(0x0205, s5k4e5ya.sensorGlobalGain & 0xFF);
	//S5K4E5YA_SetGain(s5k4e5ya.sensorGlobalGain*s5k4e5ya.ispBaseGain/s5k4e5ya.sensorBaseGain);//set preview gain	
    //S5K4E5YA_write_cmos_sensor(0x0202, 0x03); // coarse integration time				
    //S5K4E5YA_write_cmos_sensor(0x0203, 0xd0);				
    //S5K4E5YA_write_cmos_sensor(0x0204, 0x00); // Analog Gain				
    //S5K4E5YA_write_cmos_sensor(0x0205, 0x20);
    if(s5k4e5ya.maxExposureLines <= S5K4E5YA_PV_PERIOD_LINE_NUMS ) //if shutter > 0x4FC, do not reset frame lenght to 0x4FC
	{			
	    S5K4E5YA_write_cmos_sensor(0x0340, (S5K4E5YA_PV_PERIOD_LINE_NUMS>>8)&0xFF); // Frame Length				
	    S5K4E5YA_write_cmos_sensor(0x0341, S5K4E5YA_PV_PERIOD_LINE_NUMS&0xFF);	
    }			
    S5K4E5YA_write_cmos_sensor(0x0342, (S5K4E5YA_PV_PERIOD_PIXEL_NUMS>>8)&0xFF);   				
    S5K4E5YA_write_cmos_sensor(0x0343, S5K4E5YA_PV_PERIOD_PIXEL_NUMS&0xFF);				
				
//Size Setting ...					
// 1304x980				
    S5K4E5YA_write_cmos_sensor(0x30A9, 0x02); //Horizontal Binning On				
    S5K4E5YA_write_cmos_sensor(0x300E, 0x29); //Vertical Binning On				
    S5K4E5YA_write_cmos_sensor(0x302B, 0x00);				
    S5K4E5YA_write_cmos_sensor(0x3029, 0x74); // DBLR & PLA				
				                          
    S5K4E5YA_write_cmos_sensor(0x0380, 0x00); //x_even_inc 1				
    S5K4E5YA_write_cmos_sensor(0x0381, 0x01);				
    S5K4E5YA_write_cmos_sensor(0x0382, 0x00); //x_odd_inc 1				
    S5K4E5YA_write_cmos_sensor(0x0383, 0x01);				
    S5K4E5YA_write_cmos_sensor(0x0384, 0x00); //y_even_inc 1				
    S5K4E5YA_write_cmos_sensor(0x0385, 0x01);				
    S5K4E5YA_write_cmos_sensor(0x0386, 0x00); //y_odd_inc 3				
    S5K4E5YA_write_cmos_sensor(0x0387, 0x03);				
				                          
    S5K4E5YA_write_cmos_sensor(0x0344, 0x00); //x_addr_start				
    S5K4E5YA_write_cmos_sensor(0x0345, 0x00);				
    S5K4E5YA_write_cmos_sensor(0x0346, 0x00); //y_addr_start				
    S5K4E5YA_write_cmos_sensor(0x0347, 0x00);				
    S5K4E5YA_write_cmos_sensor(0x0348, 0x0A); //x_addr_end				
    S5K4E5YA_write_cmos_sensor(0x0349, 0x2F);				
    S5K4E5YA_write_cmos_sensor(0x034A, 0x07); //y_addr_end				
    S5K4E5YA_write_cmos_sensor(0x034B, 0xA7);				
				                          
    S5K4E5YA_write_cmos_sensor(0x034C, 0x05); //x_output_size_High	 //preview width = 0x518 = 1304			
    S5K4E5YA_write_cmos_sensor(0x034D, 0x18);				
    S5K4E5YA_write_cmos_sensor(0x034E, 0x03); //y_output_size_High	//preview height = 0x3D4 = 980			
    S5K4E5YA_write_cmos_sensor(0x034F, 0xD4);				                         
	
	
	// Operating START
    S5K4E5YA_write_cmos_sensor(0x0100,0x01);
	mDELAY(15);
    SENSOR_DEBUG("exit!");
}

void S5K4E5YACaptureSetting(void)
{	
    SENSOR_DEBUG("enter!");
	//if(s5k4e5ya.sensorMode != SENSOR_MODE_INIT) 
	//{
	//	S5K4E5YA_write_cmos_sensor(0x0100, 0x00);
	//}
	S5K4E5YA_write_cmos_sensor(0x0100,0x00); // stream off
	S5K4E5YA_write_cmos_sensor(0x0103,0x01); // software reset
	
	//--> This registers are for FACTORY ONLY. If you change it without prior notification.
	// YOU are RESPONSIBLE for the FAILURE that will happen in the future.
	//+++++++++++++++++++++++++++++++//
	/////////////////////////////////////////////////////////////
	/////////Should reset global settings after software reset [begin]
	/////////////////////////////////////////////////////////////
	// Analog  setting
	//// CDS timing setting ...
	S5K4E5YA_write_cmos_sensor(0x3000, 0x05);
	S5K4E5YA_write_cmos_sensor(0x3001, 0x03);
	S5K4E5YA_write_cmos_sensor(0x3002, 0x08);
	S5K4E5YA_write_cmos_sensor(0x3003, 0x0A);
	S5K4E5YA_write_cmos_sensor(0x3004, 0x50);
	S5K4E5YA_write_cmos_sensor(0x3005, 0x0E);
	S5K4E5YA_write_cmos_sensor(0x3006, 0x5E);
	S5K4E5YA_write_cmos_sensor(0x3007, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3008, 0x78);
	S5K4E5YA_write_cmos_sensor(0x3009, 0x78);
	S5K4E5YA_write_cmos_sensor(0x300A, 0x50);
	S5K4E5YA_write_cmos_sensor(0x300B, 0x08);
	S5K4E5YA_write_cmos_sensor(0x300C, 0x14);
	S5K4E5YA_write_cmos_sensor(0x300D, 0x00);
	S5K4E5YA_write_cmos_sensor(0x300F, 0x40);
	S5K4E5YA_write_cmos_sensor(0x301B, 0x77);
		   
	//// CDS option setting ...
	S5K4E5YA_write_cmos_sensor(0x3010, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3011, 0x3A);
								  
	S5K4E5YA_write_cmos_sensor(0x3012, 0x30);
	S5K4E5YA_write_cmos_sensor(0x3013, 0xA0);
	S5K4E5YA_write_cmos_sensor(0x3014, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3015, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3016, 0x52);
	S5K4E5YA_write_cmos_sensor(0x3017, 0x94);
	S5K4E5YA_write_cmos_sensor(0x3018, 0x70);
								  
	S5K4E5YA_write_cmos_sensor(0x301D, 0xD4);
								  
	S5K4E5YA_write_cmos_sensor(0x3021, 0x02);
	S5K4E5YA_write_cmos_sensor(0x3022, 0x24);
	S5K4E5YA_write_cmos_sensor(0x3024, 0x40);
	S5K4E5YA_write_cmos_sensor(0x3027, 0x08);
									
	//// Pixel option setting ...		
	S5K4E5YA_write_cmos_sensor(0x301C, 0x06);
	S5K4E5YA_write_cmos_sensor(0x30D8, 0x3F);
									
	//+++++++++++++++++++++++++++++++// 
	// ADLC setting ... 				
	S5K4E5YA_write_cmos_sensor(0x3070, 0x5F);
	S5K4E5YA_write_cmos_sensor(0x3071, 0x00);
	S5K4E5YA_write_cmos_sensor(0x3080, 0x04);
	S5K4E5YA_write_cmos_sensor(0x3081, 0x38);
	S5K4E5YA_write_cmos_sensor(0x302E, 0x0B);
	// MIPI 1304 x 980 @ 30fps				
	// MCLK 24Mhz, Vt_Pix_Clk 122Mhz,  612Mbps per lane				
				
	//+++++++++++++++++++++++++++++++//				
	// MIPI setting				
    S5K4E5YA_write_cmos_sensor(0x30BD, 0x00); //    S5K4E5YA_write_cmos_sensor(0xEL_CCP[0]				
    S5K4E5YA_write_cmos_sensor(0x3084, 0x15); //    S5K4E5YA_write_cmos_sensor(0xYNC Mode				
    S5K4E5YA_write_cmos_sensor(0x30BE, 0x1A); //M_PCLKDIV_AUTO[4], M_DIV_PCLK[3:0]				
    S5K4E5YA_write_cmos_sensor(0x30C1, 0x01); //pack video enable [0]				
    S5K4E5YA_write_cmos_sensor(0x30EE, 0x02); //DPHY enable [1]				
    S5K4E5YA_write_cmos_sensor(0x3111, 0x86); //Embedded data off [5]				
    S5K4E5YA_write_cmos_sensor(0x30E8, 0x0F); //Continuous mode 				
				                            
    S5K4E5YA_write_cmos_sensor(0x30E3, 0x38);	//According to MCLK			
    S5K4E5YA_write_cmos_sensor(0x30E4, 0x40);				
    S5K4E5YA_write_cmos_sensor(0x3113, 0x70);				
    S5K4E5YA_write_cmos_sensor(0x3114, 0x80);				
    S5K4E5YA_write_cmos_sensor(0x3115, 0x7B);				
    S5K4E5YA_write_cmos_sensor(0x3116, 0xC0);				
    S5K4E5YA_write_cmos_sensor(0x30EE, 0x12);
	/////////////////////////////////////////////////////////////
	/////////Should reset global settings after software reset [end]
	/////////////////////////////////////////////////////////////

	// PLL setting ...				
	S5K4E5YA_write_cmos_sensor(0x0305, 0x04);				
	S5K4E5YA_write_cmos_sensor(0x0306, 0x00);				
	S5K4E5YA_write_cmos_sensor(0x0307, 0x6C);//for 0x66, pclk =122.4MHz, for 0x55, pclk = 102MHz, 0x6c=129.6MHz				
	S5K4E5YA_write_cmos_sensor(0x30B5, 0x01);				
	S5K4E5YA_write_cmos_sensor(0x30E2, 0x02); //num lanes[1:0] = 2				
	S5K4E5YA_write_cmos_sensor(0x30F1, 0xB0); // 0xA0, DPHY Band Control				
					
	S5K4E5YA_write_cmos_sensor(0x30BC, 0xA8); // [7]bit : DBLR enable, [6]~[3]bit : DBLR Div				
	        // DBLR clock = Pll output/DBLR Div = 61.2Mhz				
					
	S5K4E5YA_write_cmos_sensor(0x30BF, 0xAB); //outif_enable[7], data_type[5:0](2Bh = bayer 10bit)				
	S5K4E5YA_write_cmos_sensor(0x30C0, 0x80); //video_offset[7:4]				
	S5K4E5YA_write_cmos_sensor(0x30C8, 0x0C); //video_data_length 				
	S5K4E5YA_write_cmos_sensor(0x30C9, 0xBC);				

	S5K4E5YA_write_cmos_sensor(0x3112, 0x00); //gain option sel off				
	S5K4E5YA_write_cmos_sensor(0x3030, 0x07); //old shut mode				
					
	//--> This register are for user.				
	//+++++++++++++++++++++++++++++++//				
	// Integration setting ... 				
	S5K4E5YA_write_cmos_sensor(0x0200, 0x03); // fine integration time				
	S5K4E5YA_write_cmos_sensor(0x0201, 0x5C);		
	S5K4E5YA_write_cmos_sensor(0x0202, (s5k4e5ya.shutter >> 8) & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0203, s5k4e5ya.shutter & 0xFF);
	S5K4E5YA_write_cmos_sensor(0x0204, (s5k4e5ya.sensorGlobalGain & 0xFF00) >> 8); // ANALOG_GAIN_CTRLR
	S5K4E5YA_write_cmos_sensor(0x0205, s5k4e5ya.sensorGlobalGain & 0xFF);
	//S5K4E5YA_write_cmos_sensor(0x0202, 0x03); // coarse integration time				
	//S5K4E5YA_write_cmos_sensor(0x0203, 0xd0);				
	//S5K4E5YA_write_cmos_sensor(0x0204, 0x00); // Analog Gain				
	//S5K4E5YA_write_cmos_sensor(0x0205, 0x20);
	if(s5k4e5ya.maxExposureLines <= S5K4E5YA_FULL_PERIOD_LINE_NUMS ) //if shutter > 0x9EE, do not reset frame lenght to 0x9EE
	{			
	    S5K4E5YA_write_cmos_sensor(0x0340, S5K4E5YA_FULL_PERIOD_LINE_NUMS>>8); // Frame Length				
	    S5K4E5YA_write_cmos_sensor(0x0341, S5K4E5YA_FULL_PERIOD_LINE_NUMS&0xff);	
    }						
	S5K4E5YA_write_cmos_sensor(0x0342, (S5K4E5YA_FULL_PERIOD_PIXEL_NUMS >> 8) & 0xFF);  				
	S5K4E5YA_write_cmos_sensor(0x0343, S5K4E5YA_FULL_PERIOD_PIXEL_NUMS & 0xFF);				
					
	//size setting ...				
	// 2608x1960				
	S5K4E5YA_write_cmos_sensor(0x30A9, 0x03); //Horizontal Binning Off				
	S5K4E5YA_write_cmos_sensor(0x300E, 0x28); //Vertical Binning Off				
	S5K4E5YA_write_cmos_sensor(0x302B, 0x01);				
	S5K4E5YA_write_cmos_sensor(0x3029, 0x34); // DBLR & PLA				

	S5K4E5YA_write_cmos_sensor(0x0380, 0x00); //x_even_inc 1				
	S5K4E5YA_write_cmos_sensor(0x0381, 0x01);				
	S5K4E5YA_write_cmos_sensor(0x0382, 0x00); //x_odd_inc 1				
	S5K4E5YA_write_cmos_sensor(0x0383, 0x01);				
	S5K4E5YA_write_cmos_sensor(0x0384, 0x00); //y_even_inc 1				
	S5K4E5YA_write_cmos_sensor(0x0385, 0x01);				
	S5K4E5YA_write_cmos_sensor(0x0386, 0x00); //y_odd_inc 3				
	S5K4E5YA_write_cmos_sensor(0x0387, 0x01);				
					                
	S5K4E5YA_write_cmos_sensor(0x0344, 0x00); //x_addr_start				
	S5K4E5YA_write_cmos_sensor(0x0345, 0x00);				
	S5K4E5YA_write_cmos_sensor(0x0346, 0x00); //y_addr_start				
	S5K4E5YA_write_cmos_sensor(0x0347, 0x00);				
	S5K4E5YA_write_cmos_sensor(0x0348, 0x0A); //x_addr_end				
	S5K4E5YA_write_cmos_sensor(0x0349, 0x2F);				
	S5K4E5YA_write_cmos_sensor(0x034A, 0x07); //y_addr_end				
	S5K4E5YA_write_cmos_sensor(0x034B, 0xA7);				
					                
	S5K4E5YA_write_cmos_sensor(0x034C, 0x0A); //x_output_size_High	0xA30=2608			
	S5K4E5YA_write_cmos_sensor(0x034D, 0x30);				
	S5K4E5YA_write_cmos_sensor(0x034E, 0x07); //y_output_size_High	0x7A8=1960			
	S5K4E5YA_write_cmos_sensor(0x034F, 0xA8);

	// Operating START
    S5K4E5YA_write_cmos_sensor(0x0100,0x01);
	mDELAY(20);
	SENSOR_DEBUG("exit!");
}


static void S5K4E5YA_Sensor_Init(void)
{
	SENSOR_DEBUG("enter!");
	#if defined( S5K4E5YA_1M_SIZE_PREVIEW )
		S5K4E5YAPreviewSetting(); // make sure after open sensor, there are normal signal output
	#else
		S5K4E5YACaptureSetting(); 
	#endif
    SENSOR_DEBUG("exit!");	
}   /*  S5K4E5YA_Sensor_Init  */

/*************************************************************************
* FUNCTION
*   S5K4E5YAOpen
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

UINT32 S5K4E5YAOpen(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;
	
	SENSOR_DEBUG("enter!");
	S5K4E5YA_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mDELAY(10);

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (S5K4E5YA_read_cmos_sensor(0x0000)<<8)|S5K4E5YA_read_cmos_sensor(0x0001);
		SENSOR_DEBUG("OS5K4E5YA READ ID :%x",sensor_id);
		if(sensor_id != S5K4E5YA_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}

	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.sensorMode = SENSOR_MODE_INIT;
	s5k4e5ya.DummyLines= 0;
	s5k4e5ya.DummyPixels= 0;	
	s5k4e5ya.pvPclk = S5K4E5YA_PV_PCLK/100000;	// 1224
	s5k4e5ya.capPclk = S5K4E5YA_CP_PCLK/100000; // 1020;	
	s5k4e5ya.shutter = 0x3d0;
	s5k4e5ya.maxExposureLines =S5K4E5YA_PV_PERIOD_LINE_NUMS;
	s5k4e5ya.sensorBaseGain = 0x20;
	s5k4e5ya.ispBaseGain = BASEGAIN;
	s5k4e5ya.sensorGlobalGain = (4 * s5k4e5ya.sensorBaseGain);
	s5k4e5ya.bAutoFlickerMode=KAL_FALSE;
	s5k4e5ya.bIsVideoNightMode=KAL_FALSE;
	s5k4e5ya.bIsVideoMode=KAL_FALSE;	
	spin_unlock(&s5k4e5_drv_lock);
	
	S5K4E5YA_Sensor_Init();
	SENSOR_DEBUG("s5k4e5ya.pvPclk=%d,s5k4e5ya.capPclk=%d",s5k4e5ya.pvPclk,s5k4e5ya.capPclk);
	SENSOR_DEBUG("exit!");
	
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   S5K4E5YAGetSensorID
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
UINT32 S5K4E5YAGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
	
	SENSOR_DEBUG("enter!");
	S5K4E5YA_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mDELAY(10);
	
    // check if sensor ID correct
    do {
        *sensorID = (S5K4E5YA_read_cmos_sensor(0x0000)<<8)|S5K4E5YA_read_cmos_sensor(0x0001);        
        if (*sensorID == S5K4E5YA_SENSOR_ID)
        	{
        		SENSORDB("Sensor ID = 0x%04x\n", *sensorID);
            	break; 
        	}
        SENSOR_DEBUG("Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
    } while (retry > 0);

    if (*sensorID != S5K4E5YA_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	SENSOR_DEBUG("exit!");
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   S5K4E5YA_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of S5K4E5YA to change exposure time.
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
void S5K4E5YA_SetShutter(kal_uint16 iShutter)
{
   S5K4E5YA_write_shutter(iShutter);
   return;
}   /*  S5K4E5YA_SetShutter   */



/*************************************************************************
* FUNCTION
*   S5K4E5YA_read_shutter
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
UINT16 S5K4E5YA_read_shutter(void)
{
    UINT16 shutter = 0;

	shutter = ((S5K4E5YA_read_cmos_sensor(0x0202)<<8) & 0xFF00) | (S5K4E5YA_read_cmos_sensor(0x0203)&0xFF);
	SENSOR_DEBUG(" read shutter value is %d",shutter);
	return shutter;
}

UINT16 S5K4E5YA_Read_Linelength(void)
{
    UINT16 iLinelength = 0;
	iLinelength = ((S5K4E5YA_read_cmos_sensor(0x0342)<<8) & 0xFF00) | (S5K4E5YA_read_cmos_sensor(0x0343)&0xFF);
	SENSOR_DEBUG(" read linelength value is %d",iLinelength);
	return iLinelength;
}

/*************************************************************************
* FUNCTION
*   S5K4E5YA_night_mode
*
* DESCRIPTION
*   This function night mode of S5K4E5YA.
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
void S5K4E5YA_NightMode(kal_bool bEnable)
{
}/*	S5K4E5YA_NightMode */



/*************************************************************************
* FUNCTION
*   S5K4E5YAClose
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
UINT32 S5K4E5YAClose(void)
{
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(S5K4E5YAhDrvI2C);
    //e_porting
    return ERROR_NONE;
}	/* S5K4E5YAClose() */

void S5K4E5YASetFlipMirror(kal_int32 imgMirror)
{
	//set mirror & flip
	SENSOR_DEBUG("imgMirror is %d",imgMirror);
    switch (imgMirror)
    {
        case IMAGE_NORMAL: //B
            S5K4E5YA_write_cmos_sensor(0x0101, 0x00);	//Set normal
            break;
		case IMAGE_H_MIRROR: //Gb
        	S5K4E5YA_write_cmos_sensor(0x0101, 0x01);	//Set mirror
        	break;
        case IMAGE_V_MIRROR: //Gr X
            S5K4E5YA_write_cmos_sensor(0x0101, 0x02);	//Set flip
            break;   
        case IMAGE_HV_MIRROR: //R
            S5K4E5YA_write_cmos_sensor(0x0101, 0x03);	//Set mirror and flip
            break;
    }
}


/*************************************************************************
* FUNCTION
*   S5K4E5YAPreview
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
UINT32 S5K4E5YAPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	//SENSOR_DEBUG("enter!");
	SENSOR_DEBUG("Current sensorMode is %d",s5k4e5ya.sensorMode);
	// preview size 
	//if(s5k4e5ya.sensorMode == SENSOR_MODE_INIT) 
	//{
		// do nothing	
		// preview setting already be set in S5K4E5YA_Sensor_Init , so there is no need to set it here again when first enter preview
	//}
	//else
	//{
	//SENSOR_SHUTTER_LOCK(KAL_FALSE);
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	s5k4e5ya.bIsVideoNightMode=KAL_FALSE;
	s5k4e5ya.imgMirror = sensor_config_data->SensorImageMirror;
	s5k4e5ya.DummyLines=0;
	spin_unlock(&s5k4e5_drv_lock);
	S5K4E5YAPreviewSetting();
	//After call S5K4E5YACaptureSetting(), gain is initialized, so we have to reset preview gain here.
	S5K4E5YA_SetGain(s5k4e5ya.sensorGlobalGain*s5k4e5ya.ispBaseGain/s5k4e5ya.sensorBaseGain);//set preview gain	
	//}
	//S5K4E5YA_write_shutter(s5k4e5ya.shutter);
		
	//SENSORDB("[S5K4E5YAPreview] mirror&flip: %d \n",sensor_config_data->SensorImageMirror);
	
	S5K4E5YASetFlipMirror(sensor_config_data->SensorImageMirror);
	
	//set dummy
	S5K4E5YA_SetDummy(0,0);

	//spin_lock(&s5k4e5_drv_lock);
	memcpy(&S5K4E5YASensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	//spin_unlock(&s5k4e5_drv_lock);
	
	SENSOR_DEBUG("exit!");
    return ERROR_NONE;
}	/* S5K4E5YAPreview() */

UINT32 S5K4E5YAZSDPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//SENSOR_DEBUG("enter!");
	SENSOR_DEBUG("s5k4e5ya.sensorMode is %d",s5k4e5ya.sensorMode);
	//Full size preview
	S5K4E5YACaptureSetting();
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.sensorMode = SENSOR_MODE_ZSD_PREVIEW; 
	s5k4e5ya.imgMirror = sensor_config_data->SensorImageMirror;
	s5k4e5ya.DummyPixels=0;
	s5k4e5ya.DummyLines=0;	
	spin_unlock(&s5k4e5_drv_lock);
	S5K4E5YASetFlipMirror(sensor_config_data->SensorImageMirror);
	S5K4E5YA_SetDummy( s5k4e5ya.DummyPixels, s5k4e5ya.DummyLines);
	//spin_lock(&s5k4e5_drv_lock);
	memcpy(&S5K4E5YASensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	//spin_unlock(&s5k4e5_drv_lock);
	//SENSOR_DEBUG("exit!");
    return ERROR_NONE;
}

UINT32 S5K4E5YACapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
//    kal_uint16 shutter = s5k4e5ya.pvShutter; //s5k4e5ya.shutter;
 //   kal_uint16 gain = s5k4e5ya.pvSensorGlobalGain/2;
 	kal_uint16 shutter = s5k4e5ya.shutter;
	kal_uint16 pv_line_length , cap_line_length;
	
	//SENSOR_DEBUG("enter!");
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.bAutoFlickerMode=KAL_FALSE;
	spin_unlock(&s5k4e5_drv_lock);
	//u4PreviewShutter=s5k4e5ya.shutter;//save preview_shutter for normal shot, save capture_shutter for burst short, HDR
	//if(SENSOR_MODE_CAPTURE!=s5k4e5ya.sensorMode) //For burst shot case, do not recalculate shutter
	{
		pv_line_length = S5K4E5YA_PV_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels;
		cap_line_length = S5K4E5YA_FULL_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels;	
		SENSOR_DEBUG("preview linelength=%d,capture linelength=%d,preview shutter=%d",pv_line_length,cap_line_length,shutter);
		shutter = shutter * (pv_line_length) / (cap_line_length);
		shutter = shutter * s5k4e5ya.capPclk / s5k4e5ya.pvPclk;
		//preview binning is actually subsample, so no need to do shutter*=2.
		if(shutter < 3)
	    shutter = 3;
		s5k4e5ya.shutter=shutter;
	}
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.sensorMode = SENSOR_MODE_CAPTURE;
	spin_unlock(&s5k4e5_drv_lock);
	
	S5K4E5YACaptureSetting();
		
	S5K4E5YA_write_shutter(shutter);
	//After call S5K4E5YACaptureSetting(), gain is initialized, so we have to reset preview gain here.
	S5K4E5YA_SetGain(s5k4e5ya.sensorGlobalGain*s5k4e5ya.ispBaseGain/s5k4e5ya.sensorBaseGain);//set preview gain	
	
	
	//set mirror & flip
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.imgMirror = sensor_config_data->SensorImageMirror;
	s5k4e5ya.DummyPixels =0;
	s5k4e5ya.DummyLines  =0;
	spin_unlock(&s5k4e5_drv_lock);
	S5K4E5YASetFlipMirror(sensor_config_data->SensorImageMirror);
	
	// set dummy
	S5K4E5YA_SetDummy( s5k4e5ya.DummyPixels, s5k4e5ya.DummyLines);
	
	
	
	//spin_lock(&s5k4e5_drv_lock);
    memcpy(&S5K4E5YASensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	//spin_unlock(&s5k4e5_drv_lock);
	SENSOR_DEBUG("s5k4e5ya.shutter =%d, s5k4e5ya.sensorGlobalGain = %d",s5k4e5ya.shutter,s5k4e5ya.sensorGlobalGain);
	//SENSOR_DEBUG("exit!");
    return ERROR_NONE;
}	/* S5K4E5YACapture() */

UINT32 S5K4E5YAGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    SENSOR_DEBUG("enter!");
	#if defined( S5K4E5YA_1M_SIZE_PREVIEW )
		pSensorResolution->SensorPreviewWidth	= S5K4E5YA_IMAGE_SENSOR_PV_WIDTH;
    	pSensorResolution->SensorPreviewHeight	= S5K4E5YA_IMAGE_SENSOR_PV_HEIGHT;
	#else
    	pSensorResolution->SensorPreviewWidth	= S5K4E5YA_IMAGE_SENSOR_FULL_WIDTH; //S5K4E5YAMIPI_REAL_PV_WIDTH;
    	pSensorResolution->SensorPreviewHeight	= S5K4E5YA_IMAGE_SENSOR_FULL_HEIGHT; //S5K4E5YAMIPI_REAL_PV_HEIGHT;
    #endif
    pSensorResolution->SensorFullWidth		= S5K4E5YA_IMAGE_SENSOR_FULL_WIDTH; //S5K4E5YAMIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= S5K4E5YA_IMAGE_SENSOR_FULL_HEIGHT; //S5K4E5YAMIPI_REAL_CAP_HEIGHT;
	//SENSOR_DEBUG("exit!");
    return ERROR_NONE;
}   /* S5K4E5YAGetResolution() */

UINT32 S5K4E5YAGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	//SENSOR_DEBUG("enter!");
	switch(ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		pSensorInfo->SensorPreviewResolutionX= S5K4E5YA_IMAGE_SENSOR_FULL_WIDTH; //S5K4E5YA_IMAGE_SENSOR_FULL_WIDTH;
		pSensorInfo->SensorPreviewResolutionY=S5K4E5YA_IMAGE_SENSOR_FULL_HEIGHT ;//S5K4E5YAMIPI_REAL_CAP_HEIGHT;
		pSensorInfo->SensorCameraPreviewFrameRate=15;
		break;
		default:
		pSensorInfo->SensorPreviewResolutionX= S5K4E5YA_IMAGE_SENSOR_PV_WIDTH; //S5K4E5YAMIPI_REAL_PV_WIDTH;
		pSensorInfo->SensorPreviewResolutionY= S5K4E5YA_IMAGE_SENSOR_PV_HEIGHT; //S5K4E5YAMIPI_REAL_PV_HEIGHT;
		pSensorInfo->SensorCameraPreviewFrameRate=30;
		break;
	}
	pSensorInfo->SensorFullResolutionX= S5K4E5YA_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= S5K4E5YA_IMAGE_SENSOR_FULL_HEIGHT;
	
	spin_lock(&s5k4e5_drv_lock);
	s5k4e5ya.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&s5k4e5_drv_lock);
	SENSOR_DEBUG("s5k4e5ya.imgMirror=%d", s5k4e5ya.imgMirror );

	switch(s5k4e5ya.imgMirror)
	{
		case IMAGE_NORMAL: 
			 pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_B;
			 break;
		case IMAGE_H_MIRROR: 
			 pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_Gb;
			 break;
	    case IMAGE_V_MIRROR: 
			 pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_Gr;
			 break;
	    case IMAGE_HV_MIRROR: 
			 pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_R;
			 break;
		default:
			break;
	}
	//pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_RAW_R;
	
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=15;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
   	//pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 2;
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

    pSensorInfo->CaptureDelayFrame = 2; 
    pSensorInfo->PreviewDelayFrame = 1; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;      
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0 ;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;// 1	
	SENSOR_DEBUG("ScenarioId = %d",ScenarioId);   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	5;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = S5K4E5YA_PV_X_START; 
        pSensorInfo->SensorGrabStartY = S5K4E5YA_PV_Y_START;           		
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
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	5;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = S5K4E5YA_FULL_X_START;	//2*S5K4E5YA_IMAGE_SENSOR_PV_STARTX; 
        pSensorInfo->SensorGrabStartY = S5K4E5YA_FULL_Y_START;	//2*S5K4E5YA_IMAGE_SENSOR_PV_STARTY;          			
        pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
        pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
        pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
        pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
        pSensorInfo->SensorPacketECCOrder = 1;
        break;
        default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = 1; 
        pSensorInfo->SensorGrabStartY = 1;             
        break;
    }
	//spin_lock(&s5k4e5_drv_lock);
    memcpy(pSensorConfigData, &S5K4E5YASensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	//spin_unlock(&s5k4e5_drv_lock);
	//SENSOR_DEBUG("exit!");
    return ERROR_NONE;
}   /* S5K4E5YAGetInfo() */


UINT32 S5K4E5YAControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	//SENSOR_DEBUG("enter!");	
	spin_lock(&s5k4e5_drv_lock);
	CurrentScenarioId = ScenarioId;
	spin_unlock(&s5k4e5_drv_lock);
	SENSOR_DEBUG("CurrentScenarioId = %d",CurrentScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			spin_lock(&s5k4e5_drv_lock);
			s5k4e5ya.bIsVideoMode=KAL_FALSE;
			spin_unlock(&s5k4e5_drv_lock);
			S5K4E5YAPreview(pImageWindow, pSensorConfigData);
            break;   
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			spin_lock(&s5k4e5_drv_lock);
			s5k4e5ya.bIsVideoMode=KAL_TRUE;
			spin_unlock(&s5k4e5_drv_lock);
            S5K4E5YAPreview(pImageWindow, pSensorConfigData);
            break;   
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
            S5K4E5YACapture(pImageWindow, pSensorConfigData);
            break;	
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			S5K4E5YAZSDPreview(pImageWindow, pSensorConfigData);
		break;
        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
	//SENSOR_DEBUG("exit!");
    return TRUE;
} /* S5K4E5YAControl() */

UINT32 S5K4E5YASetVideoMode(UINT16 u2FrameRate)
{
   /*
   		night mode:15fps
   		normal mode : 30fps @ 1M SIZE PREVIEW
     */
    kal_uint32 MAX_Frame_Length =0;
   kal_uint16 u2DummyLines=0;
    SENSOR_DEBUG("enter! frame rate = %d", u2FrameRate);
	
	if(u2FrameRate >30)
	{
		SENSOR_DEBUG("[error] video frame rate exceeds 30!");
		u2FrameRate = 30;
	}
	else if(u2FrameRate < 10)
	{
		SENSOR_DEBUG("[error] video frame rate is below 10!");
		//u2FrameRate = 10;
		return; //to avoid case: u2FrameRate=0
	}
	if(30==u2FrameRate)
	{
		spin_lock(&s5k4e5_drv_lock);
		s5k4e5ya.bIsVideoNightMode=KAL_FALSE;
		spin_unlock(&s5k4e5_drv_lock);

	}
	else if(15==u2FrameRate)
	{
		spin_lock(&s5k4e5_drv_lock);
		s5k4e5ya.bIsVideoNightMode=KAL_TRUE;
		spin_unlock(&s5k4e5_drv_lock);
	}
			
	SENSOR_DEBUG("s5k4e5ya.sensorMode = %d", s5k4e5ya.sensorMode);
    if(s5k4e5ya.sensorMode == SENSOR_MODE_PREVIEW)
    {
		MAX_Frame_Length = s5k4e5ya.pvPclk * 100000/(S5K4E5YA_PV_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels)/u2FrameRate;
		SENSOR_DEBUG("MAX_Frame_Length = %d", MAX_Frame_Length);

		if((MAX_Frame_Length <=S5K4E5YA_PV_PERIOD_LINE_NUMS))
		{
			MAX_Frame_Length = S5K4E5YA_PV_PERIOD_LINE_NUMS;
			SENSOR_DEBUG("current fps = %d", s5k4e5ya.pvPclk * 100000/(S5K4E5YA_PV_PERIOD_PIXEL_NUMS)/S5K4E5YA_PV_PERIOD_LINE_NUMS);
		
		}
		u2DummyLines = MAX_Frame_Length - S5K4E5YA_PV_PERIOD_LINE_NUMS;
	
		S5K4E5YA_SetDummy(s5k4e5ya.DummyPixels,u2DummyLines);
    }
	
	SENSOR_DEBUG("exit!");
    return 0;
}

UINT32 S5K4E5YASetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//kal_uint32 pv_max_frame_rate_lines = S5K4E5YA_MAX_EXPOSURE_LINES;
	if(bEnable)
	{
		spin_lock(&s5k4e5_drv_lock);
		s5k4e5ya.bAutoFlickerMode=KAL_TRUE;
		spin_unlock(&s5k4e5_drv_lock);
	}
	else
	{
		spin_lock(&s5k4e5_drv_lock);
		s5k4e5ya.bAutoFlickerMode=KAL_FALSE;
		spin_unlock(&s5k4e5_drv_lock);
		SENSOR_DEBUG("IsVideo Mode=%d",s5k4e5ya.bIsVideoMode);
		if(KAL_TRUE== s5k4e5ya.bIsVideoMode) 
		{
			if(KAL_TRUE==s5k4e5ya.bIsVideoNightMode)
			{
				S5K4E5YASetMaxFrameRate(150);
			}
			else
			{
				S5K4E5YASetMaxFrameRate(300);
			}
		}
	}
    SENSORDB("[S5K4E5YASetAutoFlickerMode] frame rate(10 base) = %d %d\n", bEnable, u2FrameRate);
    return 0;
}

static UINT32 S5K4E5YASetMaxFrameRate(UINT16 u2FrameRate)
{
	kal_int16 dummy_line;
	kal_int16 FrameHeight = s5k4e5ya.maxExposureLines;
	unsigned long flags;
	//kal_uint32 u4Pclk=S5K4E5YA_PV_PCLK;
		
	//SENSOR_DEBUG("u2FrameRate=%d,s5k4e5ya.maxExposureLines=%d",u2FrameRate,s5k4e5ya.maxExposureLines);

	if(SENSOR_MODE_PREVIEW==s5k4e5ya.sensorMode)
	{
		FrameHeight= (10 * S5K4E5YA_PV_PCLK) / u2FrameRate / S5K4E5YA_PV_PERIOD_PIXEL_NUMS;
		dummy_line = FrameHeight - S5K4E5YA_PV_PERIOD_LINE_NUMS;

	}
	else if(SENSOR_MODE_ZSD_PREVIEW==s5k4e5ya.sensorMode)
	{
		FrameHeight= (10 * S5K4E5YA_ZSD_PCLK) / u2FrameRate / S5K4E5YA_ZSD_PERIOD_PIXEL_NUMS;
		dummy_line = FrameHeight - S5K4E5YA_ZSD_PERIOD_LINE_NUMS;
	}
	spin_lock_irqsave(&s5k4e5_drv_lock,flags);
	s5k4e5ya.maxExposureLines = FrameHeight;
	spin_unlock_irqrestore(&s5k4e5_drv_lock,flags);
	SENSOR_DEBUG("dummy_line=%d",dummy_line);
	dummy_line = (dummy_line>0?dummy_line:0);
	S5K4E5YA_SetDummy(0, dummy_line); /* modify dummy_pixel must gen AE table again */	
}

UINT32 S5K4E5YASetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[S5K4E5YASetTestPatternMode] Test pattern enable:%d\n", bEnable);

    return TRUE;
}

UINT32 S5K4E5YAFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
	//SENSOR_DEBUG("enter!");
	//SENSOR_DEBUG("FeatureId is %d",FeatureId);
    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= S5K4E5YA_IMAGE_SENSOR_FULL_WIDTH;;
            *pFeatureReturnPara16= S5K4E5YA_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        			switch(CurrentScenarioId)
        			{
        				case MSDK_SCENARIO_ID_CAMERA_ZSD:
							*pFeatureReturnPara16++= S5K4E5YA_ZSD_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels;  
		            		*pFeatureReturnPara16= S5K4E5YA_ZSD_PERIOD_LINE_NUMS + s5k4e5ya.DummyLines;	
		            		SENSOR_DEBUG("sensor period:%d ,%d\n", *(pFeatureReturnPara16-1),*pFeatureReturnPara16); 
							break;
						case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
       		 			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		            		*pFeatureReturnPara16++= S5K4E5YA_FULL_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels;  
		            		*pFeatureReturnPara16= S5K4E5YA_FULL_PERIOD_LINE_NUMS + s5k4e5ya.DummyLines;	
		            		SENSOR_DEBUG("sensor period:%d ,%d\n", *(pFeatureReturnPara16-1),*pFeatureReturnPara16);
		            		*pFeatureParaLen=4;        				
        					break;
        			
        				default:	
		            		*pFeatureReturnPara16++= S5K4E5YA_PV_PERIOD_PIXEL_NUMS + s5k4e5ya.DummyPixels;  
		            		*pFeatureReturnPara16= S5K4E5YA_PV_PERIOD_LINE_NUMS + s5k4e5ya.DummyLines;
		            		SENSOR_DEBUG("sensor period:%d ,%d\n", *(pFeatureReturnPara16-1),*pFeatureReturnPara16);
		            		*pFeatureParaLen=4;
	            			break;
          				}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        			switch(CurrentScenarioId)
        			{
						case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
							*pFeatureReturnPara32 = S5K4E5YA_CP_PCLK; 
		            	 	*pFeatureParaLen=4;		         	
		         			 break;
        				case MSDK_SCENARIO_ID_CAMERA_ZSD:
		            	 	*pFeatureReturnPara32 = S5K4E5YA_ZSD_PCLK; 
		            	 	*pFeatureParaLen=4;		         	
		         			 break;
		         		default:
		            		*pFeatureReturnPara32 = S5K4E5YA_PV_PCLK; 
		            		*pFeatureParaLen=4;
		            		break;
		        	}
					SENSOR_DEBUG("sensor pclk = %d",*pFeatureReturnPara32);
		    break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            S5K4E5YA_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            S5K4E5YA_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            S5K4E5YA_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //S5K4E5YA_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            S5K4E5YA_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = S5K4E5YA_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR; //FACTORY_END_ADDR is not a big value
			spin_lock(&s5k4e5_drv_lock);
            for (i=0;i<SensorRegNumber;i++)
            {
                S5K4E5YASensorCCT[i].Addr=*pFeatureData32++;
                S5K4E5YASensorCCT[i].Para=*pFeatureData32++;
            }
			spin_unlock(&s5k4e5_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR; 
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;		
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K4E5YASensorCCT[i].Addr;
                *pFeatureData32++=S5K4E5YASensorCCT[i].Para;
            }		
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END; //ENGINEER_END is not a big value
			spin_lock(&s5k4e5_drv_lock);
            for (i=0;i<SensorRegNumber;i++)
            {
                S5K4E5YASensorReg[i].Addr=*pFeatureData32++;
                S5K4E5YASensorReg[i].Para=*pFeatureData32++;
            }
			spin_unlock(&s5k4e5_drv_lock);
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K4E5YASensorReg[i].Addr;
                *pFeatureData32++=S5K4E5YASensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=S5K4E5YA_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, S5K4E5YASensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, S5K4E5YASensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &S5K4E5YASensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            S5K4E5YA_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            S5K4E5YA_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=S5K4E5YA_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            S5K4E5YA_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            S5K4E5YA_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            S5K4E5YA_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B; //SENSOR_OUTPUT_FORMAT_RAW_R;
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
            S5K4E5YASetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            S5K4E5YAGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            S5K4E5YASetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            S5K4E5YASetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
        default:
            break;
    }
	//SENSOR_DEBUG("exit!");
    return ERROR_NONE;
}	/* S5K4E5YAFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncS5K4E5YA=
{
    S5K4E5YAOpen,
    S5K4E5YAGetInfo,
    S5K4E5YAGetResolution,
    S5K4E5YAFeatureControl,
    S5K4E5YAControl,
    S5K4E5YAClose
};

UINT32 S5K4E5YA_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncS5K4E5YA;

    return ERROR_NONE;
}   /* SensorInit() */

