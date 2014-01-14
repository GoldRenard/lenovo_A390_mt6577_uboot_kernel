/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   YUSU
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
 * 03 15 2011 koli.lin
 * [ALPS00034474] [Need Patch] [Volunteer Patch]
 * Move sensor driver current setting to isp of middleware.
 *
 * 10 12 2010 koli.lin
 * [ALPS00127101] [Camera] AE will flash
 * [Camera]Create Vsync interrupt to handle the exposure time, sensor gain and raw gain control.
 *
 * 08 27 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Check in AD5820 Constant AF function.
 *
 * 08 26 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Add AD5820 Lens driver function.
 * must disable SWIC and bus log, otherwise the lens initial time take about 30 second.(without log about 3 sec)
 *
 * 08 19 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Merge dual camera relative settings. Main OV5640, SUB O7675 ready.
 *
 * 08 18 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Mmodify ISP setting and add OV5640 sensor driver.
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define WINMO_USE           0

 #if WINMO_USE
#include <windows.h>
#include <memory.h>
#include <nkintr.h>
#include <ceddk.h>
#include <ceddk_exp.h>

#include "CameraCustomized.h"

#include "kal_release.h"
#include "i2c_exp.h"
#include "gpio_exp.h"
#include "msdk_exp.h"
#include "msdk_sensor_exp.h"
#include "msdk_isp_exp.h"
#include "base_regs.h"
#include "Sensor.h"
#include "camera_sensor_para.h"
#else 
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov5640yuv_Sensor.h"
#include "ov5640yuv_Camera_Sensor_para.h"
#include "ov5640yuv_CameraCustomized.h"

#include "kd_camera_feature.h"
#endif 

#define PFX "[kd_camera_hw] [CAMERA SENSOR]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(PFX "[tong] %s: " fmt, __FUNCTION__ ,##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#endif

void OV5640YUV_Set_Mirror_Flip(kal_uint8 image_mirror);
static void OV5640_FOCUS_AD5820_Cancel_Focus();

/*
DBGPARAM dpCurSettings = {
    TEXT("Sensor"), {
        TEXT("Preview"),TEXT("Capture"),TEXT("Init"),TEXT("Error"),
        TEXT("Gain"),TEXT("Shutter"),TEXT("Undef"),TEXT("Undef"),
        TEXT("Undef"),TEXT("Undef"),TEXT("Undef"),TEXT("Undef"),
        TEXT("Undef"),TEXT("Undef"),TEXT("Undef"),TEXT("Undef")},
    0x00FF	// ZONE_INIT | ZONE_WARNING | ZONE_ERROR
};

*/
kal_bool  OV5640YUV_MPEG4_encode_mode = KAL_FALSE;
kal_uint16  OV5640YUV_sensor_gain_base=0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 OV5640YUV_MAX_EXPOSURE_LINES = OV5640_PV_PERIOD_LINE_NUMS-4;
kal_uint8  OV5640YUV_MIN_EXPOSURE_LINES = 2;
kal_uint32 OV5640YUV_isp_master_clock;
kal_uint16 OV5640YUV_CURRENT_FRAME_LINES = OV5640_PV_PERIOD_LINE_NUMS;

static kal_uint16 OV5640YUV_dummy_pixels=0, OV5640YUV_dummy_lines=0;
kal_uint16 OV5640YUV_PV_dummy_pixels=0,OV5640YUV_PV_dummy_lines=0;

kal_uint8 OV5640YUV_sensor_write_I2C_address = OV5640_WRITE_ID;
kal_uint8 OV5640YUV_sensor_read_I2C_address = OV5640_READ_ID;

static kal_uint32 OV5640YUV_zoom_factor = 0; 

//add by lingnan for af status
static UINT8 STA_FOCUS = 0x8F; 
//static kal_uint32 MAC = 255;
//static kal_uint32 INF = 0;
static kal_uint32 AF_XS = 64;//version0.21, aug.2009
static kal_uint32 AF_YS = 48;//version0.21, aug.2009
//static kal_bool AF_INIT = FALSE;
static UINT8 ZONE[4] = {24, 18, 40, 30};////version0.21, aug.2009,center 4:3 window


#define LOG_TAG "[OV5640Yuv]"
#define PK_DBG(fmt, arg...) printk( LOG_TAG  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT

kal_uint16 OV5640YUV_g_iDummyLines = 28; 

#if WINMO_USE
HANDLE OV5640YUVhDrvI2C;
I2C_TRANSACTION OV5640YUVI2CConfig;
#endif 

UINT8 OV5640YUVPixelClockDivider=0;
kal_uint32 OV5640YUV_sensor_pclk=52000000;;
kal_uint32 OV5640YUV_PV_pclk = 5525; 

kal_uint32 OV5640YUV_CAP_pclk = 6175;

kal_uint16 OV5640YUV_pv_exposure_lines=0x100,OV5640YUV_g_iBackupExtraExp = 0,OV5640YUV_extra_exposure_lines = 0;

kal_uint16 OV5640YUV_sensor_id=0;

MSDK_SENSOR_CONFIG_STRUCT OV5640YUVSensorConfigData;

kal_uint32 OV5640YUV_FAC_SENSOR_REG;
kal_uint16 OV5640YUV_sensor_flip_value;


/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT OV5640YUVSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV5640YUVSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/



////////////////////////////////////////////////////////////////
typedef enum
{
  OV5640_720P,       //1M 1280x960
  OV5640_5M,     //5M 2592x1944
} OV5640_RES_TYPE;
OV5640_RES_TYPE OV5640YUV_g_RES=OV5640_720P;

typedef enum
{
  OV5640_MODE_PREVIEW,  //1M  	1280x960
  OV5640_MODE_CAPTURE   //5M    2592x1944
} OV5640_MODE;
OV5640_MODE g_iOV5640YUV_Mode=OV5640_MODE_PREVIEW;

//[agold][sea][add for iwb]
typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
} OV5640YUVStatus;
OV5640YUVStatus OV5640YUVCurrentStatus;
//[agold][sea][end]

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId); 
#define OV5640YUV_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV5640_WRITE_ID)
#define OV5640YUV_burst_write_cmos_sensor(pData, bytes)  iBurstWriteReg(pData, bytes, OV5640_WRITE_ID)

static UINT32 g_sensorAfStatus = 0;

static unsigned int iCapTmp = 0; //chenlang add, for multi shot AE issue

#define PROFILE 1

#if PROFILE 
static struct timeval OV5640YUV_ktv1, OV5640YUV_ktv2; 
inline void OV5640YUV_imgSensorProfileStart(void)
{
    do_gettimeofday(&OV5640YUV_ktv1);    
}

inline void OV5640YUV_imgSensorProfileEnd(char *tag)
{
    unsigned long TimeIntervalUS;    
    do_gettimeofday(&OV5640YUV_ktv2);

    TimeIntervalUS = (OV5640YUV_ktv2.tv_sec - OV5640YUV_ktv1.tv_sec) * 1000000 + (OV5640YUV_ktv2.tv_usec - OV5640YUV_ktv1.tv_usec); 
    PK_DBG("[%s]Profile = %lu\n",tag, TimeIntervalUS);
}
#else 
inline static void OV5640YUV_imgSensorProfileStart() {}
inline static void OV5640YUV_imgSensorProfileEnd(char *tag) {}
#endif 

#if 0
OV5640YUV_write_cmos_sensor(addr, para) 
{
    u16 data = 0 ; 

    OV5640YUV_imgSensorProfileStart();
    iWriteReg((u16) addr , (u32) para , 1, OV5640_WRITE_ID);
    OV5640YUV_imgSensorProfileEnd("OV5640YUV_write_cmos_sensor");


    iReadReg((u16)addr, (u8*) &data, OV5640_WRITE_ID); 

    PK_DBG("Write addr = %x , wdata = %x,  rdata= %x\n", addr, para, data); 
}
#endif 

kal_uint16 OV5640YUV_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV5640_WRITE_ID);
    return get_byte;
}


#define Sleep(ms) mdelay(ms)

#if WINMO_USE
void OV5640YUV_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    OV5640YUVI2CConfig.operation=I2C_OP_WRITE;
    OV5640YUVI2CConfig.slaveAddr=OV5640YUV_sensor_write_I2C_address>>1;
    OV5640YUVI2CConfig.transfer_num=1;	/* TRANSAC_LEN */
    OV5640YUVI2CConfig.transfer_len=3;
    OV5640YUVI2CConfig.buffer[0]=(UINT8)(addr>>8);
    OV5640YUVI2CConfig.buffer[1]=(UINT8)(addr&0xFF);
    OV5640YUVI2CConfig.buffer[2]=(UINT8)para;
    DRV_I2CTransaction(OV5640YUVhDrvI2C, &OV5640YUVI2CConfig);
}    /* OV5640YUV_write_cmos_sensor() */
#endif 

#if WINMO_USE
kal_uint32 OV5640YUV_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    
    OV5640YUVI2CConfig.operation=I2C_OP_WRITE;
    OV5640YUVI2CConfig.slaveAddr=OV5640YUV_sensor_write_I2C_address>>1;
    OV5640YUVI2CConfig.transfer_num=1;	/* TRANSAC_LEN */
    OV5640YUVI2CConfig.transfer_len=2;
    OV5640YUVI2CConfig.buffer[0]=(UINT8)(addr>>8);
    OV5640YUVI2CConfig.buffer[1]=(UINT8)(addr&0xFF);
    DRV_I2CTransaction(OV5640YUVhDrvI2C, &OV5640YUVI2CConfig);
    
    OV5640YUVI2CConfig.operation=I2C_OP_READ;
    OV5640YUVI2CConfig.slaveAddr=OV5640YUV_sensor_read_I2C_address>>1;
    OV5640YUVI2CConfig.transfer_num=1;	/* TRANSAC_LEN */
    OV5640YUVI2CConfig.transfer_len=1;
    DRV_I2CTransaction(OV5640YUVhDrvI2C, &OV5640YUVI2CConfig);
    get_byte=OV5640YUVI2CConfig.buffer[0];
    return get_byte;
}	/* OV5640YUV_read_cmos_sensor() */
#endif 

static atomic_t OV5640_SetShutter_Flag; 
static wait_queue_head_t OV5640_SetShutter_waitQueue;
void OV5640YUV_write_shutter(kal_uint16 shutter)
{
    kal_uint16 iExp = shutter;
    kal_uint16 OV5640_g_iExtra_ExpLines = 0 ;
//    kal_uint16 OV5640_g_bXGA_Mode = 0; 
    int timeOut = 0; 

    if (atomic_read(&OV5640_SetShutter_Flag) == 1) {
        timeOut = wait_event_interruptible_timeout(
            OV5640_SetShutter_waitQueue, atomic_read(&OV5640_SetShutter_Flag) == 0, 1 * HZ);        
        if (timeOut == 0) {
            PK_DBG("[OV5640YUV_SetGain] Set Gain Wait Queue time out \n"); 
            return; 
        }
    }    
    atomic_set(&OV5640_SetShutter_Flag, 1); 

    if (OV5640YUV_g_RES == OV5640_720P) {
        if (iExp <= OV5640_PV_EXPOSURE_LIMITATION) {
            OV5640_g_iExtra_ExpLines = 0;
        }else {
            OV5640_g_iExtra_ExpLines = iExp - OV5640_PV_EXPOSURE_LIMITATION ;
        }

    }else {
        if (iExp <= OV5640_FULL_EXPOSURE_LIMITATION) {
            OV5640_g_iExtra_ExpLines = 0;
        }else {
            OV5640_g_iExtra_ExpLines = iExp - OV5640_FULL_EXPOSURE_LIMITATION;
        }
    }

    PK_DBG("Set Extra-line = %d, iExp = %d \n", OV5640_g_iExtra_ExpLines, iExp);     

//    OV5640YUV_write_cmos_sensor(0x3212, 0x01); 
    if (OV5640YUV_MPEG4_encode_mode != TRUE) {
    OV5640YUV_write_cmos_sensor(0x350c, OV5640_g_iExtra_ExpLines >> 8);
    OV5640YUV_write_cmos_sensor(0x350d, OV5640_g_iExtra_ExpLines & 0x00FF);
    }
    
    OV5640YUV_write_cmos_sensor(0x3500, (iExp >> 12) & 0xFF);
    OV5640YUV_write_cmos_sensor(0x3501, (iExp >> 4 ) & 0xFF);
    OV5640YUV_write_cmos_sensor(0x3502, (iExp <<4 ) & 0xFF);
//    OV5640YUV_write_cmos_sensor(0x3212, 0x11); 
//    OV5640YUV_write_cmos_sensor(0x3212, 0xa1);     

    OV5640YUV_g_iBackupExtraExp = OV5640_g_iExtra_ExpLines;    
    atomic_set(&OV5640_SetShutter_Flag, 0);
    wake_up_interruptible(&OV5640_SetShutter_waitQueue);    
}   /* write_OV5640_shutter */

static kal_uint16 OV5640YUVReg2Gain(const kal_uint8 iReg)
{
	return iReg;
	/*
    kal_uint8 iI;
    kal_uint16 iGain = BASEGAIN;    // 1x-gain base

    // Range: 1x to 32x
    // Gain = (GAIN[7] + 1) * (GAIN[6] + 1) * (GAIN[5] + 1) * (GAIN[4] + 1) * (1 + GAIN[3:0] / 16)
    for (iI = 7; iI >= 4; iI--) {
        iGain *= (((iReg >> iI) & 0x01) + 1);
    }

    return iGain +  iGain * (iReg & 0x0F) / 16;
    */
}

static kal_uint8 OV5640YUVGain2Reg(const kal_uint16 iGain)
{
	return iGain;
/*
    kal_uint8 iReg = 0x00;

    if (iGain < 2 * BASEGAIN) {
        // Gain = 1 + GAIN[3:0](0x00) / 16
        //iReg = 16 * (iGain - BASEGAIN) / BASEGAIN;
        iReg = 16 * iGain / BASEGAIN - 16; 
    }else if (iGain < 4 * BASEGAIN) {
        // Gain = 2 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x10;
        //iReg |= 8 * (iGain - 2 * BASEGAIN) / BASEGAIN;
        iReg |= (8 *iGain / BASEGAIN - 16); 
    }else if (iGain < 8 * BASEGAIN) {
        // Gain = 4 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x30;
        //iReg |= 4 * (iGain - 4 * BASEGAIN) / BASEGAIN;
        iReg |= (4 * iGain / BASEGAIN - 16); 
    }else if (iGain < 16 * BASEGAIN) {
        // Gain = 8 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x70;
        //iReg |= 2 * (iGain - 8 * BASEGAIN) / BASEGAIN;
        iReg |= (2 * iGain / BASEGAIN - 16); 
    }else if (iGain < 32 * BASEGAIN) {
        // Gain = 16 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0xF0;
        //iReg |= (iGain - 16 * BASEGAIN) / BASEGAIN;
        iReg |= (iGain / BASEGAIN - 16); 
    }else {
        ASSERT(0);
    }

    return iReg;
*/
}

/*************************************************************************
* FUNCTION
*    OV5640YUV_SetGain
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
//! Due to the OV5640 set gain will happen race condition. 
//! It need to use a critical section to protect it. 
static atomic_t OV5640_SetGain_Flag; 
static wait_queue_head_t OV5640_SetGain_waitQueue;
void OV5640YUV_SetGain(UINT16 iGain)
{
    kal_uint8 iReg;
    int timeOut = 0; 

    //OV5640YUV_imgSensorProfileStart();
    //PK_DBG("[OV5640YUV_SetGain] E Gain = %d \n", iGain); 
    if (atomic_read(&OV5640_SetGain_Flag) == 1) {
        timeOut = wait_event_interruptible_timeout(
            OV5640_SetGain_waitQueue, atomic_read(&OV5640_SetGain_Flag) == 0, 1 * HZ);        
        if (timeOut == 0) {
            PK_DBG("[OV5640YUV_SetGain] Set Gain Wait Queue time out \n"); 
            return; 
        }
    }    
    atomic_set(&OV5640_SetGain_Flag, 1); 
    //iReg = OV5640YUVGain2Reg(iGain);
    //PK_DBG("transfer gain 0x%x(%d) to 0x%x(%d)\n",iGain,iGain,iReg,iReg);
    //! For OV5640 sensor, the set gain don't have double buffer,  
    //! it needs use group write to write sensor gain 
    OV5640YUV_write_cmos_sensor(0x3212, 0x00); 
    //OV5640YUV_write_cmos_sensor(0x350B, (kal_uint32)iReg);
    OV5640YUV_write_cmos_sensor(0x350B, (kal_uint32)iGain);
    OV5640YUV_write_cmos_sensor(0x3212, 0x10); 
    OV5640YUV_write_cmos_sensor(0x3212, 0xA0); 
    
    //OV5640YUV_imgSensorProfileEnd("OV5640YUV_SetGain"); 
    //PK_DBG("Gain = %x\n", iReg);

    atomic_set(&OV5640_SetGain_Flag, 0);
    wake_up_interruptible(&OV5640_SetGain_waitQueue);
    //PK_DBG("[OV5640YUV_SetGain] X Gain = %d \n", iGain); 
}   /*  OV5640YUV_SetGain  */


/*************************************************************************
* FUNCTION
*    read_OV5640YUV_gain
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
kal_uint16 read_OV5640YUV_gain(void)
{

    kal_uint8 temp_gain;
    kal_uint16 gain;
    //temp_gain = OV5640YUV_read_cmos_sensor(0x350B);
    //gain = OV5640YUVReg2Gain(temp_gain);
    gain = OV5640YUV_read_cmos_sensor(0x350B);
    PK_DBG("0x350b=0x%x, transfer to gain=0x%x,%d\n",temp_gain,gain,gain);
    return gain;
}  /* read_OV5640YUV_gain */

void write_OV5640YUV_gain(kal_uint16 gain)
{
    OV5640YUV_SetGain(gain);
}
void OV5640YUV_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV5640YUVSensorReg[i].Addr; i++)
    {
        OV5640YUV_write_cmos_sensor(OV5640YUVSensorReg[i].Addr, OV5640YUVSensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV5640YUVSensorReg[i].Addr; i++)
    {
        OV5640YUV_write_cmos_sensor(OV5640YUVSensorReg[i].Addr, OV5640YUVSensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        OV5640YUV_write_cmos_sensor(OV5640YUVSensorCCT[i].Addr, OV5640YUVSensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    OV5640YUV_sensor_to_camera_para
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
void OV5640YUV_sensor_to_camera_para(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV5640YUVSensorReg[i].Addr; i++)
    {
        OV5640YUVSensorReg[i].Para = OV5640YUV_read_cmos_sensor(OV5640YUVSensorReg[i].Addr);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV5640YUVSensorReg[i].Addr; i++)
    {
        OV5640YUVSensorReg[i].Para = OV5640YUV_read_cmos_sensor(OV5640YUVSensorReg[i].Addr);
    }
}


/*************************************************************************
* FUNCTION
*    OV5640YUV_get_sensor_group_count
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
kal_int32  OV5640YUV_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV5640YUV_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void OV5640YUV_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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
            temp_para=OV5640YUVSensorCCT[temp_addr].Para;
            temp_gain = OV5640YUVReg2Gain(temp_para);

            temp_gain=(temp_gain*1000)/BASEGAIN;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min=1000;
            info_ptr->Max=15875;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                    //temp_reg=OV5640YUVSensorReg[CMMCLK_CURRENT_INDEX].Para;
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
                    info_ptr->ItemValue=OV5640YUV_MAX_EXPOSURE_LINES;
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

//void OV5640YUV_set_isp_driving_current(kal_uint8 current)
//{
//}

kal_bool OV5640YUV_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16 temp_addr=0, temp_para=0;

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

            temp_para = OV5640YUVGain2Reg(ItemValue);


            OV5640YUVSensorCCT[temp_addr].Para = temp_para;
            OV5640YUV_write_cmos_sensor(OV5640YUVSensorCCT[temp_addr].Addr,temp_para);

            OV5640YUV_sensor_gain_base=read_OV5640YUV_gain();

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {
                        OV5640YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        OV5640YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        OV5640YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                        OV5640YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_8MA);
                    }
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
                    OV5640YUV_FAC_SENSOR_REG=ItemValue;
                    break;
                case 1:
                    OV5640YUV_write_cmos_sensor(OV5640YUV_FAC_SENSOR_REG,ItemValue);
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

static void OV5640YUV_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    kal_uint16 LinesOneframe;
    kal_uint16 PixelsOneline = OV5640_FULL_PERIOD_PIXEL_NUMS;
    if(OV5640_720P == OV5640YUV_g_RES)
    {
        PixelsOneline = (OV5640_PV_PERIOD_PIXEL_NUMS_HTS + iPixels );
        LinesOneframe =iLines + OV5640_PV_PERIOD_LINE_NUMS_VTS;
        if(OV5640YUV_MPEG4_encode_mode == KAL_FALSE)//for Fix video framerate
            OV5640YUV_CURRENT_FRAME_LINES = iLines + OV5640_PV_PERIOD_LINE_NUMS_VTS;
    }
    else if (OV5640_5M == OV5640YUV_g_RES)
    {
        PixelsOneline = OV5640_FULL_PERIOD_PIXEL_NUMS_HTS + iPixels;
	 LinesOneframe =iLines + OV5640_FULL_PERIOD_LINE_NUMS_VTS;

        OV5640YUV_CURRENT_FRAME_LINES = iLines + OV5640_FULL_PERIOD_LINE_NUMS_VTS;
    }
    if(iPixels)
    {
    	OV5640YUV_write_cmos_sensor(0x380c, (PixelsOneline >> 8) & 0xFF);
    	OV5640YUV_write_cmos_sensor(0x380d, PixelsOneline & 0xFF);
    }
    if(iLines)
    {
    	OV5640YUV_write_cmos_sensor(0x380e, (LinesOneframe >> 8) & 0xFF);
    	OV5640YUV_write_cmos_sensor(0x380f, LinesOneframe & 0xFF);
    }
    

#if 0 //! Don't set shutter related register 
    if(OV5640YUV_MPEG4_encode_mode == KAL_FALSE)//for Fix video framerate,set the frame lines in night mode function
    {
        //Set dummy lines.
        //The maximum shutter value = Line_Without_Dummy + Dummy_lines
        OV5640YUV_write_cmos_sensor(0x350C, (OV5640YUV_CURRENT_FRAME_LINES >> 8) & 0xFF);
        OV5640YUV_write_cmos_sensor(0x350D, OV5640YUV_CURRENT_FRAME_LINES & 0xFF);
        OV5640YUV_MAX_EXPOSURE_LINES = OV5640YUV_CURRENT_FRAME_LINES - OV5640_SHUTTER_LINES_GAP;
    }
#endif     
}   /*  OV5640YUV_SetDummy */

static void OV5640YUV_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 temp_AE_reg = 0;

    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
        temp_AE_reg = OV5640YUV_read_cmos_sensor(0x3503);
        OV5640YUV_write_cmos_sensor(0x3503, temp_AE_reg&~0x07);
  
    }
    else
    {
        // turn off AEC/AGC
        temp_AE_reg = OV5640YUV_read_cmos_sensor(0x3503);
        OV5640YUV_write_cmos_sensor(0x3503, temp_AE_reg| 0x07);

    }
}


static void OV5640YUV_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 temp_AWB_reg = 0;

    //return ;

    if (AWB_enable == KAL_TRUE)
    {
        //enable Auto WB
        temp_AWB_reg = OV5640YUV_read_cmos_sensor(0x3406);
        OV5640YUV_write_cmos_sensor(0x3406, temp_AWB_reg & ~0x01);        
    }
    else
    {
        //turn off AWB
        temp_AWB_reg = OV5640YUV_read_cmos_sensor(0x3406);
        OV5640YUV_write_cmos_sensor(0x3406, temp_AWB_reg | 0x01);        
    }
}

static void OV5640_FOCUS_AD5820_Check_MCU()
{
    kal_uint8 check[13] = {0x00};
	//mcu on
    check[0] = OV5640YUV_read_cmos_sensor(0x3000);
    check[1] = OV5640YUV_read_cmos_sensor(0x3004);
	//soft reset of mcu
    check[2] = OV5640YUV_read_cmos_sensor(0x3f00);	
	//afc on
    check[3] = OV5640YUV_read_cmos_sensor(0x3001);
    check[4] = OV5640YUV_read_cmos_sensor(0x3005);
	//gpio1,gpio2
    check[5] = OV5640YUV_read_cmos_sensor(0x3018);
    check[6] = OV5640YUV_read_cmos_sensor(0x301e);
    check[7] = OV5640YUV_read_cmos_sensor(0x301b);
    check[8] = OV5640YUV_read_cmos_sensor(0x3042);
	//y0
    check[9] = OV5640YUV_read_cmos_sensor(0x3018);
    check[10] = OV5640YUV_read_cmos_sensor(0x301e);
    check[11] = OV5640YUV_read_cmos_sensor(0x301b);
    check[12] = OV5640YUV_read_cmos_sensor(0x3042);

    int i = 0;
    for(i = 0; i < 13; i++)
    PK_DBG("check[%d]=0x%x\n", i, check[i]);

	
}


void OV5640YUV_Sensor_Init_set_720P(void);
void OV5640YUV_set_5M_init(void);
void OV5640YUV_IQ(void);
static void OV5640_FOCUS_AD5820_Init(void);



/*******************************************************************************
*
********************************************************************************/
static void OV5640YUV_Sensor_Init(void)
{
    PK_DBG("lln:: OV5640YUV_Sensor_Init, use OV5640YUV_Sensor_Init_set_720P");
    OV5640YUV_Sensor_Init_set_720P();

    //OV5640_FOCUS_AD5820_Init();// yxw 

    PK_DBG("Init Success \n");
}   /*  OV5640YUV_Sensor_Init  */


void OV5640YUV_Sensor_Init_set_720P(void)
{
	//24Mhz Mclk ,56MHz Pclk, 15fps
	OV5640YUV_write_cmos_sensor(0x3103, 0x11);
	OV5640YUV_write_cmos_sensor(0x3008, 0x82);
	OV5640YUV_write_cmos_sensor(0x3008, 0x42);
	OV5640YUV_write_cmos_sensor(0x3103, 0x03);
	OV5640YUV_write_cmos_sensor(0x3017, 0xff);
	OV5640YUV_write_cmos_sensor(0x3018, 0xff);
	OV5640YUV_write_cmos_sensor(0x3034, 0x1a);
	OV5640YUV_write_cmos_sensor(0x3035, 0x21);
	OV5640YUV_write_cmos_sensor(0x3036, 0x46);
	OV5640YUV_write_cmos_sensor(0x3037, 0x13);
	OV5640YUV_write_cmos_sensor(0x3108, 0x01);
	OV5640YUV_write_cmos_sensor(0x3630, 0x2e);
	OV5640YUV_write_cmos_sensor(0x3632, 0xe2);
	OV5640YUV_write_cmos_sensor(0x3633, 0x23);
	OV5640YUV_write_cmos_sensor(0x3621, 0xe0);
	OV5640YUV_write_cmos_sensor(0x3704, 0xa0);
	OV5640YUV_write_cmos_sensor(0x3703, 0x5a);
	OV5640YUV_write_cmos_sensor(0x3715, 0x78);
	OV5640YUV_write_cmos_sensor(0x3717, 0x01);
	OV5640YUV_write_cmos_sensor(0x370b, 0x60);
	OV5640YUV_write_cmos_sensor(0x3705, 0x1a);
	OV5640YUV_write_cmos_sensor(0x3905, 0x02);
	OV5640YUV_write_cmos_sensor(0x3906, 0x10);
	OV5640YUV_write_cmos_sensor(0x3901, 0x0a);
	OV5640YUV_write_cmos_sensor(0x3731, 0x12);
	OV5640YUV_write_cmos_sensor(0x3600, 0x08);
	OV5640YUV_write_cmos_sensor(0x3601, 0x33);
	OV5640YUV_write_cmos_sensor(0x302d, 0x60);
	OV5640YUV_write_cmos_sensor(0x3620, 0x52);
	OV5640YUV_write_cmos_sensor(0x371b, 0x20);
	OV5640YUV_write_cmos_sensor(0x471c, 0x50);
	OV5640YUV_write_cmos_sensor(0x3a18, 0x00);
	OV5640YUV_write_cmos_sensor(0x3a19, 0xf8);
	OV5640YUV_write_cmos_sensor(0x3635, 0x1c);
	OV5640YUV_write_cmos_sensor(0x3634, 0x40);
	OV5640YUV_write_cmos_sensor(0x3622, 0x01);
	OV5640YUV_write_cmos_sensor(0x3c00, 0x04);
	OV5640YUV_write_cmos_sensor(0x3c01, 0xb4);
	OV5640YUV_write_cmos_sensor(0x3c04, 0x28);
	OV5640YUV_write_cmos_sensor(0x3c05, 0x98);
	OV5640YUV_write_cmos_sensor(0x3c06, 0x00);
	OV5640YUV_write_cmos_sensor(0x3c07, 0x08);
	OV5640YUV_write_cmos_sensor(0x3c08, 0x00);
	OV5640YUV_write_cmos_sensor(0x3c09, 0x1c);
	OV5640YUV_write_cmos_sensor(0x3c0a, 0x9c);
	OV5640YUV_write_cmos_sensor(0x3c0b, 0x40);
	OV5640YUV_write_cmos_sensor(0x3820, 0x41);
	OV5640YUV_write_cmos_sensor(0x3821, 0x07);
	OV5640YUV_write_cmos_sensor(0x3814, 0x31);
	OV5640YUV_write_cmos_sensor(0x3815, 0x31);
	OV5640YUV_write_cmos_sensor(0x3800, 0x00);
	OV5640YUV_write_cmos_sensor(0x3801, 0x00);
	OV5640YUV_write_cmos_sensor(0x3802, 0x00);
	OV5640YUV_write_cmos_sensor(0x3803, 0x04);
	OV5640YUV_write_cmos_sensor(0x3804, 0x0a);
	OV5640YUV_write_cmos_sensor(0x3805, 0x3f);
	OV5640YUV_write_cmos_sensor(0x3806, 0x07);
	OV5640YUV_write_cmos_sensor(0x3807, 0x9b);
	OV5640YUV_write_cmos_sensor(0x3808, 0x05); 
	OV5640YUV_write_cmos_sensor(0x3809, 0x00);
	OV5640YUV_write_cmos_sensor(0x380a, 0x03);
	OV5640YUV_write_cmos_sensor(0x380b, 0xc0);
	OV5640YUV_write_cmos_sensor(0x380c, 0x07); //1896
	OV5640YUV_write_cmos_sensor(0x380d, 0x68);
	OV5640YUV_write_cmos_sensor(0x380e, 0x03);
	OV5640YUV_write_cmos_sensor(0x380f, 0xd8);
	OV5640YUV_write_cmos_sensor(0x3810, 0x00);
	OV5640YUV_write_cmos_sensor(0x3811, 0x04);
	OV5640YUV_write_cmos_sensor(0x3812, 0x00);
	OV5640YUV_write_cmos_sensor(0x3813, 0x00);
	OV5640YUV_write_cmos_sensor(0x3618, 0x00);
	OV5640YUV_write_cmos_sensor(0x3612, 0x29);
	OV5640YUV_write_cmos_sensor(0x3708, 0x64);
	OV5640YUV_write_cmos_sensor(0x3709, 0x52);
	OV5640YUV_write_cmos_sensor(0x370c, 0x03);
	OV5640YUV_write_cmos_sensor(0x3a02, 0x03);
	OV5640YUV_write_cmos_sensor(0x3a03, 0xd8);
	OV5640YUV_write_cmos_sensor(0x3a08, 0x00);
	OV5640YUV_write_cmos_sensor(0x3a09, 0x9F); //94
	OV5640YUV_write_cmos_sensor(0x3a0a, 0x00);
	OV5640YUV_write_cmos_sensor(0x3a0b, 0x85);//7b
	OV5640YUV_write_cmos_sensor(0x3a0e, 0x06);
	OV5640YUV_write_cmos_sensor(0x3a0d, 0x08);
	OV5640YUV_write_cmos_sensor(0x3a14, 0x03);
	OV5640YUV_write_cmos_sensor(0x3a15, 0xd8);
	OV5640YUV_write_cmos_sensor(0x4001, 0x02);
	OV5640YUV_write_cmos_sensor(0x4004, 0x02);
	OV5640YUV_write_cmos_sensor(0x3000, 0x00);
	OV5640YUV_write_cmos_sensor(0x3002, 0x1c);
	OV5640YUV_write_cmos_sensor(0x3004, 0xff);
	OV5640YUV_write_cmos_sensor(0x3006, 0xc3);
	OV5640YUV_write_cmos_sensor(0x300e, 0x58);
	OV5640YUV_write_cmos_sensor(0x302e, 0x00);
	OV5640YUV_write_cmos_sensor(0x4300, 0x30);
	OV5640YUV_write_cmos_sensor(0x501f, 0x00);
	OV5640YUV_write_cmos_sensor(0x4713, 0x03);
	OV5640YUV_write_cmos_sensor(0x4740, 0x00);
	OV5640YUV_write_cmos_sensor(0x4407, 0x04);
	OV5640YUV_write_cmos_sensor(0x460b, 0x35);
	OV5640YUV_write_cmos_sensor(0x460c, 0x20);
	OV5640YUV_write_cmos_sensor(0x3824, 0x02);
	OV5640YUV_write_cmos_sensor(0x5000, 0xa7);
	OV5640YUV_write_cmos_sensor(0x5001, 0xa3);
	OV5640YUV_write_cmos_sensor(0x5180, 0xff);
	OV5640YUV_write_cmos_sensor(0x5181, 0xf2);
	OV5640YUV_write_cmos_sensor(0x5182, 0x00);
	OV5640YUV_write_cmos_sensor(0x5183, 0x14);
	OV5640YUV_write_cmos_sensor(0x5184, 0x25);
	OV5640YUV_write_cmos_sensor(0x5185, 0x24);
	OV5640YUV_write_cmos_sensor(0x5186, 0x09);
	OV5640YUV_write_cmos_sensor(0x5187, 0x09);
	OV5640YUV_write_cmos_sensor(0x5188, 0x09);
	OV5640YUV_write_cmos_sensor(0x5189, 0x75);
	OV5640YUV_write_cmos_sensor(0x518a, 0x54);
	OV5640YUV_write_cmos_sensor(0x518b, 0xe0);
	OV5640YUV_write_cmos_sensor(0x518c, 0xb2);
	OV5640YUV_write_cmos_sensor(0x518d, 0x42);
	OV5640YUV_write_cmos_sensor(0x518e, 0x3d);
	OV5640YUV_write_cmos_sensor(0x518f, 0x56);
	OV5640YUV_write_cmos_sensor(0x5190, 0x46);
	OV5640YUV_write_cmos_sensor(0x5191, 0xf8);
	OV5640YUV_write_cmos_sensor(0x5192, 0x04);
	OV5640YUV_write_cmos_sensor(0x5193, 0x70);
	OV5640YUV_write_cmos_sensor(0x5194, 0xf0);
	OV5640YUV_write_cmos_sensor(0x5195, 0xf0);
	OV5640YUV_write_cmos_sensor(0x5196, 0x03);
	OV5640YUV_write_cmos_sensor(0x5197, 0x01);
	OV5640YUV_write_cmos_sensor(0x5198, 0x04);
	OV5640YUV_write_cmos_sensor(0x5199, 0x12);
	OV5640YUV_write_cmos_sensor(0x519a, 0x04);
	OV5640YUV_write_cmos_sensor(0x519b, 0x00);
	OV5640YUV_write_cmos_sensor(0x519c, 0x06);
	OV5640YUV_write_cmos_sensor(0x519d, 0x82);
	OV5640YUV_write_cmos_sensor(0x519e, 0x38);
	OV5640YUV_write_cmos_sensor(0x5381, 0x1c);
	OV5640YUV_write_cmos_sensor(0x5382, 0x5a);
	OV5640YUV_write_cmos_sensor(0x5383, 0x06);
	OV5640YUV_write_cmos_sensor(0x5384, 0x0a);
	OV5640YUV_write_cmos_sensor(0x5385, 0x7e);
	OV5640YUV_write_cmos_sensor(0x5386, 0x88);
	OV5640YUV_write_cmos_sensor(0x5387, 0x7c);
	OV5640YUV_write_cmos_sensor(0x5388, 0x6c);
	OV5640YUV_write_cmos_sensor(0x5389, 0x10);
	OV5640YUV_write_cmos_sensor(0x538a, 0x01);
	OV5640YUV_write_cmos_sensor(0x538b, 0x98);
	OV5640YUV_write_cmos_sensor(0x5300, 0x08);
	OV5640YUV_write_cmos_sensor(0x5301, 0x30);
	OV5640YUV_write_cmos_sensor(0x5302, 0x10);
	OV5640YUV_write_cmos_sensor(0x5303, 0x00);
	OV5640YUV_write_cmos_sensor(0x5304, 0x08);
	OV5640YUV_write_cmos_sensor(0x5305, 0x30);
	OV5640YUV_write_cmos_sensor(0x5306, 0x08);
	OV5640YUV_write_cmos_sensor(0x5307, 0x16);
	OV5640YUV_write_cmos_sensor(0x5309, 0x08);
	OV5640YUV_write_cmos_sensor(0x530a, 0x30);
	OV5640YUV_write_cmos_sensor(0x530b, 0x04);
	OV5640YUV_write_cmos_sensor(0x530c, 0x06);
	OV5640YUV_write_cmos_sensor(0x5480, 0x01);
	OV5640YUV_write_cmos_sensor(0x5481, 0x08);
	OV5640YUV_write_cmos_sensor(0x5482, 0x14);
	OV5640YUV_write_cmos_sensor(0x5483, 0x28);
	OV5640YUV_write_cmos_sensor(0x5484, 0x51);
	OV5640YUV_write_cmos_sensor(0x5485, 0x65);
	OV5640YUV_write_cmos_sensor(0x5486, 0x71);
	OV5640YUV_write_cmos_sensor(0x5487, 0x7d);
	OV5640YUV_write_cmos_sensor(0x5488, 0x87);
	OV5640YUV_write_cmos_sensor(0x5489, 0x91);
	OV5640YUV_write_cmos_sensor(0x548a, 0x9a);
	OV5640YUV_write_cmos_sensor(0x548b, 0xaa);
	OV5640YUV_write_cmos_sensor(0x548c, 0xb8);
	OV5640YUV_write_cmos_sensor(0x548d, 0xcd);
	OV5640YUV_write_cmos_sensor(0x548e, 0xdd);
	OV5640YUV_write_cmos_sensor(0x548f, 0xea);
	OV5640YUV_write_cmos_sensor(0x5490, 0x1d);
	OV5640YUV_write_cmos_sensor(0x5580, 0x02);
	OV5640YUV_write_cmos_sensor(0x5583, 0x40);
	OV5640YUV_write_cmos_sensor(0x5584, 0x10);
	OV5640YUV_write_cmos_sensor(0x5589, 0x10);
	OV5640YUV_write_cmos_sensor(0x558a, 0x00);
	OV5640YUV_write_cmos_sensor(0x558b, 0xf8);
	OV5640YUV_write_cmos_sensor(0x5800, 0x23);
	OV5640YUV_write_cmos_sensor(0x5801, 0x14);
	OV5640YUV_write_cmos_sensor(0x5802, 0x0f);
	OV5640YUV_write_cmos_sensor(0x5803, 0x0f);
	OV5640YUV_write_cmos_sensor(0x5804, 0x12);
	OV5640YUV_write_cmos_sensor(0x5805, 0x26);
	OV5640YUV_write_cmos_sensor(0x5806, 0x0c);
	OV5640YUV_write_cmos_sensor(0x5807, 0x08);
	OV5640YUV_write_cmos_sensor(0x5808, 0x05);
	OV5640YUV_write_cmos_sensor(0x5809, 0x05);
	OV5640YUV_write_cmos_sensor(0x580a, 0x08);
	OV5640YUV_write_cmos_sensor(0x580b, 0x0d);
	OV5640YUV_write_cmos_sensor(0x580c, 0x08);
	OV5640YUV_write_cmos_sensor(0x580d, 0x03);
	OV5640YUV_write_cmos_sensor(0x580e, 0x00);
	OV5640YUV_write_cmos_sensor(0x580f, 0x00);
	OV5640YUV_write_cmos_sensor(0x5810, 0x03);
	OV5640YUV_write_cmos_sensor(0x5811, 0x09);
	OV5640YUV_write_cmos_sensor(0x5812, 0x07);
	OV5640YUV_write_cmos_sensor(0x5813, 0x03);
	OV5640YUV_write_cmos_sensor(0x5814, 0x00);
	OV5640YUV_write_cmos_sensor(0x5815, 0x01);
	OV5640YUV_write_cmos_sensor(0x5816, 0x03);
	OV5640YUV_write_cmos_sensor(0x5817, 0x08);
	OV5640YUV_write_cmos_sensor(0x5818, 0x0d);
	OV5640YUV_write_cmos_sensor(0x5819, 0x08);
	OV5640YUV_write_cmos_sensor(0x581a, 0x05);
	OV5640YUV_write_cmos_sensor(0x581b, 0x06);
	OV5640YUV_write_cmos_sensor(0x581c, 0x08);
	OV5640YUV_write_cmos_sensor(0x581d, 0x0e);
	OV5640YUV_write_cmos_sensor(0x581e, 0x29);
	OV5640YUV_write_cmos_sensor(0x581f, 0x17);
	OV5640YUV_write_cmos_sensor(0x5820, 0x11);
	OV5640YUV_write_cmos_sensor(0x5821, 0x11);
	OV5640YUV_write_cmos_sensor(0x5822, 0x15);
	OV5640YUV_write_cmos_sensor(0x5823, 0x28);
	OV5640YUV_write_cmos_sensor(0x5824, 0x46);
	OV5640YUV_write_cmos_sensor(0x5825, 0x26);
	OV5640YUV_write_cmos_sensor(0x5826, 0x08);
	OV5640YUV_write_cmos_sensor(0x5827, 0x26);
	OV5640YUV_write_cmos_sensor(0x5828, 0x64);
	OV5640YUV_write_cmos_sensor(0x5829, 0x26);
	OV5640YUV_write_cmos_sensor(0x582a, 0x24);
	OV5640YUV_write_cmos_sensor(0x582b, 0x22);
	OV5640YUV_write_cmos_sensor(0x582c, 0x24);
	OV5640YUV_write_cmos_sensor(0x582d, 0x24);
	OV5640YUV_write_cmos_sensor(0x582e, 0x06);
	OV5640YUV_write_cmos_sensor(0x582f, 0x22);
	OV5640YUV_write_cmos_sensor(0x5830, 0x40);
	OV5640YUV_write_cmos_sensor(0x5831, 0x42);
	OV5640YUV_write_cmos_sensor(0x5832, 0x24);
	OV5640YUV_write_cmos_sensor(0x5833, 0x26);
	OV5640YUV_write_cmos_sensor(0x5834, 0x24);
	OV5640YUV_write_cmos_sensor(0x5835, 0x22);
	OV5640YUV_write_cmos_sensor(0x5836, 0x22);
	OV5640YUV_write_cmos_sensor(0x5837, 0x26);
	OV5640YUV_write_cmos_sensor(0x5838, 0x44);
	OV5640YUV_write_cmos_sensor(0x5839, 0x24);
	OV5640YUV_write_cmos_sensor(0x583a, 0x26);
	OV5640YUV_write_cmos_sensor(0x583b, 0x28);
	OV5640YUV_write_cmos_sensor(0x583c, 0x42);
	OV5640YUV_write_cmos_sensor(0x583d, 0xce);
	OV5640YUV_write_cmos_sensor(0x5025, 0x00);
/*	
	OV5640YUV_write_cmos_sensor(0x3a0f, 0x30);
	OV5640YUV_write_cmos_sensor(0x3a10, 0x28);
	OV5640YUV_write_cmos_sensor(0x3a1b, 0x30);
	OV5640YUV_write_cmos_sensor(0x3a1e, 0x26);
	OV5640YUV_write_cmos_sensor(0x3a11, 0x60);
	OV5640YUV_write_cmos_sensor(0x3a1f, 0x14);
*/
	OV5640YUV_write_cmos_sensor(0x3a0f, 0x38);
	OV5640YUV_write_cmos_sensor(0x3a10, 0x30);
	OV5640YUV_write_cmos_sensor(0x3a11, 0x61);
	OV5640YUV_write_cmos_sensor(0x3a1b, 0x38);
	OV5640YUV_write_cmos_sensor(0x3a1e, 0x30);
	OV5640YUV_write_cmos_sensor(0x3a1f, 0x10);
	

        OV5640YUV_IQ();

	OV5640YUV_write_cmos_sensor(0x3008, 0x02);

	//following add by thp
//AWB
OV5640YUV_write_cmos_sensor(0x5180,0xff);
OV5640YUV_write_cmos_sensor(0x5181,0xf2);
OV5640YUV_write_cmos_sensor(0x5182,0x00);
OV5640YUV_write_cmos_sensor(0x5183,0x14);
OV5640YUV_write_cmos_sensor(0x5184,0x25);
OV5640YUV_write_cmos_sensor(0x5185,0x24);
OV5640YUV_write_cmos_sensor(0x5186,0x09);
OV5640YUV_write_cmos_sensor(0x5187,0x09);
OV5640YUV_write_cmos_sensor(0x5188,0x16);
OV5640YUV_write_cmos_sensor(0x5189,0x8e);
OV5640YUV_write_cmos_sensor(0x518a,0x56);
OV5640YUV_write_cmos_sensor(0x518b,0xe0);
OV5640YUV_write_cmos_sensor(0x518c,0xb2);
OV5640YUV_write_cmos_sensor(0x518d,0x28);
OV5640YUV_write_cmos_sensor(0x518e,0x1d);
OV5640YUV_write_cmos_sensor(0x518f,0x56);
OV5640YUV_write_cmos_sensor(0x5190,0x46);
OV5640YUV_write_cmos_sensor(0x5191,0xf8);
OV5640YUV_write_cmos_sensor(0x5192,0x04);
OV5640YUV_write_cmos_sensor(0x5193,0x70);
OV5640YUV_write_cmos_sensor(0x5194,0xf0);
OV5640YUV_write_cmos_sensor(0x5195,0xf0);
OV5640YUV_write_cmos_sensor(0x5196,0x03);
OV5640YUV_write_cmos_sensor(0x5197,0x01);
OV5640YUV_write_cmos_sensor(0x5198,0x05);
OV5640YUV_write_cmos_sensor(0x5199,0x7c);
OV5640YUV_write_cmos_sensor(0x519a,0x04);
OV5640YUV_write_cmos_sensor(0x519b,0x00);
OV5640YUV_write_cmos_sensor(0x519c,0x06);
OV5640YUV_write_cmos_sensor(0x519d,0x79);
OV5640YUV_write_cmos_sensor(0x519e,0x38);

//Color
OV5640YUV_write_cmos_sensor(0x5381,0x1e);
OV5640YUV_write_cmos_sensor(0x5382,0x5b);
OV5640YUV_write_cmos_sensor(0x5383,0x08);
OV5640YUV_write_cmos_sensor(0x5384,0x05);
OV5640YUV_write_cmos_sensor(0x5385,0x72);
OV5640YUV_write_cmos_sensor(0x5386,0x77);
OV5640YUV_write_cmos_sensor(0x5387,0x6d);
OV5640YUV_write_cmos_sensor(0x5388,0x4d);
OV5640YUV_write_cmos_sensor(0x5389,0x20);
OV5640YUV_write_cmos_sensor(0x538a,0x01);
OV5640YUV_write_cmos_sensor(0x538b,0x98);

//Gamma
OV5640YUV_write_cmos_sensor(0x5480,0x01);
OV5640YUV_write_cmos_sensor(0x5481,0x06);
OV5640YUV_write_cmos_sensor(0x5482,0x0e);
OV5640YUV_write_cmos_sensor(0x5483,0x19);
OV5640YUV_write_cmos_sensor(0x5484,0x3b);
OV5640YUV_write_cmos_sensor(0x5485,0x50);
OV5640YUV_write_cmos_sensor(0x5486,0x5d);
OV5640YUV_write_cmos_sensor(0x5487,0x6a);
OV5640YUV_write_cmos_sensor(0x5488,0x75);
OV5640YUV_write_cmos_sensor(0x5489,0x80);
OV5640YUV_write_cmos_sensor(0x548a,0x8a);
OV5640YUV_write_cmos_sensor(0x548b,0x9b);
OV5640YUV_write_cmos_sensor(0x548c,0xaa);
OV5640YUV_write_cmos_sensor(0x548d,0xc0);
OV5640YUV_write_cmos_sensor(0x548e,0xd5);
OV5640YUV_write_cmos_sensor(0x548f,0xe8);
OV5640YUV_write_cmos_sensor(0x5490,0x20);

//UV
OV5640YUV_write_cmos_sensor(0x5580,0x06);
OV5640YUV_write_cmos_sensor(0x5583,0x40);
OV5640YUV_write_cmos_sensor(0x5584,0x10);
OV5640YUV_write_cmos_sensor(0x5589,0x10);
OV5640YUV_write_cmos_sensor(0x558a,0x00);
OV5640YUV_write_cmos_sensor(0x558b,0xf8);

//LensShading
OV5640YUV_write_cmos_sensor(0x5000,0xa7);
OV5640YUV_write_cmos_sensor(0x5800,0x17);
OV5640YUV_write_cmos_sensor(0x5801,0x10);
OV5640YUV_write_cmos_sensor(0x5802,0x0e);
OV5640YUV_write_cmos_sensor(0x5803,0x0e);
OV5640YUV_write_cmos_sensor(0x5804,0x11);
OV5640YUV_write_cmos_sensor(0x5805,0x1b);
OV5640YUV_write_cmos_sensor(0x5806,0x0b);
OV5640YUV_write_cmos_sensor(0x5807,0x07);
OV5640YUV_write_cmos_sensor(0x5808,0x05);
OV5640YUV_write_cmos_sensor(0x5809,0x06);
OV5640YUV_write_cmos_sensor(0x580a,0x09);
OV5640YUV_write_cmos_sensor(0x580b,0x0e);
OV5640YUV_write_cmos_sensor(0x580c,0x06);
OV5640YUV_write_cmos_sensor(0x580d,0x02);
OV5640YUV_write_cmos_sensor(0x580e,0x00);
OV5640YUV_write_cmos_sensor(0x580f,0x00);
OV5640YUV_write_cmos_sensor(0x5810,0x03);
OV5640YUV_write_cmos_sensor(0x5811,0x09);
OV5640YUV_write_cmos_sensor(0x5812,0x06);
OV5640YUV_write_cmos_sensor(0x5813,0x03);
OV5640YUV_write_cmos_sensor(0x5814,0x00);
OV5640YUV_write_cmos_sensor(0x5815,0x00);
OV5640YUV_write_cmos_sensor(0x5816,0x03);
OV5640YUV_write_cmos_sensor(0x5817,0x09);
OV5640YUV_write_cmos_sensor(0x5818,0x0b);
OV5640YUV_write_cmos_sensor(0x5819,0x08);
OV5640YUV_write_cmos_sensor(0x581a,0x05);
OV5640YUV_write_cmos_sensor(0x581b,0x05);
OV5640YUV_write_cmos_sensor(0x581c,0x08);
OV5640YUV_write_cmos_sensor(0x581d,0x0e);
OV5640YUV_write_cmos_sensor(0x581e,0x18);
OV5640YUV_write_cmos_sensor(0x581f,0x12);
OV5640YUV_write_cmos_sensor(0x5820,0x0f);
OV5640YUV_write_cmos_sensor(0x5821,0x0f);
OV5640YUV_write_cmos_sensor(0x5822,0x12);
OV5640YUV_write_cmos_sensor(0x5823,0x1a);
OV5640YUV_write_cmos_sensor(0x5824,0x0a);
OV5640YUV_write_cmos_sensor(0x5825,0x0a);
OV5640YUV_write_cmos_sensor(0x5826,0x0a);
OV5640YUV_write_cmos_sensor(0x5827,0x0a);
OV5640YUV_write_cmos_sensor(0x5828,0x46);
OV5640YUV_write_cmos_sensor(0x5829,0x2a);
OV5640YUV_write_cmos_sensor(0x582a,0x24);
OV5640YUV_write_cmos_sensor(0x582b,0x44);
OV5640YUV_write_cmos_sensor(0x582c,0x24);
OV5640YUV_write_cmos_sensor(0x582d,0x28);
OV5640YUV_write_cmos_sensor(0x582e,0x08);
OV5640YUV_write_cmos_sensor(0x582f,0x42);
OV5640YUV_write_cmos_sensor(0x5830,0x40);
OV5640YUV_write_cmos_sensor(0x5831,0x42);
OV5640YUV_write_cmos_sensor(0x5832,0x28);
OV5640YUV_write_cmos_sensor(0x5833,0x0a);
OV5640YUV_write_cmos_sensor(0x5834,0x26);
OV5640YUV_write_cmos_sensor(0x5835,0x24);
OV5640YUV_write_cmos_sensor(0x5836,0x26);
OV5640YUV_write_cmos_sensor(0x5837,0x28);
OV5640YUV_write_cmos_sensor(0x5838,0x4a);
OV5640YUV_write_cmos_sensor(0x5839,0x0a);
OV5640YUV_write_cmos_sensor(0x583a,0x0c);
OV5640YUV_write_cmos_sensor(0x583b,0x2a);
OV5640YUV_write_cmos_sensor(0x583c,0x28);
OV5640YUV_write_cmos_sensor(0x583d,0xce);
//EV
OV5640YUV_write_cmos_sensor(0x3a0f,0x40);
OV5640YUV_write_cmos_sensor(0x3a10,0x38);
OV5640YUV_write_cmos_sensor(0x3a1b,0x40);
OV5640YUV_write_cmos_sensor(0x3a1e,0x38);
OV5640YUV_write_cmos_sensor(0x3a11,0x70);
OV5640YUV_write_cmos_sensor(0x3a1f,0x14);

//Sharpness/Denoise
OV5640YUV_write_cmos_sensor(0x5300,0x08);
OV5640YUV_write_cmos_sensor(0x5301,0x30);
OV5640YUV_write_cmos_sensor(0x5302,0x2c);
OV5640YUV_write_cmos_sensor(0x5303,0x1c);
OV5640YUV_write_cmos_sensor(0x5304,0x08);
OV5640YUV_write_cmos_sensor(0x5305,0x30);
OV5640YUV_write_cmos_sensor(0x5306,0x1c);
OV5640YUV_write_cmos_sensor(0x5307,0x2c);
OV5640YUV_write_cmos_sensor(0x5309,0x08);
OV5640YUV_write_cmos_sensor(0x530a,0x30);
OV5640YUV_write_cmos_sensor(0x530b,0x04);
OV5640YUV_write_cmos_sensor(0x530c,0x06);

//AEC/AGC
OV5640YUV_write_cmos_sensor(0x3a0f,0x40);
OV5640YUV_write_cmos_sensor(0x3a10,0x38);
OV5640YUV_write_cmos_sensor(0x3a1b,0x40);
OV5640YUV_write_cmos_sensor(0x3a1e,0x38);
OV5640YUV_write_cmos_sensor(0x3a11,0x70);
OV5640YUV_write_cmos_sensor(0x3a1f,0x14);


//add 2012.6.20 for satruation
#ifdef AGOLD_E1809C_LX528
OV5640YUV_write_cmos_sensor(0x5001,0xff);
OV5640YUV_write_cmos_sensor(0x5583,0x40);
OV5640YUV_write_cmos_sensor(0x5584,0x40);
OV5640YUV_write_cmos_sensor(0x5580,0x02);
OV5640YUV_write_cmos_sensor(0x5588,0x41);
#endif
//end of the add

	//add by thp
}
void OV5640YUV_set_720P(void)
{
    int dummy_pixels, dummy_lines;
    OV5640YUV_g_RES = OV5640_720P;
    PK_DBG("OV5640YUV_set_720P Start \n"); 
    
    OV5640YUV_Sensor_Init_set_720P();

    OV5640YUV_PV_pclk = 5600; 
    OV5640YUV_sensor_pclk=56000000;



//    PK_DBG("preview start may 1 change mcu\n");
//    OV5640_FOCUS_AD5820_Check_MCU();

    PK_DBG("Set 720P End\n"); 
}
void OV5640YUV_IQ(void)
{
	OV5640YUV_write_cmos_sensor(0x5180, 0xff);   
	OV5640YUV_write_cmos_sensor(0x5181, 0xf2);   
	OV5640YUV_write_cmos_sensor(0x5182, 0x0 );  
	OV5640YUV_write_cmos_sensor(0x5183, 0x14);   
	OV5640YUV_write_cmos_sensor(0x5184, 0x25);   
	OV5640YUV_write_cmos_sensor(0x5185, 0x24);   
	OV5640YUV_write_cmos_sensor(0x5186, 0x20);   
	OV5640YUV_write_cmos_sensor(0x5187, 0x16);   
	OV5640YUV_write_cmos_sensor(0x5188, 0x17);   
	OV5640YUV_write_cmos_sensor(0x5189, 0x81);   
	OV5640YUV_write_cmos_sensor(0x518a, 0x6b);   
	OV5640YUV_write_cmos_sensor(0x518b, 0xb3);   
	OV5640YUV_write_cmos_sensor(0x518c, 0x87);   
	OV5640YUV_write_cmos_sensor(0x518d, 0x3b);   
	OV5640YUV_write_cmos_sensor(0x518e, 0x35);   
	OV5640YUV_write_cmos_sensor(0x518f, 0x63);   
	OV5640YUV_write_cmos_sensor(0x5190, 0x4b);   
	OV5640YUV_write_cmos_sensor(0x5191, 0xf8);   
	OV5640YUV_write_cmos_sensor(0x5192, 0x4 );  
	OV5640YUV_write_cmos_sensor(0x5193, 0x70);   
	OV5640YUV_write_cmos_sensor(0x5194, 0xf0);   
	OV5640YUV_write_cmos_sensor(0x5195, 0xf0);   
	OV5640YUV_write_cmos_sensor(0x5196, 0x3 );  
	OV5640YUV_write_cmos_sensor(0x5197, 0x1 );  
	OV5640YUV_write_cmos_sensor(0x5198, 0x5 );  
	OV5640YUV_write_cmos_sensor(0x5199, 0xfd);   
	OV5640YUV_write_cmos_sensor(0x519a, 0x4 );  
	OV5640YUV_write_cmos_sensor(0x519b, 0x0 );  
	OV5640YUV_write_cmos_sensor(0x519c, 0x4 );  
	OV5640YUV_write_cmos_sensor(0x519d, 0xc6);   
	OV5640YUV_write_cmos_sensor(0x519e, 0x38);   
	                                        
	//CCM                                   
	OV5640YUV_write_cmos_sensor(0x5381, 0x1c);
	OV5640YUV_write_cmos_sensor(0x5382, 0x5a);
	OV5640YUV_write_cmos_sensor(0x5383, 0x06);
	OV5640YUV_write_cmos_sensor(0x5384, 0x0a);
	OV5640YUV_write_cmos_sensor(0x5385, 0x7e);
	OV5640YUV_write_cmos_sensor(0x5386, 0x88);
	OV5640YUV_write_cmos_sensor(0x5387, 0x7c);
	OV5640YUV_write_cmos_sensor(0x5388, 0x6c);
	OV5640YUV_write_cmos_sensor(0x5389, 0x10);
	OV5640YUV_write_cmos_sensor(0x538a, 0x01);
	OV5640YUV_write_cmos_sensor(0x538b, 0x98);
	                                        
	//sharpness&noise                       
	OV5640YUV_write_cmos_sensor(0x5308, 0x25);
	OV5640YUV_write_cmos_sensor(0x5300, 0x09);
	OV5640YUV_write_cmos_sensor(0x5301, 0x11);
	OV5640YUV_write_cmos_sensor(0x5302, 0x10);
	OV5640YUV_write_cmos_sensor(0x5303, 0x00);
	OV5640YUV_write_cmos_sensor(0x5304, 0x08);
	OV5640YUV_write_cmos_sensor(0x5305, 0x11);
	OV5640YUV_write_cmos_sensor(0x5306, 0x08);
	OV5640YUV_write_cmos_sensor(0x5307, 0x18);
	OV5640YUV_write_cmos_sensor(0x5309, 0x08);
	OV5640YUV_write_cmos_sensor(0x530a, 0x30);
	OV5640YUV_write_cmos_sensor(0x530b, 0x04);
	OV5640YUV_write_cmos_sensor(0x530c, 0x06);
	//GAMMA                            
	OV5640YUV_write_cmos_sensor(0x5480, 0x01); 
	OV5640YUV_write_cmos_sensor(0x5481, 0x8 );
	OV5640YUV_write_cmos_sensor(0x5482, 0x14); 
	OV5640YUV_write_cmos_sensor(0x5483, 0x28); 
	OV5640YUV_write_cmos_sensor(0x5484, 0x51);
	OV5640YUV_write_cmos_sensor(0x5485, 0x65); 
	OV5640YUV_write_cmos_sensor(0x5486, 0x71); 
	OV5640YUV_write_cmos_sensor(0x5487, 0x7e); 
	OV5640YUV_write_cmos_sensor(0x5488, 0x88); 
	OV5640YUV_write_cmos_sensor(0x5489, 0x92); 
	OV5640YUV_write_cmos_sensor(0x548a, 0x9b); 
	OV5640YUV_write_cmos_sensor(0x548b, 0xab); 
	OV5640YUV_write_cmos_sensor(0x548c, 0xba); 
	OV5640YUV_write_cmos_sensor(0x548d, 0xd0); 
	OV5640YUV_write_cmos_sensor(0x548e, 0xe1); 
	OV5640YUV_write_cmos_sensor(0x548f, 0xf3);
	OV5640YUV_write_cmos_sensor(0x5490, 0x11);
	                                
	//UV adjust                     
	OV5640YUV_write_cmos_sensor(0x5580, 0x02);
	OV5640YUV_write_cmos_sensor(0x5583, 0x40);
	OV5640YUV_write_cmos_sensor(0x5584, 0x18);
	OV5640YUV_write_cmos_sensor(0x5589, 0x12);
	OV5640YUV_write_cmos_sensor(0x558a, 0x00);
	OV5640YUV_write_cmos_sensor(0x558b, 0x58);
	//blc                                   
	OV5640YUV_write_cmos_sensor(0x4005, 0x1a);
	                                        
	//OV5640 LENC setting
	OV5640YUV_write_cmos_sensor(0x5800, 0x28);
	OV5640YUV_write_cmos_sensor(0x5801, 0x13);
	OV5640YUV_write_cmos_sensor(0x5802, 0x10);
	OV5640YUV_write_cmos_sensor(0x5803, 0xf );
	OV5640YUV_write_cmos_sensor(0x5804, 0x12);
	OV5640YUV_write_cmos_sensor(0x5805, 0x20);
	OV5640YUV_write_cmos_sensor(0x5806, 0xa );
	OV5640YUV_write_cmos_sensor(0x5807, 0x6 );
	OV5640YUV_write_cmos_sensor(0x5808, 0x6 );
	OV5640YUV_write_cmos_sensor(0x5809, 0x6 );
	OV5640YUV_write_cmos_sensor(0x580a, 0x5 );
	OV5640YUV_write_cmos_sensor(0x580b, 0xa );
	OV5640YUV_write_cmos_sensor(0x580c, 0x8 );
	OV5640YUV_write_cmos_sensor(0x580d, 0x4 );
	OV5640YUV_write_cmos_sensor(0x580e, 0x0 );
	OV5640YUV_write_cmos_sensor(0x580f, 0x0 );
	OV5640YUV_write_cmos_sensor(0x5810, 0x3 );
	OV5640YUV_write_cmos_sensor(0x5811, 0x8 );
	OV5640YUV_write_cmos_sensor(0x5812, 0x7 );
	OV5640YUV_write_cmos_sensor(0x5813, 0x3 );
	OV5640YUV_write_cmos_sensor(0x5814, 0x0 );
	OV5640YUV_write_cmos_sensor(0x5815, 0x0 );
	OV5640YUV_write_cmos_sensor(0x5816, 0x3 );
	OV5640YUV_write_cmos_sensor(0x5817, 0x8 );
	OV5640YUV_write_cmos_sensor(0x5818, 0xa );
	OV5640YUV_write_cmos_sensor(0x5819, 0x7 );
	OV5640YUV_write_cmos_sensor(0x581a, 0x5 );
	OV5640YUV_write_cmos_sensor(0x581b, 0x5 );
	OV5640YUV_write_cmos_sensor(0x581c, 0x5 );
	OV5640YUV_write_cmos_sensor(0x581d, 0xa );
	OV5640YUV_write_cmos_sensor(0x581e, 0x1d);
	OV5640YUV_write_cmos_sensor(0x581f, 0xd );
	OV5640YUV_write_cmos_sensor(0x5820, 0xc );
	OV5640YUV_write_cmos_sensor(0x5821, 0xc );
	OV5640YUV_write_cmos_sensor(0x5822, 0xe );
	OV5640YUV_write_cmos_sensor(0x5823, 0x1a);
	OV5640YUV_write_cmos_sensor(0x5824, 0x28);
	OV5640YUV_write_cmos_sensor(0x5825, 0x28);
	OV5640YUV_write_cmos_sensor(0x5826, 0x2a);
	OV5640YUV_write_cmos_sensor(0x5827, 0x28);
	OV5640YUV_write_cmos_sensor(0x5828, 0x28);
	OV5640YUV_write_cmos_sensor(0x5829, 0x2c);
	OV5640YUV_write_cmos_sensor(0x582a, 0x28);
	OV5640YUV_write_cmos_sensor(0x582b, 0x26);
	OV5640YUV_write_cmos_sensor(0x582c, 0x26);
	OV5640YUV_write_cmos_sensor(0x582d, 0x2a);
	OV5640YUV_write_cmos_sensor(0x582e, 0x2a);
	OV5640YUV_write_cmos_sensor(0x582f, 0x24);
	OV5640YUV_write_cmos_sensor(0x5830, 0x40);
	OV5640YUV_write_cmos_sensor(0x5831, 0x42);
	OV5640YUV_write_cmos_sensor(0x5832, 0xa );
	OV5640YUV_write_cmos_sensor(0x5833, 0x2c);
	OV5640YUV_write_cmos_sensor(0x5834, 0x28);
	OV5640YUV_write_cmos_sensor(0x5835, 0x26);
	OV5640YUV_write_cmos_sensor(0x5836, 0x26);
	OV5640YUV_write_cmos_sensor(0x5837, 0xe );
	OV5640YUV_write_cmos_sensor(0x5838, 0x2e);
	OV5640YUV_write_cmos_sensor(0x5839, 0x2c);
	OV5640YUV_write_cmos_sensor(0x583a, 0x2e);
	OV5640YUV_write_cmos_sensor(0x583b, 0x2c);
	OV5640YUV_write_cmos_sensor(0x583c, 0x2c);
	OV5640YUV_write_cmos_sensor(0x583d, 0xce);
}

void OV5640YUV_set_5M_init(void)
{
	OV5640YUV_write_cmos_sensor(0x3820, 0x40); 
	OV5640YUV_write_cmos_sensor(0x3821, 0x06); 
	OV5640YUV_write_cmos_sensor(0x3814, 0x11); 
	OV5640YUV_write_cmos_sensor(0x3815, 0x11); 
	OV5640YUV_write_cmos_sensor(0x3803, 0x00); 
	OV5640YUV_write_cmos_sensor(0x3807, 0x9f); 
	OV5640YUV_write_cmos_sensor(0x3808, 0x0a); 
	OV5640YUV_write_cmos_sensor(0x3809, 0x20); 
	OV5640YUV_write_cmos_sensor(0x380a, 0x07); 
	OV5640YUV_write_cmos_sensor(0x380b, 0x98); 
	OV5640YUV_write_cmos_sensor(0x380c, 0x0b); 
	OV5640YUV_write_cmos_sensor(0x380d, 0x1c); 
	OV5640YUV_write_cmos_sensor(0x380e, 0x07); 
	OV5640YUV_write_cmos_sensor(0x380f, 0xb0); 
	OV5640YUV_write_cmos_sensor(0x3811, 0x10); //
	OV5640YUV_write_cmos_sensor(0x3813, 0x04); 
	OV5640YUV_write_cmos_sensor(0x3618, 0x04); 
	OV5640YUV_write_cmos_sensor(0x3612, 0x2b); //4b 
	OV5640YUV_write_cmos_sensor(0x3708, 0x64);
	OV5640YUV_write_cmos_sensor(0x3709, 0x12); 
	OV5640YUV_write_cmos_sensor(0x370c, 0x00); 
	OV5640YUV_write_cmos_sensor(0x3a02, 0x07); 
	OV5640YUV_write_cmos_sensor(0x3a03, 0xb0); 
	OV5640YUV_write_cmos_sensor(0x3a0e, 0x06); 
	OV5640YUV_write_cmos_sensor(0x3a0d, 0x08); 
	OV5640YUV_write_cmos_sensor(0x3a14, 0x07); 
	OV5640YUV_write_cmos_sensor(0x3a15, 0xb0); 
	OV5640YUV_write_cmos_sensor(0x4004, 0x06);
	OV5640YUV_write_cmos_sensor(0x5000, 0xa7); 
	OV5640YUV_write_cmos_sensor(0x5001, 0x83);
	OV5640YUV_write_cmos_sensor(0x519e, 0x38);
	OV5640YUV_write_cmos_sensor(0x5381, 0x1e);
	OV5640YUV_write_cmos_sensor(0x5382, 0x5b);
	OV5640YUV_write_cmos_sensor(0x5383, 0x08);
	OV5640YUV_write_cmos_sensor(0x460b, 0x37); 
	OV5640YUV_write_cmos_sensor(0x460c, 0x20); 
	OV5640YUV_write_cmos_sensor(0x3824, 0x01); 
	OV5640YUV_write_cmos_sensor(0x4005, 0x1A); 

}
void OV5640YUV_set_5M(void)
{
    PK_DBG("Set 5M begin\n"); 
    OV5640YUV_g_RES = OV5640_5M;

    OV5640YUV_set_5M_init();

    //OV5640YUV_set_AE_mode(KAL_FALSE);
    //OV5640YUV_set_AWB_mode(KAL_FALSE);

    //OV5640_Sensor_Total_5M();	
    OV5640YUV_PV_pclk = 5600; 
    OV5640YUV_sensor_pclk=56000000;
    PK_DBG("Set 5M End\n");  
}

void OV5640YUV_dump_5M(void)
{
/*
    kal_uint8 dump_5M[69] = {0x00};

    dump_5M[0] = OV5640YUV_read_cmos_sensor(0x3612);
    dump_5M[1] = OV5640YUV_read_cmos_sensor(0x3613);
    dump_5M[2] = OV5640YUV_read_cmos_sensor(0x3621);
    dump_5M[3] = OV5640YUV_read_cmos_sensor(0x3622);
    dump_5M[4] = OV5640YUV_read_cmos_sensor(0x3623);
    dump_5M[5] = OV5640YUV_read_cmos_sensor(0x3604);
    dump_5M[6] = OV5640YUV_read_cmos_sensor(0x3705);
    dump_5M[7] = OV5640YUV_read_cmos_sensor(0x370a);
    dump_5M[8] = OV5640YUV_read_cmos_sensor(0x3801);
    dump_5M[9] = OV5640YUV_read_cmos_sensor(0x3803);
    dump_5M[10] = OV5640YUV_read_cmos_sensor(0x3804);
    dump_5M[11] = OV5640YUV_read_cmos_sensor(0x3805);
    dump_5M[12] = OV5640YUV_read_cmos_sensor(0x3806);
    dump_5M[13] = OV5640YUV_read_cmos_sensor(0x3807);
    dump_5M[14] = OV5640YUV_read_cmos_sensor(0x3808);
    dump_5M[15] = OV5640YUV_read_cmos_sensor(0x3809);
    dump_5M[16] = OV5640YUV_read_cmos_sensor(0x380a);
    dump_5M[17] = OV5640YUV_read_cmos_sensor(0x3503);
    dump_5M[18] = OV5640YUV_read_cmos_sensor(0x3000);
    dump_5M[19] = OV5640YUV_read_cmos_sensor(0x3001);
    dump_5M[20] = OV5640YUV_read_cmos_sensor(0x3002);
    dump_5M[21] = OV5640YUV_read_cmos_sensor(0x3003);
    dump_5M[22] = OV5640YUV_read_cmos_sensor(0x3005);
    dump_5M[23] = OV5640YUV_read_cmos_sensor(0x3006);
    dump_5M[24] = OV5640YUV_read_cmos_sensor(0x3007);
    dump_5M[25] = OV5640YUV_read_cmos_sensor(0x350c);
    dump_5M[26] = OV5640YUV_read_cmos_sensor(0x350d);
    dump_5M[27] = OV5640YUV_read_cmos_sensor(0x3602);
    dump_5M[28] = OV5640YUV_read_cmos_sensor(0x380b);
    dump_5M[29] = OV5640YUV_read_cmos_sensor(0x380c);
    dump_5M[30] = OV5640YUV_read_cmos_sensor(0x380d);
    dump_5M[31] = OV5640YUV_read_cmos_sensor(0x380e);
    dump_5M[32] = OV5640YUV_read_cmos_sensor(0x380f);
    dump_5M[33] = OV5640YUV_read_cmos_sensor(0x3810);
    dump_5M[34] = OV5640YUV_read_cmos_sensor(0x3815);
    dump_5M[35] = OV5640YUV_read_cmos_sensor(0x3818);
    dump_5M[36] = OV5640YUV_read_cmos_sensor(0x3824);
    dump_5M[37] = OV5640YUV_read_cmos_sensor(0x3827);
    dump_5M[38] = OV5640YUV_read_cmos_sensor(0x3a00);
    dump_5M[39] = OV5640YUV_read_cmos_sensor(0x3a0d);
    dump_5M[40] = OV5640YUV_read_cmos_sensor(0x3a0e);
    dump_5M[41] = OV5640YUV_read_cmos_sensor(0x3a10);
    dump_5M[42] = OV5640YUV_read_cmos_sensor(0x3a1b);
    dump_5M[43] = OV5640YUV_read_cmos_sensor(0x3a1e);
    dump_5M[44] = OV5640YUV_read_cmos_sensor(0x3a11);
    dump_5M[45] = OV5640YUV_read_cmos_sensor(0x3a1f);
    dump_5M[46] = OV5640YUV_read_cmos_sensor(0x3a00);
    dump_5M[47] = OV5640YUV_read_cmos_sensor(0x460b);
    dump_5M[48] = OV5640YUV_read_cmos_sensor(0x471d);
    dump_5M[49] = OV5640YUV_read_cmos_sensor(0x4713);
    dump_5M[50] = OV5640YUV_read_cmos_sensor(0x471c);
    dump_5M[51] = OV5640YUV_read_cmos_sensor(0x5682);
    dump_5M[52] = OV5640YUV_read_cmos_sensor(0x5683);
    dump_5M[53] = OV5640YUV_read_cmos_sensor(0x5686);
    dump_5M[54] = OV5640YUV_read_cmos_sensor(0x5687);
    dump_5M[55] = OV5640YUV_read_cmos_sensor(0x5001);
    dump_5M[56] = OV5640YUV_read_cmos_sensor(0x589b);
    dump_5M[57] = OV5640YUV_read_cmos_sensor(0x589a);
    dump_5M[58] = OV5640YUV_read_cmos_sensor(0x4407);
    dump_5M[59] = OV5640YUV_read_cmos_sensor(0x589b);
    dump_5M[60] = OV5640YUV_read_cmos_sensor(0x589a);
    dump_5M[61] = OV5640YUV_read_cmos_sensor(0x3002);
    dump_5M[62] = OV5640YUV_read_cmos_sensor(0x3002);
    dump_5M[63] = OV5640YUV_read_cmos_sensor(0x3503);
    dump_5M[64] = OV5640YUV_read_cmos_sensor(0x3010);
    dump_5M[65] = OV5640YUV_read_cmos_sensor(0x3011);
    dump_5M[66] = OV5640YUV_read_cmos_sensor(0x3012);
    dump_5M[67] = OV5640YUV_read_cmos_sensor(0x3009);
    dump_5M[68] = OV5640YUV_read_cmos_sensor(0x300a);

    int i = 0;
    for(i = 0; i < 69; i++)
    {
         PK_DBG("dump_5M[%d] = 0x%x\n", i, dump_5M[i]);
    }
*/
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   OV5640YUVOpen
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

UINT32 OV5640YUVOpen(void)
{
    int  retry = 0; 
	int itmp = 0;
#if WINMO_USE
    OV5640YUVhDrvI2C=DRV_I2COpen(1);
    DRV_I2CSetParam(OV5640YUVhDrvI2C, I2C_VAL_FS_SAMPLE_DIV, 3);
    DRV_I2CSetParam(OV5640YUVhDrvI2C, I2C_VAL_FS_STEP_DIV, 8);
    DRV_I2CSetParam(OV5640YUVhDrvI2C, I2C_VAL_DELAY_LEN, 2);
    DRV_I2CSetParam(OV5640YUVhDrvI2C, I2C_VAL_CLK_EXT, I2C_CLKEXT_DISABLE);
    OV5640YUVI2CConfig.trans_mode=I2C_TRANSFER_FAST_MODE;
    OV5640YUVI2CConfig.slaveAddr=OV5640YUV_sensor_write_I2C_address>>1;
    OV5640YUVI2CConfig.RS_ST=I2C_TRANSFER_STOP;	/* for fast mode */
#endif 

    OV5640YUV_sensor_id = ((OV5640YUV_read_cmos_sensor(0x300A) << 8) | OV5640YUV_read_cmos_sensor(0x300B));
    // check if sensor ID correct
    retry = 3; 
    do {
        OV5640YUV_sensor_id = ((OV5640YUV_read_cmos_sensor(0x300A) << 8) | OV5640YUV_read_cmos_sensor(0x300B));        
        if (OV5640YUV_sensor_id == OV5640_SENSOR_ID)
            break; 
        printk("Read Sensor ID Fail = 0x%04x\n", OV5640YUV_sensor_id); 
        retry--; 
    } while (retry > 0);

    if (OV5640YUV_sensor_id != OV5640_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;

    OV5640YUV_Sensor_Init();


    OV5640YUV_sensor_gain_base = read_OV5640YUV_gain();
    
    OV5640YUV_g_iBackupExtraExp = 0;
    atomic_set(&OV5640_SetGain_Flag, 0); 
    atomic_set(&OV5640_SetShutter_Flag, 0); 
    init_waitqueue_head(&OV5640_SetGain_waitQueue);
    init_waitqueue_head(&OV5640_SetShutter_waitQueue);

	//following add by thp for driving
	itmp = OV5640YUV_read_cmos_sensor(0x302c);
		OV5640YUV_write_cmos_sensor(0x302c ,itmp|0xc0);
	//end add
    return ERROR_NONE;
}



/*************************************************************************
* FUNCTION
*   OV5640YUV_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of OV5640 to change exposure time.
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
void OV5640YUV_SetShutter(kal_uint16 iShutter)
{
#if 0 
    if (iShutter < 4 )
        iShutter = 4;
#else 
    if (iShutter < 1)
        iShutter = 1; 
#endif     

    OV5640YUV_pv_exposure_lines = iShutter;

    //OV5640YUV_imgSensorProfileStart();
    OV5640YUV_write_shutter(iShutter);
    //OV5640YUV_imgSensorProfileEnd("OV5640YUV_SetShutter"); 
    //PK_DBG("iShutter = %d\n", iShutter);

}   /*  OV5640YUV_SetShutter   */



/*************************************************************************
* FUNCTION
*   OV5640YUV_read_shutter
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
UINT16 OV5640YUV_read_shutter(void)
{
    kal_uint8 temp_reg1, temp_reg2, temp_reg3;
    kal_uint16 temp_reg;
    temp_reg1 = OV5640YUV_read_cmos_sensor(0x3500);
    temp_reg2 = OV5640YUV_read_cmos_sensor(0x3501);
    temp_reg3 = OV5640YUV_read_cmos_sensor(0x3502);

   // PK_DBG("ov5640read shutter 0x3500=0x%x,0x3501=0x%x,0x3502=0x%x\n",
	//	temp_reg1,temp_reg2,temp_reg3);
    temp_reg = ((temp_reg1<<12) & 0xF000) | ((temp_reg2<<4) & 0x0FF0) | ((temp_reg3>>4) & 0x0F);

    //PK_DBG("ov5640read shutter = 0x%x\n", temp_reg);
	
    return (UINT16)temp_reg;
}

/*************************************************************************
* FUNCTION
*   OV5640_night_mode
*
* DESCRIPTION
*   This function night mode of OV5640.
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
void OV5640YUV_NightMode(kal_bool bEnable)
{
    if(bEnable)
    {
        if(OV5640YUV_MPEG4_encode_mode==KAL_TRUE)
        {
		OV5640YUV_write_cmos_sensor(0x3a02 ,0x03);
		OV5640YUV_write_cmos_sensor(0x3a03 ,0xd8);
		OV5640YUV_write_cmos_sensor(0x3a14 ,0x03);
		OV5640YUV_write_cmos_sensor(0x3a15 ,0xd8);
		OV5640YUV_write_cmos_sensor(0x3a00, 0x38);
		OV5640YUV_write_cmos_sensor(0x3a19, 0xb0);
        }
        else
        {
		OV5640YUV_write_cmos_sensor(0x3a02 ,0x08);
		OV5640YUV_write_cmos_sensor(0x3a03 ,0xa6);
		OV5640YUV_write_cmos_sensor(0x3a14 ,0x0a);
		OV5640YUV_write_cmos_sensor(0x3a15 ,0x68);
		OV5640YUV_write_cmos_sensor(0x3a00, 0x3c);
		OV5640YUV_write_cmos_sensor(0x3a19, 0xb0);
        }
    }
    else
    {
        if(OV5640YUV_MPEG4_encode_mode==KAL_TRUE)
        {
		OV5640YUV_write_cmos_sensor(0x3a02 ,0x03);
		OV5640YUV_write_cmos_sensor(0x3a03 ,0xd8);
		OV5640YUV_write_cmos_sensor(0x3a14 ,0x03);
		OV5640YUV_write_cmos_sensor(0x3a15 ,0xd8);
		OV5640YUV_write_cmos_sensor(0x3a00, 0x38);
		OV5640YUV_write_cmos_sensor(0x3a19, 0xb0);
        }
        else
        {
		OV5640YUV_write_cmos_sensor(0x3a02 ,0x06);//0x05
		OV5640YUV_write_cmos_sensor(0x3a03 ,0xc1);//0xc4
		OV5640YUV_write_cmos_sensor(0x3a14 ,0x06);//0x06
		OV5640YUV_write_cmos_sensor(0x3a15 ,0xd5);//0xF0
		OV5640YUV_write_cmos_sensor(0x3a00, 0x3c);
		OV5640YUV_write_cmos_sensor(0x3a19, 0xb0);
        }

    }
    
}

/*************************************************************************
* FUNCTION
*   OV5640YUVClose
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
UINT32 OV5640YUVClose(void)
{
    //  CISModulePowerOn(FALSE);

    //s_porting
    //  DRV_I2CClose(OV5640YUVhDrvI2C);
    //e_porting
    return ERROR_NONE;
}	/* OV5640YUVClose() */

/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_AD5820_Init
*
* DESCRIPTION
*   This function is to load micro code for AF function
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
static u8 AD5820_Config[] =
{
      //0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	0x02, 0x0f, 0xd6, 0x02, 0x0a, 0x39, 0xc2, 0x01, 0x22, 0x22, 0x00, 0x02, 0x0f, 0xb2, 0xe5, 0x1f, //0x8000,
	0x70, 0x72, 0xf5, 0x1e, 0xd2, 0x35, 0xff, 0xef, 0x25, 0xe0, 0x24, 0x4e, 0xf8, 0xe4, 0xf6, 0x08, //0x8010,
	0xf6, 0x0f, 0xbf, 0x34, 0xf2, 0x90, 0x0e, 0x93, 0xe4, 0x93, 0xff, 0xe5, 0x4b, 0xc3, 0x9f, 0x50, //0x8020,
	0x04, 0x7f, 0x05, 0x80, 0x02, 0x7f, 0xfb, 0x78, 0xbd, 0xa6, 0x07, 0x12, 0x0f, 0x04, 0x40, 0x04, //0x8030,
	0x7f, 0x03, 0x80, 0x02, 0x7f, 0x30, 0x78, 0xbc, 0xa6, 0x07, 0xe6, 0x18, 0xf6, 0x08, 0xe6, 0x78, //0x8040,
	0xb9, 0xf6, 0x78, 0xbc, 0xe6, 0x78, 0xba, 0xf6, 0x78, 0xbf, 0x76, 0x33, 0xe4, 0x08, 0xf6, 0x78, //0x8050,
	0xb8, 0x76, 0x01, 0x75, 0x4a, 0x02, 0x78, 0xb6, 0xf6, 0x08, 0xf6, 0x74, 0xff, 0x78, 0xc1, 0xf6, //0x8060,
	0x08, 0xf6, 0x75, 0x1f, 0x01, 0x78, 0xbc, 0xe6, 0x75, 0xf0, 0x05, 0xa4, 0xf5, 0x4b, 0x12, 0x0a, //0x8070,
	0xff, 0xc2, 0x37, 0x22, 0x78, 0xb8, 0xe6, 0xd3, 0x94, 0x00, 0x40, 0x02, 0x16, 0x22, 0xe5, 0x1f, //0x8080,
	0xb4, 0x05, 0x23, 0xe4, 0xf5, 0x1f, 0xc2, 0x01, 0x78, 0xb6, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x78, //0x8090,
	0x4e, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0xa2, 0x37, 0xe4, 0x33, 0xf5, 0x3c, 0x90, 0x30, 0x28, 0xf0, //0x80a0,
	0x75, 0x1e, 0x10, 0xd2, 0x35, 0x22, 0xe5, 0x4b, 0x75, 0xf0, 0x05, 0x84, 0x78, 0xbc, 0xf6, 0x90, //0x80b0,
	0x0e, 0x8c, 0xe4, 0x93, 0xff, 0x25, 0xe0, 0x24, 0x0a, 0xf8, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x78, //0x80c0,
	0xbc, 0xe6, 0x25, 0xe0, 0x24, 0x4e, 0xf8, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0xef, 0x12, 0x0f, 0x0b, //0x80d0,
	0xd3, 0x78, 0xb7, 0x96, 0xee, 0x18, 0x96, 0x40, 0x0d, 0x78, 0xbc, 0xe6, 0x78, 0xb9, 0xf6, 0x78, //0x80e0,
	0xb6, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x90, 0x0e, 0x8c, 0xe4, 0x93, 0x12, 0x0f, 0x0b, 0xc3, 0x78, //0x80f0,
	0xc2, 0x96, 0xee, 0x18, 0x96, 0x50, 0x0d, 0x78, 0xbc, 0xe6, 0x78, 0xba, 0xf6, 0x78, 0xc1, 0xa6, //0x8100,
	0x06, 0x08, 0xa6, 0x07, 0x78, 0xb6, 0xe6, 0xfe, 0x08, 0xe6, 0xc3, 0x78, 0xc2, 0x96, 0xff, 0xee, //0x8110,
	0x18, 0x96, 0x78, 0xc3, 0xf6, 0x08, 0xa6, 0x07, 0x90, 0x0e, 0x95, 0xe4, 0x18, 0x12, 0x0e, 0xe9, //0x8120,
	0x40, 0x02, 0xd2, 0x37, 0x78, 0xbc, 0xe6, 0x08, 0x26, 0x08, 0xf6, 0xe5, 0x1f, 0x64, 0x01, 0x70, //0x8130,
	0x4a, 0xe6, 0xc3, 0x78, 0xc0, 0x12, 0x0e, 0xdf, 0x40, 0x05, 0x12, 0x0e, 0xda, 0x40, 0x39, 0x12, //0x8140,
	0x0f, 0x02, 0x40, 0x04, 0x7f, 0xfe, 0x80, 0x02, 0x7f, 0x02, 0x78, 0xbd, 0xa6, 0x07, 0x78, 0xb9, //0x8150,
	0xe6, 0x24, 0x03, 0x78, 0xbf, 0xf6, 0x78, 0xb9, 0xe6, 0x24, 0xfd, 0x78, 0xc0, 0xf6, 0x12, 0x0f, //0x8160,
	0x02, 0x40, 0x06, 0x78, 0xc0, 0xe6, 0xff, 0x80, 0x04, 0x78, 0xbf, 0xe6, 0xff, 0x78, 0xbe, 0xa6, //0x8170,
	0x07, 0x75, 0x1f, 0x02, 0x78, 0xb8, 0x76, 0x01, 0x02, 0x02, 0x4a, 0xe5, 0x1f, 0x64, 0x02, 0x60, //0x8180,
	0x03, 0x02, 0x02, 0x2a, 0x78, 0xbe, 0xe6, 0xff, 0xc3, 0x78, 0xc0, 0x12, 0x0e, 0xe0, 0x40, 0x08, //0x8190,
	0x12, 0x0e, 0xda, 0x50, 0x03, 0x02, 0x02, 0x28, 0x12, 0x0f, 0x02, 0x40, 0x04, 0x7f, 0xff, 0x80, //0x81a0,
	0x02, 0x7f, 0x01, 0x78, 0xbd, 0xa6, 0x07, 0x78, 0xb9, 0xe6, 0x04, 0x78, 0xbf, 0xf6, 0x78, 0xb9, //0x81b0,
	0xe6, 0x14, 0x78, 0xc0, 0xf6, 0x18, 0x12, 0x0f, 0x04, 0x40, 0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, //0x81c0,
	0x00, 0x78, 0xbf, 0xa6, 0x07, 0xd3, 0x08, 0xe6, 0x64, 0x80, 0x94, 0x80, 0x40, 0x04, 0xe6, 0xff, //0x81d0,
	0x80, 0x02, 0x7f, 0x00, 0x78, 0xc0, 0xa6, 0x07, 0xc3, 0x18, 0xe6, 0x64, 0x80, 0x94, 0xb3, 0x50, //0x81e0,
	0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, 0x33, 0x78, 0xbf, 0xa6, 0x07, 0xc3, 0x08, 0xe6, 0x64, 0x80, //0x81f0,
	0x94, 0xb3, 0x50, 0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, 0x33, 0x78, 0xc0, 0xa6, 0x07, 0x12, 0x0f, //0x8200,
	0x02, 0x40, 0x06, 0x78, 0xc0, 0xe6, 0xff, 0x80, 0x04, 0x78, 0xbf, 0xe6, 0xff, 0x78, 0xbe, 0xa6, //0x8210,
	0x07, 0x75, 0x1f, 0x03, 0x78, 0xb8, 0x76, 0x01, 0x80, 0x20, 0xe5, 0x1f, 0x64, 0x03, 0x70, 0x26, //0x8220,
	0x78, 0xbe, 0xe6, 0xff, 0xc3, 0x78, 0xc0, 0x12, 0x0e, 0xe0, 0x40, 0x05, 0x12, 0x0e, 0xda, 0x40, //0x8230,
	0x09, 0x78, 0xb9, 0xe6, 0x78, 0xbe, 0xf6, 0x75, 0x1f, 0x04, 0x78, 0xbe, 0xe6, 0x75, 0xf0, 0x05, //0x8240,
	0xa4, 0xf5, 0x4b, 0x02, 0x0a, 0xff, 0xe5, 0x1f, 0xb4, 0x04, 0x10, 0x90, 0x0e, 0x94, 0xe4, 0x78, //0x8250,
	0xc3, 0x12, 0x0e, 0xe9, 0x40, 0x02, 0xd2, 0x37, 0x75, 0x1f, 0x05, 0x22, 0x30, 0x01, 0x03, 0x02, //0x8260,
	0x04, 0xc0, 0x30, 0x02, 0x03, 0x02, 0x04, 0xc0, 0x90, 0x51, 0xa5, 0xe0, 0x78, 0x93, 0xf6, 0xa3, //0x8270,
	0xe0, 0x08, 0xf6, 0xa3, 0xe0, 0x08, 0xf6, 0xe5, 0x1f, 0x70, 0x3c, 0x75, 0x1e, 0x20, 0xd2, 0x35, //0x8280,
	0x12, 0x0c, 0x7a, 0x78, 0x7e, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x78, 0x8b, 0xa6, 0x09, 0x18, 0x76, //0x8290,
	0x01, 0x12, 0x0c, 0x5b, 0x78, 0x4e, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x78, 0x8b, 0xe6, 0x78, 0x6e, //0x82a0,
	0xf6, 0x75, 0x1f, 0x01, 0x78, 0x93, 0xe6, 0x78, 0x90, 0xf6, 0x78, 0x94, 0xe6, 0x78, 0x91, 0xf6, //0x82b0,
	0x78, 0x95, 0xe6, 0x78, 0x92, 0xf6, 0x22, 0x79, 0x90, 0xe7, 0xd3, 0x78, 0x93, 0x96, 0x40, 0x05, //0x82c0,
	0xe7, 0x96, 0xff, 0x80, 0x08, 0xc3, 0x79, 0x93, 0xe7, 0x78, 0x90, 0x96, 0xff, 0x78, 0x88, 0x76, //0x82d0,
	0x00, 0x08, 0xa6, 0x07, 0x79, 0x91, 0xe7, 0xd3, 0x78, 0x94, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, //0x82e0,
	0x80, 0x08, 0xc3, 0x79, 0x94, 0xe7, 0x78, 0x91, 0x96, 0xff, 0x12, 0x0c, 0x8e, 0x79, 0x92, 0xe7, //0x82f0,
	0xd3, 0x78, 0x95, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, 0x80, 0x08, 0xc3, 0x79, 0x95, 0xe7, 0x78, //0x8300,
	0x92, 0x96, 0xff, 0x12, 0x0c, 0x8e, 0x12, 0x0c, 0x5b, 0x78, 0x8a, 0xe6, 0x25, 0xe0, 0x24, 0x4e, //0x8310,
	0xf8, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x78, 0x8a, 0xe6, 0x24, 0x6e, 0xf8, 0xa6, 0x09, 0x78, 0x8a, //0x8320,
	0xe6, 0x24, 0x01, 0xff, 0xe4, 0x33, 0xfe, 0xd3, 0xef, 0x94, 0x0f, 0xee, 0x64, 0x80, 0x94, 0x80, //0x8330,
	0x40, 0x04, 0x7f, 0x00, 0x80, 0x05, 0x78, 0x8a, 0xe6, 0x04, 0xff, 0x78, 0x8a, 0xa6, 0x07, 0xe5, //0x8340,
	0x1f, 0xb4, 0x01, 0x0a, 0xe6, 0x60, 0x03, 0x02, 0x04, 0xc0, 0x75, 0x1f, 0x02, 0x22, 0x12, 0x0c, //0x8350,
	0x7a, 0x78, 0x80, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x12, 0x0c, 0x7a, 0x78, 0x82, 0xa6, 0x06, 0x08, //0x8360,
	0xa6, 0x07, 0x78, 0x6e, 0xe6, 0x78, 0x8c, 0xf6, 0x78, 0x6e, 0xe6, 0x78, 0x8d, 0xf6, 0x7f, 0x01, //0x8370,
	0xef, 0x25, 0xe0, 0x24, 0x4f, 0xf9, 0xc3, 0x78, 0x81, 0xe6, 0x97, 0x18, 0xe6, 0x19, 0x97, 0x50, //0x8380,
	0x0a, 0x12, 0x0c, 0x82, 0x78, 0x80, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0x74, 0x6e, 0x2f, 0xf9, 0x78, //0x8390,
	0x8c, 0xe6, 0xc3, 0x97, 0x50, 0x08, 0x74, 0x6e, 0x2f, 0xf8, 0xe6, 0x78, 0x8c, 0xf6, 0xef, 0x25, //0x83a0,
	0xe0, 0x24, 0x4f, 0xf9, 0xd3, 0x78, 0x83, 0xe6, 0x97, 0x18, 0xe6, 0x19, 0x97, 0x40, 0x0a, 0x12, //0x83b0,
	0x0c, 0x82, 0x78, 0x82, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0x74, 0x6e, 0x2f, 0xf9, 0x78, 0x8d, 0xe6, //0x83c0,
	0xd3, 0x97, 0x40, 0x08, 0x74, 0x6e, 0x2f, 0xf8, 0xe6, 0x78, 0x8d, 0xf6, 0x0f, 0xef, 0x64, 0x10, //0x83d0,
	0x70, 0x9e, 0xc3, 0x79, 0x81, 0xe7, 0x78, 0x83, 0x96, 0xff, 0x19, 0xe7, 0x18, 0x96, 0x78, 0x84, //0x83e0,
	0xf6, 0x08, 0xa6, 0x07, 0xc3, 0x79, 0x8c, 0xe7, 0x78, 0x8d, 0x96, 0x08, 0xf6, 0xd3, 0x79, 0x81, //0x83f0,
	0xe7, 0x78, 0x7f, 0x96, 0x19, 0xe7, 0x18, 0x96, 0x40, 0x05, 0x09, 0xe7, 0x08, 0x80, 0x06, 0xc3, //0x8400,
	0x79, 0x7f, 0xe7, 0x78, 0x81, 0x96, 0xff, 0x19, 0xe7, 0x18, 0x96, 0xfe, 0x78, 0x86, 0xa6, 0x06, //0x8410,
	0x08, 0xa6, 0x07, 0x79, 0x8c, 0xe7, 0xd3, 0x78, 0x8b, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, 0x80, //0x8420,
	0x08, 0xc3, 0x79, 0x8b, 0xe7, 0x78, 0x8c, 0x96, 0xff, 0x78, 0x8f, 0xa6, 0x07, 0xe5, 0x1f, 0x64, //0x8430,
	0x02, 0x70, 0x69, 0x90, 0x0e, 0x91, 0x93, 0xff, 0x18, 0xe6, 0xc3, 0x9f, 0x50, 0x72, 0x12, 0x0c, //0x8440,
	0x4a, 0x12, 0x0c, 0x2f, 0x90, 0x0e, 0x8e, 0x12, 0x0c, 0x38, 0x78, 0x80, 0x12, 0x0c, 0x6b, 0x7b, //0x8450,
	0x04, 0x12, 0x0c, 0x1d, 0xc3, 0x12, 0x06, 0x45, 0x50, 0x56, 0x90, 0x0e, 0x92, 0xe4, 0x93, 0xff, //0x8460,
	0x78, 0x8f, 0xe6, 0x9f, 0x40, 0x02, 0x80, 0x11, 0x90, 0x0e, 0x90, 0xe4, 0x93, 0xff, 0xd3, 0x78, //0x8470,
	0x89, 0xe6, 0x9f, 0x18, 0xe6, 0x94, 0x00, 0x40, 0x03, 0x75, 0x1f, 0x05, 0x12, 0x0c, 0x4a, 0x12, //0x8480,
	0x0c, 0x2f, 0x90, 0x0e, 0x8f, 0x12, 0x0c, 0x38, 0x78, 0x7e, 0x12, 0x0c, 0x6b, 0x7b, 0x40, 0x12, //0x8490,
	0x0c, 0x1d, 0xd3, 0x12, 0x06, 0x45, 0x40, 0x18, 0x75, 0x1f, 0x05, 0x22, 0xe5, 0x1f, 0xb4, 0x05, //0x84a0,
	0x0f, 0xd2, 0x01, 0xc2, 0x02, 0xe4, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x35, 0xd2, 0x33, 0xd2, 0x36, //0x84b0,
	0x22, 0xef, 0x8d, 0xf0, 0xa4, 0xa8, 0xf0, 0xcf, 0x8c, 0xf0, 0xa4, 0x28, 0xce, 0x8d, 0xf0, 0xa4, //0x84c0,
	0x2e, 0xfe, 0x22, 0xbc, 0x00, 0x0b, 0xbe, 0x00, 0x29, 0xef, 0x8d, 0xf0, 0x84, 0xff, 0xad, 0xf0, //0x84d0,
	0x22, 0xe4, 0xcc, 0xf8, 0x75, 0xf0, 0x08, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xec, 0x33, 0xfc, //0x84e0,
	0xee, 0x9d, 0xec, 0x98, 0x40, 0x05, 0xfc, 0xee, 0x9d, 0xfe, 0x0f, 0xd5, 0xf0, 0xe9, 0xe4, 0xce, //0x84f0,
	0xfd, 0x22, 0xed, 0xf8, 0xf5, 0xf0, 0xee, 0x84, 0x20, 0xd2, 0x1c, 0xfe, 0xad, 0xf0, 0x75, 0xf0, //0x8500,
	0x08, 0xef, 0x2f, 0xff, 0xed, 0x33, 0xfd, 0x40, 0x07, 0x98, 0x50, 0x06, 0xd5, 0xf0, 0xf2, 0x22, //0x8510,
	0xc3, 0x98, 0xfd, 0x0f, 0xd5, 0xf0, 0xea, 0x22, 0xe8, 0x8f, 0xf0, 0xa4, 0xcc, 0x8b, 0xf0, 0xa4, //0x8520,
	0x2c, 0xfc, 0xe9, 0x8e, 0xf0, 0xa4, 0x2c, 0xfc, 0x8a, 0xf0, 0xed, 0xa4, 0x2c, 0xfc, 0xea, 0x8e, //0x8530,
	0xf0, 0xa4, 0xcd, 0xa8, 0xf0, 0x8b, 0xf0, 0xa4, 0x2d, 0xcc, 0x38, 0x25, 0xf0, 0xfd, 0xe9, 0x8f, //0x8540,
	0xf0, 0xa4, 0x2c, 0xcd, 0x35, 0xf0, 0xfc, 0xeb, 0x8e, 0xf0, 0xa4, 0xfe, 0xa9, 0xf0, 0xeb, 0x8f, //0x8550,
	0xf0, 0xa4, 0xcf, 0xc5, 0xf0, 0x2e, 0xcd, 0x39, 0xfe, 0xe4, 0x3c, 0xfc, 0xea, 0xa4, 0x2d, 0xce, //0x8560,
	0x35, 0xf0, 0xfd, 0xe4, 0x3c, 0xfc, 0x22, 0x75, 0xf0, 0x08, 0x75, 0x82, 0x00, 0xef, 0x2f, 0xff, //0x8570,
	0xee, 0x33, 0xfe, 0xcd, 0x33, 0xcd, 0xcc, 0x33, 0xcc, 0xc5, 0x82, 0x33, 0xc5, 0x82, 0x9b, 0xed, //0x8580,
	0x9a, 0xec, 0x99, 0xe5, 0x82, 0x98, 0x40, 0x0c, 0xf5, 0x82, 0xee, 0x9b, 0xfe, 0xed, 0x9a, 0xfd, //0x8590,
	0xec, 0x99, 0xfc, 0x0f, 0xd5, 0xf0, 0xd6, 0xe4, 0xce, 0xfb, 0xe4, 0xcd, 0xfa, 0xe4, 0xcc, 0xf9, //0x85a0,
	0xa8, 0x82, 0x22, 0xb8, 0x00, 0xc1, 0xb9, 0x00, 0x59, 0xba, 0x00, 0x2d, 0xec, 0x8b, 0xf0, 0x84, //0x85b0,
	0xcf, 0xce, 0xcd, 0xfc, 0xe5, 0xf0, 0xcb, 0xf9, 0x78, 0x18, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, //0x85c0,
	0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xeb, 0x33, 0xfb, 0x10, 0xd7, 0x03, 0x99, 0x40, 0x04, 0xeb, //0x85d0,
	0x99, 0xfb, 0x0f, 0xd8, 0xe5, 0xe4, 0xf9, 0xfa, 0x22, 0x78, 0x18, 0xef, 0x2f, 0xff, 0xee, 0x33, //0x85e0,
	0xfe, 0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xc9, 0x33, 0xc9, 0x10, 0xd7, 0x05, 0x9b, 0xe9, 0x9a, //0x85f0,
	0x40, 0x07, 0xec, 0x9b, 0xfc, 0xe9, 0x9a, 0xf9, 0x0f, 0xd8, 0xe0, 0xe4, 0xc9, 0xfa, 0xe4, 0xcc, //0x8600,
	0xfb, 0x22, 0x75, 0xf0, 0x10, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xcc, 0x33, //0x8610,
	0xcc, 0xc8, 0x33, 0xc8, 0x10, 0xd7, 0x07, 0x9b, 0xec, 0x9a, 0xe8, 0x99, 0x40, 0x0a, 0xed, 0x9b, //0x8620,
	0xfd, 0xec, 0x9a, 0xfc, 0xe8, 0x99, 0xf8, 0x0f, 0xd5, 0xf0, 0xda, 0xe4, 0xcd, 0xfb, 0xe4, 0xcc, //0x8630,
	0xfa, 0xe4, 0xc8, 0xf9, 0x22, 0xeb, 0x9f, 0xf5, 0xf0, 0xea, 0x9e, 0x42, 0xf0, 0xe9, 0x9d, 0x42, //0x8640,
	0xf0, 0xe8, 0x9c, 0x45, 0xf0, 0x22, 0xe8, 0x60, 0x0f, 0xec, 0xc3, 0x13, 0xfc, 0xed, 0x13, 0xfd, //0x8650,
	0xee, 0x13, 0xfe, 0xef, 0x13, 0xff, 0xd8, 0xf1, 0x22, 0xe8, 0x60, 0x0f, 0xef, 0xc3, 0x33, 0xff, //0x8660,
	0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xd8, 0xf1, 0x22, 0xe4, 0x93, 0xfc, 0x74, //0x8670,
	0x01, 0x93, 0xfd, 0x74, 0x02, 0x93, 0xfe, 0x74, 0x03, 0x93, 0xff, 0x22, 0xe6, 0xfb, 0x08, 0xe6, //0x8680,
	0xf9, 0x08, 0xe6, 0xfa, 0x08, 0xe6, 0xcb, 0xf8, 0x22, 0xec, 0xf6, 0x08, 0xed, 0xf6, 0x08, 0xee, //0x8690,
	0xf6, 0x08, 0xef, 0xf6, 0x22, 0xa4, 0x25, 0x82, 0xf5, 0x82, 0xe5, 0xf0, 0x35, 0x83, 0xf5, 0x83, //0x86a0,
	0x22, 0xd0, 0x83, 0xd0, 0x82, 0xf8, 0xe4, 0x93, 0x70, 0x12, 0x74, 0x01, 0x93, 0x70, 0x0d, 0xa3, //0x86b0,
	0xa3, 0x93, 0xf8, 0x74, 0x01, 0x93, 0xf5, 0x82, 0x88, 0x83, 0xe4, 0x73, 0x74, 0x02, 0x93, 0x68, //0x86c0,
	0x60, 0xef, 0xa3, 0xa3, 0xa3, 0x80, 0xdf, 0x90, 0x38, 0x04, 0x78, 0x52, 0x12, 0x0b, 0xfd, 0x90, //0x86d0,
	0x38, 0x00, 0xe0, 0xfe, 0xa3, 0xe0, 0xfd, 0xed, 0xff, 0xc3, 0x12, 0x0b, 0x9e, 0x90, 0x38, 0x10, //0x86e0,
	0x12, 0x0b, 0x92, 0x90, 0x38, 0x06, 0x78, 0x54, 0x12, 0x0b, 0xfd, 0x90, 0x38, 0x02, 0xe0, 0xfe, //0x86f0,
	0xa3, 0xe0, 0xfd, 0xed, 0xff, 0xc3, 0x12, 0x0b, 0x9e, 0x90, 0x38, 0x12, 0x12, 0x0b, 0x92, 0xa3, //0x8700,
	0xe0, 0xb4, 0x31, 0x07, 0x78, 0x52, 0x79, 0x52, 0x12, 0x0c, 0x13, 0x90, 0x38, 0x14, 0xe0, 0xb4, //0x8710,
	0x71, 0x15, 0x78, 0x52, 0xe6, 0xfe, 0x08, 0xe6, 0x78, 0x02, 0xce, 0xc3, 0x13, 0xce, 0x13, 0xd8, //0x8720,
	0xf9, 0x79, 0x53, 0xf7, 0xee, 0x19, 0xf7, 0x90, 0x38, 0x15, 0xe0, 0xb4, 0x31, 0x07, 0x78, 0x54, //0x8730,
	0x79, 0x54, 0x12, 0x0c, 0x13, 0x90, 0x38, 0x15, 0xe0, 0xb4, 0x71, 0x15, 0x78, 0x54, 0xe6, 0xfe, //0x8740,
	0x08, 0xe6, 0x78, 0x02, 0xce, 0xc3, 0x13, 0xce, 0x13, 0xd8, 0xf9, 0x79, 0x55, 0xf7, 0xee, 0x19, //0x8750,
	0xf7, 0x79, 0x52, 0x12, 0x0b, 0xd9, 0x09, 0x12, 0x0b, 0xd9, 0xaf, 0x47, 0x12, 0x0b, 0xb2, 0xe5, //0x8760,
	0x44, 0xfb, 0x7a, 0x00, 0xfd, 0x7c, 0x00, 0x12, 0x04, 0xd3, 0x78, 0x5a, 0xa6, 0x06, 0x08, 0xa6, //0x8770,
	0x07, 0xaf, 0x45, 0x12, 0x0b, 0xb2, 0xad, 0x03, 0x7c, 0x00, 0x12, 0x04, 0xd3, 0x78, 0x56, 0xa6, //0x8780,
	0x06, 0x08, 0xa6, 0x07, 0xaf, 0x48, 0x78, 0x54, 0x12, 0x0b, 0xb4, 0xe5, 0x43, 0xfb, 0xfd, 0x7c, //0x8790,
	0x00, 0x12, 0x04, 0xd3, 0x78, 0x5c, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0xaf, 0x46, 0x7e, 0x00, 0x78, //0x87a0,
	0x54, 0x12, 0x0b, 0xb6, 0xad, 0x03, 0x7c, 0x00, 0x12, 0x04, 0xd3, 0x78, 0x58, 0xa6, 0x06, 0x08, //0x87b0,
	0xa6, 0x07, 0xc3, 0x78, 0x5b, 0xe6, 0x94, 0x08, 0x18, 0xe6, 0x94, 0x00, 0x50, 0x05, 0x76, 0x00, //0x87c0,
	0x08, 0x76, 0x08, 0xc3, 0x78, 0x5d, 0xe6, 0x94, 0x08, 0x18, 0xe6, 0x94, 0x00, 0x50, 0x05, 0x76, //0x87d0,
	0x00, 0x08, 0x76, 0x08, 0x78, 0x5a, 0x12, 0x0b, 0xc6, 0xff, 0xd3, 0x78, 0x57, 0xe6, 0x9f, 0x18, //0x87e0,
	0xe6, 0x9e, 0x40, 0x0e, 0x78, 0x5a, 0xe6, 0x13, 0xfe, 0x08, 0xe6, 0x78, 0x57, 0x12, 0x0c, 0x08, //0x87f0,
	0x80, 0x04, 0x7e, 0x00, 0x7f, 0x00, 0x78, 0x5e, 0x12, 0x0b, 0xbe, 0xff, 0xd3, 0x78, 0x59, 0xe6, //0x8800,
	0x9f, 0x18, 0xe6, 0x9e, 0x40, 0x0e, 0x78, 0x5c, 0xe6, 0x13, 0xfe, 0x08, 0xe6, 0x78, 0x59, 0x12, //0x8810,
	0x0c, 0x08, 0x80, 0x04, 0x7e, 0x00, 0x7f, 0x00, 0xe4, 0xfc, 0xfd, 0x78, 0x62, 0x12, 0x06, 0x99, //0x8820,
	0x78, 0x5a, 0x12, 0x0b, 0xc6, 0x78, 0x57, 0x26, 0xff, 0xee, 0x18, 0x36, 0xfe, 0x78, 0x66, 0x12, //0x8830,
	0x0b, 0xbe, 0x78, 0x59, 0x26, 0xff, 0xee, 0x18, 0x36, 0xfe, 0xe4, 0xfc, 0xfd, 0x78, 0x6a, 0x12, //0x8840,
	0x06, 0x99, 0x12, 0x0b, 0xce, 0x78, 0x66, 0x12, 0x06, 0x8c, 0xd3, 0x12, 0x06, 0x45, 0x40, 0x08, //0x8850,
	0x12, 0x0b, 0xce, 0x78, 0x66, 0x12, 0x06, 0x99, 0x78, 0x54, 0x12, 0x0b, 0xd0, 0x78, 0x6a, 0x12, //0x8860,
	0x06, 0x8c, 0xd3, 0x12, 0x06, 0x45, 0x40, 0x0a, 0x78, 0x54, 0x12, 0x0b, 0xd0, 0x78, 0x6a, 0x12, //0x8870,
	0x06, 0x99, 0x78, 0x61, 0xe6, 0x90, 0x60, 0x01, 0xf0, 0x78, 0x65, 0xe6, 0xa3, 0xf0, 0x78, 0x69, //0x8880,
	0xe6, 0xa3, 0xf0, 0x78, 0x55, 0xe6, 0xa3, 0xf0, 0x7d, 0x01, 0x78, 0x61, 0x12, 0x0b, 0xe9, 0x24, //0x8890,
	0x01, 0x12, 0x0b, 0xa6, 0x78, 0x65, 0x12, 0x0b, 0xe9, 0x24, 0x02, 0x12, 0x0b, 0xa6, 0x78, 0x69, //0x88a0,
	0x12, 0x0b, 0xe9, 0x24, 0x03, 0x12, 0x0b, 0xa6, 0x78, 0x6d, 0x12, 0x0b, 0xe9, 0x24, 0x04, 0x12, //0x88b0,
	0x0b, 0xa6, 0x0d, 0xbd, 0x05, 0xd4, 0xc2, 0x0e, 0xc2, 0x06, 0x22, 0x85, 0x08, 0x41, 0x90, 0x30, //0x88c0,
	0x24, 0xe0, 0xf5, 0x3d, 0xa3, 0xe0, 0xf5, 0x3e, 0xa3, 0xe0, 0xf5, 0x3f, 0xa3, 0xe0, 0xf5, 0x40, //0x88d0,
	0xa3, 0xe0, 0xf5, 0x3c, 0xd2, 0x34, 0xe5, 0x41, 0x12, 0x06, 0xb1, 0x09, 0x31, 0x03, 0x09, 0x35, //0x88e0,
	0x04, 0x09, 0x3b, 0x05, 0x09, 0x3e, 0x06, 0x09, 0x41, 0x07, 0x09, 0x4a, 0x08, 0x09, 0x5b, 0x12, //0x88f0,
	0x09, 0x73, 0x18, 0x09, 0x89, 0x19, 0x09, 0x5e, 0x1a, 0x09, 0x6a, 0x1b, 0x09, 0xad, 0x80, 0x09, //0x8900,
	0xb2, 0x81, 0x0a, 0x1d, 0x8f, 0x0a, 0x09, 0x90, 0x0a, 0x1d, 0x91, 0x0a, 0x1d, 0x92, 0x0a, 0x1d, //0x8910,
	0x93, 0x0a, 0x1d, 0x94, 0x0a, 0x1d, 0x98, 0x0a, 0x17, 0x9f, 0x0a, 0x1a, 0xec, 0x00, 0x00, 0x0a, //0x8920,
	0x38, 0x12, 0x0f, 0x74, 0x22, 0x12, 0x0f, 0x74, 0xd2, 0x03, 0x22, 0xd2, 0x03, 0x22, 0xc2, 0x03, //0x8930,
	0x22, 0xa2, 0x37, 0xe4, 0x33, 0xf5, 0x3c, 0x02, 0x0a, 0x1d, 0xc2, 0x01, 0xc2, 0x02, 0xc2, 0x03, //0x8940,
	0x12, 0x0d, 0x0d, 0x75, 0x1e, 0x70, 0xd2, 0x35, 0x02, 0x0a, 0x1d, 0x02, 0x0a, 0x04, 0x85, 0x40, //0x8950,
	0x4a, 0x85, 0x3c, 0x4b, 0x12, 0x0a, 0xff, 0x02, 0x0a, 0x1d, 0x85, 0x4a, 0x40, 0x85, 0x4b, 0x3c, //0x8960,
	0x02, 0x0a, 0x1d, 0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x85, 0x40, 0x31, 0x85, 0x3f, 0x30, 0x85, 0x3e, //0x8970,
	0x2f, 0x85, 0x3d, 0x2e, 0x12, 0x0f, 0x46, 0x80, 0x1f, 0x75, 0x22, 0x00, 0x75, 0x23, 0x01, 0x74, //0x8980,
	0xff, 0xf5, 0x2d, 0xf5, 0x2c, 0xf5, 0x2b, 0xf5, 0x2a, 0x12, 0x0f, 0x46, 0x85, 0x2d, 0x40, 0x85, //0x8990,
	0x2c, 0x3f, 0x85, 0x2b, 0x3e, 0x85, 0x2a, 0x3d, 0xe4, 0xf5, 0x3c, 0x80, 0x70, 0x12, 0x0f, 0x16, //0x89a0,
	0x80, 0x6b, 0x85, 0x3d, 0x45, 0x85, 0x3e, 0x46, 0xe5, 0x47, 0xc3, 0x13, 0xff, 0xe5, 0x45, 0xc3, //0x89b0,
	0x9f, 0x50, 0x02, 0x8f, 0x45, 0xe5, 0x48, 0xc3, 0x13, 0xff, 0xe5, 0x46, 0xc3, 0x9f, 0x50, 0x02, //0x89c0,
	0x8f, 0x46, 0xe5, 0x47, 0xc3, 0x13, 0xff, 0xfd, 0xe5, 0x45, 0x2d, 0xfd, 0xe4, 0x33, 0xfc, 0xe5, //0x89d0,
	0x44, 0x12, 0x0f, 0x90, 0x40, 0x05, 0xe5, 0x44, 0x9f, 0xf5, 0x45, 0xe5, 0x48, 0xc3, 0x13, 0xff, //0x89e0,
	0xfd, 0xe5, 0x46, 0x2d, 0xfd, 0xe4, 0x33, 0xfc, 0xe5, 0x43, 0x12, 0x0f, 0x90, 0x40, 0x05, 0xe5, //0x89f0,
	0x43, 0x9f, 0xf5, 0x46, 0x12, 0x06, 0xd7, 0x80, 0x14, 0x85, 0x40, 0x48, 0x85, 0x3f, 0x47, 0x85, //0x8a00,
	0x3e, 0x46, 0x85, 0x3d, 0x45, 0x80, 0x06, 0x02, 0x06, 0xd7, 0x12, 0x0d, 0x7e, 0x90, 0x30, 0x24, //0x8a10,
	0xe5, 0x3d, 0xf0, 0xa3, 0xe5, 0x3e, 0xf0, 0xa3, 0xe5, 0x3f, 0xf0, 0xa3, 0xe5, 0x40, 0xf0, 0xa3, //0x8a20,
	0xe5, 0x3c, 0xf0, 0x90, 0x30, 0x23, 0xe4, 0xf0, 0x22, 0xc0, 0xe0, 0xc0, 0x83, 0xc0, 0x82, 0xc0, //0x8a30,
	0xd0, 0x90, 0x3f, 0x0c, 0xe0, 0xf5, 0x32, 0xe5, 0x32, 0x30, 0xe3, 0x74, 0x30, 0x36, 0x66, 0x90, //0x8a40,
	0x60, 0x19, 0xe0, 0xf5, 0x0a, 0xa3, 0xe0, 0xf5, 0x0b, 0x90, 0x60, 0x1d, 0xe0, 0xf5, 0x14, 0xa3, //0x8a50,
	0xe0, 0xf5, 0x15, 0x90, 0x60, 0x21, 0xe0, 0xf5, 0x0c, 0xa3, 0xe0, 0xf5, 0x0d, 0x90, 0x60, 0x29, //0x8a60,
	0xe0, 0xf5, 0x0e, 0xa3, 0xe0, 0xf5, 0x0f, 0x90, 0x60, 0x31, 0xe0, 0xf5, 0x10, 0xa3, 0xe0, 0xf5, //0x8a70,
	0x11, 0x90, 0x60, 0x39, 0xe0, 0xf5, 0x12, 0xa3, 0xe0, 0xf5, 0x13, 0x30, 0x01, 0x06, 0x30, 0x33, //0x8a80,
	0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x09, 0x30, 0x02, 0x06, 0x30, 0x33, 0x03, 0xd3, 0x80, 0x01, //0x8a90,
	0xc3, 0x92, 0x0a, 0x30, 0x33, 0x0c, 0x30, 0x03, 0x09, 0x20, 0x02, 0x06, 0x20, 0x01, 0x03, 0xd3, //0x8aa0,
	0x80, 0x01, 0xc3, 0x92, 0x0b, 0x90, 0x30, 0x01, 0xe0, 0x44, 0x40, 0xf0, 0xe0, 0x54, 0xbf, 0xf0, //0x8ab0,
	0xe5, 0x32, 0x30, 0xe1, 0x14, 0x30, 0x34, 0x11, 0x90, 0x30, 0x22, 0xe0, 0xf5, 0x08, 0xe4, 0xf0, //0x8ac0,
	0x30, 0x00, 0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x08, 0xe5, 0x32, 0x30, 0xe5, 0x12, 0x90, 0x56, //0x8ad0,
	0xa1, 0xe0, 0xf5, 0x09, 0x30, 0x31, 0x09, 0x30, 0x05, 0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x0d, //0x8ae0,
	0x90, 0x3f, 0x0c, 0xe5, 0x32, 0xf0, 0xd0, 0xd0, 0xd0, 0x82, 0xd0, 0x83, 0xd0, 0xe0, 0x32, 0x90, //0x8af0,
	0x0e, 0x7e, 0xe4, 0x93, 0xfe, 0x74, 0x01, 0x93, 0xff, 0xc3, 0x90, 0x0e, 0x7c, 0x74, 0x01, 0x93, //0x8b00,
	0x9f, 0xff, 0xe4, 0x93, 0x9e, 0xfe, 0xe4, 0x8f, 0x3b, 0x8e, 0x3a, 0xf5, 0x39, 0xf5, 0x38, 0xab, //0x8b10,
	0x3b, 0xaa, 0x3a, 0xa9, 0x39, 0xa8, 0x38, 0xaf, 0x4b, 0xfc, 0xfd, 0xfe, 0x12, 0x05, 0x28, 0x12, //0x8b20,
	0x0d, 0xe1, 0xe4, 0x7b, 0xff, 0xfa, 0xf9, 0xf8, 0x12, 0x05, 0xb3, 0x12, 0x0d, 0xe1, 0x90, 0x0e, //0x8b30,
	0x69, 0xe4, 0x12, 0x0d, 0xf6, 0x12, 0x0d, 0xe1, 0xe4, 0x85, 0x4a, 0x37, 0xf5, 0x36, 0xf5, 0x35, //0x8b40,
	0xf5, 0x34, 0xaf, 0x37, 0xae, 0x36, 0xad, 0x35, 0xac, 0x34, 0xa3, 0x12, 0x0d, 0xf6, 0x8f, 0x37, //0x8b50,
	0x8e, 0x36, 0x8d, 0x35, 0x8c, 0x34, 0xe5, 0x3b, 0x45, 0x37, 0xf5, 0x3b, 0xe5, 0x3a, 0x45, 0x36, //0x8b60,
	0xf5, 0x3a, 0xe5, 0x39, 0x45, 0x35, 0xf5, 0x39, 0xe5, 0x38, 0x45, 0x34, 0xf5, 0x38, 0xe4, 0xf5, //0x8b70,
	0x22, 0xf5, 0x23, 0x85, 0x3b, 0x31, 0x85, 0x3a, 0x30, 0x85, 0x39, 0x2f, 0x85, 0x38, 0x2e, 0x02, //0x8b80,
	0x0f, 0x46, 0xe0, 0xa3, 0xe0, 0x75, 0xf0, 0x02, 0xa4, 0xff, 0xae, 0xf0, 0xc3, 0x08, 0xe6, 0x9f, //0x8b90,
	0xf6, 0x18, 0xe6, 0x9e, 0xf6, 0x22, 0xff, 0xe5, 0xf0, 0x34, 0x60, 0x8f, 0x82, 0xf5, 0x83, 0xec, //0x8ba0,
	0xf0, 0x22, 0x78, 0x52, 0x7e, 0x00, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x02, 0x04, 0xc1, 0xe4, 0xfc, //0x8bb0,
	0xfd, 0x12, 0x06, 0x99, 0x78, 0x5c, 0xe6, 0xc3, 0x13, 0xfe, 0x08, 0xe6, 0x13, 0x22, 0x78, 0x52, //0x8bc0,
	0xe6, 0xfe, 0x08, 0xe6, 0xff, 0xe4, 0xfc, 0xfd, 0x22, 0xe7, 0xc4, 0xf8, 0x54, 0xf0, 0xc8, 0x68, //0x8bd0,
	0xf7, 0x09, 0xe7, 0xc4, 0x54, 0x0f, 0x48, 0xf7, 0x22, 0xe6, 0xfc, 0xed, 0x75, 0xf0, 0x04, 0xa4, //0x8be0,
	0x22, 0x12, 0x06, 0x7c, 0x8f, 0x48, 0x8e, 0x47, 0x8d, 0x46, 0x8c, 0x45, 0x22, 0xe0, 0xfe, 0xa3, //0x8bf0,
	0xe0, 0xfd, 0xee, 0xf6, 0xed, 0x08, 0xf6, 0x22, 0x13, 0xff, 0xc3, 0xe6, 0x9f, 0xff, 0x18, 0xe6, //0x8c00,
	0x9e, 0xfe, 0x22, 0xe6, 0xc3, 0x13, 0xf7, 0x08, 0xe6, 0x13, 0x09, 0xf7, 0x22, 0xad, 0x39, 0xac, //0x8c10,
	0x38, 0xfa, 0xf9, 0xf8, 0x12, 0x05, 0x28, 0x8f, 0x3b, 0x8e, 0x3a, 0x8d, 0x39, 0x8c, 0x38, 0xab, //0x8c20,
	0x37, 0xaa, 0x36, 0xa9, 0x35, 0xa8, 0x34, 0x22, 0x93, 0xff, 0xe4, 0xfc, 0xfd, 0xfe, 0x12, 0x05, //0x8c30,
	0x28, 0x8f, 0x37, 0x8e, 0x36, 0x8d, 0x35, 0x8c, 0x34, 0x22, 0x78, 0x84, 0xe6, 0xfe, 0x08, 0xe6, //0x8c40,
	0xff, 0xe4, 0x8f, 0x37, 0x8e, 0x36, 0xf5, 0x35, 0xf5, 0x34, 0x22, 0x90, 0x0e, 0x8c, 0xe4, 0x93, //0x8c50,
	0x25, 0xe0, 0x24, 0x0a, 0xf8, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x22, 0xe6, 0xfe, 0x08, 0xe6, 0xff, //0x8c60,
	0xe4, 0x8f, 0x3b, 0x8e, 0x3a, 0xf5, 0x39, 0xf5, 0x38, 0x22, 0x78, 0x4e, 0xe6, 0xfe, 0x08, 0xe6, //0x8c70,
	0xff, 0x22, 0xef, 0x25, 0xe0, 0x24, 0x4e, 0xf8, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x22, 0x78, 0x89, //0x8c80,
	0xef, 0x26, 0xf6, 0x18, 0xe4, 0x36, 0xf6, 0x22, 0x75, 0x89, 0x03, 0x75, 0xa8, 0x01, 0x75, 0xb8, //0x8c90,
	0x04, 0x75, 0x34, 0xff, 0x75, 0x35, 0x0e, 0x75, 0x36, 0x15, 0x75, 0x37, 0x0d, 0x12, 0x0e, 0x9a, //0x8ca0,
	0x12, 0x00, 0x09, 0x12, 0x0f, 0x16, 0x12, 0x00, 0x06, 0xd2, 0x00, 0xd2, 0x34, 0xd2, 0xaf, 0x75, //0x8cb0,
	0x34, 0xff, 0x75, 0x35, 0x0e, 0x75, 0x36, 0x49, 0x75, 0x37, 0x03, 0x12, 0x0e, 0x9a, 0x30, 0x08, //0x8cc0,
	0x09, 0xc2, 0x34, 0x12, 0x08, 0xcb, 0xc2, 0x08, 0xd2, 0x34, 0x30, 0x0b, 0x09, 0xc2, 0x36, 0x12, //0x8cd0,
	0x02, 0x6c, 0xc2, 0x0b, 0xd2, 0x36, 0x30, 0x09, 0x09, 0xc2, 0x36, 0x12, 0x00, 0x0e, 0xc2, 0x09, //0x8ce0,
	0xd2, 0x36, 0x30, 0x0e, 0x03, 0x12, 0x06, 0xd7, 0x30, 0x35, 0xd3, 0x90, 0x30, 0x29, 0xe5, 0x1e, //0x8cf0,
	0xf0, 0xb4, 0x10, 0x05, 0x90, 0x30, 0x23, 0xe4, 0xf0, 0xc2, 0x35, 0x80, 0xc1, 0xe4, 0xf5, 0x4b, //0x8d00,
	0x90, 0x0e, 0x7a, 0x93, 0xff, 0xe4, 0x8f, 0x37, 0xf5, 0x36, 0xf5, 0x35, 0xf5, 0x34, 0xaf, 0x37, //0x8d10,
	0xae, 0x36, 0xad, 0x35, 0xac, 0x34, 0x90, 0x0e, 0x6a, 0x12, 0x0d, 0xf6, 0x8f, 0x37, 0x8e, 0x36, //0x8d20,
	0x8d, 0x35, 0x8c, 0x34, 0x90, 0x0e, 0x72, 0x12, 0x06, 0x7c, 0xef, 0x45, 0x37, 0xf5, 0x37, 0xee, //0x8d30,
	0x45, 0x36, 0xf5, 0x36, 0xed, 0x45, 0x35, 0xf5, 0x35, 0xec, 0x45, 0x34, 0xf5, 0x34, 0xe4, 0xf5, //0x8d40,
	0x22, 0xf5, 0x23, 0x85, 0x37, 0x31, 0x85, 0x36, 0x30, 0x85, 0x35, 0x2f, 0x85, 0x34, 0x2e, 0x12, //0x8d50,
	0x0f, 0x46, 0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x90, 0x0e, 0x72, 0x12, 0x0d, 0xea, 0x12, 0x0f, 0x46, //0x8d60,
	0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x90, 0x0e, 0x6e, 0x12, 0x0d, 0xea, 0x02, 0x0f, 0x46, 0xe5, 0x40, //0x8d70,
	0x24, 0xf2, 0xf5, 0x37, 0xe5, 0x3f, 0x34, 0x43, 0xf5, 0x36, 0xe5, 0x3e, 0x34, 0xa2, 0xf5, 0x35, //0x8d80,
	0xe5, 0x3d, 0x34, 0x28, 0xf5, 0x34, 0xe5, 0x37, 0xff, 0xe4, 0xfe, 0xfd, 0xfc, 0x78, 0x18, 0x12, //0x8d90,
	0x06, 0x69, 0x8f, 0x40, 0x8e, 0x3f, 0x8d, 0x3e, 0x8c, 0x3d, 0xe5, 0x37, 0x54, 0xa0, 0xff, 0xe5, //0x8da0,
	0x36, 0xfe, 0xe4, 0xfd, 0xfc, 0x78, 0x07, 0x12, 0x06, 0x56, 0x78, 0x10, 0x12, 0x0f, 0x9a, 0xe4, //0x8db0,
	0xff, 0xfe, 0xe5, 0x35, 0xfd, 0xe4, 0xfc, 0x78, 0x0e, 0x12, 0x06, 0x56, 0x12, 0x0f, 0x9d, 0xe4, //0x8dc0,
	0xff, 0xfe, 0xfd, 0xe5, 0x34, 0xfc, 0x78, 0x18, 0x12, 0x06, 0x56, 0x78, 0x08, 0x12, 0x0f, 0x9a, //0x8dd0,
	0x22, 0x8f, 0x3b, 0x8e, 0x3a, 0x8d, 0x39, 0x8c, 0x38, 0x22, 0x12, 0x06, 0x7c, 0x8f, 0x31, 0x8e, //0x8de0,
	0x30, 0x8d, 0x2f, 0x8c, 0x2e, 0x22, 0x93, 0xf9, 0xf8, 0x02, 0x06, 0x69, 0x00, 0x00, 0x00, 0x00, //0x8df0,
	0x12, 0x01, 0x17, 0x08, 0x31, 0x15, 0x53, 0x54, 0x44, 0x20, 0x20, 0x20, 0x20, 0x20, 0x13, 0x01, //0x8e00,
	0x10, 0x01, 0x56, 0x40, 0x1a, 0x30, 0x29, 0x7e, 0x00, 0x30, 0x04, 0x20, 0xdf, 0x30, 0x05, 0x40, //0x8e10,
	0xbf, 0x50, 0x03, 0x00, 0xfd, 0x50, 0x27, 0x01, 0xfe, 0x60, 0x00, 0x11, 0x00, 0x3f, 0x05, 0x30, //0x8e20,
	0x00, 0x3f, 0x06, 0x22, 0x00, 0x3f, 0x01, 0x2a, 0x00, 0x3f, 0x02, 0x00, 0x00, 0x36, 0x06, 0x07, //0x8e30,
	0x00, 0x3f, 0x0b, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x30, 0x01, 0x40, 0xbf, 0x30, 0x01, 0x00, //0x8e40,
	0xbf, 0x30, 0x29, 0x70, 0x00, 0x3a, 0x00, 0x00, 0xff, 0x3a, 0x00, 0x00, 0xff, 0x36, 0x03, 0x36, //0x8e50,
	0x02, 0x41, 0x44, 0x58, 0x20, 0x18, 0x10, 0x0a, 0x04, 0x04, 0x00, 0x03, 0xff, 0x64, 0x00, 0x00, //0x8e60,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x06, 0x00, 0x03, 0x51, 0x00, 0x7a, //0x8e70,
	0x50, 0x3c, 0x28, 0x1e, 0x10, 0x10, 0x50, 0x2d, 0x28, 0x16, 0x10, 0x10, 0x02, 0x00, 0x10, 0x0c, //0x8e80,
	0x10, 0x04, 0x0c, 0x6e, 0x06, 0x05, 0x00, 0xa5, 0x5a, 0x00, 0xae, 0x35, 0xaf, 0x36, 0xe4, 0xfd, //0x8e90,
	0xed, 0xc3, 0x95, 0x37, 0x50, 0x33, 0x12, 0x0f, 0xe2, 0xe4, 0x93, 0xf5, 0x38, 0x74, 0x01, 0x93, //0x8ea0,
	0xf5, 0x39, 0x45, 0x38, 0x60, 0x23, 0x85, 0x39, 0x82, 0x85, 0x38, 0x83, 0xe0, 0xfc, 0x12, 0x0f, //0x8eb0,
	0xe2, 0x74, 0x03, 0x93, 0x52, 0x04, 0x12, 0x0f, 0xe2, 0x74, 0x02, 0x93, 0x42, 0x04, 0x85, 0x39, //0x8ec0,
	0x82, 0x85, 0x38, 0x83, 0xec, 0xf0, 0x0d, 0x80, 0xc7, 0x22, 0x78, 0xbe, 0xe6, 0xd3, 0x08, 0xff, //0x8ed0,
	0xe6, 0x64, 0x80, 0xf8, 0xef, 0x64, 0x80, 0x98, 0x22, 0x93, 0xff, 0x7e, 0x00, 0xe6, 0xfc, 0x08, //0x8ee0,
	0xe6, 0xfd, 0x12, 0x04, 0xc1, 0x78, 0xc1, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0xd3, 0xef, 0x9d, 0xee, //0x8ef0,
	0x9c, 0x22, 0x78, 0xbd, 0xd3, 0xe6, 0x64, 0x80, 0x94, 0x80, 0x22, 0x25, 0xe0, 0x24, 0x0a, 0xf8, //0x8f00,
	0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x22, 0xe5, 0x3c, 0xd3, 0x94, 0x00, 0x40, 0x0b, 0x90, 0x0e, 0x88, //0x8f10,
	0x12, 0x0b, 0xf1, 0x90, 0x0e, 0x86, 0x80, 0x09, 0x90, 0x0e, 0x82, 0x12, 0x0b, 0xf1, 0x90, 0x0e, //0x8f20,
	0x80, 0xe4, 0x93, 0xf5, 0x44, 0xa3, 0xe4, 0x93, 0xf5, 0x43, 0xd2, 0x06, 0x30, 0x06, 0x03, 0xd3, //0x8f30,
	0x80, 0x01, 0xc3, 0x92, 0x0e, 0x22, 0xa2, 0xaf, 0x92, 0x32, 0xc2, 0xaf, 0xe5, 0x23, 0x45, 0x22, //0x8f40,
	0x90, 0x0e, 0x5d, 0x60, 0x0e, 0x12, 0x0f, 0xcb, 0xe0, 0xf5, 0x2c, 0x12, 0x0f, 0xc8, 0xe0, 0xf5, //0x8f50,
	0x2d, 0x80, 0x0c, 0x12, 0x0f, 0xcb, 0xe5, 0x30, 0xf0, 0x12, 0x0f, 0xc8, 0xe5, 0x31, 0xf0, 0xa2, //0x8f60,
	0x32, 0x92, 0xaf, 0x22, 0xd2, 0x01, 0xc2, 0x02, 0xe4, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x35, 0xd2, //0x8f70,
	0x33, 0xd2, 0x36, 0xd2, 0x01, 0xc2, 0x02, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x35, 0xd2, 0x33, 0x22, //0x8f80,
	0xfb, 0xd3, 0xed, 0x9b, 0x74, 0x80, 0xf8, 0x6c, 0x98, 0x22, 0x12, 0x06, 0x69, 0xe5, 0x40, 0x2f, //0x8f90,
	0xf5, 0x40, 0xe5, 0x3f, 0x3e, 0xf5, 0x3f, 0xe5, 0x3e, 0x3d, 0xf5, 0x3e, 0xe5, 0x3d, 0x3c, 0xf5, //0x8fa0,
	0x3d, 0x22, 0xc0, 0xe0, 0xc0, 0x83, 0xc0, 0x82, 0x90, 0x3f, 0x0d, 0xe0, 0xf5, 0x33, 0xe5, 0x33, //0x8fb0,
	0xf0, 0xd0, 0x82, 0xd0, 0x83, 0xd0, 0xe0, 0x32, 0x90, 0x0e, 0x5f, 0xe4, 0x93, 0xfe, 0x74, 0x01, //0x8fc0,
	0x93, 0xf5, 0x82, 0x8e, 0x83, 0x22, 0x78, 0x7f, 0xe4, 0xf6, 0xd8, 0xfd, 0x75, 0x81, 0xcd, 0x02, //0x8fd0,
	0x0c, 0x98, 0x8f, 0x82, 0x8e, 0x83, 0x75, 0xf0, 0x04, 0xed, 0x02, 0x06, 0xa5,                   //0x8fe0

    }; 

static void OV5640_FOCUS_AD5820_Init(void)
{
    UINT8 state=0x8F;
    UINT32 iteration = 100;
    int totalCnt = 0; 
    int sentCnt = 0; 
    int index=0; 
    u16 addr = 0x8000; 
    u8 buf[256]; 
    int len = 128;

    PK_DBG("\n");     

    OV5640YUV_imgSensorProfileStart();
    OV5640YUV_write_cmos_sensor(0x3000, 0x20);
    totalCnt = ARRAY_SIZE(AD5820_Config); 
    PK_DBG("Total Count = %d\n", totalCnt);

    #if 0//Brust mode
    while (index < totalCnt) {        
        sentCnt = totalCnt - index > len  ? len  : totalCnt - index; 
        //PK_DBG("Index = %d, sent Cnt = %d, addr = 0x%x\n", index, sentCnt, addr); 
        buf[0] = addr >> 8; 
        buf[1] = addr & 0xff; 
        memcpy(&buf[2], &AD5820_Config[index], len );
        OV5640YUV_burst_write_cmos_sensor(buf, sentCnt + 2); 
        addr += len ; 
        index += len ;
    }
   #else//single mode
//download firmware 1th	thp
    for(sentCnt =0; sentCnt<totalCnt; sentCnt++)
    {
		OV5640YUV_write_cmos_sensor(addr,AD5820_Config[sentCnt]);
		addr++;
    }
    #endif
	mdelay(10);
//init cmd reg 2th	thp
    OV5640YUV_write_cmos_sensor(0x3022, 0x00);
    OV5640YUV_write_cmos_sensor(0x3023, 0x00);
    OV5640YUV_write_cmos_sensor(0x3024, 0x00);
    OV5640YUV_write_cmos_sensor(0x3025, 0x00);
    OV5640YUV_write_cmos_sensor(0x3026, 0x00);
    OV5640YUV_write_cmos_sensor(0x3027, 0x00);
    OV5640YUV_write_cmos_sensor(0x3028, 0x00);
    OV5640YUV_write_cmos_sensor(0x3029, 0x7f);
    OV5640YUV_write_cmos_sensor(0x3000, 0x00);

    do {
        state = (UINT8)OV5640YUV_read_cmos_sensor(0x3029);
	 PK_DBG("when init af, state=0x%x\n",state);	
	 
       // Sleep(1);
	mdelay(5);
        if (iteration-- == 0)
        {
            RETAILMSG(1, (TEXT("[OV5640]STA_FOCUS state check ERROR!!0x%x \r\n")), state);
            break;
        }
    } while(state!=0x70);
    OV5640YUV_imgSensorProfileEnd("OV5640_FOCUS_AD5820_Init");
    OV5640_FOCUS_AD5820_Check_MCU();
    return;    
}   /*  OV5640_FOCUS_AD5820_Init  */




// Step to Specified position
static void OV5640_FOCUS_AD5820_Move_to(UINT16 a_u2MovePosition)
{
	//don't support for ov5640
}

static UINT32 check_retry = 0;
static int itmp = 0;
#define AGOLD_RATE_CONTROL 5
static void OV5640_FOCUS_AD5820_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
/*
    UINT32 state = OV5640YUV_read_cmos_sensor(0x3028);

    if (state == 0)
    {
        *pFeatureReturnPara32 = 0;
    }
    else
    {
        *pFeatureReturnPara32 = 1;
    }
   */

   UINT32 state;
	int state1 = 0;
	int state2 = 0;
	int state3 = 0;
	int state4 = 0;
	int state5 = 0;
	int itmp_int = 0;
   PK_DBG("OV5640_FOCUS_AD5820_Get_AF_Status\n");
   
/*	itmp_int = 0;
	do
	{
		state1 = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
		state2 = (UINT8)OV5640YUV_read_cmos_sensor(0x3028);
		state3 = (UINT8)OV5640YUV_read_cmos_sensor(0x3029);
		state4 = (UINT8)OV5640YUV_read_cmos_sensor(0x3001);
		state5 = (UINT8)OV5640YUV_read_cmos_sensor(0x3005);
		PK_DBG("[tongreg]0x3023=%x, 0x3028=%x, 0x3029=%x, 0x3001=%x, 0x3005=%x[Cnt: %d]\n", state1, state2, state3, state4, state5, itmp_int);
		mdelay(1);
		itmp_int++;	
		if (itmp_int > AGOLD_RATE_CONTROL)
		{
			break;		
		}
}while((state1 != 0x00) || (state2 == 0x0) || (state3 != 0x10));*/

	if(g_sensorAfStatus == SENSOR_AF_FOCUSED)
	{
		*pFeatureReturnPara32 = g_sensorAfStatus;
		return;
	}
		

		state1 = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
		state2 = (UINT8)OV5640YUV_read_cmos_sensor(0x3028);
		state3 = (UINT8)OV5640YUV_read_cmos_sensor(0x3029);
        
    if((state1 == 0x00) && (state2 != 0x00) && (state3 == 0x10))
    {
        PK_DBG("focused,%d!\n", itmp);
	itmp = 0;
        g_sensorAfStatus = SENSOR_AF_FOCUSED;    
    }
    else
    {
    		itmp++;
		PK_DBG("[tongreg]0x3023=%x, 0x3028=%x, 0x3029=%x,[Cnt: %d]\n", state1, state2, state3,itmp);
		//mdelay(1);
    		if(itmp > 1000)
    		{
				PK_DBG("over time! error in focus!!\n");
    				g_sensorAfStatus = SENSOR_AF_FOCUSED;
    				itmp = 0;
    		}
    			
    }
   *pFeatureReturnPara32 = g_sensorAfStatus;
   
}

static void OV5640_FOCUS_AD5820_Get_AF_Inf(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}

static void OV5640_FOCUS_AD5820_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 1023;
}

//update focus window
//@input zone[] addr
static void OV5640_FOCUS_AD5820_Set_AF_Window(UINT32 *zone_addr)
{//update global zone
    UINT8 state = 0x8F;

    //input:
    UINT32 times = 1;
    UINT32 FD_XS = 4;
    UINT32 FD_YS = 3;	
    UINT32 x0, y0, x1, y1;
    UINT32* zone = (UINT32*)zone_addr;
    x0 = *zone;
    y0 = *(zone + 1);
    x1 = *(zone + 2);
    y1 = *(zone + 3);	
    FD_XS = *(zone + 4);
    FD_YS = *(zone + 5);
    times = FD_XS / AF_XS;
    PK_DBG("x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
		x0, y0, x1, y1, FD_XS, FD_YS);	
	
    x0 = (UINT8)(x0 / times);   
    y0 = (UINT8)(y0 / times);   
    x1 = (UINT8)(x1 / times);   
    y1 = (UINT8)(y1 / times);   	

//zone changed, update global zone.
      ZONE[0]=x0;
      ZONE[1]=y0;
      ZONE[2]=x1;
      ZONE[3]=y1;
}

//set focus zone coordinates to sensor IC
//global zone[]
static void OV5640_FOCUS_AD5820_Set_AF_Window_to_IC(void)
{
#if 1
    UINT8 x_center, y_center;
    UINT8 state = 0x8F;
    UINT32 iteration = 100;  
    UINT8 cmd_reg = 0x8F;
	
    x_center = (ZONE[0] + ZONE[2]) / 2;	
    y_center = (ZONE[1] + ZONE[3]) / 2;	

    //send array mode 5 -- floate center mode cmd to firmware
    OV5640YUV_write_cmos_sensor(0x3022,0x81);//cmd_tag
    OV5640YUV_write_cmos_sensor(0x3023,0x01);
    OV5640YUV_write_cmos_sensor(0x3024,x_center);//cmd_para1, x_center
    OV5640YUV_write_cmos_sensor(0x3025,y_center);//cmd_para0, y_center
/*
    do{
        state = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
            PK_DBG("float center mode CMD_TAG= 0x%0x\n",state);
            Sleep(1);
            if(iteration-- == 0)
            {
                PK_DBG("float center mode wait CMD_TAG zero out of time!!!! %x\n",state);
                break;
            }

    }while(state != 0x00);	
*/
    return;
#endif
}

//update zone[]
static void OV5640_FOCUS_AD5820_Update_Zone(void)
{
#if 0
    UINT8 state = 0x8F;    
    UINT32 iteration = 100;  
    UINT8 cmd_reg = 0x8F;
    //send update zone cmd to firmware
    OV5640YUV_write_cmos_sensor(0x3025,0x01);//cmd_tag
    OV5640YUV_write_cmos_sensor(0x3024,0x12);

    do{
        state = (UINT8)OV5640YUV_read_cmos_sensor(0x3025);
            PK_DBG("update zone CMD_TAG= 0x%0x\n",state);
            Sleep(1);
            if(iteration-- == 0)
            {
                PK_DBG("update zone wait CMD_TAG zero out of time!!!! %x\n",state);
                break;
            }

    }while(state != 0x00);
    return;
#endif
}

//set constant focus
static void OV5640_FOCUS_AD5820_Constant_Focus(void)
{
    UINT8 state = 0x8F;
    UINT32 iteration = 300;
    g_sensorAfStatus = SENSOR_AF_FOCUSING;
	PK_DBG("OV5640_FOCUS_AD5820_Constant_Focus\n");
/*
    //send idle command to firmware
    OV5640YUV_write_cmos_sensor(0x3023,0x01);
    OV5640YUV_write_cmos_sensor(0x3022,0x08);

    iteration = 300;
    do {
    	 state = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
        if (iteration-- == 0)
        {
            RETAILMSG(1, (TEXT("[OV5640]AD5820_Single_Focus time out !!0x%x \r\n")), state);
            return ;
        }   
        Sleep(1);
    } while(state!=0x00); 
*/
    //send constant focus mode command to firmware
    OV5640YUV_write_cmos_sensor(0x3022,0x04);
    OV5640YUV_write_cmos_sensor(0x3023,0x01);
/* chenlang removed
    iteration = 1000;
    do {
    	 state = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
        
        if(state == 0x00)
        {
            PK_DBG("single focused!\n");
            g_sensorAfStatus = SENSOR_AF_FOCUSED;
            break;
        }
        Sleep(1);
        iteration --;
    }while(iteration);    
 */
    return;
}

static void OV5640_FOCUS_AD5820_Single_Focus()
{


    UINT8 state = 0x8F;
    UINT8 state_ack = 0x8F;	
    UINT8 state_cmd = 0x8F;		
    UINT32 iteration = 300;
     PK_DBG("OV5640_FOCUS_AD5820_Single_Focus\n");
    g_sensorAfStatus = SENSOR_AF_FOCUSING;

//1.update zone
    //OV5640_FOCUS_AD5820_Update_Zone();

//2.change focus window
    //OV5640_FOCUS_AD5820_Set_AF_Window_to_IC();

//3.update zone
    //OV5640_FOCUS_AD5820_Update_Zone();

//4.send single focus mode command to firmware

	itmp = 0;
	g_sensorAfStatus = SENSOR_AF_IDLE;

	OV5640YUV_write_cmos_sensor(0x3023,0x01);
    	OV5640YUV_write_cmos_sensor(0x3022,0x03);


/*	mdelay(1);
	if ((UINT8)OV5640YUV_read_cmos_sensor(0x3023) != 0)
	{
		PK_DBG("Single Focus Ack err!\n");
		g_sensorAfStatus = SENSOR_AF_FOCUSING;
		return ;
	}
	mdelay(1);

	if ((UINT8)OV5640YUV_read_cmos_sensor(0x3028) == 0)
	{
		g_sensorAfStatus = SENSOR_AF_FOCUSING;
		PK_DBG("Single Focus read CMD_PARA4 failed! go on focusing!!!\n");
		return ;
	}*/


    PK_DBG("After single focus...  \n");

//5.after sigle focus cmd, check the STA_FOCUS until S_FOCUSED 0x10
/* chenlang removed
    iteration = 1000;  
    do{
        state = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
        PK_DBG("test,Single state = 0x%x,state_ack=0x%x,state_cmd=0x%x\n",state,state_ack,state_cmd);
        if(state == 0x00)
        {
            PK_DBG("single focused!\n");
            g_sensorAfStatus = SENSOR_AF_FOCUSED;
            break;
        }			
        Sleep(1);
        iteration --;
    }while(iteration);
  */  
    return;
}

static void OV5640_FOCUS_AD5820_Pause_Focus()
{
    UINT8 state = 0x8F;
    UINT32 iteration = 300;
     PK_DBG("OV5640_FOCUS_AD5820_Pause_Focus\n");
    //send idle command to firmware
    OV5640YUV_write_cmos_sensor(0x3023,0x01);
    OV5640YUV_write_cmos_sensor(0x3022,0x06);

    iteration = 100;  
    do{
        state = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
        
        if(state == 0x00)
        {
            PK_DBG("idle!\n");
            g_sensorAfStatus = SENSOR_AF_IDLE;
            break;
        }			
        Sleep(1);
        iteration --;

    }while(iteration);

}
static void OV5640_FOCUS_AD5820_Cancel_Focus()
{
     PK_DBG("OV5640_FOCUS_AD5820_Cancel_Focus\n");

    UINT8 state = 0x8F;
    UINT8 state1 = 0x00;
    UINT32 iteration = 300;

    //send idle command to firmware
    OV5640YUV_write_cmos_sensor(0x3023,0x01);
    OV5640YUV_write_cmos_sensor(0x3022,0x08);
	
    iteration = 100;  
    do{
        state = (UINT8)OV5640YUV_read_cmos_sensor(0x3023);
	state1 = (UINT8)OV5640YUV_read_cmos_sensor(0x3029);
        if((state == 0x00) && (state1 == 0x70))
    {
            PK_DBG("Cancel_Focus:idle!\n");
            g_sensorAfStatus = SENSOR_AF_IDLE;
            break;
    }
        Sleep(1);
        iteration --;
		
    }while(iteration);
			
    }

void OV5640YUV_Set_Mirror_Flip(kal_uint8 image_mirror)
{
	UINT32 curr_mirror = 0, curr_flip = 0;

	// TODO: For debug the V project only.
	image_mirror = IMAGE_H_MIRROR;
	
	//OV5640_sensor_mirror_flip = image_mirror;

	curr_flip = (OV5640YUV_read_cmos_sensor(0x3820) & 0xF9);		/* bit[1]-Sensor Flip, bit[2]-ISP Flip */
	curr_mirror = (OV5640YUV_read_cmos_sensor(0x3821) & 0xF9);	/* bit[1]-Sensor Mirror, bit[2]-ISP Mirror */	
	
	switch (image_mirror)
	{
	case IMAGE_NORMAL:
		/* Do nothing. */
		break;
	case IMAGE_H_MIRROR:
		curr_mirror |= 0x06; 
		break;
	case IMAGE_V_MIRROR:
		curr_flip |= 0x06;
		break;
	case IMAGE_HV_MIRROR:
		curr_mirror |= 0x06;
		curr_flip |= 0x06;
		break;
	default:
		break;
	}

	#if defined(AGOLD_OV5640_YUV_HV_MIRROR) //[Agold][huyl]
	 	curr_mirror = curr_mirror ^0x06;	//02
	 	curr_flip = curr_flip ^0x06;
	#elif defined(AGOLD_OV5640_YUV_H_MIRROR)
		curr_mirror = curr_mirror ^0x06;
	#elif defined(AGOLD_OV5640_YUV_V_MIRROR)
		curr_flip = curr_flip ^0x06;
	#endif

	OV5640YUV_write_cmos_sensor(0x3820, curr_flip);
	OV5640YUV_write_cmos_sensor(0x3821, curr_mirror);
}

/*************************************************************************
* FUNCTION
*   OV5640YUVPreview
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
UINT32 OV5640YUVPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
	int itmp = 0;
	  iCapTmp = 0;

    g_iOV5640YUV_Mode = OV5640_MODE_PREVIEW;

    //if(OV5640_720P == OV5640YUV_g_RES)
    {
        OV5640YUV_set_720P();
    }

    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        OV5640YUV_MPEG4_encode_mode = KAL_TRUE;
    }
    else
    {
        OV5640YUV_MPEG4_encode_mode = KAL_FALSE;
    }

    iStartX += OV5640_IMAGE_SENSOR_PV_STARTX;
    iStartY += OV5640_IMAGE_SENSOR_PV_STARTY;

    //sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR; 
    
    OV5640YUV_Set_Mirror_Flip(sensor_config_data->SensorImageMirror);

#if 0    
    iTemp = OV5640YUV_read_cmos_sensor(0x3818) & 0x9f;	//Clear the mirror and flip bits.
    switch (sensor_config_data->SensorImageMirror)
    {
        case IMAGE_NORMAL:
            OV5640YUV_write_cmos_sensor(0x3818, iTemp | 0xc0);	//Set normal
            OV5640YUV_write_cmos_sensor(0x3621, 0xc7);
            //SET_FIRST_GRAB_COLOR(BAYER_Gr);
            break;

        case IMAGE_V_MIRROR:
            OV5640YUV_write_cmos_sensor(0x3818, iTemp | 0xe0);	//Set flip
            OV5640YUV_write_cmos_sensor(0x3621, 0xc7);

            //SET_FIRST_GRAB_COLOR(BAYER_B);
            break;

        case IMAGE_H_MIRROR:
            OV5640YUV_write_cmos_sensor(0x3818, iTemp | 0x80);	//Set mirror
            OV5640YUV_write_cmos_sensor(0x3621, 0xe7);
            //SET_FIRST_GRAB_COLOR(BAYER_Gr);
        break;

        case IMAGE_HV_MIRROR:
            OV5640YUV_write_cmos_sensor(0x3818, iTemp | 0xa0);	//Set mirror and flip
            OV5640YUV_write_cmos_sensor(0x3621, 0xe7);
            //SET_FIRST_GRAB_COLOR(BAYER_B);
            break;
    }
#endif     

    OV5640YUV_dummy_pixels = 0;  
    OV5640YUV_dummy_lines = 0;
    OV5640YUV_PV_dummy_pixels = OV5640YUV_dummy_pixels;
    OV5640YUV_PV_dummy_lines = OV5640YUV_dummy_lines;

    OV5640YUV_SetDummy(OV5640YUV_dummy_pixels, OV5640YUV_dummy_lines);
    //OV5640YUV_SetShutter(OV5640YUV_pv_exposure_lines);

    memcpy(&OV5640YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    image_window->GrabStartX= iStartX;
    image_window->GrabStartY= iStartY;
    image_window->ExposureWindowWidth= OV5640_IMAGE_SENSOR_PV_WIDTH - 2*iStartX;
    image_window->ExposureWindowHeight= OV5640_IMAGE_SENSOR_PV_HEIGHT - 2*iStartY;

	//mdelay(500); // yxw 

//following add by thp for driving
	itmp = OV5640YUV_read_cmos_sensor(0x302c);
		OV5640YUV_write_cmos_sensor(0x302c ,itmp|0xc0);
	//end add

printk("[tong]0x302c=%x\n", OV5640YUV_read_cmos_sensor(0x302c));

    return ERROR_NONE;
}	/* OV5640YUVPreview() */


UINT32 OV5640YUVCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    //kal_uint32 shutter=OV5640YUV_pv_exposure_lines;
    kal_uint32 shutter = 0;
    kal_uint32 temp_shutter = 0;	
    kal_uint16 iStartX = 0, iStartY = 0;
    kal_uint32 pv_gain = 0;	
    kal_uint8 temp = 0;//for night mode	

// chenlang add, to fix multi shot AE issue
	if (iCapTmp == 1)
	{ 
		return 0;
	}
	
	iCapTmp = 1;
	
	// chenlang , end
   
    PK_DBG("Preview Shutter = %d, Gain = %d\n", shutter, read_OV5640YUV_gain());    

    g_iOV5640YUV_Mode = OV5640_MODE_CAPTURE;

    if(sensor_config_data->EnableShutterTansfer==KAL_TRUE)
        shutter=sensor_config_data->CaptureShutter;


//1. disable night mode
    temp= OV5640YUV_read_cmos_sensor(0x3a00);
    OV5640YUV_write_cmos_sensor(0x3a00,temp&0xfb);
//2. disable AE,AWB
    OV5640YUV_set_AE_mode(KAL_FALSE);
    OV5640YUV_set_AWB_mode(KAL_FALSE);

//3. read shutter, gain
    shutter = OV5640YUV_read_shutter();
    pv_gain = read_OV5640YUV_gain();
//4. set 5M mode

    if ((image_window->ImageTargetWidth<= OV5640_IMAGE_SENSOR_PV_WIDTH) &&
        (image_window->ImageTargetHeight<= OV5640_IMAGE_SENSOR_PV_HEIGHT)) {
        OV5640YUV_dummy_pixels= 0;
        OV5640YUV_dummy_lines = 0;

        //shutter = ((UINT32)(shutter*(OV5640_PV_PERIOD_PIXEL_NUMS_HTS  + OV5640YUV_PV_dummy_pixels)))/
        //                                                ((OV5640_FULL_PERIOD_PIXEL_NUMS_HTS + OV5640YUV_dummy_pixels)) ;
        //shutter = shutter * OV5640YUV_CAP_pclk / OV5640YUV_PV_pclk; 
        
        iStartX = OV5640_IMAGE_SENSOR_PV_STARTX;
        iStartY = OV5640_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX=iStartX;
        image_window->GrabStartY=iStartY;
        image_window->ExposureWindowWidth=OV5640_IMAGE_SENSOR_PV_WIDTH - 2*iStartX;
        image_window->ExposureWindowHeight=OV5640_IMAGE_SENSOR_PV_HEIGHT- 2*iStartY;
    }
    else { // 5M  Mode
        OV5640YUV_dummy_pixels= 0;
        OV5640YUV_dummy_lines = 0;        
        OV5640YUV_set_5M();
        //sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR;      
        //OV5640SetFlipMirror(sensor_config_data->SensorImageMirror); 

 	//OV5640YUV_CAP_pclk = 5600;
 	//OV5640YUV_PV_pclk = 5600;		
	//To avoid overflow...
        OV5640YUV_CAP_pclk = 560;
        OV5640YUV_PV_pclk = 560;		
        temp_shutter = (shutter*(OV5640_PV_PERIOD_PIXEL_NUMS_HTS+OV5640YUV_PV_dummy_pixels)*OV5640YUV_CAP_pclk)
        			/(OV5640_FULL_PERIOD_PIXEL_NUMS_HTS+OV5640YUV_dummy_pixels)/OV5640YUV_PV_pclk;	
        
        shutter = (kal_uint32)(temp_shutter);
        PK_DBG("cap shutter calutaed = %d, 0x%x\n", shutter,shutter);
		//shutter = shutter*2; 
        //SVGA Internal CLK = 1/4 UXGA Internal CLK
        //shutter = 4* shutter;
       // shutter = ((UINT32)(shutter*(OV5640_IMAGE_SENSOR_720P_PIXELS_LINE + OV5640_PV_PERIOD_EXTRA_PIXEL_NUMS + OV5640YUV_PV_dummy_pixels)))/
        //                                                ((OV5640_IMAGE_SENSOR_5M_PIXELS_LINE+ OV5640_FULL_PERIOD_EXTRA_PIXEL_NUMS + OV5640YUV_dummy_pixels)) ;
        //shutter = shutter * OV5640YUV_CAP_pclk / OV5640YUV_PV_pclk; 
        iStartX = 2* OV5640_IMAGE_SENSOR_PV_STARTX;
        iStartY = 2* OV5640_IMAGE_SENSOR_PV_STARTY;

        image_window->GrabStartX=iStartX;
        image_window->GrabStartY=iStartY;
        image_window->ExposureWindowWidth=OV5640_IMAGE_SENSOR_FULL_WIDTH -2*iStartX;
        image_window->ExposureWindowHeight=OV5640_IMAGE_SENSOR_FULL_HEIGHT-2*iStartY;
    }//5M Capture
    // config flashlight preview setting
    if(OV5640_5M == OV5640YUV_g_RES) //add start
    {
        sensor_config_data->DefaultPclk = 32500000;
        sensor_config_data->Pixels = OV5640_IMAGE_SENSOR_5M_PIXELS_LINE + OV5640YUV_PV_dummy_pixels;
        sensor_config_data->FrameLines =OV5640_PV_PERIOD_LINE_NUMS+OV5640YUV_PV_dummy_lines;
    }
    else
    {
        sensor_config_data->DefaultPclk = 32500000;
        sensor_config_data->Pixels = OV5640_IMAGE_SENSOR_5M_PIXELS_LINE+OV5640YUV_dummy_pixels;
        sensor_config_data->FrameLines =OV5640_FULL_PERIOD_LINE_NUMS+OV5640YUV_dummy_lines;
    }

    sensor_config_data->Lines = image_window->ExposureWindowHeight;
    sensor_config_data->Shutter =shutter;

    OV5640YUV_SetDummy(OV5640YUV_dummy_pixels, OV5640YUV_dummy_lines);

//6.set shutter
    OV5640YUV_SetShutter(shutter);    
//7.set gain
    write_OV5640YUV_gain(pv_gain);	
//aec, awb is close?
    //PK_DBG("aec reg0x3503=0x%x, awb reg0x3406=0x%x\n",
    //OV5640YUV_read_cmos_sensor(0x3503),
    //OV5640YUV_read_cmos_sensor(0x3406));
    //PK_DBG("Capture Shutter = %d, Gain = %d\n", shutter, read_OV5640YUV_gain());     
    memcpy(&OV5640YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
#ifdef AGOLD_JHGG_CAPTURE_MIRROR
OV5640YUV_Set_Mirror_Flip(sensor_config_data->SensorImageMirror);
#endif
    return ERROR_NONE;
}	/* OV5640YUVCapture() */

UINT32 OV5640YUVGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH - 4*OV5640_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT - 4*OV5640_IMAGE_SENSOR_PV_STARTY;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH - 2*OV5640_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT - 2*OV5640_IMAGE_SENSOR_PV_STARTY;

    return ERROR_NONE;
}   /* OV5640YUVGetResolution() */
UINT32 OV5640YUVGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH - 2*OV5640_IMAGE_SENSOR_PV_STARTX;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT - 2*OV5640_IMAGE_SENSOR_PV_STARTY;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH - 4*OV5640_IMAGE_SENSOR_PV_STARTX;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT - 4*OV5640_IMAGE_SENSOR_PV_STARTY;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

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

    //pSensorInfo->CaptureDelayFrame = 1; 
    pSensorInfo->CaptureDelayFrame = 2; 
    pSensorInfo->PreviewDelayFrame = 2; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;      

//
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 1;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 1;	
	   
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
            pSensorInfo->SensorGrabStartX = OV5640_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = OV5640_IMAGE_SENSOR_PV_STARTY;             
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
            pSensorInfo->SensorClockFreq=26;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = OV5640_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = OV5640_IMAGE_SENSOR_PV_STARTY;             
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

    OV5640YUVPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &OV5640YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* OV5640YUVGetInfo() */


UINT32 OV5640YUVControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            OV5640YUVPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
            OV5640YUVCapture(pImageWindow, pSensorConfigData);
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
} /* OV5640YUVControl() */


UINT32 OV5640YUVSetVideoMode(UINT16 u2FrameRate)
{
    return TRUE;
}
kal_uint32 OV5640_set_param_wb(kal_uint32 para)
{

    switch (para)
    {
        case AWB_MODE_AUTO:
		OV5640YUV_write_cmos_sensor(0x3212, 0x03);
		OV5640YUV_write_cmos_sensor(0x3406, 0x00);
		OV5640YUV_write_cmos_sensor(0x3400, 0x04);
		OV5640YUV_write_cmos_sensor(0x3401, 0x00);
		OV5640YUV_write_cmos_sensor(0x3402, 0x04);
		OV5640YUV_write_cmos_sensor(0x3403, 0x00);
		OV5640YUV_write_cmos_sensor(0x3404, 0x04);
		OV5640YUV_write_cmos_sensor(0x3405, 0x00);
		OV5640YUV_write_cmos_sensor(0x3212, 0x13);
		OV5640YUV_write_cmos_sensor(0x3212, 0xa3);
          break;

        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy           
              OV5640YUV_write_cmos_sensor(0x3212, 0x03);
		OV5640YUV_write_cmos_sensor(0x3406, 0x01);
		OV5640YUV_write_cmos_sensor(0x3400, 0x06);
		OV5640YUV_write_cmos_sensor(0x3401, 0x48);
		OV5640YUV_write_cmos_sensor(0x3402, 0x04);
		OV5640YUV_write_cmos_sensor(0x3403, 0x00);
		OV5640YUV_write_cmos_sensor(0x3404, 0x04);
		OV5640YUV_write_cmos_sensor(0x3405, 0xd3);
		OV5640YUV_write_cmos_sensor(0x3212, 0x13);
		OV5640YUV_write_cmos_sensor(0x3212, 0xa3);            
          break;

        case AWB_MODE_DAYLIGHT: //sunny            
              OV5640YUV_write_cmos_sensor(0x3212, 0x03);
		OV5640YUV_write_cmos_sensor(0x3406, 0x01);
		OV5640YUV_write_cmos_sensor(0x3400, 0x06);
		OV5640YUV_write_cmos_sensor(0x3401, 0x1c);
		OV5640YUV_write_cmos_sensor(0x3402, 0x04);
		OV5640YUV_write_cmos_sensor(0x3403, 0x00);
		OV5640YUV_write_cmos_sensor(0x3404, 0x04);
		OV5640YUV_write_cmos_sensor(0x3405, 0xf3);
		OV5640YUV_write_cmos_sensor(0x3212, 0x13);
		OV5640YUV_write_cmos_sensor(0x3212, 0xa3);            
          break;

        case AWB_MODE_INCANDESCENT: //office
		OV5640YUV_write_cmos_sensor(0x3212, 0x03);
		OV5640YUV_write_cmos_sensor(0x3406, 0x01);
		OV5640YUV_write_cmos_sensor(0x3400, 0x05);
		OV5640YUV_write_cmos_sensor(0x3401, 0x48);
		OV5640YUV_write_cmos_sensor(0x3402, 0x04);
		OV5640YUV_write_cmos_sensor(0x3403, 0x00);
		OV5640YUV_write_cmos_sensor(0x3404, 0x07);
		OV5640YUV_write_cmos_sensor(0x3405, 0xcf);
		OV5640YUV_write_cmos_sensor(0x3212, 0x13);
		OV5640YUV_write_cmos_sensor(0x3212, 0xa3);
          break;

        case AWB_MODE_TUNGSTEN: //home
		OV5640YUV_write_cmos_sensor(0x3212, 0x03);
		OV5640YUV_write_cmos_sensor(0x3406, 0x01);
		OV5640YUV_write_cmos_sensor(0x3400, 0x04);
		OV5640YUV_write_cmos_sensor(0x3401, 0x10);
		OV5640YUV_write_cmos_sensor(0x3402, 0x04);
		OV5640YUV_write_cmos_sensor(0x3403, 0x00);
		OV5640YUV_write_cmos_sensor(0x3404, 0x08);
		OV5640YUV_write_cmos_sensor(0x3405, 0x40);
		OV5640YUV_write_cmos_sensor(0x3212, 0x13);
		OV5640YUV_write_cmos_sensor(0x3212, 0xa3);
          break;

        case AWB_MODE_FLUORESCENT:            
              OV5640YUV_write_cmos_sensor(0x3212, 0x03);
		OV5640YUV_write_cmos_sensor(0x3406, 0x01);
		OV5640YUV_write_cmos_sensor(0x3400, 0x05);
		OV5640YUV_write_cmos_sensor(0x3401, 0x8c);
		OV5640YUV_write_cmos_sensor(0x3402, 0x04);
		OV5640YUV_write_cmos_sensor(0x3403, 0x00);
		OV5640YUV_write_cmos_sensor(0x3404, 0x06);
		OV5640YUV_write_cmos_sensor(0x3405, 0xe8);
		OV5640YUV_write_cmos_sensor(0x3212, 0x13);
		OV5640YUV_write_cmos_sensor(0x3212, 0xa3);          
          break;
        default:
            return KAL_FALSE;
    }
    //[agold][sea][add for iwb]
    OV5640YUVCurrentStatus.iWB = para;
    //[agold][sea][end]
    return KAL_TRUE;
} 

kal_uint32 OV5640_set_param_exposure(kal_uint32 para)
{
    switch (para)
    {
        case AE_EV_COMP_n10:
		OV5640YUV_write_cmos_sensor(0x3a0f, 0x20);
		OV5640YUV_write_cmos_sensor(0x3a10, 0x18);
		OV5640YUV_write_cmos_sensor(0x3a11, 0x40);
		OV5640YUV_write_cmos_sensor(0x3a1b, 0x20);
		OV5640YUV_write_cmos_sensor(0x3a1e, 0x18);
		OV5640YUV_write_cmos_sensor(0x3a1f, 0x10);
        break;

        case AE_EV_COMP_n07:
		OV5640YUV_write_cmos_sensor(0x3a0f, 0x28);
		OV5640YUV_write_cmos_sensor(0x3a10, 0x20);
		OV5640YUV_write_cmos_sensor(0x3a11, 0x40);
		OV5640YUV_write_cmos_sensor(0x3a1b, 0x28);
		OV5640YUV_write_cmos_sensor(0x3a1e, 0x20);
		OV5640YUV_write_cmos_sensor(0x3a1f, 0x10);
        break;

        case AE_EV_COMP_n03:
		OV5640YUV_write_cmos_sensor(0x3a0f, 0x30);
		OV5640YUV_write_cmos_sensor(0x3a10, 0x28);
		OV5640YUV_write_cmos_sensor(0x3a11, 0x61);
		OV5640YUV_write_cmos_sensor(0x3a1b, 0x30);
		OV5640YUV_write_cmos_sensor(0x3a1e, 0x28);
		OV5640YUV_write_cmos_sensor(0x3a1f, 0x10);
        break;

        case AE_EV_COMP_00:
		OV5640YUV_write_cmos_sensor(0x3a0f, 0x38);
		OV5640YUV_write_cmos_sensor(0x3a10, 0x30);
		OV5640YUV_write_cmos_sensor(0x3a11, 0x61);
		OV5640YUV_write_cmos_sensor(0x3a1b, 0x38);
		OV5640YUV_write_cmos_sensor(0x3a1e, 0x30);
		OV5640YUV_write_cmos_sensor(0x3a1f, 0x10);
        break;

        case AE_EV_COMP_03:
		OV5640YUV_write_cmos_sensor(0x3a0f, 0x40);
		OV5640YUV_write_cmos_sensor(0x3a10, 0x38);
		OV5640YUV_write_cmos_sensor(0x3a11, 0x71);
		OV5640YUV_write_cmos_sensor(0x3a1b, 0x40);
		OV5640YUV_write_cmos_sensor(0x3a1e, 0x38);
		OV5640YUV_write_cmos_sensor(0x3a1f, 0x10);
        break;

        case AE_EV_COMP_07:
		OV5640YUV_write_cmos_sensor(0x3a0f, 0x50);
		OV5640YUV_write_cmos_sensor(0x3a10, 0x48);
		OV5640YUV_write_cmos_sensor(0x3a11, 0x90);
		OV5640YUV_write_cmos_sensor(0x3a1b, 0x50);
		OV5640YUV_write_cmos_sensor(0x3a1e, 0x48);
		OV5640YUV_write_cmos_sensor(0x3a1f, 0x20);
        break;

        case AE_EV_COMP_10:
		OV5640YUV_write_cmos_sensor(0x3a0f, 0x60);
		OV5640YUV_write_cmos_sensor(0x3a10, 0x58);
		OV5640YUV_write_cmos_sensor(0x3a11, 0xa0);
		OV5640YUV_write_cmos_sensor(0x3a1b, 0x60);
		OV5640YUV_write_cmos_sensor(0x3a1e, 0x58);
		OV5640YUV_write_cmos_sensor(0x3a1f, 0x20);
        break;

        default:
            return KAL_FALSE;
    }
    return KAL_TRUE;
} 

kal_uint32 OV5640_set_param_effect(kal_uint32 para)
{
    kal_uint32 ret = KAL_TRUE;

    switch (para)
    {
        case MEFFECT_OFF:            
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x06);
			OV5640YUV_write_cmos_sensor(0x5583, 0x40);
			#ifdef AGOLD_GDS_SATURATION
			OV5640YUV_write_cmos_sensor(0x5584, 0x70);
			#else
			OV5640YUV_write_cmos_sensor(0x5584, 0x10);
			#endif
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);
            break;

	 case MEFFECT_MONO:
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x1e);
			OV5640YUV_write_cmos_sensor(0x5583, 0x80);
			OV5640YUV_write_cmos_sensor(0x5584, 0x80);
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);
            break;
			
        case MEFFECT_SEPIA:         
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x1e);
			OV5640YUV_write_cmos_sensor(0x5583, 0x40);
			OV5640YUV_write_cmos_sensor(0x5584, 0xa0);
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);           
            break;

        case MEFFECT_NEGATIVE:
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x40);
			OV5640YUV_write_cmos_sensor(0x5583, 0x40);
			OV5640YUV_write_cmos_sensor(0x5584, 0x10);
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);
            break;

        case MEFFECT_SEPIAGREEN:           
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x1e);
			OV5640YUV_write_cmos_sensor(0x5583, 0x60);
			OV5640YUV_write_cmos_sensor(0x5584, 0x60);
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);           
            break;
 /*           
         case CAM_EFFECT_ENC_REDDISH:           
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x1e);
			OV5640YUV_write_cmos_sensor(0x5583, 0x80);
			OV5640YUV_write_cmos_sensor(0x5584, 0xc0);
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);           
            break;
*/
        case MEFFECT_SEPIABLUE:           
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x1e);
			OV5640YUV_write_cmos_sensor(0x5583, 0xa0);
			OV5640YUV_write_cmos_sensor(0x5584, 0x40);
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);                      
            break;
       /*     
        case CAM_EFFECT_ENC_GRAYSCALE:     
			OV5640YUV_write_cmos_sensor(0x3212, 0x03);
			OV5640YUV_write_cmos_sensor(0x5580, 0x1e);
			OV5640YUV_write_cmos_sensor(0x5583, 0x80);
			OV5640YUV_write_cmos_sensor(0x5584, 0x80);
			OV5640YUV_write_cmos_sensor(0x5003, 0x08);
			OV5640YUV_write_cmos_sensor(0x3212, 0x13);
			OV5640YUV_write_cmos_sensor(0x3212, 0xa3);
        	break;
        */
/*
        case CAM_EFFECT_ENC_GRAYINV:
        case CAM_EFFECT_ENC_COPPERCARVING:
        case CAM_EFFECT_ENC_BLUECARVING:
        case CAM_EFFECT_ENC_CONTRAST:
        case CAM_EFFECT_ENC_EMBOSSMENT:
        case CAM_EFFECT_ENC_SKETCH:
        case CAM_EFFECT_ENC_BLACKBOARD:
        case CAM_EFFECT_ENC_WHITEBOARD:
        case CAM_EFFECT_ENC_JEAN:
        case CAM_EFFECT_ENC_OIL:
*/
        default:
            ret = KAL_FALSE;
	return ret;
    }

    return KAL_TRUE;
} 

kal_uint32 OV5640_set_param_banding(kal_uint32 para)
{
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
		OV5640YUV_write_cmos_sensor(0x3a09,0x9F); //94
		OV5640YUV_write_cmos_sensor(0x3a0e,0x06);
		OV5640YUV_write_cmos_sensor(0x3c00,0x04);
		OV5640YUV_write_cmos_sensor(0x3c01,0x80);
	break;

        case AE_FLICKER_MODE_60HZ:
		OV5640YUV_write_cmos_sensor(0x3a0b, 0x7b); 
		OV5640YUV_write_cmos_sensor(0x3a0d, 0x08);
		OV5640YUV_write_cmos_sensor(0x3c00,0x00);
		OV5640YUV_write_cmos_sensor(0x3c01,0x80);
	break;

	default:
	  return KAL_FALSE;
    }
    return KAL_TRUE;
} 

UINT32 OV5640YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{

	PK_DBG("OV5640YUVSensorSetting: FID_SCENE_MODE=%d, cmd=%d, para = 0x%x\n", FID_SCENE_MODE, iCmd, iPara);

	switch (iCmd) {
	case FID_SCENE_MODE:	    
	    if (iPara == SCENE_MODE_OFF)
	    {
	        OV5640YUV_NightMode(FALSE); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
			OV5640YUV_NightMode(TRUE); 
	    }	    
	break; 	    
	case FID_AWB_MODE:
		OV5640_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
		OV5640_set_param_effect(iPara);
	break;
	case FID_AE_EV:
		OV5640_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
		OV5640_set_param_banding(iPara);
	break;
	case FID_ZOOM_FACTOR:
        OV5640YUV_zoom_factor = iPara; 		
	break;
		default:
			PK_DBG("Default:%d", iCmd);
	break;
    }

    return TRUE;

}

/*************************************************************************
* FUNCTION
*   OV5640YUVGetSensorID
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
UINT32 OV5640YUVGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
    
    // check if sensor ID correct
    do {
        *sensorID = ((OV5640YUV_read_cmos_sensor(0x300A) << 8) | OV5640YUV_read_cmos_sensor(0x300B));        
        if (*sensorID == OV5640_SENSOR_ID)
            break; 
        PK_DBG("Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
    } while (retry > 0);

    if (*sensorID != OV5640_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

//[agold][sea][add for iwb]
void OV5640YUVGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 1;    
    PK_DBG("*pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);

}

void OV5640YUVGetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 1;    
    PK_DBG(" OV5640YUVGetAFMaxNumMeteringAreas,*pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);

}

void OV5640YUVGetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = OV5640YUVCurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}
//[agold][sea][end]

UINT32 OV5640YUVFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{    
    UINT8   *pFeatureData8 =pFeaturePara;
    
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

#if WINMO_USE
        case SENSOR_FEATURE_GET_INIT_OPERATION_PARA:
        {
            PCAMERA_DRIVER_OPERATION_PARA_STRUCT pSensorOperData;
            pSensorOperData = (PCAMERA_DRIVER_OPERATION_PARA_STRUCT)pFeaturePara;

            pSensorOperData->CaptureDelayFrame = 2;         /* wait stable frame when sensor change mode (pre to cap) */
            pSensorOperData->PreviewDelayFrame = 3;         /* wait stable frame when sensor change mode (cap to pre) */
            pSensorOperData->PreviewDisplayWaitFrame = 2;   /* wait stable frame when sensor change mode (cap to pre) */
            pSensorOperData->AECalDelayFrame = 0;               /* The frame of calculation default 0 */
            pSensorOperData->AEShutDelayFrame = 0;              /* The frame of setting shutter default 0 for TG int */
            pSensorOperData->AESensorGainDelayFrame = 1;    /* The frame of setting sensor gain */
            pSensorOperData->AEISPGainDelayFrame = 2;          /* The frame of setting gain */
            pSensorOperData->AECalPeriod = 3;                           /* AE AWB calculation period */
            pSensorOperData->FlashlightMode=FLASHLIGHT_LED_CONSTANT;

            break;
        }
#endif 

        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
            *pFeatureReturnPara16++=OV5640_PV_PERIOD_EXTRA_PIXEL_NUMS + OV5640_PV_PERIOD_PIXEL_NUMS + OV5640YUV_dummy_pixels;//OV5640_PV_PERIOD_PIXEL_NUMS+OV5640YUV_dummy_pixels;
            *pFeatureReturnPara16=OV5640_PV_PERIOD_LINE_NUMS+OV5640YUV_dummy_lines;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *pFeatureReturnPara32 = 55250000; //19500000;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            OV5640YUV_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV5640YUV_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            OV5640YUV_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            OV5640YUV_isp_master_clock=*pFeatureData32;
            printk("MASTER_CLOCK_FREQ:%d", OV5640YUV_isp_master_clock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV5640YUV_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV5640YUV_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
                OV5640YUVSensorCCT[i].Addr=*pFeatureData32++;
                OV5640YUVSensorCCT[i].Para=*pFeatureData32++;
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV5640YUVSensorCCT[i].Addr;
                *pFeatureData32++=OV5640YUVSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
                OV5640YUVSensorReg[i].Addr=*pFeatureData32++;
                OV5640YUVSensorReg[i].Para=*pFeatureData32++;
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV5640YUVSensorReg[i].Addr;
                *pFeatureData32++=OV5640YUVSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV5640_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV5640YUVSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV5640YUVSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV5640YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV5640YUV_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV5640YUV_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV5640YUV_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV5640YUV_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
		printk("[tong]SENSOR_FEATURE_GET_ITEM_INFO\n");
            OV5640YUV_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV5640YUV_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
			//test by lingnan
            //pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YVYU;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            PK_DBG("FCtrl:SENSOR_FEATURE_INITIALIZE_AF\n");
            OV5640_FOCUS_AD5820_Init();
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            PK_DBG("SENSOR_FEATURE_CONSTANT_AF:OV5640_FOCUS_AD5820_Constant_Focus\n");
            OV5640_FOCUS_AD5820_Constant_Focus();
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            PK_DBG("SENSOR_FEATURE_MOVE_FOCUS_LENS:OV5640_FOCUS_AD5820_Move_to %d\n", *pFeatureData16);
            OV5640_FOCUS_AD5820_Move_to(*pFeatureData16);
            break;
        case SENSOR_FEATURE_GET_AF_STATUS:
            //for yuv use:
           // PK_DBG("SENSOR_FEATURE_GET_AF_STATUS pFeatureReturnPara32=0x%x\n",pFeatureReturnPara32);
            OV5640_FOCUS_AD5820_Get_AF_Status(pFeatureReturnPara32);
           // PK_DBG("SENSOR_FEATURE_GET_AF_STATUS pFeatureReturnPara32=0x%x\n",pFeatureReturnPara32);
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_AF_INF:
        	PK_DBG("SENSOR_FEATURE_GET_AF_INF\n");
            OV5640_FOCUS_AD5820_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
        case SENSOR_FEATURE_GET_AF_MACRO:
        	PK_DBG("SENSOR_FEATURE_GET_AF_MACRO\n");
            OV5640_FOCUS_AD5820_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;                
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            OV5640YUVSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_YUV_CMD:
            OV5640YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));	
            //OV5640YUVSensorSetting((FEATURE_ID)*pFeatureData8,(UINT32)(*(pFeatureData8+1)));	
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV5640YUVGetSensorID(pFeatureReturnPara32); 
            break; 				            
        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
            PK_DBG("SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
            OV5640_FOCUS_AD5820_Single_Focus();
            break;		
        case SENSOR_FEATURE_CANCEL_AF:
            PK_DBG("SENSOR_FEATURE_CANCEL_AF\n");
            OV5640_FOCUS_AD5820_Cancel_Focus();
            break;			
        case SENSOR_FEATURE_SET_AF_WINDOW:
            PK_DBG("SENSOR_FEATURE_SET_AF_WINDOW\n");
            PK_DBG("get zone addr = 0x%x\n",*pFeatureData32);			
            OV5640_FOCUS_AD5820_Set_AF_Window(pFeatureData32);
            break;

	case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
            //[agold][sea][add for iwb]
            OV5640YUVGetAFMaxNumFocusAreas(pFeatureReturnPara32);
            //*pFeatureReturnPara32 = 1;       
            //[agold][sea][end]     
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
            //[agold][sea][add for iwb]
            //*pFeatureReturnPara32 = 1; 
            OV5640YUVGetAFMaxNumMeteringAreas(pFeatureReturnPara32);   
            //[agold][sea][end]        
            *pFeatureParaLen=4;
            break;
            //[agold][sea][add for iwb]
            case SENSOR_FEATURE_GET_EXIF_INFO:
            PK_DBG("SENSOR_FEATURE_GET_EXIF_INFO\n");
            PK_DBG("EXIF addr = 0x%x\n",*pFeatureData32);          
            OV5640YUVGetExifInfo(*pFeatureData32);
            //[agold][sea][end]
            break;
		
        default:
        	PK_DBG("Fct:%d,%d\n", SENSOR_FEATURE_SET_AF_WINDOW, FeatureId);
            break;
    }
    return ERROR_NONE;
}	/* OV5640YUVFeatureControl() */

UINT32 OV5640_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	static SENSOR_FUNCTION_STRUCT SensorFuncOV5640=
	{
		.SensorOpen = OV5640YUVOpen,
		.SensorGetInfo =  OV5640YUVGetInfo,
		.SensorGetResolution = OV5640YUVGetResolution,
		.SensorFeatureControl = OV5640YUVFeatureControl,
		.SensorControl = OV5640YUVControl,
		.SensorClose = OV5640YUVClose
	};

	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV5640;
	return ERROR_NONE;
}	/* OV5640_YUV_SensorInit() */

