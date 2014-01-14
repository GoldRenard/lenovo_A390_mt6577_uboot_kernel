/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ov5640_Sensor.h
 *
 * Project:
 * --------
 *   YUSU
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
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
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

typedef enum group_enum {
    PRE_GAIN=0,
    CMMCLK_CURRENT,
    FRAME_RATE_LIMITATION,
    REGISTER_EDITOR,
    GROUP_TOTAL_NUMS
} FACTORY_GROUP_ENUM;


#define ENGINEER_START_ADDR 10
#define FACTORY_START_ADDR 0

typedef enum engineer_index
{
    CMMCLK_CURRENT_INDEX=ENGINEER_START_ADDR,
    ENGINEER_END
} FACTORY_ENGINEER_INDEX;



typedef enum register_index
{
    PRE_GAIN_INDEX=FACTORY_START_ADDR,
    GLOBAL_GAIN_INDEX,
    FACTORY_END_ADDR
} FACTORY_REGISTER_INDEX;

typedef struct
{
    SENSOR_REG_STRUCT	Reg[ENGINEER_END];
    SENSOR_REG_STRUCT	CCT[FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT, *PSENSOR_DATA_STRUCT;



#define CURRENT_MAIN_SENSOR                OV5640_OMNIVISION


//Macro for Resolution
#define OV5640_IMAGE_SENSOR_CCT_WIDTH     (2596)
#define OV5640_IMAGE_SENSOR_CCT_HEIGHT    (1950)

/* SENSOR 720P SIZE */
#define OV5640_IMAGE_SENSOR_PV_WIDTH   (1280)
#define OV5640_IMAGE_SENSOR_PV_HEIGHT  (960)

/* SENSOR 5M SIZE */
#define OV5640_IMAGE_SENSOR_FULL_WIDTH                                       (2592-16)
#define OV5640_IMAGE_SENSOR_FULL_HEIGHT                                      (1944-14)

#define OV5640_IMAGE_SENSOR_PV_STARTX                         4
#define OV5640_IMAGE_SENSOR_PV_STARTY                         3

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define OV5640_FULL_PERIOD_PIXEL_NUMS  (2592)  // 5M mode's pixel # in one HSYNC w/o dummy pixels
#define OV5640_FULL_PERIOD_LINE_NUMS    (1944)  // 5M mode's HSYNC # in one HSYNC w/o dummy lines
#define OV5640_PV_PERIOD_PIXEL_NUMS      (1280)    // 720P mode's pixel # in one HSYNC w/o dummy pixels
#define OV5640_PV_PERIOD_LINE_NUMS        (960)   // 720P mode's HSYNC # in one HSYNC w/o dummy lines

#define OV5640_FULL_PERIOD_EXTRA_PIXEL_NUMS  608 //450
#define OV5640_FULL_PERIOD_EXTRA_LINE_NUMS    56 //28
#define OV5640_PV_PERIOD_EXTRA_PIXEL_NUMS      531 //450
#define OV5640_PV_PERIOD_EXTRA_LINE_NUMS        28

#define OV5640_FULL_PERIOD_PIXEL_NUMS_HTS  (2844) 
#define OV5640_FULL_PERIOD_LINE_NUMS_VTS    (1968) 
#define OV5640_PV_PERIOD_PIXEL_NUMS_HTS      (1896) 
#define OV5640_PV_PERIOD_LINE_NUMS_VTS        (984)   


#define OV5640_IMAGE_SENSOR_5M_PIXELS_LINE           OV5640_FULL_PERIOD_PIXEL_NUMS_HTS
#define OV5640_IMAGE_SENSOR_720P_PIXELS_LINE        OV5640_PV_PERIOD_PIXEL_NUMS_HTS


    
#define MAX_FRAME_RATE	(15)
#define MIN_FRAME_RATE  (12)

/* SENSOR EXPOSURE LINE LIMITATION */
#define OV5640_FULL_EXPOSURE_LIMITATION    (1944-4)  // 5M mode
#define OV5640_PV_EXPOSURE_LIMITATION  (OV5640_PV_PERIOD_LINE_NUMS-4)  // # of lines in one 720P frame
// SENSOR VGA SIZE
//For 2x Platform camera_para.c used
#define IMAGE_SENSOR_PV_WIDTH    OV5640_IMAGE_SENSOR_PV_WIDTH
#define IMAGE_SENSOR_PV_HEIGHT   OV5640_IMAGE_SENSOR_PV_HEIGHT

#define IMAGE_SENSOR_FULL_WIDTH	  OV5640_IMAGE_SENSOR_FULL_WIDTH
#define IMAGE_SENSOR_FULL_HEIGHT	  OV5640_IMAGE_SENSOR_FULL_HEIGHT

#define OV5640_SHUTTER_LINES_GAP	  0


#define OV5640_WRITE_ID (0x78)
#define OV5640_READ_ID	(0x79)

// SENSOR CHIP VERSION

//#define OV5640_SENSOR_ID            0x5642

#define OV5640_PAGE_SETTING_REG    (0xFF)

//s_add for porting
//s_add for porting
//s_add for porting

//#define OV5640_SENSOR_ID    OV5640_SENSOR_ID
//#define OV5640_SENSOR_ID1   OV5640_SENSOR_ID_1
//#define OV5640_SENSOR_ID2   OV5640_SENSOR_ID_2
//#define OV5640_WRITE_ID     OV5640_WRITE_ID

//export functions
UINT32 OV5640YUVOpen(void);
UINT32 OV5640YUVGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 OV5640YUVGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV5640YUVControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV5640YUVFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 OV5640YUVClose(void);

//#define Sleep(ms) mdelay(ms)
//#define RETAILMSG(x,...)
//#define TEXT

//e_add for porting
//e_add for porting
//e_add for porting

#endif /* __SENSOR_H */

