#ifndef _CAMERA_INFO_A8141MIPIRAW_MT6573_H
#define _CAMERA_INFO_A8141MIPIRAW_MT6573_H

/*******************************************************************************
*   Configuration
********************************************************************************/
#define SENSOR_ID                           A8141_SENSOR_ID
#define SENSOR_DRVNAME                      SENSOR_DRVNAME_A8141_MIPI_RAW
#define INCLUDE_FILENAME_ISP_REGS_PARAM     "camera_isp_regs_a8141mipiraw_mt6573.h"
#define INCLUDE_FILENAME_ISP_PCA_PARAM      "camera_isp_pca_a8141mipiraw_mt6573.h"

/*******************************************************************************
*   
********************************************************************************/

#if defined(ISP_SUPPORT)

#define A8141MIPIRAW_CAMERA_AUTO_DSC CAM_AUTO_DSC
#define A8141MIPIRAW_CAMERA_PORTRAIT CAM_PORTRAIT
#define A8141MIPIRAW_CAMERA_LANDSCAPE CAM_LANDSCAPE
#define A8141MIPIRAW_CAMERA_SPORT CAM_SPORT
#define A8141MIPIRAW_CAMERA_FLOWER CAM_FLOWER
#define A8141MIPIRAW_CAMERA_NIGHTSCENE CAM_NIGHTSCENE
#define A8141MIPIRAW_CAMERA_DOCUMENT CAM_DOCUMENT
#define A8141MIPIRAW_CAMERA_ISO_ANTI_HAND_SHAKE CAM_ISO_ANTI_HAND_SHAKE
#define A8141MIPIRAW_CAMERA_ISO100 CAM_ISO100
#define A8141MIPIRAW_CAMERA_ISO200 CAM_ISO200
#define A8141MIPIRAW_CAMERA_ISO400 CAM_ISO400
#define A8141MIPIRAW_CAMERA_ISO800 CAM_ISO800
#define A8141MIPIRAW_CAMERA_ISO1600 CAM_ISO1600
#define A8141MIPIRAW_CAMERA_VIDEO_AUTO CAM_VIDEO_AUTO
#define A8141MIPIRAW_CAMERA_VIDEO_NIGHT CAM_VIDEO_NIGHT
#define A8141MIPIRAW_CAMERA_NO_OF_SCENE_MODE CAM_NO_OF_SCENE_MODE

#endif
#endif
