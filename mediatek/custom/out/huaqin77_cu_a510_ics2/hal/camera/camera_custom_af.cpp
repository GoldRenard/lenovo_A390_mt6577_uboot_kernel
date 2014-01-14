#include "camera_custom_af.h"

AF_COEF_T get_AF_Coef()
{
    AF_COEF_T sAFcoef;

    sAFcoef.i4AFS_STEP_MIN_ENABLE = 1;
    sAFcoef.i4AFS_STEP_MIN_MACRO = 4;
    sAFcoef.i4AFS_STEP_MIN_NORMAL = 4;
    sAFcoef.i4AFS_NOISE_LV1 = 0;    // LV1 > LV2
    sAFcoef.i4AFS_NOISE_POS_OFFSET1 = 0;
    sAFcoef.i4AFS_NOISE_GAIN1 = 0;
    sAFcoef.i4AFS_NOISE_LV2 = 0;    
    sAFcoef.i4AFS_NOISE_POS_OFFSET2 = 0;
    sAFcoef.i4AFS_NOISE_GAIN2 = 0;
    sAFcoef.i4AFS_NOISE_POS_OFFSET3 = 0;
    sAFcoef.i4AFS_NOISE_GAIN3 = 0;
    sAFcoef.i4AFC_FAIL_STOP = 4;    
    sAFcoef.i4AFC_FRAME_MODE = 1;      // 0: for 1 frame, 1: for 2 frame
    
    sAFcoef.i4AFC_RESTART_ENABLE      = 0;
    sAFcoef.i4AFC_INIT_STABLE_ENABLE  = 1;        
    sAFcoef.i4FV_1ST_STABLE_ENABLE    = 1;
    sAFcoef.i4AFC_WAIT                = 10;
    
	return sAFcoef;
}

