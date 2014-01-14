#ifndef _AF_COEF_H
#define _AF_COEF_H

#include "MediaTypes.h"

    typedef struct
    {
        MINT32  i4AFS_STEP_MIN_ENABLE;
        MINT32  i4AFS_STEP_MIN_NORMAL;
        MINT32  i4AFS_STEP_MIN_MACRO;
        MINT32  i4AFS_NOISE_LV1;
        MINT32  i4AFS_NOISE_POS_OFFSET1;
        MINT32  i4AFS_NOISE_GAIN1;
        MINT32  i4AFS_NOISE_LV2;
        MINT32  i4AFS_NOISE_POS_OFFSET2;
        MINT32  i4AFS_NOISE_GAIN2;
        MINT32  i4AFS_NOISE_POS_OFFSET3;
        MINT32  i4AFS_NOISE_GAIN3;
        MINT32  i4AFC_FAIL_STOP;
        MINT32  i4AFC_FRAME_MODE;      // 0: for 1 frame, 1: for 2 frame

        MINT32 i4AFC_RESTART_ENABLE;     // restart AFC if AE non stable                   
        MINT32 i4AFC_INIT_STABLE_ENABLE; // for waiting stable in AFC init               
        MINT32 i4FV_1ST_STABLE_ENABLE;   // for get 1st FV             
        MINT32 i4AFC_WAIT;               // AFC wait frame when i4AFC_INIT_STABLE_ENABLE = 0
        
    } AF_COEF_T;

	AF_COEF_T get_AF_Coef();
	
#endif /* _AF_COEF_H */

