#ifndef _A8141AF_H
#define _A8141AF_H

#include <linux/ioctl.h>
//#include "kd_imgsensor.h"

#define A8141AF_MAGIC 'A'
//IOCTRL(inode * ,file * ,cmd ,arg )


//Structures
typedef struct {
//current position
unsigned long u4CurrentPosition;
//macro position
unsigned long u4MacroPosition;
//Infiniti position
unsigned long u4InfPosition;
//Motor Status
bool          bIsMotorMoving;
//Motor Open?
bool          bIsMotorOpen;
} stA8141AF_MotorInfo;

//Control commnad
//S means "set through a ptr"
//T means "tell by a arg value"
//G means "get by a ptr"             
//Q means "get by return a value"
//X means "switch G and S atomically"
//H means "switch T and Q atomically"
#define A8141AFIOC_G_MOTORINFO _IOR(A8141AF_MAGIC,0,stA8141AF_MotorInfo)

#define A8141AFIOC_T_MOVETO _IOW(A8141AF_MAGIC,1,unsigned long)

#define A8141AFIOC_T_SETINFPOS _IOW(A8141AF_MAGIC,2,unsigned long)

#define A8141AFIOC_T_SETMACROPOS _IOW(A8141AF_MAGIC,3,unsigned long)

#else
#endif
