#include <linux/types.h>
#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#endif

#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#endif

#include <cust_alsps.h>
//#include <mach/mt6575_pm_ldo.h>
static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 0,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x88},
    //.als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
    //.als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
   .als_level  = { 0,  7, 15,  100, 500, 1000,  1500,  2000, 2500, 3000, 3500, 4096},
    .als_value  = {40, 90, 160, 320, 640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
    .ps_threshold = 160,	//3,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
