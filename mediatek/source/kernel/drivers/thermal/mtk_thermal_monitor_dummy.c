#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/bug.h>
#include <linux/err.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <asm/system.h>

/* Init */
static int __init mtkthermal_init(void)
{
    return 0;
}

/* Exit */
static void __exit mtkthermal_exit(void)
{
}

/*mtk thermal zone register function */
struct thermal_zone_device *mtk_thermal_zone_device_register_wrapper
(
    char *type,
    int  trips,
    void *devdata,
    const struct thermal_zone_device_ops *ops,
    int tc1,
    int tc2,
    int passive_delay,
    int polling_delay
)
{
    return NULL;
}

/*mtk thermal zone unregister function */
void mtk_thermal_zone_device_unregister_wrapper
(
    struct thermal_zone_device *tz
)
{
}

int mtk_thermal_zone_bind_cooling_device_wrapper
(
    struct thermal_zone_device *thermal,
    int trip,
    struct thermal_cooling_device *cdev
)
{
    return 0;
}

int thermal_zone_unbind_cooling_device
(
    struct thermal_zone_device *tz, 
    int trip,
    struct thermal_cooling_device *cdev
)
{
    return 0;
}

/*
 * MTK Cooling Register
 */
struct thermal_cooling_device *mtk_thermal_cooling_device_register_wrapper
(
     char *type,
     void *devdata,
     const struct thermal_cooling_device_ops *ops
)
{
    return NULL;
}

/*
 * MTK Cooling Unregister
 */
void mtk_thermal_cooling_device_unregister_wrapper
(
    struct thermal_cooling_device *cdev
)
{
}

//*********************************************
// Export Interface
//*********************************************

EXPORT_SYMBOL(mtk_thermal_zone_device_register_wrapper);
EXPORT_SYMBOL(mtk_thermal_zone_device_unregister_wrapper);
EXPORT_SYMBOL(mtk_thermal_cooling_device_register_wrapper);
EXPORT_SYMBOL(mtk_thermal_cooling_device_unregister_wrapper);
EXPORT_SYMBOL(mtk_thermal_zone_bind_cooling_device_wrapper);
EXPORT_SYMBOL(thermal_zone_unbind_cooling_device);

module_init(mtkthermal_init);
module_exit(mtkthermal_exit);
