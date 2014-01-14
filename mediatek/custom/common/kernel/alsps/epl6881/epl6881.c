/* drivers/hwmon/mt6516/amit/epl6881.c - epl6881 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif

#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif

#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6575
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "epl6881.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define EPL6881_DEV_NAME     "EPL6881"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/
#ifdef MT6575
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
	extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
	extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
	extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
										 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
										 kal_bool auto_umask);
	
#endif
#ifdef MT6516
extern void MT6516_EINTIRQUnmask(unsigned int line);
extern void MT6516_EINTIRQMask(unsigned int line);
extern void MT6516_EINT_Set_Polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void MT6516_EINT_Set_HW_Debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 MT6516_EINT_Set_Sensitivity(kal_uint8 eintno, kal_bool sens);
extern void MT6516_EINT_Registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_client *epl6881_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl6881_i2c_id[] = {{EPL6881_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_EPL6881={ I2C_BOARD_INFO("EPL6881", 0x49)};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short epl6881_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const epl6881_forces[] = { epl6881_force, NULL };
//static struct i2c_client_address_data epl6881_addr_data = { .forces = epl6881_forces,};
/*----------------------------------------------------------------------------*/
static int epl6881_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int epl6881_i2c_remove(struct i2c_client *client);
static int epl6881_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int epl6881_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl6881_i2c_resume(struct i2c_client *client);

static struct epl6881_priv *g_epl6881_ptr = NULL;

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={{0,0,0},};

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct epl6881_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/

static bool poverflag = false;
static int psensor_mode_suspend = 0;

struct epl6881_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;

    /*i2c address group*/
    struct epl6881_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;

	int intr_pin;
	
		int ps_opened;
		int als_opened;
	
		int enable_pflag;
		int enable_lflag;
		int read_flag;
		int irq;
	
		int ps_is_first_frame ;
		uint16_t ps_irq_enabled;
	
		uint32_t als_i_threshold;
		uint32_t als_f_threshold;
		uint32_t als_m_threshold;
	
		uint32_t als_ax2;
		uint32_t als_bx;
		uint32_t als_c;
		uint32_t als_ax;
		uint32_t als_b;
		
	
		uint8_t epl_irq_pin_state;
		uint8_t epl_adc_item;
		uint8_t epl_average_item;
		uint8_t epl_integrationtime_item;
		uint16_t epl_integrationtime;
		uint8_t epl_integrationtime_index;
		uint8_t epl_intpersistency;
		uint16_t epl_intthreshold_hvalue;
		uint16_t epl_intthreshold_lvalue;
		uint8_t epl_op_mode;
		uint8_t epl_sc_mode;
		uint8_t epl_sensor_gain_item;
		uint8_t epl_return_status;
		u8 mode_flag;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

#define TXBYTES                             2
#define RXBYTES                             2

#define PS_POLLING_RATE 		300
#define DUAL_POLLING_RATE	 	1000
#define ALS_POLLING_RATE 		1000

#define PACKAGE_SIZE 			2
#define I2C_RETRY_COUNT 		10
#define P_INTT 					4

//incandescent and fluorescent parameter
#define ALS_MAX_COUNT			60000
#define ALS_MIN_COUNT			15  			//ambient light mode dynamic integration time ADC low threshold
#define ALS_MAX_LUX				60000	  	//ambient light mode MAX lux
#define ALS_MIN_LUX				0

/*24 bits integer, 8 bits fraction*/
#define ALS_I_THRESHOLD		0x018F		// incandescent threshold (1.55859375)
#define ALS_F_THRESHOLD		0x0217		// fluorescent threshold (2.08984375)
#define ALS_M_THRESHOLD		0x0199 		// incandescent and fluorescent mix threshold (1.59765625)

/*24 bits integer, 8 bits fraction*/
#define ALS_AX2					0x4C433
#define ALS_BX					0x11FE3
#define ALS_C					0x4AFB3
#define ALS_AX					0x1494A
#define ALS_B					0x20028

// 0 for power-saving, 1 for less cpu communication
#define PS_IRQ_ENABLED         	0
#define P_SENSOR_LTHD			100
#define P_SENSOR_HTHD			500

typedef struct _epl_raw_data
{
    u8 raw_bytes[2];
    u16 ps_raw;
    u16 als_ch0_raw;
    u16 als_ch1_raw;
    uint32_t lux;
    uint32_t ratio;
} epl_raw_data;

struct epl6881_priv *epl_data;

static epl_raw_data	gRawData;

static uint16_t als_intT_table[] =
{
    1,
    8,
    16,
    32,
    64,
    128,
    256,
    512,
    640,
    768,
    1024,
    2048,
    4096,
    6144,
    8192,
    10240
};


static uint8_t als_intT_selection[] =
{
    1,
    7,
};


/*----------------------------------------------------------------------------*/
static struct i2c_driver epl6881_i2c_driver = {	
	.probe      = epl6881_i2c_probe,
	.remove     = epl6881_i2c_remove,
	.detect     = epl6881_i2c_detect,
	.suspend    = epl6881_i2c_suspend,
	.resume     = epl6881_i2c_resume,
	.id_table   = epl6881_i2c_id,
//	.address_data = &epl6881_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = epl6881_DEV_NAME,
	},
};

static struct epl6881_priv *epl6881_obj = NULL;
static struct platform_driver epl6881_alsps_driver;

/*----------------------------------------------------------------------------*/
static int epl6881_get_als_value(struct epl6881_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}

int epl6881_get_addr(struct alsps_hw *hw, struct epl6881_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void epl6881_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "EPL6881")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "EPL6881")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}

static int elan_epl6881_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry,val;
    struct elan_epl_data *epld = epl_data;

    buffer[0] = (regaddr<<3) | bytecount ;
    buffer[1] = data;


    //printk("---buffer data (%x) (%x)---\n",buffer[0],buffer[1]);

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        //val = gpio_get_value(epld->intr_pin);

        //printk("[EPL6881] %s, INTERRUPT GPIO val = %d\n", __func__, val);

        msleep(10);
    }


    if(retry>=I2C_RETRY_COUNT)
    {
        printk(KERN_ERR "[ELAN epl6881 error] %s i2c write retry over %d\n",__func__, I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}


static int elan_epl6881_I2C_Read(struct i2c_client *client)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i =0;
    int retry,val;
    struct elan_epl_data *epld = epl_data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {

        ret = i2c_master_recv(client, buffer, RXBYTES);

        if (ret == RXBYTES)
            break;

        //val = gpio_get_value(epld->intr_pin);

        printk("[EPL6881] %s, INTERRUPT GPIO val = %d\n", __func__, val);

        //printk("i2c read error,RXBYTES %d\r\n",ret);
        msleep(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        printk(KERN_ERR "[ELAN epl6881 error] %s i2c read retry over %d\n",__func__, I2C_RETRY_COUNT);
        return -EINVAL;
    }

    for(i=0; i<PACKAGE_SIZE; i++)
    {
        gRawData.raw_bytes[i] = buffer[i];
    }

    //printk("-----Receive data byte1 (%x) byte2 (%x)-----\n",buffer[0],buffer[1]);


    return ret;
}

#if 0
static int elan_epl6881_psensor_enable(struct epl6881_priv *epld)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

    printk("--- Proximity sensor Enable --- \n");

    //set register 0
    //average cycle = 8 times, sensing mode = continuous, sensor mode = ps, gain = low
    //mutex_lock(&ps_enable_mutex);

    epl_data->ps_is_first_frame = 1;
    epl_data->epl_average_item = EPL_SENSING_8_TIME;
    epl_data->epl_sc_mode = epld->ps_irq_enabled ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE;
    epl_data->epl_op_mode = EPL_PS_MODE;
    epl_data->epl_sensor_gain_item = EPL_L_GAIN;
    regdata = epl_data->epl_average_item | epl_data->epl_sc_mode | epl_data->epl_op_mode | epl_data->epl_sensor_gain_item;
    ret = elan_epl6881_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

    //set register 1
    epl_data->epl_intpersistency = EPL_PST_1_TIME;
    epl_data->epl_adc_item = EPL_10BIT_ADC;
    regdata = P_INTT << 4 | epl_data->epl_intpersistency | epl_data->epl_adc_item;
    ret = elan_epl6881_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

    //set register 9
    elan_epl6881_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_FRAME_ENABLE);  //setting interrupt pin = binary

    /*restart sensor*/
    elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    ret = elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
    //printk("--- Proximity sensor setting finish --- \n");
    msleep(2);

    if (ret != 0x02)
    {
        epld->enable_pflag = 0;
        printk("[EPL6881] P-sensor i2c err\n");
    }
    //else
    //{
       //wake_lock(&g_ps_wlock);
    //}

    //mutex_unlock(&ps_enable_mutex);
    return ret;
}


static int elan_epl6881_lsensor_enable(struct epl6881_priv *epld)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

    printk("--- ALS sensor Enable --- \n");
    //mutex_lock(&als_enable_mutex);

    //set register 0
    //average cycle = 32 times, sensing mode = continuous, sensor mode = als, gain = auto
    epl_data->epl_average_item = EPL_SENSING_32_TIME;
    epl_data->epl_sc_mode = EPL_S_SENSING_MODE;
    epl_data->epl_op_mode = EPL_ALS_MODE;
    epl_data->epl_sensor_gain_item = EPL_AUTO_GAIN;
    regdata = epl_data->epl_average_item | epl_data->epl_sc_mode | epl_data->epl_op_mode | epl_data->epl_sensor_gain_item;
    ret = elan_epl6881_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

    //set register 1
    epl_data->epl_intpersistency = EPL_PST_1_TIME;
    epl_data->epl_adc_item = EPL_10BIT_ADC;
    epld->epl_integrationtime_item = als_intT_selection[epld->epl_integrationtime_index];
    regdata = epld->epl_integrationtime_item << 4 | epl_data->epl_intpersistency | epl_data->epl_adc_item;
    epld->epl_integrationtime = als_intT_table[epld->epl_integrationtime_item];
    ret = elan_epl6881_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

    //set register 9
    elan_epl6881_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, EPL_INT_FRAME_ENABLE);  // set frame enable

    /*restart sensor*/
    elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    ret = elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
    printk("--- ALS sensor setting finish --- \n");
    msleep(2);

    if(ret != 0x02)
    {
        epld->enable_lflag = 0;
        printk("[EPL6881] ALS-sensor i2c err\n");
    }

    //mutex_unlock(&als_enable_mutex);

    return ret;

}



static int elan_epl6881_psensor_disable(struct epl6881_priv *epld)
{
    printk("--- Proximity sensor disable --- \n");
    printk("--- wake_unlock --- \n");

    epld->enable_pflag=0;
    //wake_unlock(&g_ps_wlock);
    if(epld->enable_lflag==1)
    {
        elan_epl6881_lsensor_enable(epld);
    }

    return 0;
}

static int elan_epl6881_lsensor_disable(struct epl6881_priv *epld)
{
    printk("--- Light sensor disable --- \n");

    epld->enable_lflag=0;
    if(epld->enable_pflag==1)
    {
        elan_epl6881_psensor_enable(epld);
    }

    return 0;
}
#endif
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    int ret = 0;
    struct epl6881_priv *epld;
    struct i2c_client *client = epld->client;

    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    //printk("[ELAN epl6881] %s: low_thd = 0x%X, high_thd = 0x%x \n",__func__, low_thd, high_thd);
    //mutex_lock(&ps_get_sensor_value_mutex);
    elan_epl6881_I2C_Write(epld->client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
    elan_epl6881_I2C_Write(epld->client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
    elan_epl6881_I2C_Write(epld->client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
    elan_epl6881_I2C_Write(epld->client,REG_5,W_SINGLE_BYTE,0x02,low_msb);
    //mutex_unlock(&ps_get_sensor_value_mutex);

    return ret;
}

/*----------------------------------------------------------------------------*/
static long epl6881_enable_als(struct i2c_client *client, int enable)
{
		struct epl6881_priv *epld = i2c_get_clientdata(client);
		u8 databuf[2];	  
		long res = 0;
		int ret;
    	uint8_t regdata = 0;
		u8 buffer[1];
		u8 reg_value[1];
	
		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUL NULL\n");
			return -1;
		}
		
		if(enable)
		{
			//databuf[0] = EPL6881_CMM_ENABLE;
		  regdata = EPL_SENSING_32_TIME| EPL_S_SENSING_MODE | EPL_ALS_MODE | EPL_AUTO_GAIN;
    	  ret = elan_epl6881_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);
		  //res = i2c_master_send(client, buffer, 0x1);


		  regdata = 0x11;
          epld->epl_integrationtime = als_intT_table[epld->epl_integrationtime_item];
          ret = elan_epl6881_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

	      
		  elan_epl6881_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, EPL_INT_DISABLE);  // set frame enable
		  
			  /*restart sensor*/
		   elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
		   ret = elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
		   printk("--- epl6881 ALS sensor setting finish --- \n");
		   msleep(2);

		   if(ret != 0x02)
    		{
        		epld->enable_lflag = 0;
        		printk("[EPL6881] ALS-sensor i2c err\n");
    		}

    		//mutex_unlock(&als_enable_mutex);

				printk("[EPL6881] ALS-sensor \n");
		}
		else
		{
			epld->enable_lflag=0;
           epld->enable_pflag==1;
          
         	regdata = 0x67;
    		ret = elan_epl6881_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

		}
		return 0;
		
	EXIT_ERR:
		APS_ERR("epl6881_enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
static long epl6881_enable_ps(struct i2c_client *client, int enable)
{
	struct epl6881_priv *epld = i2c_get_clientdata(client);
	
	u8 databuf[2];    
	long res = 0;
	u8 buffer[1];
	u8 reg_value[1];
	int ret;
    uint8_t regdata = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	printk("[EPL6881]enable ps\n");
	if(enable)
	{
		//mutex_lock(&ps_enable_mutex);

    
    	
    	regdata = 0x67;
    	ret = elan_epl6881_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

    //set register 1
    	
    	regdata = 0x41;
    	ret = elan_epl6881_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

    //set register 9
    	elan_epl6881_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);  //setting interrupt pin = binary

    /*restart sensor*/
    	elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    	ret = elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
    printk("--- Proximity sensor setting finish --- \n");
    	msleep(2);

    	if (ret != 0x02)
    	{
        	epld->enable_pflag = 0;
        	printk("[EPL6881] P-sensor i2c err\n");
    	}
    	//else
    	//{
        	//wake_lock(&g_ps_wlock);
    	//}

    	//mutex_unlock(&ps_enable_mutex);

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == epld->hw->polling_mode_ps)
		{								
			//modify by ivy
		//schedule_delayed_work(&epld->eint_work,120);
			
		}
	}
	else
	{
		//databuf[0] = EPL6881_CMM_ENABLE;    
		epld->enable_pflag = 0; 
		epld->enable_lflag = 1;
        epl6881_enable_als(epld->client, 1);

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("epl6881_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
#if 0
static int epl6881_enable(struct i2c_client *client, int enable)
{
	struct epl6881_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	/* modify to restore reg setting after cali ---liaoxl.lenovo */
	buffer[0]=EPL6881_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	if(enable)
	{
		databuf[0] = EPL6881_CMM_ENABLE;    
		databuf[1] = reg_value[0] | 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_DBG("epl6881 power on\n");
	}
	else
	{
		databuf[0] = EPL6881_CMM_ENABLE;    
		databuf[1] = reg_value[0] & 0xFE;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		/*Lenovo-sw chenlj2 add 2011-06-03,close als_deb_on */
		atomic_set(&obj->als_deb_on, 0);
		APS_DBG("epl6881 power off\n");
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("epl6881_enable fail\n");
	return res;
}
#endif
/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int epl6881_check_and_clear_intr(struct i2c_client *client) 
{
	struct epl6881_priv *epld = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];
	u8 reg_value[1];//by  ivy
	uint8_t regdata = 0;

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	    return 0;
	
	elan_epl6881_I2C_Write(epld->client,REG_13,R_SINGLE_BYTE,0x01,0);
	elan_epl6881_I2C_Read(epld->client);
	epld->epl_return_status = gRawData.raw_bytes[0];
	regdata= gRawData.raw_bytes[0]&(3<<4);
	
	res = 1;
	intp = 0;
	intl = 0;
	if(regdata==0x10)
	{
		res = 0;
		intp = 1;
	}
	if(regdata==0x00)
	{
		res = 0;
		intl = 1;		
	}
	
	res = elan_epl6881_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}
	

	return res;

EXIT_ERR:
	APS_ERR("epl6881_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/
void epl6881_eint_func(void)
{
	struct epl6881_priv *obj = g_epl6881_ptr;
	if(!obj)
	{
		return;
	}
	
	schedule_delayed_work(&obj->eint_work,0);
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int epl6881_setup_eint(struct i2c_client *client)
{
	struct epl6881_priv *obj = i2c_get_clientdata(client);        

	g_epl6881_ptr = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, epl6881_eint_func, 0);

	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}

/*----------------------------------------------------------------------------*/
#if 0
static int epl6881_init_client_for_cali(struct i2c_client *client)
{

	struct epl6881_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = epl6881_CMM_ENABLE;    
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return EPL6881_ERR_I2C;
	}	
	//databuf[0] = EPL6881_CMM_ATIME;    
	//databuf[1] = 0xEE;//0xEE
	/res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return EPL6881_ERR_I2C;
	}

	databuf[0] = EPL6881_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return EPL6881_ERR_I2C;
	}

	databuf[0] = EPL6881_CMM_WTIME;    
	databuf[1] = 0xFF;//0xFF
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return EPL6881_ERR_I2C;
	}

	databuf[0] = EPL6881_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return EPL6881_ERR_I2C;
	}

	databuf[0] = EPL6881_CMM_PPCOUNT;    
	databuf[1] = EPL6881_CMM_PPCOUNT_VALUE;//0x02
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return EPL6881_ERR_I2C;
	}

	databuf[0] = EPL6881_CMM_CONTROL;    
	databuf[1] = 0x20;//0x22
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return EPL6881_ERR_I2C;
	}
	databuf[0] = EPL6881_CMM_ENABLE;	
		databuf[1] = 0x0F;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return EPL6881_ERR_I2C;
		}

	return EPL6881_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
#endif
static int epl6881_init_client(struct i2c_client *client)
{
	struct epl6881_priv *epld = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	int ret = 0;
 
	printk(KERN_INFO"%s enter!\n",__func__);
 
	 ret = elan_epl6881_I2C_Read(client);
 
	 if(ret < 0)
		 return -EINVAL;
						
    //Set proximity sensor High threshold and Low threshold
   		//set_psensor_intr_threshold(100,500);

    /*restart sensor*/
    	//elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    	//ret = elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
		

    	//msleep(2);

   		 epld->ps_irq_enabled = PS_IRQ_ENABLED;

    	epld->als_ax2 = ALS_AX2;
    	epld->als_bx = ALS_BX;
    	epld->als_c = ALS_C;
    	epld->als_ax = ALS_AX;
    	epld->als_b = ALS_B;

    	epld->als_f_threshold= ALS_F_THRESHOLD;
    	epld->als_i_threshold = ALS_I_THRESHOLD;
    	epld->als_m_threshold = ALS_M_THRESHOLD;

    	epld->enable_lflag = 0;
    	epld->enable_pflag = 0;

			
	if(0 == epld->hw->polling_mode_ps)
	{
		elan_epl6881_I2C_Write(client,REG_9,W_SINGLE_BYTE,0X02,EPL_INT_ACTIVE_LOW);
		
	}
	//if(res = epl6881_setup_eint(client))
	//{
		//APS_ERR("setup eint: %d\n", res);
		//return res;
	//}
	//if(res = epl6881_check_and_clear_intr(client))
	//{
		//APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	//}
	printk(KERN_INFO"%s init OK!\n",__func__);
	
	return EPL6881_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int epl6881_read_als(struct i2c_client *client, u16 *data)
{
	struct epl6881_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 atio;
	u16 als_value;
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
//get adc channel 0 value
	
	elan_epl6881_I2C_Write(obj->client,REG_14,R_TWO_BYTE,0x01,0x00);
    elan_epl6881_I2C_Read(obj->client);
    gRawData.als_ch0_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    elan_epl6881_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
    elan_epl6881_I2C_Read(obj->client);
    gRawData.als_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    *data = (gRawData.als_ch0_raw > gRawData.als_ch1_raw) ? gRawData.als_ch0_raw : gRawData.als_ch1_raw;
	
	
	
	//APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

//get adc channel 1 value

	

	//APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);

	
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("epl6881_read_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
long epl6881_read_ps(struct i2c_client *client, u16 *data)
{
	struct epl6881_priv *obj = i2c_get_clientdata(client);    
	u16 ps_value;    
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	elan_epl6881_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
				
    elan_epl6881_I2C_Read(obj->client);
				
    gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
				
	
	*data = gRawData.ps_raw ;
	//APS_DBG("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;    

EXIT_ERR:
	APS_ERR("epl6881_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int epl6881_get_ps_value(struct epl6881_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;
	 /*Lenovo-sw chenlj2 add 2011-10-12 begin*/
	 u16 temp_ps[1];
	 /*Lenovo-sw chenlj2 add 2011-10-12 end*/
	 
	
	//APS_LOG("epl6881_get_ps_value  1 %d," ,ps_cali.close);
	//APS_LOG("epl6881_get_ps_value  2 %d," ,ps_cali.far_away);
	//APS_LOG("epl6881_get_ps_value  3 %d,", ps_cali.valid);

	//APS_LOG("epl6881_get_ps_value  ps %d,", ps);
    /*Lenovo-sw zhuhc delete 2011-10-12 begin*/
	//return 1;
    /*Lenovo-sw zhuhc delete 2011-10-12 end*/

        mdelay(160);
	epl6881_read_ps(obj->client,temp_ps);
	if(ps_cali.valid == 1)
		{
			//APS_LOG("epl6881_get_ps_value val_temp  = %d",val_temp);
			if((ps >ps_cali.close)&&(temp_ps[0] >ps_cali.close))
			{
				val = 0;  /*close*/
				val_temp = 0;
			}
			else if((ps <ps_cali.far_away)&&(temp_ps[0] < ps_cali.far_away))
			{
				val = 1;  /*far away*/
				val_temp = 1;
			}
			else
				val = val_temp;

			//APS_LOG("epl6881_get_ps_value val  = %d",val);
	}
	else
	{
			if((ps > atomic_read(&obj->ps_thd_val))&&(temp_ps[0]  > atomic_read(&obj->ps_thd_val)))
			{
				val = 0;  /*close*/
				val_temp = 0;
			}
			else if((ps < atomic_read(&obj->ps_thd_val))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val)))
			{
				val = 1;  /*far away*/
				val_temp = 1;
			}
			else
			       val = val_temp;	
			
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
static int elan_epl_ps_poll_rawdata(void)
{

    int value = 0;

    struct epl6881_priv *epld = epl_data;

    printk("--- Polling ps rawdata --- \n");

    gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    value = gRawData.ps_raw;

    if(gRawData.ps_raw > epld->epl_intthreshold_hvalue)
        poverflag = true;

    if(gRawData.ps_raw < epld->epl_intthreshold_lvalue && poverflag == true)
        poverflag = false;

    value = (poverflag == true) ? 0 : 1;

	return value;

   // printk("### ps_raw_data (%x) (%d), value(%d) ###\n\n",gRawData.ps_raw,gRawData.ps_raw, value);

   // input_report_abs(epld->ps_input_dev, ABS_DISTANCE, value);
   // input_sync(epld->ps_input_dev);

}

/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void epl6881_eint_work(struct work_struct *work)
{
	struct epl6881_priv *obj = g_epl6881_ptr;
	int err;
	hwm_sensor_data sensor_data;
    APS_DBG("Sidney epl6881_eint_work !\n");
	if((err = epl6881_check_and_clear_intr(obj->client)))
	{
		APS_ERR("epl6881_eint_work check intrs: %d\n", err);
	}
	else
	{
        elan_epl6881_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
        elan_epl6881_I2C_Read(obj->client);
        elan_epl_ps_poll_rawdata();//get raw data
		
		APS_DBG("epl6881_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = elan_epl_ps_poll_rawdata();
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);      
}


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int epl6881_open(struct inode *inode, struct file *file)
{
	file->private_data = epl6881_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int epl6881_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static epl6881_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{

	   APS_LOG("epl6881_WriteCalibration  1 %d," ,data_cali->close);
		   APS_LOG("epl6881_WriteCalibration  2 %d," ,data_cali->far_away);
		   APS_LOG("epl6881_WriteCalibration  3 %d,", data_cali->valid);
		   
	  if(data_cali->valid == 1)
	  {
	      if(data_cali->close < 100)
	      	{
		  	ps_cali.close = 200;
			ps_cali.far_away= 150;
			ps_cali.valid = 1;
	      	}
		  else if(data_cali->close > 900)
		  {
		  	ps_cali.close = 900;
			ps_cali.far_away= 750;
			ps_cali.valid = 1;
	      	}
		  else
		  {
			  ps_cali.close = data_cali->close;
			ps_cali.far_away= data_cali->far_away;
			ps_cali.valid = 1;
		  }
	  }
	  

}

static int epl6881_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
     int i=0 ,err = 0,j = 0;
	 u16 data[21],sum,data_cali;

	 for(i = 0;i<20;i++)
	 	{
	 		mdelay(5);//50
			if(err = epl6881_read_ps(client,&data[i]))
			{
				APS_ERR("epl6881_read_data_for_cali fail: %d\n", i); 
				break;
			}
			else
				{
					sum += data[i];
			}
			mdelay(55);//160
	 	}
	 
	 for(j = 0;j<20;j++)
	 	APS_LOG("%d\t",data[j]);
	 
	 if(i == 20)
	 	{
			data_cali = sum/20;
			APS_LOG("epl6881_read_data_for_cali data = %d",data_cali);
			if(data_cali>600)
			return -1;
			if(data_cali<=100)
			{
				ps_data_cali->close =data_cali*22/10;
				ps_data_cali->far_away = data_cali*19/10;
				ps_data_cali->valid =1;
			}
			else if(100<data_cali&&data_cali<300)
			{
				ps_data_cali->close = data_cali*2;
				ps_data_cali->far_away =data_cali*17/10;
				ps_data_cali->valid = 1;
			}
			else
			{
				ps_data_cali->close = data_cali*18/10;
				ps_data_cali->far_away =data_cali*15/10;
				ps_data_cali->valid = 1;
			}
		        if(ps_data_cali->close > 900)
		       {
		  	ps_data_cali->close = 900;
			ps_data_cali->far_away = 750;
			err= 0;
	         	}
			else  if(ps_data_cali->close < 100)
			{
			   ps_data_cali->close = 200;
			   ps_data_cali->far_away = 150;
			   err= 0;
			}

			ps_cali.close = ps_data_cali->close;
			ps_cali.far_away= ps_data_cali->far_away;
			ps_cali.valid = 1;
			APS_LOG("epl6881_read_data_for_cali close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
	
	 	}
	 else
	 	{
	 	ps_data_cali->valid = 0;
	 	err=  -1;
	 	}
	 return err;
	 	

}
static long epl6881_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct epl6881_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	int value;
	void __user *ptr = (void __user*) arg;
	int dat,regdata;
	uint32_t enable;
	struct PS_CALI_DATA_STRUCT ps_cali_temp;
	printk("epl6881 _ unlocked _ioctl");
			   

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = epl6881_enable_ps(obj->client, 1))
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if(err = epl6881_enable_ps(obj->client, 0))
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
				elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
				
            	elan_epl6881_I2C_Read(client);
				
            	dat = elan_epl_ps_poll_rawdata();
				
			   if(copy_to_user(ptr, &dat, sizeof(dat)))
		      {
				 err = -EFAULT;
				 goto err_out;
			  }  
			   printk("elan proximity Sensor get data1 (%d) \n",dat);
			   
				break;
		
			case ALSPS_GET_PS_RAW_DATA:  
					
				elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
				
            	elan_epl6881_I2C_Read(client);
				
            	gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

				elan_epl6881_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
				elan_epl6881_I2C_Read(client);
				regdata = gRawData.raw_bytes[0];

				elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
				
				 printk("elan ps get data regdata[0x0D](%d) \n",regdata);
				 printk("elan PS get raw data(%d) \n",gRawData.ps_raw);
				
				if(copy_to_user(ptr, &gRawData.ps_raw , sizeof(gRawData.ps_raw)))
					return -EFAULT;
				
				break;				  
		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = epl6881_enable_als(obj->client, 1))
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if(err = epl6881_enable_als(obj->client, 0))
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

	case ALSPS_GET_ALS_DATA: 
					
		      elan_epl6881_I2C_Write(client,REG_14,R_TWO_BYTE,0x01,0x00);
              elan_epl6881_I2C_Read(client);
              gRawData.als_ch0_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

              elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
              elan_epl6881_I2C_Read(client);
              gRawData.als_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
			  dat = (gRawData.als_ch0_raw > gRawData.als_ch1_raw) ? gRawData.als_ch0_raw : gRawData.als_ch1_raw;
			   printk("elan proximity Sensor get data3 (%d) \n",dat);
			 value = epl6881_get_als_value(obj, dat);
					if(copy_to_user(ptr, &value, sizeof(value)))
					{
						err = -EFAULT;
						goto err_out;
					}			   
					break;
		
			case ALSPS_GET_ALS_RAW_DATA:	

			elan_epl6881_I2C_Write(client,REG_14,R_TWO_BYTE,0x01,0x00);
        	elan_epl6881_I2C_Read(client);
        	gRawData.als_ch0_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

        	elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
       		elan_epl6881_I2C_Read(client);
        	gRawData.als_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
			dat = (gRawData.als_ch0_raw > gRawData.als_ch1_raw) ? gRawData.als_ch0_raw : gRawData.als_ch1_raw;

			elan_epl6881_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);

			elan_epl6881_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
			elan_epl6881_I2C_Read(client);
			regdata = gRawData.raw_bytes[0];
			
			 printk("elan ALS get data regdata[0X0d](%d) \n",regdata);
			 printk("elan ALS get raw data (%d) \n",dat);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;

/*		case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp,dat, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;	  
			}
			epl6881_WriteCalibration(&ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			break;
		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			epl6881_init_client_for_cali(obj->client);
			err = epl6881_read_data_for_cali(obj->client,&ps_cali_temp);
			if(err)
			{
			   goto err_out;
			}
			epl6881_init_client(obj->client);
			// epl6881_enable_ps(obj->client, 1);
			epl6881_enable(obj->client, 0);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}


/*----------------------------------------------------------------------------*/
#if 0
switch(cmd)
    {
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if(err = elan_epl6881_psensor_enable(epld))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						goto err_out;
					}
						
					set_bit(CMC_BIT_PS, &epld->enable_pflag);
				}
				else
				{
					if(err = elan_epl6881_psensor_disable(epld))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						goto err_out;
					}
						
					clear_bit(CMC_BIT_PS, &epld->enable_pflag);
				}
				break;		
		
			case ALSPS_GET_PS_MODE:
				enable = test_bit(CMC_BIT_PS, &epld->enable_pflag) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
		
			case ALSPS_GET_PS_DATA:    
				elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
				
            	elan_epl6881_I2C_Read(client);
				
            	dat = elan_epl_ps_poll_rawdata();
				
			   if(copy_to_user(ptr, &dat, sizeof(dat)))
		      {
				 err = -EFAULT;
				 goto err_out;
			  }  
			   printk("elan proximity Sensor get data (%d) \n",value);
			   
				break;
		
			case ALSPS_GET_PS_RAW_DATA:  
					
				elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
				
            	elan_epl6881_I2C_Read(client);
				
            	gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
				
				if(copy_to_user(ptr, &gRawData.ps_raw , sizeof(gRawData.ps_raw)))
					return -EFAULT;
				
				break;				
		
				case ALSPS_SET_ALS_MODE:
					if(copy_from_user(&enable, ptr, sizeof(enable)))
					{
						err = -EFAULT;
						goto err_out;
					}
					if(enable)
					{
						if(err = elan_epl6881_lsensor_enable(epld))
						{
							APS_ERR("enable als fail: %d\n", err); 
							goto err_out;
						}
						set_bit(CMC_BIT_ALS, &epld->enable_lflag);
					}
					else
					{
						if(err = elan_epl6881_lsensor_disable(epld))
						{
							APS_ERR("disable als fail: %d\n", err); 
							goto err_out;
						}
						clear_bit(CMC_BIT_ALS, &epld->enable_lflag);
					}
					break;
		
				case ALSPS_GET_ALS_MODE:
					enable = test_bit(CMC_BIT_ALS, epld->enable_lflag) ? (1) : (0);
					if(copy_to_user(ptr,&enable, sizeof(enable)))
					{
						err = -EFAULT;
						goto err_out;
					}
					break;
		
				case ALSPS_GET_ALS_DATA: 
					
		      elan_epl6881_I2C_Write(client,REG_14,R_TWO_BYTE,0x01,0x00);
              elan_epl6881_I2C_Read(client);
              gRawData.als_ch0_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

              elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
              elan_epl6881_I2C_Read(client);
              gRawData.als_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
			  dat = (gRawData.als_ch0_raw > gRawData.als_ch1_raw) ? gRawData.als_ch0_raw : gRawData.als_ch1_raw;
			  
			 value = epl6881_get_als_value(epld, dat);
					if(copy_to_user(ptr, &value, sizeof(value)))
					{
						err = -EFAULT;
						goto err_out;
					}			   
					break;
		
			case ALSPS_GET_ALS_RAW_DATA:	

			elan_epl6881_I2C_Write(client,REG_14,R_TWO_BYTE,0x01,0x00);
        	elan_epl6881_I2C_Read(client);
        	gRawData.als_ch0_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

        	elan_epl6881_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
       		elan_epl6881_I2C_Read(client);
        	gRawData.als_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
			dat = (gRawData.als_ch0_raw > gRawData.als_ch1_raw) ? gRawData.als_ch0_raw : gRawData.als_ch1_raw;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
				
			default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
			
    }
	err_out:
	return err;

}
#endif
/*----------------------------------------------------------------------------*/
static struct file_operations epl6881_fops = {
	.owner = THIS_MODULE,
	.open = epl6881_open,
	.release = epl6881_release,
	.unlocked_ioctl = epl6881_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice epl6881_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &epl6881_fops,
};
/*----------------------------------------------------------------------------*/
static int epl6881_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct epl6881_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = epl6881_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = epl6881_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		epl6881_power(obj->hw, 0);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int epl6881_i2c_resume(struct i2c_client *client)
{
	struct epl6881_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	epl6881_power(obj->hw, 1);
	if(err = epl6881_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = epl6881_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = epl6881_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static void epl6881_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct epl6881_priv *obj = container_of(h, struct epl6881_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	#if 0
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = epl6881_enable_als(obj->client, 0))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static void epl6881_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct epl6881_priv *obj = container_of(h, struct epl6881_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 0
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = epl6881_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
}

int epl6881_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct epl6881_priv *obj = (struct epl6881_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if(err = epl6881_enable_ps(obj->client, 1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					if(err = epl6881_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if(err = epl6881_enable_ps(obj->client, 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					if(err = epl6881_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				epl6881_read_ps(obj->client, &obj->ps);
				
                                //mdelay(160);
				//epl6881_read_als_ch0(obj->client, &obj->als);
				APS_ERR("epl6881_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = epl6881_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int epl6881_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct epl6881_priv *obj = (struct epl6881_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if(err = epl6881_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if(err = epl6881_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				epl6881_read_als(obj->client, &obj->als);
								
				sensor_data->values[0] = epl6881_get_als_value(obj, obj->als);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int epl6881_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, EPL6881_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int epl6881_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct epl6881_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	epl6881_obj = obj;

	obj->hw = get_cust_alsps_hw();
	epl6881_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	INIT_DELAYED_WORK(&obj->eint_work, epl6881_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	//obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100 //16
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	
	epl6881_i2c_client = client;

	
	if(err = epl6881_init_client(client))
	{
		goto exit_init_failed;
	}
	APS_LOG("epl6881_init_client() OK!\n");

	if(err = misc_register(&epl6881_device))
	{
		APS_ERR("epl6881_device register failed\n");
		goto exit_misc_device_register_failed;
	}
/*
	if(err = epl6881_create_attr(&epl6881_alsps_driver.driver))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
*/
	obj_ps.self = epl6881_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = epl6881_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = epl6881_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = epl6881_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = epl6881_early_suspend,
	obj->early_drv.resume   = epl6881_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&epl6881_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	epl6881_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int epl6881_i2c_remove(struct i2c_client *client)
{
	int err;	
/*	
	if(err = epl6881_delete_attr(&epl6881_i2c_driver.driver))
	{
		APS_ERR("epl6881_delete_attr fail: %d\n", err);
	} 
*/
	if(err = misc_deregister(&epl6881_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	epl6881_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int epl6881_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	//epl6881_power(hw, 1);    
	//epl6881_force[0] = hw->i2c_num;
	//epl6881_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",epl6881_force[0],epl6881_force[1]);
	if(i2c_add_driver(&epl6881_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int epl6881_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	epl6881_power(hw, 0);    
	i2c_del_driver(&epl6881_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver epl6881_alsps_driver = {
	.probe      = epl6881_probe,
	.remove     = epl6881_remove,    
	.driver     = {
		.name  = "als_ps",
//		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init epl6881_init(void)
{
	APS_FUN();
	i2c_register_board_info(0, &i2c_EPL6881, 1);
	if(platform_driver_register(&epl6881_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit epl6881_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&epl6881_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(epl6881_init);
module_exit(epl6881_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("epl6881 driver");
MODULE_LICENSE("GPL");
