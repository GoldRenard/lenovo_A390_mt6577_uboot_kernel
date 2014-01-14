#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include "cust_gpio_usage.h"

#include "tpd_custom_himax.h"         
#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif
#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#endif


extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 
 #if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
static int P_ID = 0;
static int p_soft_key = 0xFF;
static int g_soft_key = 0xFF;


#define TPD_OK 0

struct touch_info {
    u32 y[4];
    u32 x[4];
    u32 p[4];
    int count;
};
 
#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK,KEY_HOME,KEY_SEARCH,KEY_MENU};//TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] ={{67,840,100,80},{218,840,100,80},{348,840,100,80},{450,840,100,80}}; //TPD_KEYS_DIM;
#endif

 static const struct i2c_device_id tpd_id[] = {{"himax",0},{}};
// unsigned short force[] = {0,0x94,I2C_CLIENT_END,I2C_CLIENT_END}; 
// static const unsigned short * const forces[] = { force, NULL };
// static struct i2c_client_address_data addr_data = { .forces = forces, };
 static struct i2c_board_info __initdata himax_i2c_tpd={ I2C_BOARD_INFO("himax", (0x94>>1))};
  
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "himax",
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = tpd_id,
  .detect = tpd_detect,
 // .address_data = &addr_data,
 };
 
static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 printk("D[%4d %4d %4d] ", x, y, p);
	 input_mt_sync(tpd->dev);
	 TPD_DOWN_DEBUG_TRACK(x,y);
	 if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(x, y, 1);  
    }
 }
 
static  int tpd_up(int x, int y,int *count) {
	//	 input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
	//	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	//	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 printk("U[%4d %4d %4d] ", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_UP_DEBUG_TRACK(x,y);
		// (*count)--;
		if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(x, y, 0); 
    }       
	    return 0;
 }


static int tpd_i2c_read_data(struct i2c_client *client, u8 command, char *buf, int count)
{	
/*	int ret = 0;	
	u16 old_flag = client->addr;	
	client->addr = client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;	
	buf[0] = command;	
	ret = i2c_master_send(client, buf, (count << 8 | 1));		
	client->addr = old_flag;*/
    int ret =0;
	buf[0] = command;
	ret = i2c_master_send(client, buf, 1);
	if (ret < 0)
	{
		TPD_DMESG("Read data error\n");
		return ret;
	}

	ret = i2c_master_recv(client, buf, count);

	return ret;
}

static int tpd_i2c_write_byte_data(struct i2c_client *client, u8 command, u8 data)
{	
	int ret = 0;	
	u8 buf[2] = {command, data};	
	ret = i2c_master_send(client, buf, 2);	
	return ret;
}


static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 { 
	int i = 0;
	//int soft_key = 0xFF;
	//char data[24] = {0};
	char data[30] = {0};
	//char buf_data[2] = {0};
	//static u16 l_info_x[4]={0},l_info_y[4]={0};
	u16 high_byte,low_byte;
	u8   u8_id = 0;
	u8 check_sum_cal = 0;
	//u8 check_sum_FW;
	
	const u8 PT_NUM_MAX = 4;
	const u16 RESOLUTION_Y = 800;
	const u16 RESOLUTION_X = 480;
	const u8 PT_LEAVE = 1;
	const u8 CHECK_SUM_LENGTH = 24;

	p_point_num = point_num;
	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));

	i2c_smbus_read_i2c_block_data(i2c_client, 0x86, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x86, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x86, 8, &(data[16]));


	TPD_DMESG("data: \n");

	for (i = 0;  i<=23; i++) {
		printk("%x ", data[i]);
		if (i % 8 == 0)
			printk("\n");
	}
	printk("\n");
	
	if ((data[20] > 0xF4 && data[20] != 0xff )|| data[20] < 0xF0)
	{
		return false;
	}
	
	g_soft_key = data[22];	
	if(data[20] == 0xFF){
		point_num = 0;	
	}else{
		point_num= data[20] & 0x07;
		//printk("Ghong_zguoqing read the hx8526 touch points number is = %x,\n",point_num);
	}
	if(g_soft_key != 0xFF)
	{
		point_num = 0;
		switch (g_soft_key)
		{
			case 1:
				input_report_key(tpd->kpd,KEY_MENU,1);
				input_sync(tpd->kpd);
				break;
			case 2:
				input_report_key(tpd->kpd,KEY_SEARCH,1);
				input_sync(tpd->kpd);
				break;
			case 3:
				input_report_key(tpd->kpd,KEY_HOME,1);
				input_sync(tpd->kpd);
				break;
			case 4:
				input_report_key(tpd->kpd,KEY_BACK,1);
				input_sync(tpd->kpd);
				break;
			default:
				;
		}
		p_soft_key = g_soft_key;
		printk("input key softkey %d\n",g_soft_key);
		return 0;
	}
	else if(p_soft_key != 0xFF) 
	{
		point_num = 0;
		switch (p_soft_key)
		{
			case 1:
				input_report_key(tpd->kpd,KEY_MENU,0);
				input_sync(tpd->kpd);
				break;
			case 2:
				input_report_key(tpd->kpd,KEY_SEARCH,0);
				input_sync(tpd->kpd);
				break;
			case 3:
				input_report_key(tpd->kpd,KEY_HOME,0);
				input_sync(tpd->kpd);
				break;
			case 4:
				input_report_key(tpd->kpd,KEY_BACK,0);
				input_sync(tpd->kpd);
				break;
			default:
				;
		}
		p_soft_key = g_soft_key;
		printk("Release input key softkey %d\n",g_soft_key);
		return 0;
	}

	p_soft_key = g_soft_key;
	for(i = 0; i < PT_NUM_MAX; i++)
	{
		if (data[4*i] != 0xFF)
		{
			/*get the X coordinate, 2 bytes*/
			high_byte = data[4*i+2];
			high_byte <<= 8;
			low_byte = data[4*i+3];
			high_byte = high_byte | low_byte;
			if (high_byte <= RESOLUTION_X)
			{
				cinfo->x[i] = high_byte;
			}
			else
			{
				cinfo->x[i] = 0xFFFF;
				cinfo->y[i] = 0xFFFF;
				continue;
			}

			/*get the Y coordinate, 2 bytes*/
			high_byte = data[4*i];
			high_byte <<= 8;
			//high_byte &= 0x0f00;
			low_byte = data[4*i + 1];
			high_byte = high_byte | low_byte;
			if (high_byte <= RESOLUTION_Y)
			{
				cinfo->y[i] = high_byte;
			}
			else
			{
				cinfo->x[i] = 0xFFFF;
				cinfo->y[i] = 0xFFFF;
				continue;
			}
			printk(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,cinfo->x[i],i, cinfo->y[i],i, cinfo->p[i]);
			cinfo->count++;
			//cinfo->p[i] = 0;
		}
		else
		{
			cinfo->x[i] = 0xFFFF;
			cinfo->y[i] = 0xFFFF;
			printk(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,cinfo->x[i],i, cinfo->y[i],i, cinfo->p[i]);
		}
	}
	return true;

 };




static int touch_event_handler(void *unused)
 {
  
    struct touch_info cinfo = {0};
	struct touch_info pinfo = {0};      
	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
	 //	msleep(15);
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 set_current_state(TASK_INTERRUPTIBLE); 
		  wait_event_interruptible(waiter,tpd_flag!=0);
						 
			 tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 

		  if (tpd_touchinfo(&cinfo, &pinfo)) {
		  TPD_DEBUG("point_num = %d\n",point_num);
		  
            if(point_num >0) {
               
                tpd_down(cinfo.x[0], cinfo.y[0], 1); 
                if(point_num>1){
                   tpd_down(cinfo.x[1], cinfo.y[1], 1);
			       if(point_num >2) {
				   	tpd_down(cinfo.x[2], cinfo.y[2], 1);
			       	} if(point_num>3) tpd_down(cinfo.x[3], cinfo.y[3], 1);
             	}
                input_sync(tpd->dev);
				TPD_DEBUG("press --->\n");
				
            } else{
            TPD_DEBUG("release --->\n"); 
               tpd_up(pinfo.x[0], pinfo.y[0], 0);
                input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
        }

 }while(!kthread_should_stop());
 
	 return 0;
 }
 

static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 TPD_DMESG("TPD 3 has been triggered\n");
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	char data[3];
	i2c_client = client;
    //client->timing = 300;
	TPD_DMESG("ENTER tpd_probe\n");
	 #ifdef MT6577
    //power on, need confirm with SA
       hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
   //    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");      
       #endif
/*
	#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
	#else*/
      //  hwPowerOn(MT65XX_POWER_LDO_VMC,VOL_3300,"SD");
      
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);///hhhhhhhhhh
        msleep(10);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);//lllllllllllllllll
	    msleep(10);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);//hhhhhhhhhhhhhhhhhhhhhhh
        msleep(50);
		TPD_DMESG("Enter RST pin configuraion!\n");
 
	//#endif
        
	

		/*data[0] = 0x81;
		if((i2c_master_send(i2c_client, data, 1)< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }

	msleep(120);

		 if((tpd_i2c_write_byte_data(i2c_client, 0x42, 2))< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }
          
         if((tpd_i2c_write_byte_data(i2c_client, 0x35, 2)< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }
	   msleep(1);
        
           data[0] =0x02;
         if((i2c_smbus_write_i2c_block_data(i2c_client, 0x35, 1, &data[0]))< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }

         msleep(1);
           data[0] =0x0f;data[1] =0x53;
         if((i2c_smbus_write_i2c_block_data(i2c_client, 0x36, 2, &data[0]))< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }

         msleep(1);
           data[0] =0x06;data[1] =0x2;
         if((i2c_smbus_write_i2c_block_data(i2c_client, 0xdd, 2, &data[0]))< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }

         msleep(1);
           data[0] =0x00;
         if((i2c_smbus_write_i2c_block_data(i2c_client, 0xe9, 1, &data[0]))< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }
         msleep(1);
 
         if((i2c_smbus_write_i2c_block_data(i2c_client, 0x83, 0, &data[0]))< 0)
	   {
		   TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }
         msleep(200);*/

	 data[0] =0x02;
			 if((i2c_smbus_write_i2c_block_data(i2c_client, 0x42, 1, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
	
			  msleep(100);
	
		if((i2c_smbus_write_i2c_block_data(i2c_client, 0x81, 0, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
			  msleep(160);//120//Ghong_zguoqing
	
			   data[0] =0x02;
			 if((i2c_smbus_write_i2c_block_data(i2c_client, 0x35, 1, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
	
		msleep(100);
	
			   data[0] =0x0f;data[1] =0x53;
			 if((i2c_smbus_write_i2c_block_data(i2c_client, 0x36, 2, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
	
		msleep(100);
			   data[0] =0x06;data[1] =0x2;
	//GHong_zguoqing_modified ,@ default is data[0]=0x05
			 if((i2c_smbus_write_i2c_block_data(i2c_client, 0xdd, 2, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
	
		msleep(100);
			   data[0] =0x00;
			 if((i2c_smbus_write_i2c_block_data(i2c_client, 0xe9, 1, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
	msleep(100);
	/*-------------------------------Ghong_zguoqing,@20120522 check sum-------------------------------*/
	/*
				data[0] =0x10;
			 if((i2c_smbus_write_i2c_block_data(i2c_client, 0xAB, 1, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
	
			  msleep(100);
			data[0] =0x1d;data[1] =0x04;
		if((i2c_smbus_write_i2c_block_data(i2c_client, 0xAC, 2, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
			  msleep(120);
		
		if((i2c_smbus_read_i2c_block_data(i2c_client, 0xAB, 1, &data[0]))< 0)
		   {
			   //printk(" Ghong_zguoqing_marked ,@check sum func_EEEEEEEEEEEError, line: %d\n ,the data of 0xAB is = %x\n", __LINE__,data[0]);
			   return -1; 
		   }
		 //printk(" Ghong_zguoqing_marked ,@check sum func_SSSSSSSSSSSSSSSSSuccess, line: %d\n ,the data of 0xAB is = %x\n", __LINE__,data[0]);
	
		if((i2c_smbus_read_i2c_block_data(i2c_client, 0x31, 3, &data[0]))< 0)
		   {
			   //printk(" Ghong_zguoqing_marked ,@I2C transfer error in the func tpd_touchinfo(), line: %d\n ,the data[0] = %x,data[1] = %x,data[2] = %x\n", __LINE__,data[0],data[1],data[2]);
			   return -1; 
		   }
		
			//printk("Ghong_zguoqing read the hx8526 id_SSSSSSSSSSSSSSSSSSSuccess is = %x\n,%x\n,%x\n",data[0],data[1],data[2]);
	
		*/
	/*-------------------------------Ghong_zguoqing,@20120522 check sum-------------------------------*/
			 if((i2c_smbus_write_i2c_block_data(i2c_client, 0x83, 0, &data[0]))< 0)
		   {
			   //printk("Ghong_zguoqing_marked I2C transfer error, line: %d\n", __LINE__);
			   return -1; 
		   }
	  msleep(200);

	
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}


	      mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
		  mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		  mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
		  mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	 
		  mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_LEVEL_SENSITIVE);
		  mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
		  mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 0); 
		  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

		tpd_load_status = 1;
	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
        return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 
 {
   
	 TPD_DEBUG("TPD removed\n");
 
   return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG("Focaltech HIMAX_TS I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

    if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("unable to add i2c driver.\n");
      	return -1;
    }

	if(tpd_load_status == 0) 
    {
    	TPD_DMESG("himax add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }

 static int tpd_resume(struct i2c_client *client)
 {
  int retval = TPD_OK;
  static char data[2];
   TPD_DEBUG("TPD wake up\n");
   
	         data[0] =0x00;
      i2c_smbus_write_i2c_block_data(i2c_client, 0xD7, 1, &data[0]);
	  msleep(1);

	         data[0] =0x02;
      i2c_smbus_write_i2c_block_data(i2c_client, 0x42, 1, &data[0]);
	  //msleep(120);

         msleep(1);

     i2c_smbus_write_i2c_block_data(i2c_client, 0x81, 0, &data[0]);

          msleep(200);
           data[0] =0x02;
      //i2c_smbus_write_i2c_block_data(i2c_client, 0x42, 1, &data[0]);
       //msleep(100);

		  
           data[0] =0x02;
    i2c_smbus_write_i2c_block_data(i2c_client, 0x35, 1, &data[0]);
     msleep(1);

           data[0] =0x0f;data[1] =0x53;
     i2c_smbus_write_i2c_block_data(i2c_client, 0x36, 2, &data[0]);
      msleep(1);
	  
           data[0] =0x06;data[1] =0x2;
       i2c_smbus_write_i2c_block_data(i2c_client, 0xdd, 2, &data[0]);


       msleep(1);
           data[0] =0x00;
       i2c_smbus_write_i2c_block_data(i2c_client, 0xe9, 1, &data[0]);

        msleep(1);
 
       i2c_smbus_write_i2c_block_data(i2c_client, 0x83, 0, &data[0]);
       msleep(1);

 mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	 return retval;
 }
 
 static int tpd_suspend(struct i2c_client *client, pm_message_t message)
 {
	 int retval = TPD_OK;
	 static char data[2];
 
	 TPD_DEBUG("TPD enter sleep\n");
	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
//         #ifdef TPD_CLOSE_POWER_IN_SLEEP	
//	 hwPowerDown(TPD_POWER_SOURCE,"TP");
//         #else
         i2c_smbus_write_i2c_block_data(i2c_client, 0x82, 0, &data[0]);  //TP enter sleep mode
         msleep(120);

         //data[0] =0x02;
         i2c_smbus_write_i2c_block_data(i2c_client, 0x80, 0, &data[0]);

		 msleep(120);

         data[0] =0x01;
      i2c_smbus_write_i2c_block_data(i2c_client, 0xD7, 1, &data[0]);
	
		// msleep(120);


      msleep(100);
     //    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
     //    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
     //    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
//         #endif
	 return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "himax",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
	 printk("MediaTek HIMAX_TS touch panel driver init\n");
	     i2c_register_board_info(0, &himax_i2c_tpd, 1);
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add HIMAX_TS driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek HIMAX_TS touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);












