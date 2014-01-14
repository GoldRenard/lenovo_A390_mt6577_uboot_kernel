#include <linux/string.h>
#ifdef BUILD_UBOOT
#include <asm/arch/mt6575_gpio.h>
#else
#include <mach/mt6575_gpio.h>
#endif
#include "lcm_drv.h"

//#if 1
// #define dbg_print printf
//#else if 0
// #define dbg_print printk
//#endif
#define dbg_print

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define RST_GPIO_PIN (GPIO_AST_RST_PIN)
#define LSA0_GPIO_PIN (GPIO_DISP_LSA0_PIN)
#define LSCE_GPIO_PIN (GPIO_DISP_LSCE_PIN)
#define LSCK_GPIO_PIN (GPIO_DISP_LSCK_PIN)
#define LSDA_GPIO_PIN (GPIO_DISP_LSDA_PIN)

/* #define LSCE_GPIO_PIN (GPIO_DISP_LSCE_PIN) */
/* #define LSCK_GPIO_PIN (GPIO_DISP_LSCK_PIN) */
/* #define LSDA_GPIO_PIN (GPIO_DISP_LSA0_PIN) */
/* #define LSA0_GPIO_PIN (GPIO_DISP_LSDA_PIN) */

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)  (lcm_util.set_gpio_out((n), (v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define SET_RST_LOW    SET_GPIO_OUT(RST_GPIO_PIN,0)
#define SET_RST_HIGH    SET_GPIO_OUT(RST_GPIO_PIN,1)
#define SET_LSCE_LOW   SET_GPIO_OUT(LSCE_GPIO_PIN, 0)
#define SET_LSCE_HIGH  SET_GPIO_OUT(LSCE_GPIO_PIN, 1)
#define SET_LSCK_LOW   SET_GPIO_OUT(LSCK_GPIO_PIN, 0)
#define SET_LSCK_HIGH  SET_GPIO_OUT(LSCK_GPIO_PIN, 1)
#define SET_LSDA_LOW   SET_GPIO_OUT(LSDA_GPIO_PIN, 0)
#define SET_LSDA_HIGH  SET_GPIO_OUT(LSDA_GPIO_PIN, 1)

#define ICID_MODE 0
#define SET_ICID_HIGH SET_GPIO_OUT(GPIO36, 1)
#define SET_ICID_LOW SET_GPIO_OUT(GPIO36, 0)

#define SET_DISP_TE_HIGH SET_GPIO_OUT(GPIO137, 1)
#define SET_DISP_TE_LOW SET_GPIO_OUT(GPIO137, 0)
#define GET_LSA0_IN mt_get_gpio_in(LSA0_GPIO_PIN)

static __inline void spi_send_data(unsigned int data)
{
    unsigned int i;

//    UDELAY(200);

    for (i = 0; i < 8; ++ i) //24
    {
        SET_LSCK_LOW;
        if (data & (1 << 7)) {
            SET_LSDA_HIGH;
        } else {
            SET_LSDA_LOW;
        }
        UDELAY(200);
        SET_LSCK_HIGH;
        UDELAY(200);
        data <<= 1;
    }

}

#if 0
static __inline void spi_send_data(unsigned short data)
{
  unsigned int i;

  int clk_num;
  int clk_mask;

  UDELAY(200);
  SET_LSCK_HIGH;
  SET_LSDA_HIGH;
  UDELAY(200);

  clk_num = 16;
  clk_mask = 0x8000;
  dbg_print("\n^");
  UDELAY(100);
  for (i = 0; i < clk_num; ++ i)
    {
	  SET_LSCK_LOW;
	  if (data & (clk_mask)) {  // 0x80= (1 << (9-1))
		SET_LSDA_HIGH;
		dbg_print("~1");
	  } else {
		SET_LSDA_LOW;
		dbg_print("~0");
	  }
	  UDELAY(300);
	  SET_LSCK_HIGH;
	  UDELAY(300);
	  data <<= 1;
    }
  dbg_print("\n");

  SET_ICID_HIGH;

  SET_LSDA_HIGH;
}
#endif
static __inline void spi_send_data_ICID(unsigned int data)
{
  unsigned int i;

  UDELAY(100);
  SET_LSCK_HIGH;
  SET_LSDA_HIGH;
  UDELAY(100);

  for (i = 0; i < 9; ++ i)
    {
	  SET_LSCK_LOW;
	  if (data & (0x100)) {  // 0x80= (1 << (9-1))
		SET_LSDA_HIGH;
		dbg_print("~1");
	  } else {
		SET_LSDA_LOW;
		dbg_print("~0");
	  }
	  UDELAY(100);
	  SET_LSCK_HIGH;
	  UDELAY(100);
	  data <<= 1;
    }

  SET_LSDA_HIGH;
}

static __inline unsigned int spi_read_data(void)
{
  unsigned int i;
  s32 inA0 = -1;

  UDELAY(100);
  SET_LSCK_HIGH;
  SET_LSDA_HIGH;
  UDELAY(100);

  for (i = 0; i < 9; ++ i)
    {
	  SET_LSCK_LOW;
	  /* if (data & (0x100)) {  // 0x80= (1 << (9-1)) */
	  /* 	SET_LSDA_HIGH; */
	  /* 	dbg_print("~1"); */
	  /* } else { */
	  /* 	SET_LSDA_LOW; */
	  /* 	dbg_print("~0"); */
	  /* } */
	  UDELAY(100);
	  inA0 = GET_LSA0_IN;
	  //printf("`%d",inA0);
	  SET_LSCK_HIGH;
	  UDELAY(100);
    }
  //printf("\n");


  SET_LSDA_HIGH;
}

static __inline void send_ctrl_cmd(unsigned int cmd)
{
  dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): cmd:0x%x \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__,cmd); 

  unsigned int out = (cmd & 0xFF); 
  spi_send_data(out);
}

static __inline void send_data_cmd(unsigned int data)
{
  dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): : data:0x%x \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__, data); 

  unsigned int out = (data & 0xFF)| 0x100;
  spi_send_data(out);
}

static __inline void send_data(unsigned short data)
{
  dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): : data:0x%x \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__, data); 

  //unsigned int out = (data & 0xFF)| 0x100;
  spi_send_data(data);
}




#define LCD_WRITE_CMD(cmd) SET_LSCE_LOW;send_ctrl_cmd(cmd);SET_LSCE_HIGH
#define LCD_WRITE_DATA(data) SET_LSCE_LOW;send_data_cmd(data);SET_LSCE_HIGH
#define LCD_CtrlWrite(cmd) SET_LSCE_LOW;send_ctrl_cmd(cmd);SET_LSCE_HIGH
#define LCD_DatWrite(data) SET_LSCE_LOW;send_data_cmd(data);SET_LSCE_HIGH

static __inline void LCM_wcomm(unsigned int cmd)
{
    unsigned char temp1 = (unsigned char)((cmd >> 8) & 0xFF);
    unsigned char temp2 = (unsigned char)(cmd & 0xFF);
    
    SET_LSCE_LOW;
    UDELAY(200);
    spi_send_data(0x20);
    spi_send_data(temp1);
    UDELAY(200);
    SET_LSCE_LOW;
    UDELAY(200);
    spi_send_data(0x00);
    spi_send_data(temp2);
    SET_LSCE_HIGH;
}


static __inline void LCM_wdata(unsigned int data)
{
    SET_LSCE_LOW;
    UDELAY(200);
    spi_send_data(0x40);
    spi_send_data(data);
    SET_LSCE_HIGH;
}


/* END: added by jimmyteng */

static void InitLCD_NT35510_DPI()
{
#if 0
// truly
// NT35516 + AUO 4.29'  for 9K1325
// VCC=IOVCC=3.3V  RGB_24Bit
 	SET_RST_HIGH;
  	MDELAY(10);
  	SET_RST_LOW;
  	MDELAY(50);
  	SET_RST_HIGH;
  	MDELAY(120);

 //TEST Commands
  LCM_wcomm(0xFF00); LCM_wdata(0xAA);//AA
  LCM_wcomm(0xFF01); LCM_wdata(0x55);//55
  LCM_wcomm(0xFF02); LCM_wdata(0x25);//08
  LCM_wcomm(0xFF03); LCM_wdata(0x01);//01
  
  //LCM_wcomm(0xFA0F); LCM_wdata(0x20);

  LCM_wcomm(0xF300); LCM_wdata(0x02);
  LCM_wcomm(0xF303); LCM_wdata(0x15);

////###### Page 0 #####//
  //ENABLE PAGE 0    
  LCM_wcomm(0xF000); LCM_wdata(0x55); //Manufacture Command Set Control   
  LCM_wcomm(0xF001); LCM_wdata(0xAA);
  LCM_wcomm(0xF002); LCM_wdata(0x52);
  LCM_wcomm(0xF003); LCM_wdata(0x08);
  LCM_wcomm(0xF004); LCM_wdata(0x00);

  LCM_wcomm(0xB800); LCM_wdata(0x01); 
  LCM_wcomm(0xB801); LCM_wdata(0x02);
  LCM_wcomm(0xB802); LCM_wdata(0x02);
  LCM_wcomm(0xB803); LCM_wdata(0x02);

  LCM_wcomm(0xBC00); LCM_wdata(0x05); //Zig-Zag Inversion  
  LCM_wcomm(0xBC01); LCM_wdata(0x05);
  LCM_wcomm(0xBC02); LCM_wdata(0x05);

  LCM_wcomm(0x4C00); LCM_wdata(0x11); //DB4=1,Enable Vivid Color,DB4=0 Disable Vivid Color

//###### Page 1#####//
// ENABLE PAGE 1   
  LCM_wcomm(0xF000); LCM_wdata(0x55); //Manufacture Command Set Control      
  LCM_wcomm(0xF001); LCM_wdata(0xAA);
  LCM_wcomm(0xF002); LCM_wdata(0x52);
  LCM_wcomm(0xF003); LCM_wdata(0x08);
  LCM_wcomm(0xF004); LCM_wdata(0x01);//Page1

  LCM_wcomm(0xB000); LCM_wdata(0x05); // Setting AVDD Voltage 6V
  LCM_wcomm(0xB001); LCM_wdata(0x05);
  LCM_wcomm(0xB002); LCM_wdata(0x05);

  LCM_wcomm(0xB600); LCM_wdata(0x44); // Setting AVEE boosting time 2.5*vpnl 
  LCM_wcomm(0xB601); LCM_wdata(0x44);
  LCM_wcomm(0xB602); LCM_wdata(0x44);

  LCM_wcomm(0xB100); LCM_wdata(0x05); // Setting AVEE Voltage -6V
  LCM_wcomm(0xB101); LCM_wdata(0x05);
  LCM_wcomm(0xB102); LCM_wdata(0x05);



//Setting AVEE boosting time -2.5xVPNL
  LCM_wcomm(0xB700); LCM_wdata(0x34); 
  LCM_wcomm(0xB701); LCM_wdata(0x34);
  LCM_wcomm(0xB702); LCM_wdata(0x34);

//Setting VGLX boosting time  AVEE-AVDD
  LCM_wcomm(0xBA00); LCM_wdata(0x14); //0x24 --> 0x14
  LCM_wcomm(0xBA01); LCM_wdata(0x14);
  LCM_wcomm(0xBA02); LCM_wdata(0x14);

//Gamma Voltage
  LCM_wcomm(0xBC00); LCM_wdata(0x00); 
  LCM_wcomm(0xBC01); LCM_wdata(0xA0);//VGMP 0x88=4.7V  0x78=4.5V   0xA0=5.0V  
  LCM_wcomm(0xBC02); LCM_wdata(0x00);//VGSP 

//Gamma Voltage
  LCM_wcomm(0xBD00); LCM_wdata(0x00); 
  LCM_wcomm(0xBD01); LCM_wdata(0xA0);//VGMN 0x88=-4.7V 0x78=-4.5V   0xA0=-5.0V
  LCM_wcomm(0xBD02); LCM_wdata(0x00);//VGSN  

  LCM_wcomm(0xBE00); LCM_wdata(0x57); // Setting VCOM Offset Voltage  0x4E to 0x57  20111019 LIYAN

//GAMMA RED Positive       
  LCM_wcomm(0xD100); LCM_wdata(0x00);
  LCM_wcomm(0xD101); LCM_wdata(0x32);
  LCM_wcomm(0xD102); LCM_wdata(0x00);
  LCM_wcomm(0xD103); LCM_wdata(0x41);
  LCM_wcomm(0xD104); LCM_wdata(0x00);
  LCM_wcomm(0xD105); LCM_wdata(0x54);
  LCM_wcomm(0xD106); LCM_wdata(0x00);
  LCM_wcomm(0xD107); LCM_wdata(0x67);
  LCM_wcomm(0xD108); LCM_wdata(0x00);
  LCM_wcomm(0xD109); LCM_wdata(0x7A);
  LCM_wcomm(0xD10A); LCM_wdata(0x00);
  LCM_wcomm(0xD10B); LCM_wdata(0x98);
  LCM_wcomm(0xD10C); LCM_wdata(0x00);
  LCM_wcomm(0xD10D); LCM_wdata(0xB0);
  LCM_wcomm(0xD10E); LCM_wdata(0x00);
  LCM_wcomm(0xD10F); LCM_wdata(0xDB);
  LCM_wcomm(0xD200); LCM_wdata(0x01);
  LCM_wcomm(0xD201); LCM_wdata(0x01);
  LCM_wcomm(0xD202); LCM_wdata(0x01);
  LCM_wcomm(0xD203); LCM_wdata(0x3F);
  LCM_wcomm(0xD204); LCM_wdata(0x01);
  LCM_wcomm(0xD205); LCM_wdata(0x70);
  LCM_wcomm(0xD206); LCM_wdata(0x01);
  LCM_wcomm(0xD207); LCM_wdata(0xB4);
  LCM_wcomm(0xD208); LCM_wdata(0x01);
  LCM_wcomm(0xD209); LCM_wdata(0xEC);
  LCM_wcomm(0xD20A); LCM_wdata(0x01);
  LCM_wcomm(0xD20B); LCM_wdata(0xED);
  LCM_wcomm(0xD20C); LCM_wdata(0x02);
  LCM_wcomm(0xD20D); LCM_wdata(0x1E);
  LCM_wcomm(0xD20E); LCM_wdata(0x02);
  LCM_wcomm(0xD20F); LCM_wdata(0x51);
  LCM_wcomm(0xD300); LCM_wdata(0x02);
  LCM_wcomm(0xD301); LCM_wdata(0x6C);
  LCM_wcomm(0xD302); LCM_wdata(0x02);
  LCM_wcomm(0xD303); LCM_wdata(0x8D);
  LCM_wcomm(0xD304); LCM_wdata(0x02);
  LCM_wcomm(0xD305); LCM_wdata(0xA5);
  LCM_wcomm(0xD306); LCM_wdata(0x02);
  LCM_wcomm(0xD307); LCM_wdata(0xC9);
  LCM_wcomm(0xD308); LCM_wdata(0x02);
  LCM_wcomm(0xD309); LCM_wdata(0xEA);
  LCM_wcomm(0xD30A); LCM_wdata(0x03);
  LCM_wcomm(0xD30B); LCM_wdata(0x19);
  LCM_wcomm(0xD30C); LCM_wdata(0x03);
  LCM_wcomm(0xD30D); LCM_wdata(0x45);
  LCM_wcomm(0xD30E); LCM_wdata(0x03);
  LCM_wcomm(0xD30F); LCM_wdata(0x7A);
  LCM_wcomm(0xD400); LCM_wdata(0x03);
  LCM_wcomm(0xD401); LCM_wdata(0xB0);
  LCM_wcomm(0xD402); LCM_wdata(0x03);
  LCM_wcomm(0xD403); LCM_wdata(0xF4);

//Positive Gamma for GREEN
  LCM_wcomm(0xD500); LCM_wdata(0x00);
  LCM_wcomm(0xD501); LCM_wdata(0x37);
  LCM_wcomm(0xD502); LCM_wdata(0x00);
  LCM_wcomm(0xD503); LCM_wdata(0x41);
  LCM_wcomm(0xD504); LCM_wdata(0x00);
  LCM_wcomm(0xD505); LCM_wdata(0x54);
  LCM_wcomm(0xD506); LCM_wdata(0x00);
  LCM_wcomm(0xD507); LCM_wdata(0x67);
  LCM_wcomm(0xD508); LCM_wdata(0x00);
  LCM_wcomm(0xD509); LCM_wdata(0x7A);
  LCM_wcomm(0xD50A); LCM_wdata(0x00);
  LCM_wcomm(0xD50B); LCM_wdata(0x98);
  LCM_wcomm(0xD50C); LCM_wdata(0x00);
  LCM_wcomm(0xD50D); LCM_wdata(0xB0);
  LCM_wcomm(0xD50E); LCM_wdata(0x00);
  LCM_wcomm(0xD50F); LCM_wdata(0xDB);
  LCM_wcomm(0xD600); LCM_wdata(0x01);
  LCM_wcomm(0xD601); LCM_wdata(0x01);
  LCM_wcomm(0xD602); LCM_wdata(0x01);
  LCM_wcomm(0xD603); LCM_wdata(0x3F);
  LCM_wcomm(0xD604); LCM_wdata(0x01);
  LCM_wcomm(0xD605); LCM_wdata(0x70);
  LCM_wcomm(0xD606); LCM_wdata(0x01);
  LCM_wcomm(0xD607); LCM_wdata(0xB4);
  LCM_wcomm(0xD608); LCM_wdata(0x01);
  LCM_wcomm(0xD609); LCM_wdata(0xEC);
  LCM_wcomm(0xD60A); LCM_wdata(0x01);
  LCM_wcomm(0xD60B); LCM_wdata(0xED);
  LCM_wcomm(0xD60C); LCM_wdata(0x02);
  LCM_wcomm(0xD60D); LCM_wdata(0x1E);
  LCM_wcomm(0xD60E); LCM_wdata(0x02);
  LCM_wcomm(0xD60F); LCM_wdata(0x51);
  LCM_wcomm(0xD700); LCM_wdata(0x02);
  LCM_wcomm(0xD701); LCM_wdata(0x6C);
  LCM_wcomm(0xD702); LCM_wdata(0x02);
  LCM_wcomm(0xD703); LCM_wdata(0x8D);
  LCM_wcomm(0xD704); LCM_wdata(0x02);
  LCM_wcomm(0xD705); LCM_wdata(0xA5);
  LCM_wcomm(0xD706); LCM_wdata(0x02);
  LCM_wcomm(0xD707); LCM_wdata(0xC9);
  LCM_wcomm(0xD708); LCM_wdata(0x02);
  LCM_wcomm(0xD709); LCM_wdata(0xEA);
  LCM_wcomm(0xD70A); LCM_wdata(0x03);
  LCM_wcomm(0xD70B); LCM_wdata(0x19);
  LCM_wcomm(0xD70C); LCM_wdata(0x03);
  LCM_wcomm(0xD70D); LCM_wdata(0x45);
  LCM_wcomm(0xD70E); LCM_wdata(0x03);
  LCM_wcomm(0xD70F); LCM_wdata(0x7A);
  LCM_wcomm(0xD800); LCM_wdata(0x03);
  LCM_wcomm(0xD801); LCM_wdata(0xA0);
  LCM_wcomm(0xD802); LCM_wdata(0x03);
  LCM_wcomm(0xD803); LCM_wdata(0xF4);

//Positive Gamma for BLUE
  LCM_wcomm(0xD900); LCM_wdata(0x00);
  LCM_wcomm(0xD901); LCM_wdata(0x32);
  LCM_wcomm(0xD902); LCM_wdata(0x00);
  LCM_wcomm(0xD903); LCM_wdata(0x41);
  LCM_wcomm(0xD904); LCM_wdata(0x00);
  LCM_wcomm(0xD905); LCM_wdata(0x54);
  LCM_wcomm(0xD906); LCM_wdata(0x00);
  LCM_wcomm(0xD907); LCM_wdata(0x67);
  LCM_wcomm(0xD908); LCM_wdata(0x00);
  LCM_wcomm(0xD909); LCM_wdata(0x7A);
  LCM_wcomm(0xD90A); LCM_wdata(0x00);
  LCM_wcomm(0xD90B); LCM_wdata(0x98);
  LCM_wcomm(0xD90C); LCM_wdata(0x00);
  LCM_wcomm(0xD90D); LCM_wdata(0xB0);
  LCM_wcomm(0xD90E); LCM_wdata(0x00);
  LCM_wcomm(0xD90F); LCM_wdata(0xDB);
  LCM_wcomm(0xDD00); LCM_wdata(0x01);
  LCM_wcomm(0xDD01); LCM_wdata(0x01);
  LCM_wcomm(0xDD02); LCM_wdata(0x01);
  LCM_wcomm(0xDD03); LCM_wdata(0x3F);
  LCM_wcomm(0xDD04); LCM_wdata(0x01);
  LCM_wcomm(0xDD05); LCM_wdata(0x70);
  LCM_wcomm(0xDD06); LCM_wdata(0x01);
  LCM_wcomm(0xDD07); LCM_wdata(0xB4);
  LCM_wcomm(0xDD08); LCM_wdata(0x01);
  LCM_wcomm(0xDD09); LCM_wdata(0xEC);
  LCM_wcomm(0xDD0A); LCM_wdata(0x01);
  LCM_wcomm(0xDD0B); LCM_wdata(0xED);
  LCM_wcomm(0xDD0C); LCM_wdata(0x02);
  LCM_wcomm(0xDD0D); LCM_wdata(0x1E);
  LCM_wcomm(0xDD0E); LCM_wdata(0x02);
  LCM_wcomm(0xDD0F); LCM_wdata(0x51);
  LCM_wcomm(0xDE00); LCM_wdata(0x02);
  LCM_wcomm(0xDE01); LCM_wdata(0x6C);
  LCM_wcomm(0xDE02); LCM_wdata(0x02);
  LCM_wcomm(0xDE03); LCM_wdata(0x8D);
  LCM_wcomm(0xDE04); LCM_wdata(0x02);
  LCM_wcomm(0xDE05); LCM_wdata(0xA5);
  LCM_wcomm(0xDE06); LCM_wdata(0x02);
  LCM_wcomm(0xDE07); LCM_wdata(0xC9);
  LCM_wcomm(0xDE08); LCM_wdata(0x02);
  LCM_wcomm(0xDE09); LCM_wdata(0xEA);
  LCM_wcomm(0xDE0A); LCM_wdata(0x03);
  LCM_wcomm(0xDE0B); LCM_wdata(0x19);
  LCM_wcomm(0xDE0C); LCM_wdata(0x03);
  LCM_wcomm(0xDE0D); LCM_wdata(0x45);
  LCM_wcomm(0xDE0E); LCM_wdata(0x03);
  LCM_wcomm(0xDE0F); LCM_wdata(0x7A);
  LCM_wcomm(0xDF00); LCM_wdata(0x03);
  LCM_wcomm(0xDF01); LCM_wdata(0xA0);
  LCM_wcomm(0xDF02); LCM_wdata(0x03);
  LCM_wcomm(0xDF03); LCM_wdata(0xF4);

//Negative Gamma for RED
  LCM_wcomm(0xE000); LCM_wdata(0x00);
  LCM_wcomm(0xE001); LCM_wdata(0x32);
  LCM_wcomm(0xE002); LCM_wdata(0x00);
  LCM_wcomm(0xE003); LCM_wdata(0x41);
  LCM_wcomm(0xE004); LCM_wdata(0x00);
  LCM_wcomm(0xE005); LCM_wdata(0x54);
  LCM_wcomm(0xE006); LCM_wdata(0x00);
  LCM_wcomm(0xE007); LCM_wdata(0x67);
  LCM_wcomm(0xE008); LCM_wdata(0x00);
  LCM_wcomm(0xE009); LCM_wdata(0x7A);
  LCM_wcomm(0xE00A); LCM_wdata(0x00);
  LCM_wcomm(0xE00B); LCM_wdata(0x98);
  LCM_wcomm(0xE00C); LCM_wdata(0x00);
  LCM_wcomm(0xE00D); LCM_wdata(0xB0);
  LCM_wcomm(0xE00E); LCM_wdata(0x00);
  LCM_wcomm(0xE00F); LCM_wdata(0xDB);
  LCM_wcomm(0xE100); LCM_wdata(0x01);
  LCM_wcomm(0xE101); LCM_wdata(0x01);
  LCM_wcomm(0xE102); LCM_wdata(0x01);
  LCM_wcomm(0xE103); LCM_wdata(0x3F);
  LCM_wcomm(0xE104); LCM_wdata(0x01);
  LCM_wcomm(0xE105); LCM_wdata(0x70);
  LCM_wcomm(0xE106); LCM_wdata(0x01);
  LCM_wcomm(0xE107); LCM_wdata(0xB4);
  LCM_wcomm(0xE108); LCM_wdata(0x01);
  LCM_wcomm(0xE109); LCM_wdata(0xEC);
  LCM_wcomm(0xE10A); LCM_wdata(0x01);
  LCM_wcomm(0xE10B); LCM_wdata(0xED);
  LCM_wcomm(0xE10C); LCM_wdata(0x02);
  LCM_wcomm(0xE10D); LCM_wdata(0x1E);
  LCM_wcomm(0xE10E); LCM_wdata(0x02);
  LCM_wcomm(0xE10F); LCM_wdata(0x51);
  LCM_wcomm(0xE200); LCM_wdata(0x02);
  LCM_wcomm(0xE201); LCM_wdata(0x6C);
  LCM_wcomm(0xE202); LCM_wdata(0x02);
  LCM_wcomm(0xE203); LCM_wdata(0x8D);
  LCM_wcomm(0xE204); LCM_wdata(0x02);
  LCM_wcomm(0xE205); LCM_wdata(0xA5);
  LCM_wcomm(0xE206); LCM_wdata(0x02);
  LCM_wcomm(0xE207); LCM_wdata(0xC9);
  LCM_wcomm(0xE208); LCM_wdata(0x02);
  LCM_wcomm(0xE209); LCM_wdata(0xEA);
  LCM_wcomm(0xE20A); LCM_wdata(0x03);
  LCM_wcomm(0xE20B); LCM_wdata(0x19);
  LCM_wcomm(0xE20C); LCM_wdata(0x03);
  LCM_wcomm(0xE20D); LCM_wdata(0x45);
  LCM_wcomm(0xE20E); LCM_wdata(0x03);
  LCM_wcomm(0xE20F); LCM_wdata(0x7A);
  LCM_wcomm(0xE300); LCM_wdata(0x03);
  LCM_wcomm(0xE301); LCM_wdata(0xA0);
  LCM_wcomm(0xE302); LCM_wdata(0x03);
  LCM_wcomm(0xE303); LCM_wdata(0xF4);

//Negative Gamma for GERREN
  LCM_wcomm(0xE400); LCM_wdata(0x00);
  LCM_wcomm(0xE401); LCM_wdata(0x32);
  LCM_wcomm(0xE402); LCM_wdata(0x00);
  LCM_wcomm(0xE403); LCM_wdata(0x41);
  LCM_wcomm(0xE404); LCM_wdata(0x00);
  LCM_wcomm(0xE405); LCM_wdata(0x54);
  LCM_wcomm(0xE406); LCM_wdata(0x00);
  LCM_wcomm(0xE407); LCM_wdata(0x67);
  LCM_wcomm(0xE408); LCM_wdata(0x00);
  LCM_wcomm(0xE409); LCM_wdata(0x7A);
  LCM_wcomm(0xE40A); LCM_wdata(0x00);
  LCM_wcomm(0xE40B); LCM_wdata(0x98);
  LCM_wcomm(0xE40C); LCM_wdata(0x00);
  LCM_wcomm(0xE40D); LCM_wdata(0xB0);
  LCM_wcomm(0xE40E); LCM_wdata(0x00);
  LCM_wcomm(0xE40F); LCM_wdata(0xDB);
  LCM_wcomm(0xE500); LCM_wdata(0x01);
  LCM_wcomm(0xE501); LCM_wdata(0x01);
  LCM_wcomm(0xE502); LCM_wdata(0x01);
  LCM_wcomm(0xE503); LCM_wdata(0x3F);
  LCM_wcomm(0xE504); LCM_wdata(0x01);
  LCM_wcomm(0xE505); LCM_wdata(0x70);
  LCM_wcomm(0xE506); LCM_wdata(0x01);
  LCM_wcomm(0xE507); LCM_wdata(0xB4);
  LCM_wcomm(0xE508); LCM_wdata(0x01);
  LCM_wcomm(0xE509); LCM_wdata(0xEC);
  LCM_wcomm(0xE50A); LCM_wdata(0x01);
  LCM_wcomm(0xE50B); LCM_wdata(0xED);
  LCM_wcomm(0xE50C); LCM_wdata(0x02);
  LCM_wcomm(0xE50D); LCM_wdata(0x1E);
  LCM_wcomm(0xE50E); LCM_wdata(0x02);
  LCM_wcomm(0xE50F); LCM_wdata(0x51);
  LCM_wcomm(0xE600); LCM_wdata(0x02);
  LCM_wcomm(0xE601); LCM_wdata(0x6C);
  LCM_wcomm(0xE602); LCM_wdata(0x02);
  LCM_wcomm(0xE603); LCM_wdata(0x8D);
  LCM_wcomm(0xE604); LCM_wdata(0x02);
  LCM_wcomm(0xE605); LCM_wdata(0xA5);
  LCM_wcomm(0xE606); LCM_wdata(0x02);
  LCM_wcomm(0xE607); LCM_wdata(0xC9);
  LCM_wcomm(0xE608); LCM_wdata(0x02);
  LCM_wcomm(0xE609); LCM_wdata(0xEA);
  LCM_wcomm(0xE60A); LCM_wdata(0x03);
  LCM_wcomm(0xE60B); LCM_wdata(0x19);
  LCM_wcomm(0xE60C); LCM_wdata(0x03);
  LCM_wcomm(0xE60D); LCM_wdata(0x45);
  LCM_wcomm(0xE60E); LCM_wdata(0x03);
  LCM_wcomm(0xE60F); LCM_wdata(0x7A);
  LCM_wcomm(0xE700); LCM_wdata(0x03);
  LCM_wcomm(0xE701); LCM_wdata(0xA0);
  LCM_wcomm(0xE702); LCM_wdata(0x03);
  LCM_wcomm(0xE703); LCM_wdata(0xF4);

//Negative Gamma for BLUE
  LCM_wcomm(0xE800); LCM_wdata(0x00);
  LCM_wcomm(0xE801); LCM_wdata(0x32);
  LCM_wcomm(0xE802); LCM_wdata(0x00);
  LCM_wcomm(0xE803); LCM_wdata(0x41);
  LCM_wcomm(0xE804); LCM_wdata(0x00);
  LCM_wcomm(0xE805); LCM_wdata(0x54);
  LCM_wcomm(0xE806); LCM_wdata(0x00);
  LCM_wcomm(0xE807); LCM_wdata(0x67);
  LCM_wcomm(0xE808); LCM_wdata(0x00);
  LCM_wcomm(0xE809); LCM_wdata(0x7A);
  LCM_wcomm(0xE80A); LCM_wdata(0x00);
  LCM_wcomm(0xE80B); LCM_wdata(0x98);
  LCM_wcomm(0xE80C); LCM_wdata(0x00);
  LCM_wcomm(0xE80D); LCM_wdata(0xB0);
  LCM_wcomm(0xE80E); LCM_wdata(0x00);
  LCM_wcomm(0xE80F); LCM_wdata(0xDB);
  LCM_wcomm(0xE900); LCM_wdata(0x01);
  LCM_wcomm(0xE901); LCM_wdata(0x01);
  LCM_wcomm(0xE902); LCM_wdata(0x01);
  LCM_wcomm(0xE903); LCM_wdata(0x3F);
  LCM_wcomm(0xE904); LCM_wdata(0x01);
  LCM_wcomm(0xE905); LCM_wdata(0x70);
  LCM_wcomm(0xE906); LCM_wdata(0x01);
  LCM_wcomm(0xE907); LCM_wdata(0xB4);
  LCM_wcomm(0xE908); LCM_wdata(0x01);
  LCM_wcomm(0xE909); LCM_wdata(0xEC);
  LCM_wcomm(0xE90A); LCM_wdata(0x01);
  LCM_wcomm(0xE90B); LCM_wdata(0xED);
  LCM_wcomm(0xE90C); LCM_wdata(0x02);
  LCM_wcomm(0xE90D); LCM_wdata(0x1E);
  LCM_wcomm(0xE90E); LCM_wdata(0x02);
  LCM_wcomm(0xE90F); LCM_wdata(0x51);
  LCM_wcomm(0xEA00); LCM_wdata(0x02);
  LCM_wcomm(0xEA01); LCM_wdata(0x6C);
  LCM_wcomm(0xEA02); LCM_wdata(0x02);
  LCM_wcomm(0xEA03); LCM_wdata(0x8D);
  LCM_wcomm(0xEA04); LCM_wdata(0x02);
  LCM_wcomm(0xEA05); LCM_wdata(0xA5);
  LCM_wcomm(0xEA06); LCM_wdata(0x02);
  LCM_wcomm(0xEA07); LCM_wdata(0xC9);
  LCM_wcomm(0xEA08); LCM_wdata(0x02);
  LCM_wcomm(0xEA09); LCM_wdata(0xEA);
  LCM_wcomm(0xEA0A); LCM_wdata(0x03);
  LCM_wcomm(0xEA0B); LCM_wdata(0x19);
  LCM_wcomm(0xEA0C); LCM_wdata(0x03);
  LCM_wcomm(0xEA0D); LCM_wdata(0x45);
  LCM_wcomm(0xEA0E); LCM_wdata(0x03);
  LCM_wcomm(0xEA0F); LCM_wdata(0x7A);
  LCM_wcomm(0xEB00); LCM_wdata(0x03);
  LCM_wcomm(0xEB01); LCM_wdata(0xA0);
  LCM_wcomm(0xEB02); LCM_wdata(0x03);
  LCM_wcomm(0xEB03); LCM_wdata(0xF4);
  
  LCM_wcomm(0x3A00); LCM_wdata(0x07);

  LCM_wcomm(0x3500); LCM_wdata(0x00);
  
  LCM_wcomm(0x1100); // Sleep out
  MDELAY(120);
  
  LCM_wcomm(0x2900); // Display On

#else
//BYD
	unsigned int i,j;
	
 	SET_RST_HIGH;
  	MDELAY(10);
  	SET_RST_LOW;
  	MDELAY(50);
  	SET_RST_HIGH;
  	MDELAY(120);

//for (i=0; i<100000; i++) {
//	LCM_wcomm(0x1100);
//}
	
	LCM_wcomm(0xFF00); LCM_wdata(0xAA);
	LCM_wcomm(0xFF01); LCM_wdata(0x55);
	LCM_wcomm(0xFF02); LCM_wdata(0x25);
	LCM_wcomm(0xFF03); LCM_wdata(0x01);
	LCM_wcomm(0xFF04); LCM_wdata(0x01);

	LCM_wcomm(0xF200); LCM_wdata(0x00);
	LCM_wcomm(0xF201); LCM_wdata(0x00);
	LCM_wcomm(0xF202); LCM_wdata(0x4A);
	LCM_wcomm(0xF203); LCM_wdata(0x0A);
	LCM_wcomm(0xF204); LCM_wdata(0xA8);
	LCM_wcomm(0xF205); LCM_wdata(0x00);
	LCM_wcomm(0xF206); LCM_wdata(0x00);
	LCM_wcomm(0xF207); LCM_wdata(0x00);
	LCM_wcomm(0xF208); LCM_wdata(0x00);
	LCM_wcomm(0xF209); LCM_wdata(0x00);
	LCM_wcomm(0xF20A); LCM_wdata(0x00);
	LCM_wcomm(0xF20B); LCM_wdata(0x00);
	LCM_wcomm(0xF20C); LCM_wdata(0x00);
	LCM_wcomm(0xF20D); LCM_wdata(0x00);
	LCM_wcomm(0xF20E); LCM_wdata(0x00);
	LCM_wcomm(0xF20F); LCM_wdata(0x00);
	LCM_wcomm(0xF210); LCM_wdata(0x00);
	LCM_wcomm(0xF211); LCM_wdata(0x0B);
	LCM_wcomm(0xF212); LCM_wdata(0x00);
	LCM_wcomm(0xF213); LCM_wdata(0x00);
	LCM_wcomm(0xF214); LCM_wdata(0x00);
	LCM_wcomm(0xF215); LCM_wdata(0x00);
	LCM_wcomm(0xF216); LCM_wdata(0x00);
	LCM_wcomm(0xF217); LCM_wdata(0x00);
	LCM_wcomm(0xF218); LCM_wdata(0x00);
	LCM_wcomm(0xF219); LCM_wdata(0x00);
	LCM_wcomm(0xF21A); LCM_wdata(0x00);
	LCM_wcomm(0xF21B); LCM_wdata(0x00);
	LCM_wcomm(0xF21C); LCM_wdata(0x40);
	LCM_wcomm(0xF21D); LCM_wdata(0x01);
	LCM_wcomm(0xF21E); LCM_wdata(0x51);
	LCM_wcomm(0xF21F); LCM_wdata(0x00);
	LCM_wcomm(0xF220); LCM_wdata(0x01);
	LCM_wcomm(0xF221); LCM_wdata(0x00);
	LCM_wcomm(0xF222); LCM_wdata(0x01);

	LCM_wcomm(0xF300); LCM_wdata(0x02);
	LCM_wcomm(0xF301); LCM_wdata(0x03);
	LCM_wcomm(0xF302); LCM_wdata(0x07);
	LCM_wcomm(0xF303); LCM_wdata(0x45);
	LCM_wcomm(0xF304); LCM_wdata(0x88);
	LCM_wcomm(0xF305); LCM_wdata(0xD1);
	LCM_wcomm(0xF306); LCM_wdata(0x0D);


	LCM_wcomm(0xF000); LCM_wdata(0x55);
	LCM_wcomm(0xF001); LCM_wdata(0xAA);
	LCM_wcomm(0xF002); LCM_wdata(0x52);
	LCM_wcomm(0xF003); LCM_wdata(0x08);
	LCM_wcomm(0xF004); LCM_wdata(0x00);//PAGE 0

	LCM_wcomm(0xB100); LCM_wdata(0xCC);
	LCM_wcomm(0xB101); LCM_wdata(0x00);
	LCM_wcomm(0xB102); LCM_wdata(0x00);

	LCM_wcomm(0xB800); LCM_wdata(0x01);
	LCM_wcomm(0xB801); LCM_wdata(0x02);
	LCM_wcomm(0xB802); LCM_wdata(0x02);
	LCM_wcomm(0xB803); LCM_wdata(0x02);

	LCM_wcomm(0xC900); LCM_wdata(0x63);
	LCM_wcomm(0xC901); LCM_wdata(0x06);
	LCM_wcomm(0xC902); LCM_wdata(0x0D);
	LCM_wcomm(0xC903); LCM_wdata(0x1A);
	LCM_wcomm(0xC904); LCM_wdata(0x17);
	LCM_wcomm(0xC905); LCM_wdata(0x00);

	LCM_wcomm(0xF000); LCM_wdata(0x55);
	LCM_wcomm(0xF001); LCM_wdata(0xAA);
	LCM_wcomm(0xF002); LCM_wdata(0x52);
	LCM_wcomm(0xF003); LCM_wdata(0x08);
	LCM_wcomm(0xF004); LCM_wdata(0x01);//PAGE1



	LCM_wcomm(0xB000); LCM_wdata(0x05);//0x55
	LCM_wcomm(0xB001); LCM_wdata(0x05);//0x55
	LCM_wcomm(0xB002); LCM_wdata(0x05);//0x55
//	LCM_wcomm(0xB003); LCM_wdata(0x08);//0x55
//	LCM_wcomm(0xB004); LCM_wdata(0x08);//0x55
	
	LCM_wcomm(0xB100); LCM_wdata(0x05);//0x55
	LCM_wcomm(0xB101); LCM_wdata(0x05);//0x55
	LCM_wcomm(0xB102); LCM_wdata(0x05);//0x55

	LCM_wcomm(0xB200); LCM_wdata(0x01);
	LCM_wcomm(0xB201); LCM_wdata(0x01);
	LCM_wcomm(0xB202); LCM_wdata(0x01);

	LCM_wcomm(0xB300); LCM_wdata(0x0E);
	LCM_wcomm(0xB301); LCM_wdata(0x0E);
	LCM_wcomm(0xB302); LCM_wdata(0x0E);

	LCM_wcomm(0xB400); LCM_wdata(0x08);
	LCM_wcomm(0xB401); LCM_wdata(0x08);
	LCM_wcomm(0xB402); LCM_wdata(0x08);

	LCM_wcomm(0xB600); LCM_wdata(0x44);
	LCM_wcomm(0xB601); LCM_wdata(0x44);
	LCM_wcomm(0xB602); LCM_wdata(0x44);

	LCM_wcomm(0xB700); LCM_wdata(0x34);
	LCM_wcomm(0xB701); LCM_wdata(0x34);
	LCM_wcomm(0xB702); LCM_wdata(0x34);

	LCM_wcomm(0xB800); LCM_wdata(0x10);
	LCM_wcomm(0xB801); LCM_wdata(0x10);
	LCM_wcomm(0xB802); LCM_wdata(0x10);

	LCM_wcomm(0xB900); LCM_wdata(0x26);
	LCM_wcomm(0xB901); LCM_wdata(0x26);
	LCM_wcomm(0xB902); LCM_wdata(0x26);

	LCM_wcomm(0xBA00); LCM_wdata(0x34);
	LCM_wcomm(0xBA01); LCM_wdata(0x34);
	LCM_wcomm(0xBA02); LCM_wdata(0x34);

	LCM_wcomm(0xBC00); LCM_wdata(0x00);
	LCM_wcomm(0xBC01); LCM_wdata(0xC8);
	LCM_wcomm(0xBC02); LCM_wdata(0x00);

	LCM_wcomm(0xBD00); LCM_wdata(0x00);
	LCM_wcomm(0xBD01); LCM_wdata(0xC8);
	LCM_wcomm(0xBD02); LCM_wdata(0x00);

	LCM_wcomm(0xBE00); LCM_wdata(0x92);//VCOM

	LCM_wcomm(0xC001); LCM_wdata(0x04);
	LCM_wcomm(0xC002); LCM_wdata(0x00);

	LCM_wcomm(0xCA00); LCM_wdata(0x00);

	LCM_wcomm(0xD000); LCM_wdata(0x0A);
	LCM_wcomm(0xD001); LCM_wdata(0x10);
	LCM_wcomm(0xD002); LCM_wdata(0x0D);
	LCM_wcomm(0xD003); LCM_wdata(0x0F);

	LCM_wcomm(0xD100); LCM_wdata(0x00);
	LCM_wcomm(0xD101); LCM_wdata(0x70);
	LCM_wcomm(0xD102); LCM_wdata(0x00);
	LCM_wcomm(0xD103); LCM_wdata(0xCE);
	LCM_wcomm(0xD104); LCM_wdata(0x00);
	LCM_wcomm(0xD105); LCM_wdata(0xF7);
	LCM_wcomm(0xD106); LCM_wdata(0x01);
	LCM_wcomm(0xD107); LCM_wdata(0x10);
	LCM_wcomm(0xD108); LCM_wdata(0x01);
	LCM_wcomm(0xD109); LCM_wdata(0x21);
	LCM_wcomm(0xD10A); LCM_wdata(0x01);
	LCM_wcomm(0xD10B); LCM_wdata(0x44);
	LCM_wcomm(0xD10C); LCM_wdata(0x01);
	LCM_wcomm(0xD10D); LCM_wdata(0x62);
	LCM_wcomm(0xD10E); LCM_wdata(0x01);
	LCM_wcomm(0xD10F); LCM_wdata(0x8D);

	LCM_wcomm(0xD200); LCM_wdata(0x01);
	LCM_wcomm(0xD201); LCM_wdata(0xAF);
	LCM_wcomm(0xD202); LCM_wdata(0x01);
	LCM_wcomm(0xD203); LCM_wdata(0xE4);
	LCM_wcomm(0xD204); LCM_wdata(0x02);
	LCM_wcomm(0xD205); LCM_wdata(0x0C);
	LCM_wcomm(0xD206); LCM_wdata(0x02);
	LCM_wcomm(0xD207); LCM_wdata(0x4D);
	LCM_wcomm(0xD208); LCM_wdata(0x02);
	LCM_wcomm(0xD209); LCM_wdata(0x82);
	LCM_wcomm(0xD20A); LCM_wdata(0x02);
	LCM_wcomm(0xD20B); LCM_wdata(0x84);
	LCM_wcomm(0xD20C); LCM_wdata(0x02);
	LCM_wcomm(0xD20D); LCM_wdata(0xB8);
	LCM_wcomm(0xD20E); LCM_wdata(0x02);
	LCM_wcomm(0xD20F); LCM_wdata(0xF0);

	LCM_wcomm(0xD300); LCM_wdata(0x03);
	LCM_wcomm(0xD301); LCM_wdata(0x14);
	LCM_wcomm(0xD302); LCM_wdata(0x03);
	LCM_wcomm(0xD303); LCM_wdata(0x42);
	LCM_wcomm(0xD304); LCM_wdata(0x03);
	LCM_wcomm(0xD305); LCM_wdata(0x5E);
	LCM_wcomm(0xD306); LCM_wdata(0x03);
	LCM_wcomm(0xD307); LCM_wdata(0x80);
	LCM_wcomm(0xD308); LCM_wdata(0x03);
	LCM_wcomm(0xD309); LCM_wdata(0x97);
	LCM_wcomm(0xD30A); LCM_wdata(0x03);
	LCM_wcomm(0xD30B); LCM_wdata(0xB0);
	LCM_wcomm(0xD30C); LCM_wdata(0x03);
	LCM_wcomm(0xD30D); LCM_wdata(0xC0);
	LCM_wcomm(0xD30E); LCM_wdata(0x03);
	LCM_wcomm(0xD30F); LCM_wdata(0xDF);

	LCM_wcomm(0xD400); LCM_wdata(0x03);
	LCM_wcomm(0xD401); LCM_wdata(0xFD);
	LCM_wcomm(0xD402); LCM_wdata(0x03);
	LCM_wcomm(0xD403); LCM_wdata(0xFF);

	LCM_wcomm(0xD500); LCM_wdata(0x00);
	LCM_wcomm(0xD501); LCM_wdata(0x70);
	LCM_wcomm(0xD502); LCM_wdata(0x00);
	LCM_wcomm(0xD503); LCM_wdata(0xCE);
	LCM_wcomm(0xD504); LCM_wdata(0x00);
	LCM_wcomm(0xD505); LCM_wdata(0xF7);
	LCM_wcomm(0xD506); LCM_wdata(0x01);
	LCM_wcomm(0xD507); LCM_wdata(0x10);
	LCM_wcomm(0xD508); LCM_wdata(0x01);
	LCM_wcomm(0xD509); LCM_wdata(0x21);
	LCM_wcomm(0xD50A); LCM_wdata(0x01);
	LCM_wcomm(0xD50B); LCM_wdata(0x44);
	LCM_wcomm(0xD50C); LCM_wdata(0x01);
	LCM_wcomm(0xD50D); LCM_wdata(0x62);
	LCM_wcomm(0xD50E); LCM_wdata(0x01);
	LCM_wcomm(0xD50F); LCM_wdata(0x8D);

	LCM_wcomm(0xD600); LCM_wdata(0x01);
	LCM_wcomm(0xD601); LCM_wdata(0xAF);
	LCM_wcomm(0xD602); LCM_wdata(0x01);
	LCM_wcomm(0xD603); LCM_wdata(0xE4);
	LCM_wcomm(0xD604); LCM_wdata(0x02);
	LCM_wcomm(0xD605); LCM_wdata(0x0C);
	LCM_wcomm(0xD606); LCM_wdata(0x02);
	LCM_wcomm(0xD607); LCM_wdata(0x4D);
	LCM_wcomm(0xD608); LCM_wdata(0x02);
	LCM_wcomm(0xD609); LCM_wdata(0x82);
	LCM_wcomm(0xD60A); LCM_wdata(0x02);
	LCM_wcomm(0xD60B); LCM_wdata(0x84);
	LCM_wcomm(0xD60C); LCM_wdata(0x02);
	LCM_wcomm(0xD60D); LCM_wdata(0xB8);
	LCM_wcomm(0xD60E); LCM_wdata(0x02);
	LCM_wcomm(0xD60F); LCM_wdata(0xF0);

	LCM_wcomm(0xD700); LCM_wdata(0x03);
	LCM_wcomm(0xD701); LCM_wdata(0x14);
	LCM_wcomm(0xD702); LCM_wdata(0x03);
	LCM_wcomm(0xD703); LCM_wdata(0x42);
	LCM_wcomm(0xD704); LCM_wdata(0x03);
	LCM_wcomm(0xD705); LCM_wdata(0x5E);
	LCM_wcomm(0xD706); LCM_wdata(0x03);
	LCM_wcomm(0xD707); LCM_wdata(0x80);
	LCM_wcomm(0xD708); LCM_wdata(0x03);
	LCM_wcomm(0xD709); LCM_wdata(0x97);
	LCM_wcomm(0xD70A); LCM_wdata(0x03);
	LCM_wcomm(0xD70B); LCM_wdata(0xB0);
	LCM_wcomm(0xD70C); LCM_wdata(0x03);
	LCM_wcomm(0xD70D); LCM_wdata(0xC0);
	LCM_wcomm(0xD70E); LCM_wdata(0x03);
	LCM_wcomm(0xD70F); LCM_wdata(0xDF);

	LCM_wcomm(0xD800); LCM_wdata(0x03);
	LCM_wcomm(0xD801); LCM_wdata(0xFD);
	LCM_wcomm(0xD802); LCM_wdata(0x03);
	LCM_wcomm(0xD803); LCM_wdata(0xFF);

	LCM_wcomm(0xD900); LCM_wdata(0x00);
	LCM_wcomm(0xD901); LCM_wdata(0x70);
	LCM_wcomm(0xD902); LCM_wdata(0x00);
	LCM_wcomm(0xD903); LCM_wdata(0xCE);
	LCM_wcomm(0xD904); LCM_wdata(0x00);
	LCM_wcomm(0xD905); LCM_wdata(0xF7);
	LCM_wcomm(0xD906); LCM_wdata(0x01);
	LCM_wcomm(0xD907); LCM_wdata(0x10);
	LCM_wcomm(0xD908); LCM_wdata(0x01);
	LCM_wcomm(0xD909); LCM_wdata(0x21);
	LCM_wcomm(0xD90A); LCM_wdata(0x01);
	LCM_wcomm(0xD90B); LCM_wdata(0x44);
	LCM_wcomm(0xD90C); LCM_wdata(0x01);
	LCM_wcomm(0xD90D); LCM_wdata(0x62);
	LCM_wcomm(0xD90E); LCM_wdata(0x01);
	LCM_wcomm(0xD90F); LCM_wdata(0x8D);

	LCM_wcomm(0xDD00); LCM_wdata(0x01);
	LCM_wcomm(0xDD01); LCM_wdata(0xAF);
	LCM_wcomm(0xDD02); LCM_wdata(0x01);
	LCM_wcomm(0xDD03); LCM_wdata(0xE4);
	LCM_wcomm(0xDD04); LCM_wdata(0x02);
	LCM_wcomm(0xDD05); LCM_wdata(0x0C);
	LCM_wcomm(0xDD06); LCM_wdata(0x02);
	LCM_wcomm(0xDD07); LCM_wdata(0x4D);
	LCM_wcomm(0xDD08); LCM_wdata(0x02);
	LCM_wcomm(0xDD09); LCM_wdata(0x82);
	LCM_wcomm(0xDD0A); LCM_wdata(0x02);
	LCM_wcomm(0xDD0B); LCM_wdata(0x84);
	LCM_wcomm(0xDD0C); LCM_wdata(0x02);
	LCM_wcomm(0xDD0D); LCM_wdata(0xB8);
	LCM_wcomm(0xDD0E); LCM_wdata(0x02);
	LCM_wcomm(0xDD0F); LCM_wdata(0xF0);

	LCM_wcomm(0xDE00); LCM_wdata(0x03);
	LCM_wcomm(0xDE01); LCM_wdata(0x14);
	LCM_wcomm(0xDE02); LCM_wdata(0x03);
	LCM_wcomm(0xDE03); LCM_wdata(0x42);
	LCM_wcomm(0xDE04); LCM_wdata(0x03);
	LCM_wcomm(0xDE05); LCM_wdata(0x5E);
	LCM_wcomm(0xDE06); LCM_wdata(0x03);
	LCM_wcomm(0xDE07); LCM_wdata(0x80);
	LCM_wcomm(0xDE08); LCM_wdata(0x03);
	LCM_wcomm(0xDE09); LCM_wdata(0x97);
	LCM_wcomm(0xDE0A); LCM_wdata(0x03);
	LCM_wcomm(0xDE0B); LCM_wdata(0xB0);
	LCM_wcomm(0xDE0C); LCM_wdata(0x03);
	LCM_wcomm(0xDE0D); LCM_wdata(0xC0);
	LCM_wcomm(0xDE0E); LCM_wdata(0x03);
	LCM_wcomm(0xDE0F); LCM_wdata(0xDF);

	LCM_wcomm(0xDF00); LCM_wdata(0x03);
	LCM_wcomm(0xDF01); LCM_wdata(0xFD);
	LCM_wcomm(0xDF02); LCM_wdata(0x03);
	LCM_wcomm(0xDF03); LCM_wdata(0xFF);

	LCM_wcomm(0xE000); LCM_wdata(0x00);
	LCM_wcomm(0xE001); LCM_wdata(0x70);
	LCM_wcomm(0xE002); LCM_wdata(0x00);
	LCM_wcomm(0xE003); LCM_wdata(0xCE);
	LCM_wcomm(0xE004); LCM_wdata(0x00);
	LCM_wcomm(0xE005); LCM_wdata(0xF7);
	LCM_wcomm(0xE006); LCM_wdata(0x01);
	LCM_wcomm(0xE007); LCM_wdata(0x10);
	LCM_wcomm(0xE008); LCM_wdata(0x01);
	LCM_wcomm(0xE009); LCM_wdata(0x21);
	LCM_wcomm(0xE00A); LCM_wdata(0x01);
	LCM_wcomm(0xE00B); LCM_wdata(0x44);
	LCM_wcomm(0xE00C); LCM_wdata(0x01);
	LCM_wcomm(0xE00D); LCM_wdata(0x62);
	LCM_wcomm(0xE00E); LCM_wdata(0x01);
	LCM_wcomm(0xE00F); LCM_wdata(0x8D);

	LCM_wcomm(0xE100); LCM_wdata(0x01);
	LCM_wcomm(0xE101); LCM_wdata(0xAF);
	LCM_wcomm(0xE102); LCM_wdata(0x01);
	LCM_wcomm(0xE103); LCM_wdata(0xE4);
	LCM_wcomm(0xE104); LCM_wdata(0x02);
	LCM_wcomm(0xE105); LCM_wdata(0x0C);
	LCM_wcomm(0xE106); LCM_wdata(0x02);
	LCM_wcomm(0xE107); LCM_wdata(0x4D);
	LCM_wcomm(0xE108); LCM_wdata(0x02);
	LCM_wcomm(0xE109); LCM_wdata(0x82);
	LCM_wcomm(0xE10A); LCM_wdata(0x02);
	LCM_wcomm(0xE10B); LCM_wdata(0x84);
	LCM_wcomm(0xE10C); LCM_wdata(0x02);
	LCM_wcomm(0xE10D); LCM_wdata(0xB8);
	LCM_wcomm(0xE10E); LCM_wdata(0x02);
	LCM_wcomm(0xE10F); LCM_wdata(0xF0);

	LCM_wcomm(0xE200); LCM_wdata(0x03);
	LCM_wcomm(0xE201); LCM_wdata(0x14);
	LCM_wcomm(0xE202); LCM_wdata(0x03);
	LCM_wcomm(0xE203); LCM_wdata(0x42);
	LCM_wcomm(0xE204); LCM_wdata(0x03);
	LCM_wcomm(0xE205); LCM_wdata(0x5E);
	LCM_wcomm(0xE206); LCM_wdata(0x03);
	LCM_wcomm(0xE207); LCM_wdata(0x80);
	LCM_wcomm(0xE208); LCM_wdata(0x03);
	LCM_wcomm(0xE209); LCM_wdata(0x97);
	LCM_wcomm(0xE20A); LCM_wdata(0x03);
	LCM_wcomm(0xE20B); LCM_wdata(0xB0);
	LCM_wcomm(0xE20C); LCM_wdata(0x03);
	LCM_wcomm(0xE20D); LCM_wdata(0xC0);
	LCM_wcomm(0xE20E); LCM_wdata(0x03);
	LCM_wcomm(0xE20F); LCM_wdata(0xDF);

	LCM_wcomm(0xE300); LCM_wdata(0x03);
	LCM_wcomm(0xE301); LCM_wdata(0xFD);
	LCM_wcomm(0xE302); LCM_wdata(0x03);
	LCM_wcomm(0xE303); LCM_wdata(0xFF);

	LCM_wcomm(0xE400); LCM_wdata(0x00);
	LCM_wcomm(0xE401); LCM_wdata(0x70);
	LCM_wcomm(0xE402); LCM_wdata(0x00);
	LCM_wcomm(0xE403); LCM_wdata(0xCE);
	LCM_wcomm(0xE404); LCM_wdata(0x00);
	LCM_wcomm(0xE405); LCM_wdata(0xF7);
	LCM_wcomm(0xE406); LCM_wdata(0x01);
	LCM_wcomm(0xE407); LCM_wdata(0x10);
	LCM_wcomm(0xE408); LCM_wdata(0x01);
	LCM_wcomm(0xE409); LCM_wdata(0x21);
	LCM_wcomm(0xE40A); LCM_wdata(0x01);
	LCM_wcomm(0xE40B); LCM_wdata(0x44);
	LCM_wcomm(0xE40C); LCM_wdata(0x01);
	LCM_wcomm(0xE40D); LCM_wdata(0x62);
	LCM_wcomm(0xE40E); LCM_wdata(0x01);
	LCM_wcomm(0xE40F); LCM_wdata(0x8D);

	LCM_wcomm(0xE500); LCM_wdata(0x01);
	LCM_wcomm(0xE501); LCM_wdata(0xAF);
	LCM_wcomm(0xE502); LCM_wdata(0x01);
	LCM_wcomm(0xE503); LCM_wdata(0xE4);
	LCM_wcomm(0xE504); LCM_wdata(0x02);
	LCM_wcomm(0xE505); LCM_wdata(0x0C);
	LCM_wcomm(0xE506); LCM_wdata(0x02);
	LCM_wcomm(0xE507); LCM_wdata(0x4D);
	LCM_wcomm(0xE508); LCM_wdata(0x02);
	LCM_wcomm(0xE509); LCM_wdata(0x82);
	LCM_wcomm(0xE50A); LCM_wdata(0x02);
	LCM_wcomm(0xE50B); LCM_wdata(0x84);
	LCM_wcomm(0xE50C); LCM_wdata(0x02);
	LCM_wcomm(0xE50D); LCM_wdata(0xB8);
	LCM_wcomm(0xE50E); LCM_wdata(0x02);
	LCM_wcomm(0xE50F); LCM_wdata(0xF0);

	LCM_wcomm(0xE600); LCM_wdata(0x03);
	LCM_wcomm(0xE601); LCM_wdata(0x14);
	LCM_wcomm(0xE602); LCM_wdata(0x03);
	LCM_wcomm(0xE603); LCM_wdata(0x42);
	LCM_wcomm(0xE604); LCM_wdata(0x03);
	LCM_wcomm(0xE605); LCM_wdata(0x5E);
	LCM_wcomm(0xE606); LCM_wdata(0x03);
	LCM_wcomm(0xE607); LCM_wdata(0x80);
	LCM_wcomm(0xE608); LCM_wdata(0x03);
	LCM_wcomm(0xE609); LCM_wdata(0x97);
	LCM_wcomm(0xE60A); LCM_wdata(0x03);
	LCM_wcomm(0xE60B); LCM_wdata(0xB0);
	LCM_wcomm(0xE60C); LCM_wdata(0x03);
	LCM_wcomm(0xE60D); LCM_wdata(0xC0);
	LCM_wcomm(0xE60E); LCM_wdata(0x03);
	LCM_wcomm(0xE60F); LCM_wdata(0xDF);

	LCM_wcomm(0xE700); LCM_wdata(0x03);
	LCM_wcomm(0xE701); LCM_wdata(0xFD);
	LCM_wcomm(0xE702); LCM_wdata(0x03);
	LCM_wcomm(0xE703); LCM_wdata(0xFF);

	LCM_wcomm(0xE800); LCM_wdata(0x00);
	LCM_wcomm(0xE801); LCM_wdata(0x70);
	LCM_wcomm(0xE802); LCM_wdata(0x00);
	LCM_wcomm(0xE803); LCM_wdata(0xCE);
	LCM_wcomm(0xE804); LCM_wdata(0x00);
	LCM_wcomm(0xE805); LCM_wdata(0xF7);
	LCM_wcomm(0xE806); LCM_wdata(0x01);
	LCM_wcomm(0xE807); LCM_wdata(0x10);
	LCM_wcomm(0xE808); LCM_wdata(0x01);
	LCM_wcomm(0xE809); LCM_wdata(0x21);
	LCM_wcomm(0xE80A); LCM_wdata(0x01);
	LCM_wcomm(0xE80B); LCM_wdata(0x44);
	LCM_wcomm(0xE80C); LCM_wdata(0x01);
	LCM_wcomm(0xE80D); LCM_wdata(0x62);
	LCM_wcomm(0xE80E); LCM_wdata(0x01);
	LCM_wcomm(0xE80F); LCM_wdata(0x8D);

	LCM_wcomm(0xE900); LCM_wdata(0x01);
	LCM_wcomm(0xE901); LCM_wdata(0xAF);
	LCM_wcomm(0xE902); LCM_wdata(0x01);
	LCM_wcomm(0xE903); LCM_wdata(0xE4);
	LCM_wcomm(0xE904); LCM_wdata(0x02);
	LCM_wcomm(0xE905); LCM_wdata(0x0C);
	LCM_wcomm(0xE906); LCM_wdata(0x02);
	LCM_wcomm(0xE907); LCM_wdata(0x4D);
	LCM_wcomm(0xE908); LCM_wdata(0x02);
	LCM_wcomm(0xE909); LCM_wdata(0x82);
	LCM_wcomm(0xE90A); LCM_wdata(0x02);
	LCM_wcomm(0xE90B); LCM_wdata(0x84);
	LCM_wcomm(0xE90C); LCM_wdata(0x02);
	LCM_wcomm(0xE90D); LCM_wdata(0xB8);
	LCM_wcomm(0xE90E); LCM_wdata(0x02);
	LCM_wcomm(0xE90F); LCM_wdata(0xF0);

	LCM_wcomm(0xEA00); LCM_wdata(0x03);
	LCM_wcomm(0xEA01); LCM_wdata(0x14);
	LCM_wcomm(0xEA02); LCM_wdata(0x03);
	LCM_wcomm(0xEA03); LCM_wdata(0x42);
	LCM_wcomm(0xEA04); LCM_wdata(0x03);
	LCM_wcomm(0xEA05); LCM_wdata(0x5E);
	LCM_wcomm(0xEA06); LCM_wdata(0x03);
	LCM_wcomm(0xEA07); LCM_wdata(0x80);
	LCM_wcomm(0xEA08); LCM_wdata(0x03);
	LCM_wcomm(0xEA09); LCM_wdata(0x97);
	LCM_wcomm(0xEA0A); LCM_wdata(0x03);
	LCM_wcomm(0xEA0B); LCM_wdata(0xB0);
	LCM_wcomm(0xEA0C); LCM_wdata(0x03);
	LCM_wcomm(0xEA0D); LCM_wdata(0xC0);
	LCM_wcomm(0xEA0E); LCM_wdata(0x03);
	LCM_wcomm(0xEA0F); LCM_wdata(0xDF);

	LCM_wcomm(0xEB00); LCM_wdata(0x03);
	LCM_wcomm(0xEB01); LCM_wdata(0xFD);
	LCM_wcomm(0xEB02); LCM_wdata(0x03);
	LCM_wcomm(0xEB03); LCM_wdata(0xFF);

	LCM_wcomm(0x4e00); LCM_wdata(0x01);

	LCM_wcomm(0x1100);
	MDELAY(120);

	
	LCM_wcomm(0x1300);
	LCM_wcomm(0x2900);
	LCM_wcomm(0x2c00);

//spi to rgb 
	LCM_wcomm(0xF000); LCM_wdata(0x55);
	LCM_wcomm(0xF001); LCM_wdata(0xAA);
	LCM_wcomm(0xF002); LCM_wdata(0x52);
	LCM_wcomm(0xF003); LCM_wdata(0x08);
	LCM_wcomm(0xF004); LCM_wdata(0x01);//PAGE 0
		
	LCM_wcomm(0x4e00); LCM_wdata(0x00);
//ADD 2012-3-15	
//	Write_Initial_IC(0xB0, 0x00, 0x007b);//RGBCTR
#endif
}

static void config_gpio(void)
{
  dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): :  \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__); 

  const unsigned int USED_GPIOS[] = 
  { 
	  LSCE_GPIO_PIN,
	  LSCK_GPIO_PIN,
	  LSDA_GPIO_PIN
    };

  unsigned int i;

  lcm_util.set_gpio_mode(GPIO137, 0x0);
  lcm_util.set_gpio_mode(RST_GPIO_PIN, GPIO_AST_RST_PIN_M_GPIO);
  lcm_util.set_gpio_mode(LSA0_GPIO_PIN, GPIO_DISP_LSA0_PIN_M_GPIO);
  lcm_util.set_gpio_mode(LSCE_GPIO_PIN, GPIO_DISP_LSCE_PIN_M_GPIO);
  lcm_util.set_gpio_mode(LSCK_GPIO_PIN, GPIO_DISP_LSCK_PIN_M_GPIO);
  lcm_util.set_gpio_mode(LSDA_GPIO_PIN, GPIO_DISP_LSDA_PIN_M_GPIO);



  for (i = 0; i < ARY_SIZE(USED_GPIOS); ++ i)
    {
	  lcm_util.set_gpio_dir(USED_GPIOS[i], 1);               // GPIO out
	  lcm_util.set_gpio_pull_enable(USED_GPIOS[i], 0);
    }
	  lcm_util.set_gpio_dir(RST_GPIO_PIN, 1);               // GPIO out
	  lcm_util.set_gpio_pull_enable(RST_GPIO_PIN, 0);
	  lcm_util.set_gpio_dir(GPIO137, 1);               // GPIO out
	  lcm_util.set_gpio_pull_enable(GPIO137, 0);

  // Swithc LSA0 pin to GPIO mode to avoid data contention,
  // since A0 is connected to LCM's SPI SDO pin
  //
  lcm_util.set_gpio_dir(LSA0_GPIO_PIN, 0);                   // GPIO in
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): :  \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__); 

  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
  dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): :  \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__); 

  memset(params, 0, sizeof(LCM_PARAMS));

  params->type   = LCM_TYPE_DPI;
  params->ctrl   = LCM_CTRL_GPIO;
  params->width  = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  // CLK
  //params->dpi.mipi_pll_clk_ref  = 2;
  params->dpi.mipi_pll_clk_div1 = 26;//8
  params->dpi.mipi_pll_clk_div2 = 5;//3
  params->dpi.dpi_clk_div       = 2;//4
  params->dpi.dpi_clk_duty      = 1; //2

#if 0 // By Fritz
  params->dpi.clk_pol           = LCM_POLARITY_FALLING;//FALLING
  //params->dpi.de_pol            = LCM_POLARITY_RISING;
  params->dpi.de_pol            = LCM_POLARITY_RISING;//LCM_POLARITY_RISING;
  params->dpi.vsync_pol         = LCM_POLARITY_FALLING;
  params->dpi.hsync_pol         = LCM_POLARITY_FALLING;
#else
  params->dpi.clk_pol           = LCM_POLARITY_FALLING;//FALLING
  //params->dpi.de_pol            = LCM_POLARITY_RISING;
  params->dpi.de_pol            = LCM_POLARITY_RISING;//LCM_POLARITY_RISING;
  params->dpi.vsync_pol         = LCM_POLARITY_FALLING;
  params->dpi.hsync_pol         = LCM_POLARITY_FALLING;
#endif

  // Hsync Vsync
#if 0
  params->dpi.hsync_pulse_width = 3;//10;//1
  params->dpi.hsync_back_porch  = 8;//30;//0
  params->dpi.hsync_front_porch = 8;//30;//0
  params->dpi.vsync_pulse_width = 3;//10;
  params->dpi.vsync_back_porch  = 8;//30;//0
  params->dpi.vsync_front_porch = 8;//30;//0
#else
  params->dpi.hsync_pulse_width = 2;//1
  params->dpi.hsync_back_porch  = 20;//0
  params->dpi.hsync_front_porch = 20;//0
  params->dpi.vsync_pulse_width = 2;
  params->dpi.vsync_back_porch  = 32;//0
  params->dpi.vsync_front_porch = 32;//0
#endif

  params->dpi.format            = LCM_DPI_FORMAT_RGB888;//666
  params->dpi.rgb_order         = LCM_COLOR_ORDER_RGB;
  params->dpi.is_serial_output  = 0;

  params->dpi.intermediat_buffer_num = 1;

  params->dpi.io_driving_current = LCM_DRIVING_CURRENT_4MA;
}


static void lcm_init(void)
{
	dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): :  \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__); 

	config_gpio();
/*
	SET_RST_HIGH;	
	MDELAY(10); // By Fritz
	//SET_DISP_TE_LOW;	  
	SET_RST_LOW;
	MDELAY(50); // By Fritz
	SET_RST_HIGH;
	SET_LSCE_HIGH;
	SET_LSCK_HIGH;
*/

//int i=0;
//for (i=0; i<100; i++)
	InitLCD_NT35510_DPI();
}


static void lcm_suspend(void)
{
	dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): :  \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__); 
	//MDELAY(20);
	LCM_wcomm(0x2800);	
	MDELAY(50);
	LCM_wcomm(0x1000);	//Sleep in
	MDELAY(50);
	//LCM_wcomm(0x4f00);	
	//LCM_wdata(0x01);	
}


static void lcm_resume(void)
{
	dbg_print("===> [NT35510_DPI] <%s %s> %s:%d %s(): :  \n", __DATE__, __TIME__, __FILE__, __LINE__, __func__); 
	//SET_RESET_PIN(0); // to exit deep sleep
	//MDELAY(20); // By Fritz
	//SET_RESET_PIN(1);
	LCM_wcomm(0x1100);	//Sleep out
	MDELAY(120);

	LCM_wcomm(0x1300);	//16BIT 65K，调颜色深度

	LCM_wcomm(0x2900);	//Display on
	LCM_wcomm(0x2c00);	//Display on
}

void lcm_set_backlight(unsigned int level)
{
   dbg_print("%s\n",__FUNCTION__);

}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------


LCM_DRIVER nt35510_dpi_lcm_drv =
  {
	.name = "nt35510_dpi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.set_backlight  = lcm_set_backlight
  };



const LCM_DRIVER* LCM_GetDriver()
{
    static const LCM_DRIVER LCM_DRV =
    {
        .set_util_funcs = lcm_set_util_funcs,
        .get_params     = lcm_get_params,
        .init           = lcm_init,
        .suspend        = lcm_suspend,
        .resume         = lcm_resume
    };

    return &LCM_DRV;
}

