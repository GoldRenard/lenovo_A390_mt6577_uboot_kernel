#include <linux/string.h>

#if defined(BUILD_UBOOT)

#if defined(MT6575)
#include <asm/arch/mt6575_gpio.h>
#endif
#if defined(MT6577)
#include <asm/arch/mt6577_gpio.h>
#endif

#else

#if defined(MT6575)
#include <mach/mt6575_gpio.h>
#endif
#if defined(MT6577)
#include <mach/mt6577_gpio.h>
#endif

#endif
extern unsigned char agold_driver_lcm_has_TE();
extern unsigned char agold_driver_lcm_is_rotate_180();
#include "lcm_drv.h"

#if defined(BUILD_UBOOT)
	#define LCM_DEBUG  printf
	#define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
	#define LCM_DEBUG  printk
	#define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif

//  Local Constants
#if defined(BUILD_UBOOT)
	#define LCM_DEBUG  printf
	#define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
	#define LCM_DEBUG  printk
	#define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif

#define UDELAY(n)	(lcm_util.udelay(n))
#define MDELAY(n)	(lcm_util.mdelay(n))

#define GPIO_DPI_MODE_DPI	(1)
#define GPIO_DPI_CLK_PIN	(41)
#define GPIO_DPI_DE_PIN		(30)
#define GPIO_DPI_VSYNC_PIN	(21)
#define GPIO_DPI_HSYNC_PIN	(19)

#define GPIO_LCM_RST_MODE	(1)
#define GPIO_LCM_RST_PIN	(18)

#define GPIO_SPI_MODE_GPIO	(0)
#define GPIO_SPI_CS_PIN		(47)
#define GPIO_SPI_CLK_PIN	(51)
#define GPIO_SPI_DA_PIN		(52)

#define GPIO_POWER_EN_PIN	(183)

//#define SPI_DATA_NUM		(16)	//change num of bit
#define SPI_CLK_POLARITY_RISING	1	//1 for rising

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)	(lcm_util.set_gpio_out((n), (v)))

#define SET_SPI_CLK_LOW		SET_GPIO_OUT(GPIO_SPI_CLK_PIN, 0)
#define SET_SPI_CLK_HIGH	SET_GPIO_OUT(GPIO_SPI_CLK_PIN, 1)
#define SET_SPI_CS_LOW		SET_GPIO_OUT(GPIO_SPI_CS_PIN, 0)
#define SET_SPI_CS_HIGH		SET_GPIO_OUT(GPIO_SPI_CS_PIN, 1)
#define SET_SPI_DA_LOW		SET_GPIO_OUT(GPIO_SPI_DA_PIN, 0)
#define SET_SPI_DA_HIGH		SET_GPIO_OUT(GPIO_SPI_DA_PIN, 1)

#if 0//(SPI_CLK_POLARITY_RISING == 1)
#define SET_SPI_CLK_RST SET_SPI_CLK_LOW
#define SET_SPI_CLK_INV SET_SPI_CLK_HIGH
#else
#define SET_SPI_CLK_RST SET_SPI_CLK_HIGH
#define SET_SPI_CLK_INV SET_SPI_CLK_LOW
#endif
#define SET_SPI_CS_RST SET_SPI_CS_HIGH
#define SET_SPI_CS_INV SET_SPI_CS_LOW

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (800)

#define SPI_DATA_NUM (9)

#define LCM_ID      0x00 //(0x8194)

//  Local Variables
static LCM_UTIL_FUNCS lcm_util = {0};

#define UDELAY(n)	(lcm_util.udelay(n))
#define MDELAY(n)	(lcm_util.mdelay(n))

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)	(lcm_util.set_gpio_out((n), (v)))

#define SET_SPI_CLK_LOW		SET_GPIO_OUT(GPIO_SPI_CLK_PIN, 0)
#define SET_SPI_CLK_HIGH	SET_GPIO_OUT(GPIO_SPI_CLK_PIN, 1)
#define SET_SPI_CS_LOW		SET_GPIO_OUT(GPIO_SPI_CS_PIN, 0)
#define SET_SPI_CS_HIGH		SET_GPIO_OUT(GPIO_SPI_CS_PIN, 1)
#define SET_SPI_DA_LOW		SET_GPIO_OUT(GPIO_SPI_DA_PIN, 0)
#define SET_SPI_DA_HIGH		SET_GPIO_OUT(GPIO_SPI_DA_PIN, 1)

static __inline void  WriteCtrlH8Bit(unsigned int i)
{
    unsigned int n;
	unsigned int  m;

    SET_SPI_CLK_HIGH;
	UDELAY(1);
    SET_SPI_CS_LOW;
	UDELAY(1);
    m=0x80;
    for(n=0;n<8;n++)
    {
            if(0x20&m)
                SET_SPI_DA_HIGH;
            else
                SET_SPI_DA_LOW;

	    UDELAY(1);

             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
            SET_SPI_CLK_HIGH;
		UDELAY(1);

            m=m>>1;
    }

    m=0x80;

    for(n=0;n<8;n++)
    {
            if(i&m)
                SET_SPI_DA_HIGH;
            else
                SET_SPI_DA_LOW;
		UDELAY(1);

             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
            SET_SPI_CLK_HIGH;
		UDELAY(1);

            m=m>>1;
    }
    SET_SPI_CS_HIGH;
}

static __inline void  WriteCtrlL8Bit(unsigned int i)
{
    unsigned int  n;
    unsigned int  m;

    SET_SPI_CLK_HIGH;
	UDELAY(1);
    SET_SPI_CS_LOW;
	UDELAY(1);

    m=0x80;
    for(n=0;n<8;n++)
    {
            if(0x00&m)
                SET_SPI_DA_HIGH;
            else
                SET_SPI_DA_LOW;
		UDELAY(1);

             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
            SET_SPI_CLK_HIGH;
		UDELAY(1);
            m=m>>1;
    }

    m=0x80;
    for(n=0;n<8;n++)
    {
            if(i&m)
                SET_SPI_DA_HIGH;
            else
                SET_SPI_DA_LOW;
		UDELAY(1);

             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
            SET_SPI_CLK_HIGH;
		UDELAY(1);
            m=m>>1;
    }
    SET_SPI_CS_HIGH;
}

static __inline void WriteData8Bit(unsigned int  i)
{
    unsigned int  n;
    unsigned int  m;

    SET_SPI_CLK_HIGH;
	UDELAY(1);
    SET_SPI_CS_LOW;
	UDELAY(1);

    m=0x80;
    for(n=0;n<8;n++)
    {
            if(0x40&m)
                SET_SPI_DA_HIGH;
            else
                SET_SPI_DA_LOW;
		UDELAY(1);

             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
            SET_SPI_CLK_HIGH;
		UDELAY(1);

            m=m>>1;
    }

    m=0x80;

    for(n=0;n<8;n++)
    {
            if(i&m)
                SET_SPI_DA_HIGH;
            else
                SET_SPI_DA_LOW;
		UDELAY(1);


             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
             SET_SPI_CLK_LOW;
		UDELAY(1);
            SET_SPI_CLK_HIGH;
		UDELAY(1);

            m=m>>1;
    }

        SET_SPI_CS_HIGH;
}

static __inline void send_ctrl_cmd(unsigned int cmd)
{
  	WriteCtrlH8Bit((cmd >>8) & 0xff);
	WriteCtrlL8Bit(cmd & 0xff);
}

static __inline void send_data_cmd(unsigned int data)
{
    WriteData8Bit(data);
}

static void init_lcm_registers(void)
{
	LCM_DEBUG("\n######################################\n"
		"##### %s, %s %s\n"
		"########################################\n",
		__FILE__, __DATE__, __TIME__);

	//**********LV2 Page 1 enable***********//
	send_ctrl_cmd(0xF000);	send_data_cmd(0x55);
	send_ctrl_cmd(0xF001);	send_data_cmd(0xAA);
	send_ctrl_cmd(0xF002);	send_data_cmd(0x52);
	send_ctrl_cmd(0xF003);	send_data_cmd(0x08);
	send_ctrl_cmd(0xF004);	send_data_cmd(0x01);

	//*************AVDD Set AVDD 5.2V*************//
	send_ctrl_cmd(0xB000);	send_data_cmd(0x0D);
	send_ctrl_cmd(0xB001);	send_data_cmd(0x0D);
	send_ctrl_cmd(0xB002);	send_data_cmd(0x0D);

	//************AVDD ratio****************//
	send_ctrl_cmd(0xB600);	send_data_cmd(0x34);
	send_ctrl_cmd(0xB601);	send_data_cmd(0x34);
	send_ctrl_cmd(0xB602);	send_data_cmd(0x34);

	//************AVEE  -5.2V****************//
	send_ctrl_cmd(0xB100);	send_data_cmd(0x0D);
	send_ctrl_cmd(0xB101);	send_data_cmd(0x0D);
	send_ctrl_cmd(0xB102);	send_data_cmd(0x0D);

	//***********AVEE ratio*************//
	send_ctrl_cmd(0xB700);	send_data_cmd(0x34);
	send_ctrl_cmd(0xB701);	send_data_cmd(0x34);
	send_ctrl_cmd(0xB702);	send_data_cmd(0x34);

	//***********VCL  -2.5V*************//
	send_ctrl_cmd(0xB200);	send_data_cmd(0x00);
	send_ctrl_cmd(0xB201);	send_data_cmd(0x00);
	send_ctrl_cmd(0xB202);	send_data_cmd(0x00);

	//**************VCL ratio*****************//
	send_ctrl_cmd(0xB800);	send_data_cmd(0x24);
	send_ctrl_cmd(0xB801);	send_data_cmd(0x24);
	send_ctrl_cmd(0xB802);	send_data_cmd(0x24);


	//*************VGH 15V  (Free pump)*********//
	send_ctrl_cmd(0xBF00);	send_data_cmd(0x01);
	send_ctrl_cmd(0xB300);	send_data_cmd(0x0F);
	send_ctrl_cmd(0xB301);	send_data_cmd(0x0F);
	send_ctrl_cmd(0xB302);	send_data_cmd(0x0F);

	//*************VGH ratio*****************//
	send_ctrl_cmd(0xB900);	send_data_cmd(0x34);
	send_ctrl_cmd(0xB901);	send_data_cmd(0x34);
	send_ctrl_cmd(0xB902);	send_data_cmd(0x34);

	//***************VGL_REG -10V**************//
	send_ctrl_cmd(0xB500);	send_data_cmd(0x08);
	send_ctrl_cmd(0xB501);	send_data_cmd(0x08);
	send_ctrl_cmd(0xB502);	send_data_cmd(0x08);

	send_ctrl_cmd(0xC200);	send_data_cmd(0x03);

	//*****************VGLX ratio***************//
	send_ctrl_cmd(0xBA00);	send_data_cmd(0x24);
	send_ctrl_cmd(0xBA01);	send_data_cmd(0x24);
	send_ctrl_cmd(0xBA02);	send_data_cmd(0x24);

	//*************VGMP/VGSP 4.5V/0V*************//
	send_ctrl_cmd(0xBC00);	send_data_cmd(0x00);
	send_ctrl_cmd(0xBC01);	send_data_cmd(0x78);
	send_ctrl_cmd(0xBC02);	send_data_cmd(0x00);

	//************VGMN/VGSN -4.5V/0V****************//
	send_ctrl_cmd(0xBD00);	send_data_cmd(0x00);
	send_ctrl_cmd(0xBD01);	send_data_cmd(0x78);
	send_ctrl_cmd(0xBD02);	send_data_cmd(0x00);

	//************VCOM  -1.25V****************//
	send_ctrl_cmd(0xBE00);	send_data_cmd(0x00);
	send_ctrl_cmd(0xBE01);	send_data_cmd(0x67);

	//************Gamma Setting******************//
	send_ctrl_cmd(0xD100);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD101);	send_data_cmd(0x32);
	send_ctrl_cmd(0xD102);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD103);	send_data_cmd(0x33);
	send_ctrl_cmd(0xD104);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD105);	send_data_cmd(0x41);
	send_ctrl_cmd(0xD106);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD107);	send_data_cmd(0x5A);
	send_ctrl_cmd(0xD108);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD109);	send_data_cmd(0x76);
	send_ctrl_cmd(0xD10A);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD10B);	send_data_cmd(0xA7);
	send_ctrl_cmd(0xD10C);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD10D);	send_data_cmd(0xCF);
	send_ctrl_cmd(0xD10E);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD10F);	send_data_cmd(0x09);
	send_ctrl_cmd(0xD110);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD111);	send_data_cmd(0x36);
	send_ctrl_cmd(0xD112);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD113);	send_data_cmd(0x73);
	send_ctrl_cmd(0xD114);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD115);	send_data_cmd(0x9F);
	send_ctrl_cmd(0xD116);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD117);	send_data_cmd(0xDF);
	send_ctrl_cmd(0xD118);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD119);	send_data_cmd(0x10);
	send_ctrl_cmd(0xD11A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD11B);	send_data_cmd(0x11);
	send_ctrl_cmd(0xD11C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD11D);	send_data_cmd(0x3D);
	send_ctrl_cmd(0xD11E);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD11F);	send_data_cmd(0x69);
	send_ctrl_cmd(0xD120);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD121);	send_data_cmd(0x81);
	send_ctrl_cmd(0xD122);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD123);	send_data_cmd(0x9D);
	send_ctrl_cmd(0xD124);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD125);	send_data_cmd(0xAD);
	send_ctrl_cmd(0xD126);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD127);	send_data_cmd(0xC3);
	send_ctrl_cmd(0xD128);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD129);	send_data_cmd(0xD0);
	send_ctrl_cmd(0xD12A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD12B);	send_data_cmd(0xE2);
	send_ctrl_cmd(0xD12C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD12D);	send_data_cmd(0xEE);
	send_ctrl_cmd(0xD12E);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD12F);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD130);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD131);	send_data_cmd(0x26);
	send_ctrl_cmd(0xD132);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD133);	send_data_cmd(0x8E);

	send_ctrl_cmd(0xD200);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD201);	send_data_cmd(0x32);
	send_ctrl_cmd(0xD202);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD203);	send_data_cmd(0x33);
	send_ctrl_cmd(0xD204);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD205);	send_data_cmd(0x41);
	send_ctrl_cmd(0xD206);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD207);	send_data_cmd(0x5A);
	send_ctrl_cmd(0xD208);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD209);	send_data_cmd(0x76);
	send_ctrl_cmd(0xD20A);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD20B);	send_data_cmd(0xA7);
	send_ctrl_cmd(0xD20C);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD20D);	send_data_cmd(0xCF);
	send_ctrl_cmd(0xD20E);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD20F);	send_data_cmd(0x09);
	send_ctrl_cmd(0xD210);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD211);	send_data_cmd(0x36);
	send_ctrl_cmd(0xD212);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD213);	send_data_cmd(0x73);
	send_ctrl_cmd(0xD214);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD215);	send_data_cmd(0x9F);
	send_ctrl_cmd(0xD216);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD217);	send_data_cmd(0xDF);
	send_ctrl_cmd(0xD218);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD219);	send_data_cmd(0x10);
	send_ctrl_cmd(0xD21A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD21B);	send_data_cmd(0x11);
	send_ctrl_cmd(0xD21C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD21D);	send_data_cmd(0x3D);
	send_ctrl_cmd(0xD21E);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD21F);	send_data_cmd(0x69);
	send_ctrl_cmd(0xD220);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD221);	send_data_cmd(0x81);
	send_ctrl_cmd(0xD222);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD223);	send_data_cmd(0x9D);
	send_ctrl_cmd(0xD224);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD225);	send_data_cmd(0xAD);
	send_ctrl_cmd(0xD226);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD227);	send_data_cmd(0xC3);
	send_ctrl_cmd(0xD228);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD229);	send_data_cmd(0xD0);
	send_ctrl_cmd(0xD22A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD22B);	send_data_cmd(0xE2);
	send_ctrl_cmd(0xD22C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD22D);	send_data_cmd(0xEE);
	send_ctrl_cmd(0xD22E);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD22F);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD230);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD231);	send_data_cmd(0x26);
	send_ctrl_cmd(0xD232);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD233);	send_data_cmd(0x8E);

	send_ctrl_cmd(0xD300);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD301);	send_data_cmd(0x32);
	send_ctrl_cmd(0xD302);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD303);	send_data_cmd(0x33);
	send_ctrl_cmd(0xD304);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD305);	send_data_cmd(0x41);
	send_ctrl_cmd(0xD306);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD307);	send_data_cmd(0x5A);
	send_ctrl_cmd(0xD308);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD309);	send_data_cmd(0x76);
	send_ctrl_cmd(0xD30A);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD30B);	send_data_cmd(0xA7);
	send_ctrl_cmd(0xD30C);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD30D);	send_data_cmd(0xCF);
	send_ctrl_cmd(0xD30E);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD30F);	send_data_cmd(0x09);
	send_ctrl_cmd(0xD310);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD311);	send_data_cmd(0x36);
	send_ctrl_cmd(0xD312);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD313);	send_data_cmd(0x73);
	send_ctrl_cmd(0xD314);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD315);	send_data_cmd(0x9F);
	send_ctrl_cmd(0xD316);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD317);	send_data_cmd(0xDF);
	send_ctrl_cmd(0xD318);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD319);	send_data_cmd(0x10);
	send_ctrl_cmd(0xD31A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD31B);	send_data_cmd(0x11);
	send_ctrl_cmd(0xD31C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD31D);	send_data_cmd(0x3D);
	send_ctrl_cmd(0xD31E);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD31F);	send_data_cmd(0x69);
	send_ctrl_cmd(0xD320);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD321);	send_data_cmd(0x81);
	send_ctrl_cmd(0xD322);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD323);	send_data_cmd(0x9D);
	send_ctrl_cmd(0xD324);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD325);	send_data_cmd(0xAD);
	send_ctrl_cmd(0xD326);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD327);	send_data_cmd(0xC3);
	send_ctrl_cmd(0xD328);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD329);	send_data_cmd(0xD0);
	send_ctrl_cmd(0xD32A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD32B);	send_data_cmd(0xE2);
	send_ctrl_cmd(0xD32C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD32D);	send_data_cmd(0xEE);
	send_ctrl_cmd(0xD32E);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD32F);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD330);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD331);	send_data_cmd(0x26);
	send_ctrl_cmd(0xD332);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD333);	send_data_cmd(0x8E);

	send_ctrl_cmd(0xD400);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD401);	send_data_cmd(0x32);
	send_ctrl_cmd(0xD402);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD403);	send_data_cmd(0x33);
	send_ctrl_cmd(0xD404);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD405);	send_data_cmd(0x41);
	send_ctrl_cmd(0xD406);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD407);	send_data_cmd(0x5A);
	send_ctrl_cmd(0xD408);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD409);	send_data_cmd(0x76);
	send_ctrl_cmd(0xD40A);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD40B);	send_data_cmd(0xA7);
	send_ctrl_cmd(0xD40C);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD40D);	send_data_cmd(0xCF);
	send_ctrl_cmd(0xD40E);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD40F);	send_data_cmd(0x09);
	send_ctrl_cmd(0xD410);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD411);	send_data_cmd(0x36);
	send_ctrl_cmd(0xD412);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD413);	send_data_cmd(0x73);
	send_ctrl_cmd(0xD414);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD415);	send_data_cmd(0x9F);
	send_ctrl_cmd(0xD416);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD417);	send_data_cmd(0xDF);
	send_ctrl_cmd(0xD418);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD419);	send_data_cmd(0x10);
	send_ctrl_cmd(0xD41A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD41B);	send_data_cmd(0x11);
	send_ctrl_cmd(0xD41C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD41D);	send_data_cmd(0x3D);
	send_ctrl_cmd(0xD41E);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD41F);	send_data_cmd(0x69);
	send_ctrl_cmd(0xD420);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD421);	send_data_cmd(0x81);
	send_ctrl_cmd(0xD422);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD423);	send_data_cmd(0x9D);
	send_ctrl_cmd(0xD424);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD425);	send_data_cmd(0xAD);
	send_ctrl_cmd(0xD426);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD427);	send_data_cmd(0xC3);
	send_ctrl_cmd(0xD428);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD429);	send_data_cmd(0xD0);
	send_ctrl_cmd(0xD42A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD42B);	send_data_cmd(0xE2);
	send_ctrl_cmd(0xD42C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD42D);	send_data_cmd(0xEE);
	send_ctrl_cmd(0xD42E);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD42F);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD430);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD431);	send_data_cmd(0x26);
	send_ctrl_cmd(0xD432);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD433);	send_data_cmd(0x8E);

	send_ctrl_cmd(0xD500);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD501);	send_data_cmd(0x32);
	send_ctrl_cmd(0xD502);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD503);	send_data_cmd(0x33);
	send_ctrl_cmd(0xD504);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD505);	send_data_cmd(0x41);
	send_ctrl_cmd(0xD506);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD507);	send_data_cmd(0x5A);
	send_ctrl_cmd(0xD508);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD509);	send_data_cmd(0x76);
	send_ctrl_cmd(0xD50A);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD50B);	send_data_cmd(0xA7);
	send_ctrl_cmd(0xD50C);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD50D);	send_data_cmd(0xCF);
	send_ctrl_cmd(0xD50E);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD50F);	send_data_cmd(0x09);
	send_ctrl_cmd(0xD510);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD511);	send_data_cmd(0x36);
	send_ctrl_cmd(0xD512);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD513);	send_data_cmd(0x73);
	send_ctrl_cmd(0xD514);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD515);	send_data_cmd(0x9F);
	send_ctrl_cmd(0xD516);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD517);	send_data_cmd(0xDF);
	send_ctrl_cmd(0xD518);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD519);	send_data_cmd(0x10);
	send_ctrl_cmd(0xD51A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD51B);	send_data_cmd(0x11);
	send_ctrl_cmd(0xD51C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD51D);	send_data_cmd(0x3D);
	send_ctrl_cmd(0xD51E);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD51F);	send_data_cmd(0x69);
	send_ctrl_cmd(0xD520);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD521);	send_data_cmd(0x81);
	send_ctrl_cmd(0xD522);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD523);	send_data_cmd(0x9D);
	send_ctrl_cmd(0xD524);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD525);	send_data_cmd(0xAD);
	send_ctrl_cmd(0xD526);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD527);	send_data_cmd(0xC3);
	send_ctrl_cmd(0xD528);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD529);	send_data_cmd(0xD0);
	send_ctrl_cmd(0xD52A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD52B);	send_data_cmd(0xE2);
	send_ctrl_cmd(0xD52C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD52D);	send_data_cmd(0xEE);
	send_ctrl_cmd(0xD52E);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD52F);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD530);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD531);	send_data_cmd(0x26);
	send_ctrl_cmd(0xD532);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD533);	send_data_cmd(0x8E);

	send_ctrl_cmd(0xD600);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD601);	send_data_cmd(0x32);
	send_ctrl_cmd(0xD602);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD603);	send_data_cmd(0x33);
	send_ctrl_cmd(0xD604);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD605);	send_data_cmd(0x41);
	send_ctrl_cmd(0xD606);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD607);	send_data_cmd(0x5A);
	send_ctrl_cmd(0xD608);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD609);	send_data_cmd(0x76);
	send_ctrl_cmd(0xD60A);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD60B);	send_data_cmd(0xA7);
	send_ctrl_cmd(0xD60C);	send_data_cmd(0x00);
	send_ctrl_cmd(0xD60D);	send_data_cmd(0xCF);
	send_ctrl_cmd(0xD60E);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD60F);	send_data_cmd(0x09);
	send_ctrl_cmd(0xD610);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD611);	send_data_cmd(0x36);
	send_ctrl_cmd(0xD612);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD613);	send_data_cmd(0x73);
	send_ctrl_cmd(0xD614);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD615);	send_data_cmd(0x9F);
	send_ctrl_cmd(0xD616);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD617);	send_data_cmd(0xDF);
	send_ctrl_cmd(0xD618);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD619);	send_data_cmd(0x10);
	send_ctrl_cmd(0xD61A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD61B);	send_data_cmd(0x11);
	send_ctrl_cmd(0xD61C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD61D);	send_data_cmd(0x3D);
	send_ctrl_cmd(0xD61E);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD61F);	send_data_cmd(0x69);
	send_ctrl_cmd(0xD620);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD621);	send_data_cmd(0x81);
	send_ctrl_cmd(0xD622);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD623);	send_data_cmd(0x9D);
	send_ctrl_cmd(0xD624);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD625);	send_data_cmd(0xAD);
	send_ctrl_cmd(0xD626);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD627);	send_data_cmd(0xC3);
	send_ctrl_cmd(0xD628);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD629);	send_data_cmd(0xD0);
	send_ctrl_cmd(0xD62A);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD62B);	send_data_cmd(0xE2);
	send_ctrl_cmd(0xD62C);	send_data_cmd(0x02);
	send_ctrl_cmd(0xD62D);	send_data_cmd(0xEE);
	send_ctrl_cmd(0xD62E);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD62F);	send_data_cmd(0x01);
	send_ctrl_cmd(0xD630);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD631);	send_data_cmd(0x26);
	send_ctrl_cmd(0xD632);	send_data_cmd(0x03);
	send_ctrl_cmd(0xD633);	send_data_cmd(0x8E);


	//**************LV2 Page 0 enable*************//
	send_ctrl_cmd(0xF000);	send_data_cmd(0x55);
	send_ctrl_cmd(0xF001);	send_data_cmd(0xAA);
	send_ctrl_cmd(0xF002);	send_data_cmd(0x52);
	send_ctrl_cmd(0xF003);	send_data_cmd(0x08);
	send_ctrl_cmd(0xF004);	send_data_cmd(0x00);

	//*************480x800*********************//
	send_ctrl_cmd(0xB500);	send_data_cmd(0x50);

	//***************Display control**************//
	send_ctrl_cmd(0xB100);	send_data_cmd(0xCC);
	send_ctrl_cmd(0xB101);	send_data_cmd(0x00);

	//***************Source hold time*************//
	send_ctrl_cmd(0xB600);	send_data_cmd(0x05);

	//**************Gate EQ control***************//
	send_ctrl_cmd(0xB700);	send_data_cmd(0x70);
	send_ctrl_cmd(0xB701);	send_data_cmd(0x70);

	//*************Source EQ control (Mode 2)************//
	send_ctrl_cmd(0xB800);	send_data_cmd(0x01);
	send_ctrl_cmd(0xB801);	send_data_cmd(0x03);
	send_ctrl_cmd(0xB802);	send_data_cmd(0x03);
	send_ctrl_cmd(0xB803);	send_data_cmd(0x03);

	//************Inversion mode  (2-dot)****************//
	send_ctrl_cmd(0xBC00);	send_data_cmd(0x02);
	send_ctrl_cmd(0xBC01);	send_data_cmd(0x00);
	send_ctrl_cmd(0xBC02);	send_data_cmd(0x00);

	//***************Frame rate***************//
	send_ctrl_cmd(0xBD00);	send_data_cmd(0x01);
	send_ctrl_cmd(0xBD01);	send_data_cmd(0x84);
	send_ctrl_cmd(0xBD02);	send_data_cmd(0x1C);
	send_ctrl_cmd(0xBD03);	send_data_cmd(0x1C);
	send_ctrl_cmd(0xBD04);	send_data_cmd(0x00);

	//***********Timing control 4H w/ 4-delay *******//
	send_ctrl_cmd(0xC900);	send_data_cmd(0xD0);
	send_ctrl_cmd(0xC901);	send_data_cmd(0x02);
	send_ctrl_cmd(0xC902);	send_data_cmd(0x50);
	send_ctrl_cmd(0xC903);	send_data_cmd(0x50);
	send_ctrl_cmd(0xC904);	send_data_cmd(0x50);

	send_ctrl_cmd(0x3600);	send_data_cmd(0x03);
	send_ctrl_cmd(0x3500);	send_data_cmd(0x00);
	send_ctrl_cmd(0x3A00);  send_data_cmd(0x77);

	send_ctrl_cmd(0x1100);
	MDELAY(120);
	send_ctrl_cmd(0x2900);
	MDELAY(50);
}

static void config_gpio(void)
{
	//power on
	lcm_util.set_gpio_mode(GPIO_POWER_EN_PIN, 0);
	lcm_util.set_gpio_dir(GPIO_POWER_EN_PIN,1);
	lcm_util.set_gpio_out(GPIO_POWER_EN_PIN,1);
	MDELAY(100);

	//set spi
	const unsigned int SPI_GPIOS[] =
	{
		GPIO_SPI_CLK_PIN,
		GPIO_SPI_CS_PIN,
		GPIO_SPI_DA_PIN
	};

	unsigned int i;

	lcm_util.set_gpio_mode(GPIO_SPI_CS_PIN, 1);

	for (i = 0; i < ARY_SIZE(SPI_GPIOS); ++ i)
	{
		lcm_util.set_gpio_mode(SPI_GPIOS[i], GPIO_SPI_MODE_GPIO);
		lcm_util.set_gpio_dir(SPI_GPIOS[i], 1);			//GPIO out
		lcm_util.set_gpio_pull_enable(SPI_GPIOS[i], 0);
	}

	//set dpi
	lcm_util.set_gpio_mode(GPIO_DPI_CLK_PIN, GPIO_DPI_MODE_DPI);	//DPI CLOCK pin
	lcm_util.set_gpio_mode(GPIO_DPI_DE_PIN, GPIO_DPI_MODE_DPI);	//DPI DE pin
	lcm_util.set_gpio_mode(GPIO_DPI_VSYNC_PIN, GPIO_DPI_MODE_DPI);	//DPI VSYNC pin
	lcm_util.set_gpio_mode(GPIO_DPI_HSYNC_PIN, GPIO_DPI_MODE_DPI);	//DPI HSYNC pin
	lcm_util.set_gpio_mode(GPIO_LCM_RST_PIN, GPIO_LCM_RST_MODE);	//lcm reset pin
}

//  LCM Driver Implementations
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DPI;
    params->ctrl   = LCM_CTRL_GPIO;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->dpi.mipi_pll_clk_ref  = 0;
    params->dpi.mipi_pll_clk_div1 = 33;
    params->dpi.mipi_pll_clk_div2 = 9;
    params->dpi.dpi_clk_div       = 2;
    params->dpi.dpi_clk_duty      = 1;			//24.576

    params->dpi.clk_pol           = LCM_POLARITY_FALLING;	           //LCM_POLARITY_FALLING;
    params->dpi.de_pol            = LCM_POLARITY_RISING;			//LCM_POLARITY_FALLING;
    params->dpi.vsync_pol         = LCM_POLARITY_FALLING;
    params->dpi.hsync_pol         = LCM_POLARITY_FALLING;

    params->dpi.hsync_pulse_width = 2;
    params->dpi.hsync_back_porch  = 30;
    params->dpi.hsync_front_porch = 30;
    params->dpi.vsync_pulse_width = 2;
    params->dpi.vsync_back_porch  = 30;	//6
    params->dpi.vsync_front_porch = 30;	//6

    params->dpi.format            = LCM_DPI_FORMAT_RGB888;
    params->dpi.rgb_order         = LCM_COLOR_ORDER_RGB;
    params->dpi.is_serial_output  = 0;

    params->dpi.intermediat_buffer_num = 2;

    params->dpi.io_driving_current = LCM_DRIVING_CURRENT_8MA;

}

static void lcm_init(void)
{
	#if defined(BUILD_UBOOT)
	printf("Eric trace: enter lcm_init\n");
	#else
	printk("Eric trace: enter lcm_init\n");
	#endif
    config_gpio();

    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);

    MDELAY(50);
	 //SET_SPI_CS_LOW;
    SET_SPI_CS_HIGH;

    init_lcm_registers();
	//SET_SPI_CS_HIGH;
}

static void lcm_suspend(void)
{
#if 0
	send_ctrl_cmd(0x2800);
	MDELAY(50);
	send_ctrl_cmd(0x1000);
	MDELAY(200);

	//lcm_util.set_gpio_mode(GPIO_POWER_EN_PIN, 0);
	//lcm_util.set_gpio_dir(GPIO_POWER_EN_PIN,1);
	//lcm_util.set_gpio_out(GPIO_POWER_EN_PIN,0);

#else

	unsigned int i;
	const unsigned int PIN_GPIOS[] =
	{

		//GPIO_LCM_RST_PIN,
		GPIO_DPI_CLK_PIN,
		GPIO_DPI_DE_PIN,
		GPIO_DPI_VSYNC_PIN,
		GPIO_DPI_HSYNC_PIN,
		GPIO_SPI_CS_PIN,
		GPIO_SPI_CLK_PIN,
		GPIO_SPI_DA_PIN
	//GPIO_POWER_EN_PIN
	};


	//send_ctrl_cmd(0x2800);
	//MDELAY(50);
	send_ctrl_cmd(0x1000);

	MDELAY(150);

	//lcm_util.set_gpio_mode(GPIO_POWER_EN_PIN,0);
	//lcm_util.set_gpio_dir(GPIO_POWER_EN_PIN,1);
	//lcm_util.set_gpio_out(GPIO_POWER_EN_PIN,0);
	//MDELAY(100);


	//close all pin
	for (i = 0; i < ARY_SIZE(PIN_GPIOS); ++ i)
	{
		lcm_util.set_gpio_mode(PIN_GPIOS[i], 0);
		lcm_util.set_gpio_pull_enable(PIN_GPIOS[i],1);
		mt_set_gpio_pull_select(PIN_GPIOS[i],0);
		lcm_util.set_gpio_dir(PIN_GPIOS[i], 1);
		lcm_util.set_gpio_out(PIN_GPIOS[i],0);
	}
	MDELAY(200);

#endif
}

static void lcm_resume(void)
{


	//send_ctrl_cmd(0x2900);
	//MDELAY(50);

	lcm_init();

	send_ctrl_cmd(0x1100);
	MDELAY(150);

}

//  Get LCM Driver Hooks
LCM_DRIVER RX_50NT_408A_lcm_drv =
{
    .name			= "RX_50NT_408A",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
};
