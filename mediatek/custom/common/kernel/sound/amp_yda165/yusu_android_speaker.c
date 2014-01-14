/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/
#include <asm/uaccess.h>
#include <linux/xlog.h>
#include <linux/i2c.h>
#include "yusu_android_speaker.h"

#include <mach/mt6575_gpio.h>
#include <linux/delay.h>


/*****************************************************************************
*                          DEBUG INFO
******************************************************************************
*/

static bool eamp_log_on = true;

#define EAMP_PRINTK(fmt, arg...) \
	do { \
		if (eamp_log_on) xlog_printk(ANDROID_LOG_INFO,"EAMP", "[YDA165]: %s() "fmt"\n", __func__,##arg); \
	}while (0)


/*****************************************************************************
*				For I2C defination
******************************************************************************
*/	

// device address
#define EAMP_SLAVE_ADDR_WRITE	0xD8
#define EAMP_I2C_CHANNEL     	0       //I2C Channel 0
#define EAMP_I2C_DEVNAME "EAMP_I2C_YDA165"


// speaker, headphone status and path;
static bool gsk_on = false;
static bool ghp_on = false;

#define YDA165_SW_I2C
#ifdef YDA165_SW_I2C
/*****************************************************************************
*                          I 2 C P A R T
******************************************************************************
*/
#define GPIO_SDA 	49
#define GPIO_SCL	50

#define SET_SCCB_CLK_OUTPUT						mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT)
#define SET_SCCB_CLK_INPUT						mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_IN)
#define SET_SCCB_DATA_OUTPUT					mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT)
#define SET_SCCB_DATA_INPUT						mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_IN)

#define SET_SCCB_CLK_HIGH						mt_set_gpio_out(GPIO_SCL, GPIO_OUT_ONE)
#define SET_SCCB_CLK_LOW						mt_set_gpio_out(GPIO_SCL, GPIO_OUT_ZERO)
#define SET_SCCB_DATA_HIGH						mt_set_gpio_out(GPIO_SDA, GPIO_OUT_ONE)
#define SET_SCCB_DATA_LOW						mt_set_gpio_out(GPIO_SDA, GPIO_OUT_ZERO)

#define GET_SCCB_DATA_BIT						mt_get_gpio_in(GPIO_SDA)

#define I2C_DELAY								2

static int i2c_delay(unsigned int n)
{
	udelay(n);
}

#define I2C_START_TRANSMISSION \
{ \
	volatile unsigned char j; \
	SET_SCCB_CLK_OUTPUT; \
	SET_SCCB_DATA_OUTPUT; \
	SET_SCCB_CLK_HIGH; \
	SET_SCCB_DATA_HIGH; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY); 	\
	SET_SCCB_DATA_LOW; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY);	\
	SET_SCCB_CLK_LOW; \
}

#define I2C_STOP_TRANSMISSION \
{ \
	volatile unsigned char j; \
	SET_SCCB_CLK_OUTPUT; \
	SET_SCCB_DATA_OUTPUT; \
	SET_SCCB_CLK_LOW; \
	SET_SCCB_DATA_LOW; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY);	\
	SET_SCCB_CLK_HIGH; \
	/*for(j=0;j<I2C_DELAY;j++);*/\
	i2c_delay(I2C_DELAY);	\
	SET_SCCB_DATA_HIGH; \
}

uint Init_i2c(void)
{
	mt_set_gpio_mode(GPIO_SDA, GPIO_MODE_00);
	mt_set_gpio_mode(GPIO_SCL, GPIO_MODE_00);

	mt_set_gpio_dir(GPIO_SDA, GPIO_DIR_OUT);
	mt_set_gpio_dir(GPIO_SCL, GPIO_DIR_OUT);
	
	mt_set_gpio_pull_enable(GPIO_SDA, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_enable(GPIO_SCL, GPIO_PULL_ENABLE);
	
	mt_set_gpio_pull_select(GPIO_SDA, GPIO_PULL_UP);
	mt_set_gpio_pull_select(GPIO_SCL, GPIO_PULL_UP);

  return 0;
}

static void SCCB_send_byte(unsigned char send_byte)
{
	volatile signed char i;
	volatile unsigned int j;

	for (i=7;i>=0;i--)
	{	/* data bit 7~0 */
		if (send_byte & (1<<i))
		{
			SET_SCCB_DATA_HIGH;
		}
		else
		{
			SET_SCCB_DATA_LOW;
		}
		i2c_delay(I2C_DELAY);
		SET_SCCB_CLK_HIGH;
		i2c_delay(I2C_DELAY);
		SET_SCCB_CLK_LOW;
		i2c_delay(I2C_DELAY);
	}
	/* don't care bit, 9th bit */
	SET_SCCB_DATA_LOW;
	SET_SCCB_DATA_INPUT;
	SET_SCCB_CLK_HIGH;
	i2c_delay(I2C_DELAY);
	SET_SCCB_CLK_LOW;
	SET_SCCB_DATA_OUTPUT;
}	/* SCCB_send_byte() */

static unsigned char SCCB_get_byte(void)
{
	volatile signed char i;
	volatile unsigned char j;
	unsigned char get_byte=0;

	SET_SCCB_DATA_INPUT;

	for (i=7;i>=0;i--)
	{	/* data bit 7~0 */
		SET_SCCB_CLK_HIGH;
		//for(j=0;j<I2C_DELAY;j++);
		i2c_delay(I2C_DELAY);
		if (GET_SCCB_DATA_BIT)
			get_byte |= (1<<i);
		//for(j=0;j<I2C_DELAY;j++);
		i2c_delay(I2C_DELAY);
		SET_SCCB_CLK_LOW;
		//for(j=0;j<I2C_DELAY;j++);
		i2c_delay(I2C_DELAY);
	}
	/* don't care bit, 9th bit */
	SET_SCCB_DATA_OUTPUT;
	SET_SCCB_DATA_HIGH;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SET_SCCB_CLK_HIGH;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SET_SCCB_CLK_LOW;

	return get_byte;
}	/* SCCB_send_byte() */

static void I2CWrite(unsigned char addr, unsigned char para)
{
	volatile unsigned int i, j;

	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SCCB_send_byte(EAMP_SLAVE_ADDR_WRITE);  // slave address of YDA165

	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SCCB_send_byte(addr);

	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SCCB_send_byte(para);

	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	i2c_delay(I2C_DELAY);
	i2c_delay(I2C_DELAY);
	I2C_STOP_TRANSMISSION;
}

static unsigned int I2CRead( unsigned char addr)
{
	unsigned int get_byte;
	volatile unsigned int i, j;

	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SCCB_send_byte(EAMP_SLAVE_ADDR_WRITE);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SCCB_send_byte(addr);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_STOP_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_START_TRANSMISSION;
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	SCCB_send_byte(EAMP_SLAVE_ADDR_WRITE|0x1);
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	get_byte=SCCB_get_byte();
	//for(j=0;j<I2C_DELAY;j++);
	i2c_delay(I2C_DELAY);
	I2C_STOP_TRANSMISSION;

	return get_byte;
}

#else
// I2C variable
static bool i2c_init=false;
static struct i2c_client *new_client = NULL;
// new I2C register method
static const struct i2c_device_id eamp_i2c_id[] = {{EAMP_I2C_DEVNAME,0},{}}; 
struct i2c_board_info __initdata  eamp_dev={I2C_BOARD_INFO(EAMP_I2C_DEVNAME,EAMP_SLAVE_ADDR_WRITE>>1)};

//function declration
static int eamp_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int eamp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int eamp_i2c_remove(struct i2c_client *client);
//i2c driver 
struct i2c_driver eamp_i2c_driver = { 
    .probe = eamp_i2c_probe,								   
    .remove = eamp_i2c_remove,							 
    //.detect = eamp_i2c_detect,							 
    .driver = {
    	.name =EAMP_I2C_DEVNAME,
    	},
    .id_table = eamp_i2c_id,							 
    //.address_data = &addr_data, 					   
};

//read one register
ssize_t static eamp_read_byte(u8 addr, u8 *returnData)
{
	if(!new_client)
	{	
		EAMP_PRINTK("I2C client not initialized!!");
		return -1;
	}	
	char	 cmd_buf[1]={0x00};
	char	 readData = 0;
	int 	ret=0;
	cmd_buf[0] = addr;
	ret = i2c_master_send(new_client, &cmd_buf[0], 1);
	if (ret < 0) {
		EAMP_PRINTK("read sends command error!!");
		return -1;
	}
	ret = i2c_master_recv(new_client, &readData, 1);
	if (ret < 0) {
		EAMP_PRINTK("reads recv data error!!");
		return -1;
	} 
	*returnData = readData;
	EAMP_PRINTK("reads recv data 0x%x",readData);
	return 0;
}
	
//read one register
static u8	I2CRead(u8 addr)
{
	u8 regvalue;
	eamp_read_byte(addr,&regvalue);
	return regvalue;
}

// write register
static ssize_t	I2CWrite(u8 addr, u8 writeData)
{
	if(!new_client)
	{	
		EAMP_PRINTK("I2C client not initialized!!");
		return -1;
	}	
	char	write_data[2] = {0};
	int    ret=0;	
	write_data[0] = addr;		  // ex. 0x01
	write_data[1] = writeData;
	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		EAMP_PRINTK("write sends command error!!");
		return -1;
	}
	EAMP_PRINTK("write data 0x%x",writeData);
	return 0;
}
	
//write register
static ssize_t	eamp_write_byte(u8 addr, u8 writeData)
{
	if(!new_client)
	{	
		EAMP_PRINTK("I2C client not initialized!!");
		return -1;
	}	
	char	write_data[2] = {0};
	int    ret=0;	
	write_data[0] = addr;		  // ex. 0x01
	write_data[1] = writeData;
	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		EAMP_PRINTK("write sends command error!!");
		return -1;
	}
	EAMP_PRINTK("write data 0x%x",writeData);
	return 0;
}
#endif		
	
//set speaker volume
static ssize_t eamp_set_speaker_vol(u8	vol)
{
EAMP_PRINTK("eamp_set_speaker_vol");
	return 0;
}
	
//set  headphone  volume
static ssize_t eamp_set_headPhone_vol(u8 HP_vol)
{
	EAMP_PRINTK("eamp_set_headPhone_vol");	
	return 0;
}
	
//set  headphone left  volume
static ssize_t eamp_set_headPhone_lvol(u8 HPL_Vol)
{
	EAMP_PRINTK("eamp_set_headPhone_lvol");	
	return 0;
}
		
//set  headphone right volume
static ssize_t eamp_set_headPhone_rvol(u8 HPR_Vol)
{
	EAMP_PRINTK("eamp_set_headPhone_rvol");	
	return 0;
}
	
	
//**********************************functions to control devices***********************************

// set registers to default value
static ssize_t eamp_resetRegister()
{
	EAMP_PRINTK("eamp_resetRegister");
	I2CWrite(0X80,0X80);
	return 0;
}
	
		
static ssize_t eamp_openheadPhone()
{
	u8 temp_control_reg = 0;
		EAMP_PRINTK("eamp_openheadPhone");
	I2CWrite(0x85, 0x5f);
	I2CWrite(0x86, 0x5f);

	I2CWrite(0x87, 0x03);
	
	ghp_on = true;
	return 0;
}

static ssize_t eamp_closeheadPhone()
{

	u8 temp_control_reg = 0;
	EAMP_PRINTK("eamp_closeheadPhone");
	I2CWrite(0x86, 0x40);	
	I2CWrite(0x87, 0x00);
	
	if(gsk_on)
	  {
	  	I2CWrite(0x85, 0x5f);	
		I2CWrite(0x87, 0x30);
		}
	
	ghp_on = false;
	return 0;
}
	
static ssize_t eamp_openspeaker()
{
	u8 temp_control_reg = 0;
	I2CWrite(0x83, 0x86);
	I2CWrite(0x84, 0x44);
	I2CWrite(0x85, 0x5f);	
	if(ghp_on)
		I2CWrite(0x87, 0x33);
		else
		I2CWrite(0x87, 0x30);
	gsk_on = true;
	return 0;
}

static ssize_t eamp_closespeaker()
{
EAMP_PRINTK("eamp_closespeaker");
	I2CWrite(0x85, 0x40);
	I2CWrite(0x87, 0x00);
	
	if(ghp_on)
	{
		I2CWrite(0x86, 0x5f);	
		I2CWrite(0x87, 0x03);
	}

	gsk_on = false;
	return 0;
}
		
static ssize_t eamp_suspend()  
{
	EAMP_PRINTK("eamp_suspend");
	eamp_resetRegister();
	return 0;
}
	
static ssize_t eamp_resume()
{
	EAMP_PRINTK("eamp_resume");
	I2CWrite(0x80, 0x01);
	if(gsk_on)
	{
		eamp_openspeaker();
	}
	if(ghp_on)
	{
		eamp_openheadPhone();
	}
	return 0;
}
	
 int eamp_command( unsigned int  type)
{
	switch(type)
	{
		case EAMP_SPEAKER_CLOSE:
		{
			eamp_closespeaker();
			break;
		}
		case EAMP_SPEAKER_OPEN:
		{
			eamp_openspeaker();
			break;
		}
		case EAMP_HEADPHONE_CLOSE:
		{
			eamp_closeheadPhone();
			break;
		}
		case EAMP_HEADPHONE_OPEN:
		{
			eamp_openheadPhone();
			break;
		}	
		default:
			EAMP_PRINTK("eamp_command TYPE = %d",type);
			return 0;
	}	
	return 0;
}
	
int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count)
{
	return eamp_command(type);
}

#if !defined(YDA165_SW_I2C)	
static int eamp_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {		   
	strcpy(info->type, EAMP_I2C_DEVNAME);														  
	return 0;																						
}																								   
	
static int eamp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {			   
	
	EAMP_PRINTK("+++++++++++++++eamp_i2c_probe++++++++++++++++++"); //add for test
	new_client = client;
	eamp_resetRegister();
	EAMP_PRINTK("client=%x !!",client);
	return 0;																						
} 
	
static int eamp_i2c_remove(struct i2c_client *client)
{
	EAMP_PRINTK("");
	new_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&eamp_i2c_driver);
	return 0;
} 
#endif

static void eamp_poweron(void)
{
	I2CWrite(0x80,0x01);
	return;
}
	
static void eamp_powerdown(void)
{
	return;
}
	
static int eamp_init()
{
#if !defined(YDA165_SW_I2C)	
	if(i2c_init)
		return 0;
	i2c_init=true;
	EAMP_PRINTK("");
	//i2c_register_board_info(EAMP_I2C_CHANNEL,&eamp_dev,1);
	if (i2c_add_driver(&eamp_i2c_driver)){
		EAMP_PRINTK("fail to add device into i2c");
		return -1;
	}
#else
	Init_i2c();
#endif
		eamp_poweron();
	return 0;
}
	
static int eamp_deinit()
{
	EAMP_PRINTK("");
	eamp_powerdown();
#if !defined(YDA165_SW_I2C)		
	i2c_init = false;
#endif
	return 0;
}
	
/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
extern void Yusu_Sound_AMP_Switch(BOOL enable);

bool Speaker_Init(void)
{
	printk("+");
	eamp_init();
	printk("-");
	
	return true;
}

bool Speaker_Register(void)
{
    return false;
}

int ExternalAmp(void)
{
	return 0;
}	
	
void Sound_SpeakerL_SetVolLevel(int level)
{
	EAMP_PRINTK("level=%d",level);
}
	
void Sound_SpeakerR_SetVolLevel(int level)
{
	EAMP_PRINTK("level=%d",level);
}
	
void Sound_Speaker_Turnon(int channel)
{
	EAMP_PRINTK("Sound_Speaker_Turnon channel = %d",channel);
	eamp_command(EAMP_SPEAKER_OPEN);
}
	
void Sound_Speaker_Turnoff(int channel)
{
	EAMP_PRINTK("Sound_Speaker_Turnoff channel = %d",channel);
	eamp_command(EAMP_SPEAKER_CLOSE);
}
	
void Sound_Speaker_SetVolLevel(int level)
{
	//Speaker_Volume =level;	
}	

void Sound_Headset_Turnon(void)
{
	EAMP_PRINTK("");
	//eamp_command(EAMP_SPEAKER_OPEN);
}
void Sound_Headset_Turnoff(void)
{
	EAMP_PRINTK("");
	//eamp_command(EAMP_HEADPHONE_CLOSE);
}
	
//kernal use
void AudioAMPDevice_Suspend(void)
{
	EAMP_PRINTK("AudioAMPDevice_Suspend");
	eamp_suspend();
}

void AudioAMPDevice_Resume(void)
{
	EAMP_PRINTK("AudioAMPDevice_Resume");
	eamp_resume();	
}
	
// for AEE beep sound
void AudioAMPDevice_SpeakerLouderOpen(void)
{
	EAMP_PRINTK("");
	if(gsk_on)
	{
		return;
	}

	eamp_openspeaker();
	return ;
}
	
// for AEE beep sound
void AudioAMPDevice_SpeakerLouderClose(void)
{
	EAMP_PRINTK("");
	if(!gsk_on)
		{
			return;
		}
	eamp_openspeaker();
}
	
// mute device when INIT_DL1_STREAM
void AudioAMPDevice_mute(void)
{
	if(ghp_on)
		eamp_closeheadPhone();	
	if(gsk_on)
		eamp_closespeaker();
	// now not control earpiece.
}
	
bool Speaker_DeInit(void)
{
	eamp_deinit();
	return true;
}
	
	
static char *ExtFunArray[] =
{
	"InfoMATVAudioStart",
	"InfoMATVAudioStop",
	"End",
};
	
kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size)
{
	int i = 0;
	int funNum = -1;
	
	//Search the supported function defined in ExtFunArray
	while(strcmp("End",ExtFunArray[i]) != 0 ) {		//while function not equal to "End"
		
		if (strcmp(name,ExtFunArray[i]) == 0 ) {		//When function name equal to table, break
			funNum = i;
			break;
		}
		i++;
	}
	
	switch (funNum) {
	case 0:			//InfoMATVAudioStart
		printk("InfoMATVAudioStart");
		break;
		
	case 1:			//InfoMATVAudioStop
		printk("InfoMATVAudioStop");
		break;
		
	default:
		break;
	}
	return 1;
}
	
