#define TPD_DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>


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

#include "tpd.h"
#include <cust_eint.h>
#include <linux/jiffies.h>

#ifndef TPD_NO_GPIO
#include "cust_gpio_usage.h"
#endif

#include "tpd_custom_GT9006.h"
//#include "gt9006_master.h"
//#include "gt9006_dsp.h"
//#include "gt9006_dsp_isp.h"
//#include "gt9006_bootcode.h"

#ifdef FIRMWARE_WITH_HEADER
#define TPD_FW_HEADER_SIZE               20
#else
#define TPD_FW_HEADER_SIZE               0
#endif

#define TPD_VERSION_INFO_REG              0x8140
#define TPD_MASTER_RESET_REG              0x4180
#define TPD_MASTER_STATUS_REG             0x6088
#define TPD_MASTER_CODE_REG               0xC000
#define TPD_DSP_ISP_BOOT_CODE_REG         0x9000
#define TPD_SRAM_BAND_REG                 0x4048
#define TPD_FIRMWARE_SIZE                 (0x8000+TPD_FW_HEADER_SIZE)
#define TPD_DSP_FIRMWARE_SIZE             (0x1000+TPD_FW_HEADER_SIZE)
#define TPD_BAND_SIZE                     0x4000
#define TPD_SS51_DOWNLOAD_SIZE            0x2000
#define TPD_DSP_ISP_DOWNLOAD_SIZE         0x1000
#define TPD_DSP_DOWNLOAD_SIZE             0x1000
#define TPD_BOOTCODE_DOWNLOAD_SIZE        0x800
#define TPD_TOUCH_INFO_REG_BASE           0x814E
#define TPD_POINT_INFO_REG_BASE           0x814F
#define TPD_KEY_INFO_REG_BASE             0x819F
#define TPD_DIFF_DATA_REG                 0xBB10
#define TPD_RAW_DATA_REG                  0x8B98
#define TPD_FLASH_RUN_REG                 0x4283
#define TPD_CONFIG_REG_BASE               0x8047
#define TPD_BUFFER_STATUS_SHIFT           7
#define TPD_FIRMWARE_VERSION              0x8147
#define TPD_NVRAM_I2C_ADDRESS             0x5094
#define TPD_NVRAM_OPERATION_SIZE          0x5096
#define TPD_NVRAM_FILE_OFFSET             0x4196
#define TPD_NVRAM_OPERATION_CHECKSUM      0x4194
#define TPD_FLASH_POWER_OFF               0x4284

#define TPD_SENSING_LINE                  10
#define TPD_DRIVING_LINE                  15

#define TPD_DEFAULT_CALI_VALUE            76

#define TPD_POINT_INFO_LEN                8
#define TPD_MAX_POINTS                    10
#define MAX_TRANSACTION_LENGTH            255
#define MAX_READ_TRANSACTION_LENGTH       8
#define I2C_DEVICE_ADDRESS_LEN            2
#define I2C_INIT_MASTER_CLOCK             300
#define I2C_MASTER_CLOCK                  3000

#define TPD_INIT_I2C_ADDR                 (0xBA>>1)
#define TPD_WORKING_I2C_ADDR              0xBA

#define TPD_CALI_FILE_NAME                "/data/nvram/APCFG/APRDEB/TPD"
#define TPD_NVRAM_FILE_NAME               "/data/TPD_data"
#define TPD_FIRMWARE_FILE_NAME            "/data/TPD_firmware/GT9006.BIN"
#define TPD_DSP_FIRMWARE_FILE_NAME        "/data/TPD_firmware/DSP.BIN"
#define TPD_CONFIG_FILE_NAME              "/data/TPD_firmware/default.gtx"
#define TPD_CONFIG_PROC_FILE              "tpd_config"
#define TPD_COMMAND_PROC_FILE             "tpd_command"

#define TPD_DELAY_INIT_CHECK_CIRCLE       100

#define TPD_SRAM_BANK                     0x4048
#define TPD_MEM_CD_EN                     0x4049
#define TPD_CACHE_EN                      0x404B
#define TPD_TMR0_EN                       0x40B0
#define TPD_SWRST_B0_                     0x4180
#define TPD_CPU_SWRST_PULSE               0x4184
#define TPD_BOOTCTL_B0_                   0x4190
#define TPD_BOOT_OPT_B0_                  0x4218
#define TPD_BOOT_CTL_                     0x5094//0x4283
#define TPD_ANA_RXADC_B0_                 0x4250
#define TPD_RG_LDO_A18_PWD                0x426f
#define TPD_RG_BG_PWD                     0x426a
#define TPD_RG_CLKGEN_PWD                 0x4269
#define TPD_RG_DMY                        0x4282
#define TPD_RG_OSC_CALIB                  0x4268
#define TPD_OSC_CK_SEL                    0x4030

#define _FW_TOTAL_PAGES                   32
#define _FW_FRAME_PAGES	                  16
#define _DSP_TOTAL_PAGES                  4 // CCH modified
#define _BOOTCODE_TOTAL_PAGES             32
#define _FW_FRAME_BYTES	                  (_FW_FRAME_PAGES*1024)
#define _DSP_TOTAL_BYTES                  (_DSP_TOTAL_PAGES*1024)
#define _BOOTCODE_TOTAL_BYTES             (_BOOTCODE_TOTAL_PAGES*1024)

#ifdef MT6573
#define CHR_CON0                          (0xF7000000+0x2FA00)
#endif
#if defined(MT6575) || defined(MT6577)
extern kal_bool upmu_is_chr_det(void);
#endif

#define MAX_I2C_TRANSFER_SIZE             (MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)

#define CONFIG_LEN (186)


//for test

int i2c_clock = I2C_INIT_MASTER_CLOCK;

extern struct tpd_device *tpd;


int power_off_mode = 1;
module_param(power_off_mode, int, 00644);

u8 *master_firmware = gt9006_master_firmware;
u8 *dsp_firmware = gt9006_dsp_firmware;



int firmware_version = 0;
int tpd_flag = 0;
static int tpd_halt = 0;

static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(tpd_waiter);

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#ifdef TPD_HAVE_TOUCH_KEY
const u16 touch_key_array[TPD_KEY_COUNT] = TPD_KEYS;
#define TPD_TOUCH_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

void tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);




struct tpd_info_t
{
    u8 vendor_id_1;
    u8 vendor_id_2;
    u8 product_id_1;
    u8 product_id_2;
    u8 version_1;
    u8 version_2;
};

struct tpd_cali_t
{
    u8 clk_cali;
    u8 clk_cali_checksum;
};



static struct i2c_client *i2c_client = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"gt9006", 0}, {}};
static unsigned short force[] = {0, TPD_INIT_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };
//for android 2.3 and before
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//end for android 2.3 and before
//for android 4.0 and after
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("gt9006", (0xBA >> 1))};
//end for android 4.0 and after
static struct tpd_cali_t tpd_cali = {TPD_DEFAULT_CALI_VALUE, ~(TPD_DEFAULT_CALI_VALUE) };

static struct i2c_driver tpd_i2c_driver =
{
    .probe = tpd_i2c_probe,
    .remove = tpd_i2c_remove,
    .detect = tpd_i2c_detect,
    .driver.name = "gt9006",
    .id_table = tpd_i2c_id,
    //for android 2.3 and before
    //.address_data = &addr_data,
    //end for android 2.3 and before
    //for android 4.0 and after
    .address_list = (const unsigned short *)forces,
    //end for android 4.0 and after


};
struct tpd_info_t tpd_info;

struct my_work_t
{
    struct delayed_work my_work;
    struct i2c_client *i2c_client;
};

/* proc file system */
static struct proc_dir_entry *gt9006_config_proc = NULL;
static struct proc_dir_entry *gt9006_command_proc = NULL;

static int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
static int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);

static struct work_struct tpd_fw_loading_work;
static struct work_struct tpd_nvram_read_work;
static struct work_struct tpd_nvram_write_work;

static void tpd_set_i2c_clock(int clock)
{
    i2c_clock = clock;
}


void clock_boost(u8 bDACCode)
{
    int err;
    u8 buf[1];
    buf[0] = 0x00;
    err = i2c_read_bytes(i2c_client, TPD_ANA_RXADC_B0_, buf, 1);

    buf[0] = (buf[0] & (~(1 << 4)));
    err = i2c_write_bytes(i2c_client, TPD_ANA_RXADC_B0_, buf, 1);

    buf[0] = 0x00;
    err = i2c_write_bytes(i2c_client, TPD_RG_LDO_A18_PWD, buf, 1);
    err = i2c_write_bytes(i2c_client, TPD_RG_BG_PWD, buf, 1);
    err = i2c_write_bytes(i2c_client, TPD_RG_CLKGEN_PWD, buf, 1);

    err = i2c_read_bytes(i2c_client, TPD_ANA_RXADC_B0_, buf, 1);

    buf[0] = (buf[0] & (~0x01)) ;
    err = i2c_write_bytes(i2c_client, TPD_ANA_RXADC_B0_, buf, 1);

    err = i2c_read_bytes(i2c_client, TPD_ANA_RXADC_B0_, buf, 1);

    buf[0] = (buf[0] & (~0x02)) ;
    err = i2c_write_bytes(i2c_client, TPD_ANA_RXADC_B0_, buf, 1);

    err = i2c_read_bytes(i2c_client, TPD_RG_DMY, buf, 1);
    buf[0] = (buf[0] & 0xFCF) | ((bDACCode & 0x03) << 4);
    err = i2c_write_bytes(i2c_client, TPD_RG_DMY, buf, 1);
    buf[0] = (bDACCode >> 2) & 0x3F;
    err = i2c_write_bytes(i2c_client, TPD_RG_OSC_CALIB, buf, 1);

    buf[0] = 0x01;
    err = i2c_write_bytes(i2c_client, TPD_OSC_CK_SEL, buf, 1);
}

static void tpd_power_on_init(void)
{

#ifdef MT6573
    // power on CTP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif
#if defined(MT6575) || defined(MT6577)
    //power off
    hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
    hwPowerDown(MT65XX_POWER_LDO_VGP, "TP");

    msleep(10);
    //power on, need confirm with SA
    hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");

#endif

    // set INT pull low
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);// pull low(0xBA) or high(0x28)
    //mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ONE);// pull low(0xBA) or high(0x28)
    //mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

    // reset
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(20);

    // set INT as interrupt mode
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

    //mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

}

static void do_tpd_setup_eint(int eintPolarity)
{
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN,
                             eintPolarity, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}

static void tpd_setup_eint(void)
{

    if ((gt9006_config[6] & 0x03) == 0x00)
    {
        do_tpd_setup_eint(CUST_EINT_POLARITY_HIGH);
        TPD_DMESG("CUST_EINT_POLARITY_HIGH\n");
    }
    else if ((gt9006_config[6] & 0x03) == 0x01)
    {
        do_tpd_setup_eint(CUST_EINT_POLARITY_LOW);
        TPD_DMESG("CUST_EINT_POLARITY_LOW\n");
    }

}




static int
nvram_read(
    char *filename,
    char *buf,
    ssize_t len,
    int offset)
{

    struct file *fd;
    int retLen = -1;

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fd = filp_open(filename, O_RDONLY, 0644);

    if (IS_ERR(fd))
    {
        TPD_DMESG("[nvram_read] : failed to open!!\n");
        return -1;
    }

    do
    {
        if ((fd->f_op == NULL) || (fd->f_op->read == NULL))
        {
            TPD_DMESG("[nvram_read] : file can not be read!!\n");
            break;
        }

        if (fd->f_pos != offset)
        {
            if (fd->f_op->llseek)
            {
                if (fd->f_op->llseek(fd, offset, 0) != offset)
                {
                    TPD_DMESG("[nvram_read] : failed to seek!!\n");
                    break;
                }
            }
            else
            {
                fd->f_pos = offset;
            }
        }

        retLen = fd->f_op->read(fd, buf, len, &fd->f_pos);

    }
    while (FALSE);

    filp_close(fd, NULL);
    set_fs(old_fs);
    return retLen;
}

static int nvram_write(char *filename, char *buf, ssize_t len, int offset)
{
    struct file *fd;
    //ssize_t ret;
    int retLen = -1;

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

    fd = filp_open(filename, O_WRONLY | O_CREAT, 0644);

    if (IS_ERR(fd))
    {
        TPD_DMESG("[nvram_write] : failed to open!!\n");
        return -1;
    }

    do
    {
        if ((fd->f_op == NULL) || (fd->f_op->write == NULL))
        {
            TPD_DMESG("[nvram_write] : file can not be write!!\n");
            break;
        } /* End of if */

        if (fd->f_pos != offset)
        {
            if (fd->f_op->llseek)
            {
                if (fd->f_op->llseek(fd, offset, 0) != offset)
                {
                    TPD_DMESG("[nvram_write] : failed to seek!!\n");
                    break;
                }
            }
            else
            {
                fd->f_pos = offset;
            }
        }

        retLen = fd->f_op->write(fd,
                                 buf,
                                 len,
                                 &fd->f_pos);

    }
    while (false);

    filp_close(fd, NULL);
    set_fs(old_fs);
    return retLen;
}

static void tpd_nvram_write(struct work_struct *work)
{
    u16 addr;
    u16 size;
    u16 offset;
    u16 checksum;
    u8 *buf;
    int ret;
    int i;
    u16 temp_checksum;

    i2c_read_bytes(i2c_client, TPD_NVRAM_I2C_ADDRESS, (u8 *)&addr, 2);
    i2c_read_bytes(i2c_client, TPD_NVRAM_OPERATION_SIZE, (u8 *)&size, 2);
    i2c_read_bytes(i2c_client, TPD_NVRAM_FILE_OFFSET, (u8 *)&offset, 2);
    i2c_read_bytes(i2c_client, TPD_NVRAM_OPERATION_CHECKSUM, (u8 *)&checksum, 2);

    buf = kmalloc(sizeof(u8) * size, GFP_KERNEL);

    i2c_read_bytes(i2c_client, addr, buf, size);

    temp_checksum = 0;


    if (size == 1)
    {
        temp_checksum = buf[0];
    }
    else
    {
        for (i = 0; i < size; i += 2)
        {
            temp_checksum += buf[i] * 256 + buf[i + 1];
            temp_checksum &= 0xFFFF;
        }
    }

    temp_checksum = 65536 - temp_checksum;

    TPD_DMESG("addr = %04X\n", addr);
    TPD_DMESG("size = %04X\n", size);
    TPD_DMESG("offset = %04X\n", offset);
    TPD_DMESG("checksum = %04X\n", checksum);

    if (temp_checksum != checksum)
    {
        TPD_DMESG("check sum error~! [%04X]:[%04X]\n", temp_checksum, checksum);
    }
    else
    {
        ret = nvram_write(TPD_NVRAM_FILE_NAME, buf, size, offset);

        if (ret != size)
        {
            TPD_DMESG("nvram_write failed!\n");
        }
    }

    size = 0;
    i2c_write_bytes(i2c_client, TPD_NVRAM_OPERATION_SIZE, (u8 *)&size, 2);
    kfree(buf);

}

static void tpd_nvram_read(struct work_struct *work)
{
    u16 addr;
    u16 size;
    u16 offset;
    u16 checksum;
    u8 *buf;
    int i;
    i2c_read_bytes(i2c_client, TPD_NVRAM_I2C_ADDRESS, (u8 *)&addr, 2);
    i2c_read_bytes(i2c_client, TPD_NVRAM_OPERATION_SIZE, (u8 *)&size, 2);
    i2c_read_bytes(i2c_client, TPD_NVRAM_FILE_OFFSET, (u8 *)&offset, 2);

    buf = kmalloc(sizeof(u8) * size, GFP_KERNEL);

    TPD_DMESG("addr = %04X\n", addr);
    TPD_DMESG("size = %04X\n", size);
    TPD_DMESG("offset = %04X\n", offset);


    nvram_read(TPD_NVRAM_FILE_NAME, buf, size, offset);

    checksum = 0;

    if (size == 1)
    {
        checksum = buf[0];
    }
    else
    {
        for (i = 0; i < size; i += 2)
        {
            checksum += buf[i] * 256 + buf[i + 1];
            checksum &= 0xFFFF;
        }
    }

    checksum = 65536 - checksum;


    TPD_DMESG("checksum = %04X\n", checksum);

    i2c_write_bytes(i2c_client, addr, buf, size);

    i2c_write_bytes(i2c_client, TPD_NVRAM_OPERATION_CHECKSUM, (u8 *)&checksum, 2);

    kfree(buf);

}


static void cali_data_init(void)
{

    int ret;
    ret = nvram_read(TPD_CALI_FILE_NAME, (char *) &tpd_cali, sizeof(struct tpd_cali_t), 0);

    if (ret == -1)
    {
        TPD_DMESG("read calibration data error~!\n");
        tpd_cali.clk_cali = TPD_DEFAULT_CALI_VALUE;
        tpd_cali.clk_cali_checksum = 255 - TPD_DEFAULT_CALI_VALUE;
    }

    if (tpd_cali.clk_cali != (255 - tpd_cali.clk_cali_checksum))
    {

        TPD_DMESG("calibration data check error~!\n");
        tpd_cali.clk_cali = TPD_DEFAULT_CALI_VALUE;
        tpd_cali.clk_cali_checksum = 255 - TPD_DEFAULT_CALI_VALUE;
    }

    TPD_DMESG("calibration data : %d\n", tpd_cali.clk_cali);
    //gt9006_config[20] = tpd_cali.clk_cali;


}

#ifdef FIRMWARE_TO_FLASH
static int do_download_to_flash(struct i2c_client *client,
                                unsigned char *gt9006_dsp_isp_firmware,
                                unsigned char *gt9006_master_firmware,
                                unsigned char *gt9006_dsp_firmware,
                                unsigned char *gt9006_bootcode_firmware
                               )
{
    int err;
    int i;
    int j;
    u8 buf[1];
    u8 bBank;
    unsigned int bootchksum;
    unsigned char chksumH;
    unsigned char chksumL;
    u8 *readback_buf = kmalloc(TPD_FIRMWARE_SIZE, GFP_KERNEL);

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_BOOT_CTL_, buf, 1);
    msleep(5);

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_TMR0_EN, buf, 1);
    msleep(5);

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_CACHE_EN, buf, 1);
    msleep(5);

    buf[0] = 0x0C;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
    msleep(5);

    buf[0] = 0x02;
    err = i2c_write_bytes(client, TPD_BOOTCTL_B0_, buf, 1);
    msleep(5);

    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_CPU_SWRST_PULSE, buf, 1);
    msleep(5);


download_dsp_isp:

    TPD_DMESG("download dsp isp\n");

    buf[0] = 0x02;
    err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);   // switch bank
    msleep(5);
    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1);
    msleep(5);

    err = i2c_write_bytes(client,
                          TPD_MASTER_CODE_REG,
                          gt9006_dsp_isp_firmware,
                          _DSP_TOTAL_BYTES); // write the second bank

    err = i2c_read_bytes(client, TPD_MASTER_CODE_REG, readback_buf, _DSP_TOTAL_BYTES);

    // compare
    for (i = 0 ; i < _DSP_TOTAL_BYTES ; i++)
    {
        if (gt9006_dsp_isp_firmware[i] != readback_buf[i])
        {
            TPD_DMESG("{%d} %02X:%02X\n", i, gt9006_dsp_isp_firmware[i], readback_buf[i]);
            goto download_dsp_isp;
        }
    }

    buf[0] = 0x00;//scramble
    err = i2c_write_bytes(client, TPD_BOOT_OPT_B0_, buf, 1);
    msleep(5);

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
    msleep(5);

    chksumH = 0;
    chksumL = 0;
    bootchksum = 0;

    for (j = 0; j < (TPD_FIRMWARE_SIZE - 2); j += 2)
    {
        bootchksum += gt9006_master_firmware[j] * 256 + gt9006_master_firmware[j + 1];
        bootchksum &= 0xFFFF;
    }

    bootchksum = 65536 - bootchksum;
    chksumH = (bootchksum >> 8) & 0xFF;
    chksumL = bootchksum & 0xFF;
    gt9006_master_firmware[32 * 1024 - 2] = chksumH;
    gt9006_master_firmware[32 * 1024 - 1] = chksumL;

    // Burn into Flash
    for (i =  0; i < 4; i++)
    {
flash_download:
        buf[0] = 0x0C;
        err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
        msleep(10);
        bBank = (i >> 1);    // bank
        buf[0] = bBank;
        err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);
        msleep(5);
        TPD_DMESG("bBank %d\n", bBank);
        buf[0] = 0x01;
        msleep(5);
        err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1);
        TPD_DMESG("flash download part %d download to 0x%X\n", i, TPD_MASTER_CODE_REG + (i % 2) * TPD_SS51_DOWNLOAD_SIZE);
        msleep(5);

        err = i2c_write_bytes(client, TPD_MASTER_CODE_REG + (i % 2) * TPD_SS51_DOWNLOAD_SIZE,
                              gt9006_master_firmware + i * TPD_SS51_DOWNLOAD_SIZE,
                              TPD_SS51_DOWNLOAD_SIZE);  // write the first bank

        err = i2c_read_bytes(client, TPD_MASTER_CODE_REG + (i % 2) * TPD_SS51_DOWNLOAD_SIZE,
                             readback_buf, TPD_SS51_DOWNLOAD_SIZE);

        // compare
        for (j = 0 ; j < TPD_SS51_DOWNLOAD_SIZE ; j++)
        {
            if (gt9006_master_firmware[j + i * TPD_SS51_DOWNLOAD_SIZE] != readback_buf[j])
            {
                TPD_DMESG("[%d] %02X:%02X\n", j,
                          gt9006_master_firmware[j + i * TPD_SS51_DOWNLOAD_SIZE],
                          readback_buf[j]);
                goto flash_download;
            }
        }

        buf[0] = 0x04;
        err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
        msleep(10);

        buf[0] = (i + 1);
        err = i2c_write_bytes(client, TPD_BOOT_CTL_, buf, 1);
        msleep(10);

        do
        {
            err = i2c_read_bytes(client, TPD_BOOT_CTL_, buf, 1);
            TPD_DMESG("%d:rRW_MISCTL__BOOT_CTL_ =%x\n", __LINE__, buf[0]);
            msleep(10);
        }
        while (buf[0] == (i + 1));

        err = i2c_read_bytes(client, TPD_BOOT_CTL_ + 1, buf, 1);

        if (buf[0] == 0xEE)
        {
            TPD_DMESG("Flash write error~~!\n");
        }
        else
        {
            TPD_DMESG("Flash write OK~~!\n");
        }
    }

    //dsp_flash download
dsp_flash_download:
    TPD_DMESG("flash download dsp\n");
    bBank = 3;    // bank
    buf[0] = bBank;
    err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);
    buf[0] = 0x0C;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
    msleep(10);

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);

    msleep(10);
    chksumH = 0;
    chksumL = 0;
    bootchksum = 0;

    for (j = 0; j < (TPD_DSP_ISP_DOWNLOAD_SIZE - 2); j += 2)
    {
        bootchksum += gt9006_dsp_firmware[j] * 256 + gt9006_dsp_firmware[j + 1];
        bootchksum &= 0xFFFF;
    }

    bootchksum = 65536 - bootchksum;
    chksumH = (bootchksum >> 8) & 0xFF;
    chksumL = bootchksum & 0xFF;
    gt9006_dsp_firmware[TPD_DSP_ISP_DOWNLOAD_SIZE - 2] = chksumH;
    gt9006_dsp_firmware[TPD_DSP_ISP_DOWNLOAD_SIZE - 1] = chksumL;

    err = i2c_write_bytes(client, TPD_DSP_ISP_BOOT_CODE_REG,
                          gt9006_dsp_firmware,
                          TPD_DSP_ISP_DOWNLOAD_SIZE);  // write the first bank

    buf[0] = bBank;
    err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);
    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1);

    err = i2c_read_bytes(client, TPD_DSP_ISP_BOOT_CODE_REG,
                         readback_buf, TPD_DSP_ISP_DOWNLOAD_SIZE);

    for (j = 0 ; j < TPD_DSP_ISP_DOWNLOAD_SIZE  ; j++)
    {
        if (gt9006_dsp_firmware[j] != readback_buf[j])
        {
            TPD_DMESG("[%d] %02X:%02X\n", j, gt9006_dsp_firmware[j], readback_buf[j]);
            goto dsp_flash_download;
        }
    }

    buf[0] = 0x05;
    err = i2c_write_bytes(client, TPD_BOOT_CTL_, buf, 1);
    msleep(10);

    do
    {
        err = i2c_read_bytes(client, TPD_BOOT_CTL_, buf, 1);
        msleep(10);
        TPD_DMESG("%d:rRW_MISCTL__BOOT_CTL_ =%x\n", __LINE__, buf[0]);
    }
    while (buf[0] == 0x05);

    err = i2c_read_bytes(client, TPD_BOOT_CTL_ + 1, buf, 1);

    if (buf[0] == 0xEE)
    {
        TPD_DMESG("Flash write error~~!\n");
    }
    else
    {
        TPD_DMESG("Flash write OK~~!\n");
    }

bootcode_download:
    ///download bootcode
    TPD_DMESG("flash download bootcode\n");
    buf[0] = 0x0C;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
    msleep(10);
    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
    msleep(10);

    buf[0] = bBank;
    err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);
    msleep(5);

    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1);
    msleep(5);

    chksumH = 0;
    chksumL = 0;
    bootchksum = 0;

    for (j = 0; j < (TPD_BOOTCODE_DOWNLOAD_SIZE - 2); j += 2)
    {
        bootchksum += gt9006_bootcode_firmware[j] * 256 + gt9006_bootcode_firmware[j + 1];
        bootchksum &= 0xFFFF;
    }

    bootchksum = 65536 - bootchksum;
    chksumH = (bootchksum >> 8) & 0xFF;
    chksumL = bootchksum & 0xFF;
    gt9006_bootcode_firmware[(2 * 1024 - 2)] = chksumH;
    gt9006_bootcode_firmware[(2 * 1024 - 1)] = chksumL;

    err = i2c_write_bytes(client, TPD_DSP_ISP_BOOT_CODE_REG,
                          gt9006_bootcode_firmware,
                          TPD_BOOTCODE_DOWNLOAD_SIZE);  // write the first bank

    err = i2c_read_bytes(client, TPD_DSP_ISP_BOOT_CODE_REG,
                         readback_buf, TPD_BOOTCODE_DOWNLOAD_SIZE);

    // compare
    for (j = 0 ; j < TPD_BOOTCODE_DOWNLOAD_SIZE  ; j++)
    {
        if (gt9006_bootcode_firmware[j] != readback_buf[j])
        {
            TPD_DMESG("[%d] %02X:%02X\n", j, gt9006_bootcode_firmware[j], readback_buf[j]);
            goto bootcode_download;
        }
    }

    buf[0] = 0x06;
    err = i2c_write_bytes(client, TPD_BOOT_CTL_, buf, 1);
    msleep(10);

    do
    {
        err = i2c_read_bytes(client, TPD_BOOT_CTL_, buf, 1);
        msleep(10);
        TPD_DMESG("%d:rRW_MISCTL__BOOT_CTL_ =%x\n", __LINE__, buf[0]);
    }
    while (buf[0] == 0x06);

    err = i2c_read_bytes(client, TPD_BOOT_CTL_ + 1, buf, 1);

    if (buf[0] == 0xEE)
    {
        TPD_DMESG("Flash write error~~!\n");
    }
    else
    {
        TPD_DMESG("Flash write OK~~!\n");
    }


    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1);
    // clear boot status for download dsp code
    buf[0] = 0x99;
    err = i2c_write_bytes(client, TPD_BOOT_CTL_, buf, 1);

    buf[0] = 0x08;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);

    /////////////////

    kfree(readback_buf);
    return 0;
}
#endif

static int do_download(struct i2c_client *client,
                       unsigned char *gt9006_dsp_isp_firmware,
                       unsigned char *gt9006_master_firmware,
                       unsigned char *gt9006_dsp_firmware,
                       unsigned char *gt9006_bootcode_firmware
                      )
{
    int err;
    u8 buf[1];

#ifdef DOWNLOAD_COMPARE
    int i;
    u8 *readback_buf = kmalloc(TPD_FIRMWARE_SIZE, GFP_KERNEL);
#endif

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_BOOT_CTL_, buf, 1);

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_TMR0_EN, buf, 1);

    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_CACHE_EN, buf, 1);

    buf[0] = 0x0C;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);

    msleep(5);

    buf[0] = 0x02;
    err = i2c_write_bytes(client, TPD_BOOTCTL_B0_, buf, 1);

    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_CPU_SWRST_PULSE, buf, 1);

    msleep(5);

#ifdef DOWNLOAD_COMPARE
download_part1:
#endif
    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);   // switch bank
    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1); // switch bank
    TPD_DMESG("download part1\n");
    err = i2c_write_bytes(client, TPD_MASTER_CODE_REG, gt9006_master_firmware, _FW_FRAME_BYTES);   // write the first bank


#ifdef DOWNLOAD_COMPARE
    err = i2c_read_bytes(client, TPD_MASTER_CODE_REG, readback_buf,
                         _FW_FRAME_BYTES);

    // compare
    for (i = 0 ; i < _FW_FRAME_BYTES ; i++)
    {

        if (gt9006_master_firmware[i] != readback_buf[i])
        {
            TPD_DMESG("[%d] %02X:%02X\n", i, gt9006_master_firmware[i], readback_buf[i]);
            goto download_part1;
        }
    }

#endif

#ifdef DOWNLOAD_COMPARE
download_part2:
#endif
    TPD_DMESG("download part2\n");
    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);   // switch bank
    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1); // switch bank
    err = i2c_write_bytes(client,
                          TPD_MASTER_CODE_REG,
                          &gt9006_master_firmware[_FW_FRAME_BYTES],
                          _FW_FRAME_BYTES);  // write the second bank


#ifdef DOWNLOAD_COMPARE
    err = i2c_read_bytes(client, TPD_MASTER_CODE_REG, readback_buf,
                         _FW_FRAME_BYTES);

    for (i = 0 ; i < _FW_FRAME_BYTES ; i++)
    {
        if (gt9006_master_firmware[i + _FW_FRAME_BYTES] != readback_buf[i])
        {
            TPD_DMESG("[%d] %02X:%02X\n", i + _FW_FRAME_BYTES, gt9006_master_firmware[i], readback_buf[i]);
            goto download_part2;
        }
    }

#endif

#ifdef DOWNLOAD_COMPARE
download_dsp:
#endif
    TPD_DMESG("download dsp\n");
    buf[0] = 0x02;
    err = i2c_write_bytes(client, TPD_SRAM_BANK, buf, 1);   // switch bank
    buf[0] = 0x01;
    err = i2c_write_bytes(client, TPD_MEM_CD_EN, buf, 1); // switch bank

    err = i2c_write_bytes(client,
                          TPD_MASTER_CODE_REG,
                          gt9006_dsp_firmware,
                          _DSP_TOTAL_BYTES); // write the second bank

#ifdef DOWNLOAD_COMPARE
    err = i2c_read_bytes(client, TPD_MASTER_CODE_REG, readback_buf, _DSP_TOTAL_BYTES);

    // compare
    for (i = 0 ; i < _DSP_TOTAL_BYTES ; i++)
    {
        if (gt9006_dsp_firmware[i] != readback_buf[i])
        {
            TPD_DMESG("{%d} %02X:%02X\n", i, gt9006_dsp_firmware[i], readback_buf[i]);
            goto download_dsp;
        }
    }

#endif

    buf[0] = 0x00;//scramble
    err = i2c_write_bytes(client, TPD_BOOT_OPT_B0_, buf, 1);
    buf[0] = 0xAA;
    err = i2c_write_bytes(client, TPD_BOOT_CTL_, buf, 1);
    buf[0] = 0x00;
    err = i2c_write_bytes(client, TPD_SWRST_B0_, buf, 1);
    msleep(10);

#ifdef DOWNLOAD_COMPARE

    kfree(readback_buf);

#endif

    return 0;
}

static void tpd_fw_loading_func(struct work_struct *work)
{
    u8 err;
    TPD_DMESG("fw_loading_func\n");

    TPD_DMESG("TPD f/w download starts at CLK %d~~!!!\n", i2c_clock);
    do_download(i2c_client,
                NULL,
                gt9006_master_firmware + TPD_FW_HEADER_SIZE,
                gt9006_dsp_firmware + TPD_FW_HEADER_SIZE,
                NULL
               );
    i2c_write_bytes(i2c_client, TPD_CONFIG_REG_BASE, gt9006_config, CONFIG_LEN);
    TPD_DMESG("TPD f/w download ends at CLK %d~~!!!\n", i2c_clock);

    msleep(5);

    err = i2c_read_bytes(i2c_client, TPD_VERSION_INFO_REG, (u8 *)&tpd_info, sizeof(struct tpd_info_t));

    if (err)
    {
        TPD_DMESG(TPD_DEVICE " fail to get tpd info %d\n", err);
    }
    else
    {
        TPD_DMESG("TPD info\n");
        TPD_DMESG("vendor %02X %02X\n", tpd_info.vendor_id_1, tpd_info.vendor_id_2);
        TPD_DMESG("product %02X %02X\n", tpd_info.product_id_1, tpd_info.product_id_2);
        TPD_DMESG("version %02X %02X\n", tpd_info.version_1, tpd_info.version_2);
    }


}

#ifdef FIRMWARE_TO_FLASH
static void tpd_fw_to_flash_func(struct work_struct *work)
{
    u8 err;
    TPD_DMESG("fw_to_flash_func\n");

    TPD_DMESG("TPD f/w to flash starts at CLK %d~~!!!\n", i2c_clock);
    do_download_to_flash(i2c_client,
                         gt9006_dsp_isp_firmware,
                         gt9006_master_firmware + TPD_FW_HEADER_SIZE,
                         gt9006_dsp_firmware + TPD_FW_HEADER_SIZE,
                         gt9006_bootcode_firmware
                        );
    i2c_write_bytes(i2c_client, TPD_CONFIG_REG_BASE, gt9006_config, CONFIG_LEN);
    TPD_DMESG("TPD f/w to flash ends at CLK %d~~!!!\n", i2c_clock);

    msleep(5);

    err = i2c_read_bytes(i2c_client, TPD_VERSION_INFO_REG, (u8 *)&tpd_info, sizeof(struct tpd_info_t));

    if (err)
    {
        TPD_DMESG(TPD_DEVICE " fail to get tpd info %d\n", err);
    }
    else
    {
        TPD_DMESG("TPD info\n");
        TPD_DMESG("vendor %02X %02X\n", tpd_info.vendor_id_1, tpd_info.vendor_id_2);
        TPD_DMESG("product %02X %02X\n", tpd_info.product_id_1, tpd_info.product_id_2);
        TPD_DMESG("version %02X %02X\n", tpd_info.version_1, tpd_info.version_2);
    }
}
#endif

static void tpd_fw_select_func(void)
{
#ifdef FIRMWARE_WITH_HEADER
    u8 headerBuf[TPD_FW_HEADER_SIZE];
    int defaultVer;
    int updatedVer;
    int *tmp;
    int fwMask;
    int updatedFwMask;
#else
    u8 buf[1];
    int tmp_firmware_version;
    u8 *master_firmware_buf;
    u8 *dsp_firmware_buf;
#endif

    u8 *config_buf;
    int ret;


#ifdef FIRMWARE_WITH_HEADER
    u8 *master_firmware_buf =  kmalloc(TPD_FIRMWARE_SIZE, GFP_KERNEL);
    u8 *dsp_firmware_buf = kmalloc(TPD_DSP_FIRMWARE_SIZE, GFP_KERNEL);

    cali_data_init();

    defaultVer = ((int)gt9006_master_firmware[1] << 8) + ((int)gt9006_master_firmware[2]);
    tmp = (int *)&gt9006_master_firmware[3];
    fwMask = *tmp;
    TPD_DMESG("defaultVer: %x\n", defaultVer);
    TPD_DMESG("fwMask: %x\n", fwMask);

    ret = nvram_read(TPD_FIRMWARE_FILE_NAME, headerBuf, TPD_FW_HEADER_SIZE, 0);

    if (ret == -1)
    {
        TPD_DMESG("NO FIRMWARE in FS\n");
    }
    else
    {
        updatedVer = ((int)headerBuf[1] << 8) + ((int)headerBuf[2]);
        tmp = (int *)&headerBuf[3];
        updatedFwMask = *tmp;

        if (updatedFwMask == fwMask && updatedVer > defaultVer)
        {
            ret = nvram_read(TPD_FIRMWARE_FILE_NAME, master_firmware_buf, TPD_FIRMWARE_SIZE, 0);

            if (ret == -1)
            {
                TPD_DMESG("FIRMWARE in FS read error\n");
            }
            else
            {
                memcpy(gt9006_master_firmware, master_firmware_buf, TPD_FIRMWARE_SIZE);
            }

        }
    }

    defaultVer = ((int)gt9006_dsp_firmware[1] << 8) + ((int)gt9006_dsp_firmware[2]);
    tmp = (int *)&gt9006_dsp_firmware[3];
    fwMask = *tmp;

    ret = nvram_read(TPD_DSP_FIRMWARE_FILE_NAME, headerBuf, TPD_FW_HEADER_SIZE, 0);

    if (ret == -1)
    {
        TPD_DMESG("NO DSP FIRMWARE in FS\n");
    }
    else
    {
        updatedVer = ((int)headerBuf[1] << 8) + ((int)headerBuf[2]);
        tmp = (int *)&headerBuf[3];
        updatedFwMask = *tmp;

        if (updatedFwMask == fwMask && updatedVer > defaultVer)
        {
            ret = nvram_read(TPD_DSP_FIRMWARE_FILE_NAME, dsp_firmware_buf, TPD_DSP_FIRMWARE_SIZE, 0);

            if (ret == -1)
            {
                TPD_DMESG("DSP FIRMWARE in FS read error\n");
            }
            else
            {
                memcpy(gt9006_dsp_firmware, dsp_firmware_buf, TPD_DSP_FIRMWARE_SIZE);
            }

        }
    }

    tpd_fw_loading_func(NULL);

#else


    cali_data_init();

    master_firmware_buf =  kmalloc(TPD_FIRMWARE_SIZE, GFP_KERNEL);
    dsp_firmware_buf = kmalloc(TPD_DSP_FIRMWARE_SIZE, GFP_KERNEL);

    tpd_fw_loading_func(NULL);

    i2c_read_bytes(i2c_client, TPD_FIRMWARE_VERSION, buf, 2);
    firmware_version = buf[0] + (((int)buf[1]) << 8);
    TPD_DMESG("TPD_FIRMWARE_VERSION %x\n", firmware_version);
    buf[0] = 0;
    buf[1] = 0;
    i2c_write_bytes(i2c_client, TPD_FIRMWARE_VERSION, buf, 2);



    memcpy(master_firmware_buf, gt9006_master_firmware, TPD_FIRMWARE_SIZE);
    memcpy(dsp_firmware_buf, gt9006_dsp_firmware , TPD_DSP_FIRMWARE_SIZE);


    ret = nvram_read(TPD_FIRMWARE_FILE_NAME, gt9006_master_firmware, TPD_FIRMWARE_SIZE, 0);

    if (ret == -1)
    {
        TPD_DMESG("NO FIRMWARE in FS\n");
        memcpy(gt9006_master_firmware, master_firmware_buf, TPD_FIRMWARE_SIZE);
    }
    else
    {
        ret = nvram_read(TPD_DSP_FIRMWARE_FILE_NAME, gt9006_dsp_firmware, TPD_DSP_FIRMWARE_SIZE, 0);

        if (ret == -1)
        {
            TPD_DMESG("NO DSP FIRMWARE in FS\n");
            memcpy(gt9006_dsp_firmware, dsp_firmware_buf , TPD_DSP_FIRMWARE_SIZE);
        }
        else
        {
            TPD_DMESG("Download start \n");
            tpd_fw_loading_func(NULL);

            i2c_read_bytes(i2c_client, TPD_FIRMWARE_VERSION, buf, 2);
            tmp_firmware_version = buf[0] + (((int)buf[1]) << 8);
            TPD_DMESG("NEW TPD_FIRMWARE_VERSION %x\n", firmware_version);

            if (tmp_firmware_version < firmware_version)
            {
                master_firmware = gt9006_master_firmware;
                memcpy(gt9006_master_firmware, master_firmware_buf, TPD_FIRMWARE_SIZE);
                memcpy(gt9006_dsp_firmware, dsp_firmware_buf , TPD_DSP_FIRMWARE_SIZE);
                tpd_fw_loading_func(NULL);
            }
        }
    }

#endif

    config_buf =  kmalloc(CONFIG_LEN , GFP_KERNEL);

    ret = nvram_read(TPD_CONFIG_FILE_NAME, config_buf, CONFIG_LEN, 0);

    if (ret == -1)
    {
        TPD_DMESG("NO CONFIG in FS\n");

    }
    else
    {
        memcpy(gt9006_config, config_buf , CONFIG_LEN);
    }

    kfree(config_buf);
    kfree(master_firmware_buf);
    kfree(dsp_firmware_buf);
}



static int gt9006_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    char *ptr = page;
    char temp_data[CONFIG_LEN] = {0};
    int i;


    ptr += sprintf(ptr, "==== GT9006 config value====\n");

    for (i = 0 ; i < CONFIG_LEN ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", gt9006_config[i]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }



    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9006 config real value read from i2c ====\n");
    i2c_read_bytes(i2c_client, TPD_CONFIG_REG_BASE, temp_data, CONFIG_LEN);

    for (i = 0 ; i < CONFIG_LEN ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");
    *eof = 1;
    return (ptr - page);
}

static int gt9006_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{




    if (count != (CONFIG_LEN))
    {
        TPD_DMESG("size not match [%d:%ld]\n", CONFIG_LEN, count);
        return -EFAULT;
    }



    if (copy_from_user(gt9006_config, buffer, CONFIG_LEN))
    {
        TPD_DEBUG("copy from user fail\n");
        return -EFAULT;
    }



    i2c_write_bytes(i2c_client, TPD_CONFIG_REG_BASE, gt9006_config, CONFIG_LEN);


    return count;
}

static int gt9006_command_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{

    char input_data[10];

    if (copy_from_user(input_data, buffer, count))
    {

        TPD_DMESG("copy from user fail\n");
        return -EFAULT;
    }

    TPD_DMESG("%s\n", input_data);

#ifdef TPD_FLASHLESS_MODE

    if (strncmp(input_data, "INIT_START", 10) == 0)
    {
        TPD_DMESG("INIT_START\n");
        tpd_fw_select_func();
        tpd_fw_loading_func(NULL);
        clock_boost(tpd_cali.clk_cali);//probe boost clock with default value, so do it again

    }

#endif

    if (strncmp(input_data, "FW_SELECT", 9) == 0)
    {
        TPD_DMESG("FW_SELECT\n");
        tpd_fw_select_func();
        tpd_fw_loading_func(NULL);
    }

#ifdef FIRMWARE_TO_FLASH

    if (strncmp(input_data, "FW_TO_FLASH", 11) == 0)
    {
        TPD_DMESG("FW_TO_FLASH\n");
        tpd_fw_select_func();
        tpd_fw_to_flash_func(NULL);
    }

#endif

    return count;
}

static int gt9006_command_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    char *ptr = page;
    return (ptr - page);
}



static int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buffer[I2C_DEVICE_ADDRESS_LEN];
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG) | (I2C_PUSHPULL_FLAG)),
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .flags = 0,
            .buf = buffer,
            .len = I2C_DEVICE_ADDRESS_LEN,
            .timing = i2c_clock
        },
        {
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG) | (I2C_PUSHPULL_FLAG)),
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .flags = I2C_M_RD,
            .timing = i2c_clock
        },
    };


    if (rxbuf == NULL)
        return -1;

    TPD_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;
        msg[1].buf = &rxbuf[offset];

        if (left > MAX_READ_TRANSACTION_LENGTH)
        {
            msg[1].len = MAX_READ_TRANSACTION_LENGTH;
            left -= MAX_READ_TRANSACTION_LENGTH;
            offset += MAX_READ_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while (i2c_transfer(client->adapter, &msg[0], 2) != 2)
        {
            retry++;

            if (retry == 20)
            {
                TPD_DEBUG("I2C read 0x%X length=%d failed\n", addr + offset, len);

                return -1;
            }
        }
    }

    return 0;
}

static int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG) | (I2C_PUSHPULL_FLAG)),
        //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
        .flags = 0,
        .buf = buffer,
        .timing = i2c_clock,
    };

    if (txbuf == NULL)
        return -1;

    if (len == 1)
    {
        TPD_DEBUG("i2c_write_byte to device %02X address %04X value %X\n", client->addr, addr, txbuf[0]);
    }
    else
    {
        TPD_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);
    }

    while (left > 0)
    {
        retry = 0;

        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;


        if (left > MAX_I2C_TRANSFER_SIZE)
        {
            memcpy(&buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy(&buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left);
            msg.len = left + I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }


        //TPD_DEBUG("byte left %d offset %d\n", left, offset);

        while (i2c_transfer(client->adapter, &msg, 1) != 1)
        {
            retry++;

            if (retry == 20)
            {
                TPD_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);

                return -1;
            }
            else
                TPD_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);
        }
    }

    return 0;
}




static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, "mtk-tpd");
    return 0;
}




static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    int err;
#ifdef TPD_HAVE_TOUCH_KEY
    int idx;
#endif

    TPD_DMESG(TPD_DEVICE " tpd_i2c_probe\n");

    tpd_power_on_init();

    tpd_set_i2c_clock(I2C_INIT_MASTER_CLOCK);

    err = i2c_read_bytes(client, TPD_VERSION_INFO_REG, (u8 *)&tpd_info, sizeof(struct tpd_info_t));

    if (err)
    {
        TPD_DMESG(TPD_DEVICE " fail to get tpd info %d\n", err);
        return err;
    }
    else
    {
#ifndef TPD_FLASHLESS_MODE
        TPD_DMESG("TPD info\n");
        TPD_DMESG("vendor %02X %02X\n", tpd_info.vendor_id_1, tpd_info.vendor_id_2);
        TPD_DMESG("product %02X %02X\n", tpd_info.product_id_1, tpd_info.product_id_2);
        TPD_DMESG("version %02X %02X\n", tpd_info.version_1, tpd_info.version_2);
#endif
    }

    i2c_client = client;
    tpd_set_i2c_clock(I2C_INIT_MASTER_CLOCK);
    clock_boost(tpd_cali.clk_cali);
    tpd_set_i2c_clock(I2C_MASTER_CLOCK);

#ifndef TPD_FLASHLESS_MODE
    i2c_write_bytes(i2c_client, TPD_CONFIG_REG_BASE, gt9006_config, CONFIG_LEN);
#endif

    gt9006_config_proc = create_proc_entry(TPD_CONFIG_PROC_FILE, 0666, NULL);

    if (gt9006_config_proc == NULL)
    {
        TPD_DEBUG("create_proc_entry %s failed\n", TPD_CONFIG_PROC_FILE);
    }
    else
    {
        gt9006_config_proc->read_proc = gt9006_config_read_proc;
        gt9006_config_proc->write_proc = gt9006_config_write_proc;
    }

    gt9006_command_proc = create_proc_entry(TPD_COMMAND_PROC_FILE, 0666, NULL);

    if (gt9006_command_proc == NULL)
    {
        TPD_DEBUG("create_proc_entry %s failed\n", TPD_COMMAND_PROC_FILE);
    }
    else
    {
        gt9006_command_proc->read_proc = gt9006_command_read_proc;
        gt9006_command_proc->write_proc = gt9006_command_write_proc;
    }

    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if (IS_ERR(thread))
    {
        err = PTR_ERR(thread);
        TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }

#ifdef TPD_HAVE_TOUCH_KEY

    for (idx = 0; idx < TPD_TOUCH_KEY_NUM; idx++)
    {
        input_set_capability(tpd->dev, EV_KEY, touch_key_array[idx]);
    }

#endif

#ifdef TPD_FLASHLESS_MODE
    INIT_WORK(&tpd_fw_loading_work, tpd_fw_loading_func);
    INIT_WORK(&tpd_nvram_read_work, tpd_nvram_read);
    INIT_WORK(&tpd_nvram_write_work, tpd_nvram_write);
#endif


    tpd_setup_eint();

    tpd_load_status = 1;
    return 0;
}


void tpd_eint_interrupt_handler(void)
{
    TPD_DEBUG_PRINT_INT;
    tpd_flag = 1;
    wake_up_interruptible(&tpd_waiter);
}


static int tpd_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static void tpd_down(int x, int y, int size, int id)
{
    if ((!size) && (!id))
    {
        input_report_abs(tpd->dev, ABS_PRESSURE, 1);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    }
    else
    {
        input_report_abs(tpd->dev, ABS_PRESSURE, size / 100);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size / 100);
        /* track id Start 0 */
        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id - 1);
    }

    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id - 1, 1);

    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        TPD_DEBUG("tpd_button~~~~!!!\n");
        tpd_button(x, y, 1);
    }
}

static void tpd_up(int x, int y, int id)
{
    input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id, 0);

    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 0);
    }
}

static int touch_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    int x, y, id, size, finger_num = 0;
    static u8 buffer[ TPD_POINT_INFO_LEN * TPD_MAX_POINTS ];
    static char buf_status;
    int wrap_x, wrap_y = 0;
    int idx;
    static u8 last_finger_num = 0;
    u8  key = 0;



#if (defined(TPD_HAVE_TOUCH_KEY) || defined(TPD_HAVE_BUTTON))
    u8 key1 = 0, key2 = 0, key3 = 0, key4 = 0;
    static u8 key1_old = 0, key2_old = 0, key3_old = 0, key4_old = 0;
    static u8  last_key = 0;
    unsigned int  count = 0;
#endif

    sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);

        while (tpd_halt)
        {
            tpd_flag = 0;
            msleep(20);
        }

        wait_event_interruptible(tpd_waiter, tpd_flag != 0);
        tpd_flag = 0;
        TPD_DEBUG_SET_TIME;
        set_current_state(TASK_RUNNING);

        i2c_read_bytes(i2c_client, TPD_TOUCH_INFO_REG_BASE, buffer, 1);
        TPD_DEBUG("[mtk-tpd] STATUS : %x\n", buffer[0]);


        finger_num = buffer[0] & 0x0f;
        buf_status = buffer[0] & 0xf0;

        if (tpd == NULL || tpd->dev == NULL)
        {
            TPD_DMESG("tpd NULL\n");
            buffer[0] = 0;
            i2c_write_bytes(i2c_client, TPD_TOUCH_INFO_REG_BASE, buffer, 1);
            continue;
        }

        if ((buf_status & 0x80) == 0x00) // buffer not ready
        {
            TPD_DMESG("Buffer not ready\n");
            continue;
        }

        if (finger_num == 0x0d)
        {
            schedule_work(&tpd_nvram_write_work);
            buffer[0] = 0;
            i2c_write_bytes(i2c_client, TPD_TOUCH_INFO_REG_BASE, buffer, 1);
            continue;
        }
        else if (finger_num == 0x0e)
        {
            schedule_work(&tpd_nvram_read_work);
            buffer[0] = 0;
            i2c_write_bytes(i2c_client, TPD_TOUCH_INFO_REG_BASE, buffer, 1);
            continue;
        }
        else if (finger_num > 10)          //abnormal state so return
        {
            TPD_DMESG("abnormal finger num = %d\n", finger_num);
            buffer[0] = 0;
            i2c_write_bytes(i2c_client, TPD_TOUCH_INFO_REG_BASE, buffer, 1);
            continue;
        }

        if (finger_num)
        {
            i2c_read_bytes(i2c_client, TPD_POINT_INFO_REG_BASE, buffer, finger_num * TPD_POINT_INFO_LEN);

        }



#if (defined(TPD_HAVE_TOUCH_KEY) || defined(TPD_HAVE_BUTTON))

        if ((buffer[0] & 0x10) != 0)
        {
            i2c_read_bytes(i2c_client, TPD_POINT_INFO_REG_BASE + finger_num * TPD_POINT_INFO_LEN, &key, 1);
            TPD_DEBUG("[mtk-tpd] KEY STATUS : %d\n", key);
            key = key & 0x0f;
        }
        else
        {
            key = 0;
        }

#ifdef TPD_HAVE_TOUCH_KEY

        if (last_key || key)
        {
            for (idx = 0; idx < TPD_TOUCH_KEY_NUM; idx++)
            {
                input_report_key(tpd->dev, touch_key_array[idx], !!(key & (0x01 << idx)));
                TPD_DEBUG("input_report_key key: %X value : %X\n", touch_key_array[idx], !!(key & (0x01 << idx)));
            }
        }

        last_key = key;
#else
        key1 = (key & 0x01);
        key2 = (key & 0x02);
        key3 = (key & 0x04);
        key4 = (key & 0x08);

        if (key1 == 1)
        {
            tpd_down(key_1, 0, 0);
        }
        else if ((key1_old == 1) & (key1 == 0))
        {
            tpd_up(key_1, 0);
        }

        if (key2 == 2)
        {
            tpd_down(key_2, 0, 0);
        }
        else if ((key2_old == 2) & (key2 == 0))
        {
            tpd_up(key_2, 0);
        }

        if (key3 == 4)
        {
            tpd_down(key_3, 0, 0);
        }
        else if ((key3_old == 4) & (key3 == 0))
        {
            tpd_up(key_3, 0);
        }

        if (key4 == 8)
        {
            tpd_down(key_4, 0, 0);
        }
        else if ((key4_old == 8) & (key4 == 0))
        {
            tpd_up(key_4, 0);
        }

        key1_old = key1;
        key2_old = key2;
        key3_old = key3;
        key4_old = key4;

#endif
#endif

        for (idx = 0 ; idx < finger_num ; idx++)
        {
            u8 *ptr = &buffer[ idx * TPD_POINT_INFO_LEN ];
            id = ptr[0];

            if (id < TPD_MAX_POINTS + 1)
            {
                y = ptr[1] + (((int)ptr[2]) << 8);
                x = ptr[3] + (((int)ptr[4]) << 8);
                size = ptr[5] + (((int)ptr[6]) << 8);

                wrap_x = TPD_WARP_X(x);
                wrap_y = TPD_WARP_Y(y);

                tpd_down(wrap_x, wrap_y, size, id);

                TPD_DEBUG("id: %3d x: %3d y: %3d \n", id, x, y);
            }
            else
            {
                TPD_DEBUG("Invalid id %d\n", id);
            }
        }

        if ((!finger_num) && (!key) && (last_finger_num))
        {
            tpd_up(0, 0, 0);
        }

        last_finger_num = finger_num;

        if (tpd != NULL && tpd->dev != NULL)
        {
            input_sync(tpd->dev);
        }

        buffer[0] = 0;
        i2c_write_bytes(i2c_client, TPD_TOUCH_INFO_REG_BASE, buffer, 1);
    }
    while (!kthread_should_stop());

    return 0;
}

static int tpd_local_init(void)
{

    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        TPD_DMESG("unable to add i2c driver.\n");
        return -1;
    }

    if (tpd_load_status == 0)
    {
        TPD_DMESG("add error touch panel driver.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

    // set vendor string
    tpd->dev->id.vendor = (tpd_info.vendor_id_2 << 8) | tpd_info.vendor_id_1;
    tpd->dev->id.product = (tpd_info.product_id_2 << 8) | tpd_info.product_id_1;
    tpd->dev->id.version = (tpd_info.version_2 << 8) | tpd_info.version_1;



    set_bit(ABS_MT_TRACKING_ID, tpd->dev->absbit);

    tpd_type_cap = 1;

    return 0;
}



/* Function to manage low power suspend */
//void tpd_suspend(struct i2c_client *client, pm_message_t message)
static void tpd_suspend(struct early_suspend *h)
{
    tpd_halt = 1;
    msleep(1);
#ifdef TPD_SUSPEND_POWER_OFF_MODE

    if (power_off_mode)
    {
        //power off
        TPD_DMESG("TPD power off\n");
        hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
        hwPowerDown(MT65XX_POWER_LDO_VGP, "TP");

    }

#endif
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
}

/* Function to manage power-on resume */
//void tpd_resume(struct i2c_client *client)
static void tpd_resume(struct early_suspend *h)
{
    TPD_DMESG(" tpd_resume start \n");
    tpd_halt = 0;

#ifdef TPD_SUSPEND_POWER_OFF_MODE
    tpd_power_on_init();
    //boost tpd clock
    tpd_set_i2c_clock(I2C_INIT_MASTER_CLOCK);
    clock_boost(tpd_cali.clk_cali);
    tpd_set_i2c_clock(I2C_MASTER_CLOCK);
#endif

#ifdef  TPD_FLASHLESS_MODE

    if (power_off_mode)
    {
        schedule_work(&tpd_fw_loading_work);
    }

#endif

#ifndef TPD_FLASHLESS_MODE
    i2c_write_bytes(i2c_client, TPD_CONFIG_REG_BASE, gt9006_config, CONFIG_LEN);
#endif

    tpd_setup_eint();
    //mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    TPD_DMESG(" tpd_resume end \n");
}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "gt9006",
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
static int __init tpd_driver_init(void)
{
    TPD_DMESG("MediaTek gt9006 touch panel driver init\n");

    i2c_register_board_info(0, &i2c_tpd, 1);

    if (tpd_driver_add(&tpd_device_driver) < 0)
        TPD_DMESG("add generic driver failed\n");

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    TPD_DMESG("MediaTek gt9006 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


MODULE_LICENSE("GPL");
