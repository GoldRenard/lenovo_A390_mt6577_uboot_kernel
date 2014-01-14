/*****************************************************************************
*
* Filename:
* ---------
*   bq24158.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq24158 header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _bq24158_SW_H_
#define _bq24158_SW_H_

//#define HIGH_BATTERY_VOLTAGE_SUPPORT   //mtk71259 20120525

#define bq24158_CON0      0x00
#define bq24158_CON1      0x01
#define bq24158_CON2      0x02
#define bq24158_CON3      0x03
#define bq24158_CON4      0x04
#define bq24158_CON5      0x05
#define bq24158_CON6      0x06

/**********************************************************
  *
  *   [MASK/SHIFT] 
  *
  *********************************************************/
//CON0
#define CON0_TMR_RST_MASK 	0x01
#define CON0_TMR_RST_SHIFT 	7

//debug  no use
#define CON0_SLRST_MASK 	0x01
#define CON0_SLRST_SHIFT 	7

#define CON0_EN_STAT_MASK 	0x01
#define CON0_EN_STAT_SHIFT 	6

#define CON0_STAT_MASK 		0x03
#define CON0_STAT_SHIFT 	4

#define CON0_FAULT_MASK 	0x07
#define CON0_FAULT_SHIFT 	0

//CON1
#define CON1_LIN_LIMIT_MASK 	0x03
#define CON1_LIN_LIMIT_SHIFT 	6

#define CON1_LOW_V_2_MASK 	0x01
#define CON1_LOW_V_2_SHIFT 	5

#define CON1_LOW_V_1_MASK 	0x01
#define CON1_LOW_V_1_SHIFT 	4

#define CON1_TE_MASK 	0x01
#define CON1_TE_SHIFT 	3

#define CON1_CE_MASK 	0x01
#define CON1_CE_SHIFT 	2

#define CON1_HZ_MODE_MASK 	0x01
#define CON1_HZ_MODE_SHIFT 	1

#define CON1_OPA_MODE_MASK 	0x01
#define CON1_OPA_MODE_SHIFT 0

//CON2
#define CON2_CV_VTH_MASK 	0x3F
#define CON2_CV_VTH_SHIFT 	2

#define CON2_OTG_PL_MASK 	0x01
#define CON2_OTG_PL_SHIFT 	1

#define CON2_OTG_EN_MASK 	0x01
#define CON2_OTG_EN_SHIFT 	0

//CON3
#define CON3_VENDER_CODE_MASK 	0x07
#define CON3_VENDER_CODE_SHIFT 	5

#define CON3_PIN_MASK 	0x03
#define CON3_PIN_SHIFT 	3

#define CON3_REVISION_MASK 		0x07
#define CON3_REVISION_SHIFT 	0

//CON4
#define CON4_RESET_MASK		0x01
#define CON4_RESET_SHIFT 	7

//#define CON4_I_CHR_MASK		0x0F
//#define CON4_I_CHR_SHIFT 	3


#define CON4_I_CHR_MASK		0x07
#define CON4_I_CHR_SHIFT 	4

#define CON4_I_TERM_MASK	0x07
#define CON4_I_TERM_SHIFT 	0

//CON5
#define CON5_LOW_CHG_MASK	0x01
#define CON5_LOW_CHG_SHIFT 	5

#define CON5_DPM_STATUS_MASK	0x01
#define CON5_DPM_STATUS_SHIFT 	4

#define CON5_CD_STATUS_MASK		0x01
#define CON5_CD_STATUS_SHIFT 	3

#define CON5_VSREG_MASK		0x07
#define CON5_VSREG_SHIFT 	0

//CON6
//#define CON6_MCHRG_MASK		0x0F
//#define CON6_MCHRG_SHIFT 	4

#define CON6_MCHRG_MASK		0x07
#define CON6_MCHRG_SHIFT 	3

#define CON6_MREG_MASK		0x0F
#define CON6_MREG_SHIFT 	0

/**********************************************************
  *
  *   [Extern Function] 
  *
  *********************************************************/
//CON0
extern void bq24158_set_tmr_rst(unsigned int val);
extern unsigned int bq24158_get_slrst_status(void);
extern void bq24158_set_en_stat(unsigned int val);
extern unsigned int bq24158_get_chip_status(void);
extern unsigned int bq24158_get_fault_reason(void);

//CON1
extern void bq24158_set_lin_limit(unsigned int val);
extern void bq24158_set_lowv_2(unsigned int val);
extern void bq24158_set_lowv_1(unsigned int val);
extern void bq24158_set_te(unsigned int val);
extern void bq24158_set_ce(unsigned int val);
extern void bq24158_set_hz_mode(unsigned int val);
extern void bq24158_set_opa_mode(unsigned int val);

//CON2
extern void bq24158_set_cv_vth(unsigned int val);
extern void bq24158_set_otg_pl(unsigned int val);
extern void bq24158_set_otg_en(unsigned int val);

//CON3
extern unsigned int bq24158_get_vender_code(void);
extern unsigned int bq24158_get_pin(void);
extern unsigned int bq24158_get_revision(void);

//CON4
extern void bq24158_set_reset(unsigned int val);
extern void bq24158_set_ac_charging_current(unsigned int val);
extern void bq24158_set_termination_current(unsigned int val);

//CON5
extern void bq24158_set_low_chg(unsigned int val);
extern unsigned int bq24158_get_dpm_status(void);
extern unsigned int bq24158_get_cd_status(void);
extern void bq24158_set_vsreg(unsigned int val);

//CON6
extern void bq24158_set_mchrg(unsigned int val);
extern void bq24158_set_mreg(unsigned int val);

extern void bq24158_dump_register(void);

extern unsigned int  bq24158_config_interface_reg (unsigned char RegNum, unsigned char val);
#endif // _bq24158_SW_H_

