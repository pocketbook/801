#ifndef __MC13892_CHECK_BATTERY_H_
#define __MC13892_CHECK_BATTERY_H_


#if defined(CONFIG_IMX_CSPI)
#include <imx_spi.h>
#include <asm/arch/imx_spi_pmic.h>

// check_battery for MC13892

#define MC13892_REG_ISENSE0			2
#define MC13892_REG_INT_STATUS1             3
#define MC13892_REG_POWER_CTL0		13
#define MC13892_REG_POWER_CTL2            15
#define MC13892_REG_24				24
#define MC13892_REG_25				25
#define MC13892_REG_SETTING_0               30
#define MC13892_REG_MODE_0            32
#define MC13892_REG_MODE_1            33
#define MC13892_REG_ADC0			43
#define MC13892_REG_ADC1			44
#define MC13892_REG_ADC2			45
#define MC13892_REG_ADC3			46
#define MC13892_REG_ADC4			47
#define MC13892_REG_CHARGE0			48

#define MC13892_ISENSE0_VBUSVALIDS	(1<<3)	
#define MC13892_ISENSE0_CHGDETS		(1<<6)	
#define MC13892_ISENSE0_CHGCURRS	(1<<11)
#define IsChargerPresence(a)	( ((a) & (MC13892_ISENSE0_VBUSVALIDS|MC13892_ISENSE0_CHGDETS)) ==  \
								  (MC13892_ISENSE0_VBUSVALIDS|MC13892_ISENSE0_CHGDETS) )

#define MC13892_ADC0_CHRGICON_MASK		(1<<1)
#define MC13892_ADC0_BATICON_MASK		(1<<2)
#define MC13892_ADC0_ADRESET		(1 << 8)
#define MC13892_ADC0_ADREFEN		(1 << 10)
#define MC13892_ADC0_ADREFMODE		(1 << 11)
#define MC13892_ADC0_TSMOD0		(1 << 12)
#define MC13892_ADC0_TSMOD1		(1 << 13)
#define MC13892_ADC0_TSMOD2		(1 << 14)
#define MC13892_ADC0_ADINC1		(1 << 16)
#define MC13892_ADC0_ADINC2		(1 << 17)

#define MC13892_ADC0_TSMOD_MASK		(MC13892_ADC0_TSMOD0 | \
					MC13892_ADC0_TSMOD1 | \
					MC13892_ADC0_TSMOD2)


#define MC13892_ADC1_ADEN		(1 << 0)
#define MC13892_ADC1_RAND		(1 << 1)
#define MC13892_ADC1_ADCCAL		(1 << 2)
#define MC13892_ADC1_ADSEL		(1 << 3)
#define MC13892_ADC1_ASC		(1 << 20)
#define MC13892_ADC1_ADTRIGIGN		(1 << 21)
#define MC13892_ADC1_DEFAULT_ATO	(0x080800)

#define MC13892_ADC1_ADA1_SHITT      5
#define MC13892_ADC1_ADA1_MASK       0xE0
#define MC13892_ADC1_ADA2_SHITT      8
#define MC13892_ADC1_ADA2_MASK       0x700								

#define MC13892_ADC2_ADD1_SHIFT		    2
#define MC13892_ADC2_ADD1_MASK		    0x3FF
#define MC13892_ADC2_ADD2_SHIFT		    14
#define MC13892_ADC2_ADD2_MASK		    0x3FF


#define MC13892_CHARGE0_ACKLPB			(1<<8)
#define MC13892_CHARGE0_CHGAUTOVIB		(1<<23)
#define MC13892_CHARGE0_ICHRG_SHIFT		3
#define MC13892_CHARGE0_ICHRG_MASK		0x78
#define MC13892_CHARGE0_PLIM_SHIFT		(15)

#define MC13892_POWER_CTL0_CLK32KMCUEN  (1<<6)
#define MC13892_POWER_CTL0_DRM  (1<<4)
#define MC13892_REG_INT_STATUS1_TODAI  (1<<1)


#define  TIMEOUT_NUM   360/* 360*/
#define VBATT_SAFE_BOOT_VOLT      305/*320 3.6V*/
#define VBATT_CURRENT_MA_OK            150  /*150,300 exit trickle charge*/
#define  VBATT_TRICKLE_POWER_VOLT    300/*3.0V*/
#define CHARGER_VOLT   45 /*4.5V*/
#define ADC_RETRIES		5


void check_battery(void);

#endif

#endif
