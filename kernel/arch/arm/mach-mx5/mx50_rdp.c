/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
    
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
#include <linux/mfd/pm_sensor_lm75a.h>
#endif
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/videodev2.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/gpmi-nfc.h>
#include <linux/powerkey.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/flash.h>
#include <asm/mach/keypad.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <mach/arc_otg.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx50.h>
#include <mach/i2c.h>
#include <mach/check_fuse.h>
#include <linux/gpio_keys.h>


#include "devices.h"
#include "usb.h"
#include "crm_regs.h"
#include "dma-apbh.h"
#include"mx50_io_cfg.h"


extern int __init mx50_rdp_init_mc13892(void);

extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
extern struct dvfs_wp *(*get_dvfs_core_wp)(int *wp);
extern int lcdif_sel_lcd;
extern int lcd_seiko_on_j12;
extern void __iomem *apll_base;

extern int xrz_wifi_open(void);
extern int xrz_wifi_close(void);
extern int xrz_3g_open(void);
extern int xrz_3g_close(void);
extern int skymedi_open_card();

static void mx50_suspend_enter(void);
static void mx50_suspend_exit(void);
static int mx50_rdp_edpc_set_3v3_voltage(int ivalue);
static void fec_gpio_iomux_init(void);
static void fec_gpio_iomux_deinit(void);
static int mx50_rdp_sdcard_control(bool enable);
int air_touch_gpio_suspendresume_control(bool enable);
int  mx50_rdp_light_control(bool bOpen);
//static int  mx50_rdp_backlight_control(bool bOpen);
void  mx50_rdp_backlight_control(bool bOpen);
int  mx50_rdp_tp_power_control(bool bOpen);

#if  defined(CONFIG_MFD_PM_SENSOR_LM75A)
static int pm_sensor_lm75a_init(struct pm_sensor *pm_sensor);
#endif
static int num_cpu_wp;

/*
xrz suspend mode
1.normal state: (iXrzSuspendMode == 0)
          (1).press the power button enter the suspend mode;
          (2).when screen timeout off,enter the suspend mode;
          (3).we can use the power button,the gpio button and usb detect to wakeup
            the device.
2.ebookReading state:(iXrzSuspendMode == 1)
	  (1).in reading ebook,we can set the device into the mode,
	     in this mode,we can use the touch irq ,touch button irq to wakeup the device,
	     and it can't update the screen
 */
int iXrzSuspendMode =1;

static iomux_v3_cfg_t mx50_rdp[] = {
	/* SD1 */
	MX50_PAD_ECSPI2_SS0__GPIO_4_19,
	MX50_PAD_EIM_CRE__GPIO_1_27,
	MX50_PAD_SD1_CMD__SD1_CMD,

	MX50_PAD_SD1_CLK__SD1_CLK,
	MX50_PAD_SD1_D0__SD1_D0,
	MX50_PAD_SD1_D1__SD1_D1,
	MX50_PAD_SD1_D2__SD1_D2,
	MX50_PAD_SD1_D3__SD1_D3,

	/* SD2 --wen we only support sd1 and sd3*/
	/*MX50_PAD_SD2_CD__GPIO_5_17,
	MX50_PAD_SD2_WP__GPIO_5_16,
	MX50_PAD_SD2_CMD__SD2_CMD,
	MX50_PAD_SD2_CLK__SD2_CLK,
	MX50_PAD_SD2_D0__SD2_D0,
	MX50_PAD_SD2_D1__SD2_D1,
	MX50_PAD_SD2_D2__SD2_D2,
	MX50_PAD_SD2_D3__SD2_D3,
	MX50_PAD_SD2_D4__SD2_D4,
	MX50_PAD_SD2_D5__GPIO_5_13,
	MX50_PAD_SD2_D6__GPIO_5_14,
	MX50_PAD_SD2_D7__SD2_D7,*/


	/* SD3 */
	MX50_PAD_SD3_CMD__SD3_CMD,
	MX50_PAD_SD3_CLK__SD3_CLK,
	MX50_PAD_SD3_D0__SD3_D0,
	MX50_PAD_SD3_D1__SD3_D1,
	MX50_PAD_SD3_D2__SD3_D2,
	MX50_PAD_SD3_D3__SD3_D3,
	MX50_PAD_SD3_D4__SD3_D4,
	MX50_PAD_SD3_D5__SD3_D5,
	MX50_PAD_SD3_D6__SD3_D6,
	MX50_PAD_SD3_D7__SD3_D7,
	////////////////////////////////////////////////
	//use for codec SGTL5000 GPIO INIT
	/////////////////////////////////////////////////////
#if defined(CONFIG_SND_SOC_SGTL5000)
	MX50_PAD_SSI_RXD__SSI_RXD,
	MX50_PAD_SSI_TXD__SSI_TXD,
	MX50_PAD_SSI_TXC__SSI_TXC,
	MX50_PAD_SSI_TXFS__SSI_TXFS,

	/* HP_DET_B (headphone detect) */
	MX50_PAD_ECSPI1_SS0__GPIO_4_15,

	// wen gpio6_26..codec clock
	MX50_PAD_OWIRE__SSI_EXT1_CLK,
	//MX50_PAD_EPITO__SSI_EXT2_CLK,

#endif

        /// TP and ata5701 power control
        //only use for c036
       // MX50_PAD_CSPI_MISO__GPIO_4_10,
       MX50_PAD_KEY_COL0__GPIO_4_0,

       ////light control
       MX50_PAD_KEY_COL2__GPIO_4_4,
       ///backlight enable or disable control
      // MX50_PAD_EPDC_PWRSTAT__GPIO_3_28,
       MX50_PAD_DISP_D4__GPIO_2_4,
      //charger gpio control,when battary voltage < 3.6v,set to high
      MX50_PAD_KEY_COL3__GPIO_4_6,

#if defined (CONFIG_LEDS_GPIO_PLATFORM)
	MX50_PAD_KEY_COL2__GPIO_4_4,
#endif
	/* UART pad setting */
	MX50_PAD_UART1_TXD__UART1_TXD,
	MX50_PAD_UART1_RXD__UART1_RXD,
	MX50_PAD_UART1_RTS__UART1_RTS,
	MX50_PAD_UART2_TXD__UART2_TXD,
	MX50_PAD_UART2_RXD__UART2_RXD,
	MX50_PAD_UART2_CTS__UART2_CTS,
	MX50_PAD_UART2_RTS__UART2_RTS,


	MX50_PAD_I2C1_SCL__I2C1_SCL,
	MX50_PAD_I2C1_SDA__I2C1_SDA,

	MX50_PAD_I2C2_SCL__I2C2_SCL,
	MX50_PAD_I2C2_SDA__I2C2_SDA,

	/*I2C3 */
	MX50_PAD_I2C3_SCL__I2C3_SCL,
	MX50_PAD_I2C3_SDA__I2C3_SDA,

	///////////////////////////////////////
	///// elink screen init
	////////////////////////////////////////
#if defined(CONFIG_E_INK_EDPC)	
	/* EPDC pins */
	MX50_PAD_EPDC_D0__EPDC_D0,
	MX50_PAD_EPDC_D1__EPDC_D1,
	MX50_PAD_EPDC_D2__EPDC_D2,
	MX50_PAD_EPDC_D3__EPDC_D3,
	MX50_PAD_EPDC_D4__EPDC_D4,
	MX50_PAD_EPDC_D5__EPDC_D5,
	MX50_PAD_EPDC_D6__EPDC_D6,
	MX50_PAD_EPDC_D7__EPDC_D7,
	MX50_PAD_EPDC_D8__EPDC_D8,
	MX50_PAD_EPDC_D9__EPDC_D9,
	MX50_PAD_EPDC_D10__EPDC_D10,
	MX50_PAD_EPDC_D11__EPDC_D11,
	MX50_PAD_EPDC_D12__EPDC_D12,
	MX50_PAD_EPDC_D13__EPDC_D13,
	MX50_PAD_EPDC_D14__EPDC_D14,
	MX50_PAD_EPDC_D15__EPDC_D15,
	MX50_PAD_EPDC_GDCLK__EPDC_GDCLK,
	MX50_PAD_EPDC_GDSP__EPDC_GDSP,
	MX50_PAD_EPDC_GDOE__EPDC_GDOE	,
	/*hardware don't connect*/
	//MX50_PAD_EPDC_GDRL__EPDC_GDRL,
	MX50_PAD_EPDC_SDCLK__EPDC_SDCLK,
	MX50_PAD_EPDC_SDOE__EPDC_SDOE,
	MX50_PAD_EPDC_SDLE__EPDC_SDLE,
	MX50_PAD_EPDC_SDCE0__EPDC_SDCE0,
	//MX50_PAD_EPDC_SDCE1__EPDC_SDCE1,
	//MX50_PAD_EPDC_SDCE2__EPDC_SDCE2,

	MX50_PAD_EPDC_BDR0__EPDC_BDR0,
	MX50_PAD_EPDC_BDR1__EPDC_BDR1,


	/* EPD PMIC WAKEUP */
	MX50_PAD_UART4_TXD__GPIO_6_16,

	/* EPD PMIC intr */
	MX50_PAD_UART4_RXD__GPIO_6_17,

	/* EPD PMIC powerup0 */
	MX50_PAD_EPDC_PWRCTRL0__GPIO_3_29,
	/* EPD PMIC powerup1*/
	MX50_PAD_EPDC_PWRCTRL1__GPIO_3_30,
	/* EPD PMIC powerup2 */
	MX50_PAD_EPDC_PWRCTRL2__GPIO_3_31,
#endif

	///////////////////////////////////
	/////	 EPDC input pins 
	///////////////////////////////////
#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
	MX50_PAD_EPDC_VCOM0__GPIO_4_21,	
#endif 

	///////////////////////////////////
	/////encrypt  init
	///////////////////////////////////
#if defined(CONFIG_MXC_ENCRYPT_GPIO)
	/*Simulation of I2C :SCL and SDA*/
	MX50_PAD_UART1_CTS__GPIO_6_8,
	MX50_PAD_EPITO__GPIO_6_27,
#endif
	/////////////////////////////////////
	////Keypad  and GPIO button define start
	//////////////////////////////////////

	/*use as KEY_DOWN button*/
	MX50_PAD_DISP_RS__GPIO_2_17,
	/*use as KEY_BACK */
	MX50_PAD_DISP_D12__GPIO_2_12,
	/*KEY_MENU button*/
	MX50_PAD_UART3_TXD__GPIO_6_14,	
	/*KEY_UP button init*/ 
	MX50_PAD_DISP_D15__GPIO_2_15,
	//////////////////////////////////
	///keypad and GPIO button define end
	/////////////////////////////////

	///////////////////////////////////
	/////EIM splash init
	///////////////////////////////////
	/*EIM pads */
	MX50_PAD_EIM_DA8__GPIO_1_8,
	MX50_PAD_EIM_DA9__GPIO_1_9,
	MX50_PAD_EIM_DA10__GPIO_1_10,
	MX50_PAD_EIM_DA11__GPIO_1_11,
	MX50_PAD_EIM_DA12__GPIO_1_12,
	MX50_PAD_EIM_DA13__GPIO_1_13,
	MX50_PAD_EIM_DA14__GPIO_1_14,
	MX50_PAD_EIM_DA15__GPIO_1_15,
	MX50_PAD_EIM_CS2__GPIO_1_16,
	MX50_PAD_EIM_CS1__GPIO_1_17,
	MX50_PAD_EIM_CS0__GPIO_1_18,
	MX50_PAD_EIM_EB0__GPIO_1_19,
	MX50_PAD_EIM_EB1__GPIO_1_20,
	MX50_PAD_EIM_WAIT__GPIO_1_21,
	MX50_PAD_EIM_BCLK__GPIO_1_22,
	MX50_PAD_EIM_RDY__GPIO_1_23,
	MX50_PAD_EIM_OE__GPIO_1_24,
	MX50_PAD_EIM_RW__GPIO_1_25,
	MX50_PAD_EIM_LBA__GPIO_1_26,
	///////////////////////////////////
	/////usb wifi  init
	///////////////////////////////////
#if defined(CONFIG_USB_WIFI_RTL8192CU)
	/*3g and wifi total power gpio*/
	//MX50_PAD_DISP_RD__GPIO_2_19,
	/*usb wifi power gpio*/
	MX50_PAD_ECSPI2_MOSI__GPIO_4_17,
	/*wifi 3g switch gpio*/
	//MX50_PAD_ECSPI2_SCLK__GPIO_4_16,
#endif
	///////////////////////////////////
	/////usb detect  init
	///////////////////////////////////
#if defined(CONFIG_USB_DETECT_CHECK)
	//wen add.for c027 DCDC detect.
	MX50_PAD_SD2_WP__GPIO_5_16,
#endif
	///////////////////////////////////////////////////
	////////ata5701 touch 
	/////////////////////////////////////////////////
#if defined(CONFIG_ATA5701_TOUCH)
	MX50_PAD_SD2_CMD__GPIO_5_7,
#endif
	///////////////////////////////////////////////////
	////////cy8c21434  touch 
	/////////////////////////////////////////////////
#if defined(CONFIG_CY8C_TOUCH)
	MX50_PAD_SD2_CMD__GPIO_5_7,
	MX50_PAD_KEY_COL1__GPIO_4_2,
#endif
	///////////////////////////////////
	/////backlight init
	///////////////////////////////////
	/*PWM2 use for backlight*/
	MX50_PAD_PWM2__PWMO,
	///////////////////////////////////
	/////TP touch init
	///////////////////////////////////
#if defined(CONFIG_MXC_GT813)
	/* tp interception gpio*/
	MX50_PAD_SD2_D5__GPIO_5_13,
	/* reset*/
	MX50_PAD_SD2_D6__GPIO_5_14,
#endif

};

static iomux_v3_cfg_t mx50_gpmi_nand[] = {
	MX50_PIN_EIM_DA8__NANDF_CLE,
	MX50_PIN_EIM_DA9__NANDF_ALE,
	MX50_PIN_EIM_DA10__NANDF_CE0,
	MX50_PIN_EIM_DA11__NANDF_CE1,
	MX50_PIN_EIM_DA12__NANDF_CE2,
	MX50_PIN_EIM_DA13__NANDF_CE3,
	MX50_PIN_EIM_DA14__NANDF_READY,
	MX50_PIN_EIM_DA15__NANDF_DQS,
	MX50_PIN_SD3_D4__NANDF_D0,
	MX50_PIN_SD3_D5__NANDF_D1,
	MX50_PIN_SD3_D6__NANDF_D2,
	MX50_PIN_SD3_D7__NANDF_D3,
	MX50_PIN_SD3_D0__NANDF_D4,
	MX50_PIN_SD3_D1__NANDF_D5,
	MX50_PIN_SD3_D2__NANDF_D6,
	MX50_PIN_SD3_D3__NANDF_D7,
	MX50_PIN_SD3_CLK__NANDF_RDN,
	MX50_PIN_SD3_CMD__NANDF_WRN,
	MX50_PIN_SD3_WP__NANDF_RESETN,
};

static iomux_v3_cfg_t suspend_enter_pads[] = {
	MX50_PAD_EIM_DA0__GPIO_1_0,
	MX50_PAD_EIM_DA1__GPIO_1_1,
	MX50_PAD_EIM_DA2__GPIO_1_2,
	MX50_PAD_EIM_DA3__GPIO_1_3,
	MX50_PAD_EIM_DA4__GPIO_1_4,
	MX50_PAD_EIM_DA5__GPIO_1_5,
	MX50_PAD_EIM_DA6__GPIO_1_6,
	MX50_PAD_EIM_DA7__GPIO_1_7,

	MX50_PAD_EIM_DA8__GPIO_1_8,
	MX50_PAD_EIM_DA9__GPIO_1_9,
	MX50_PAD_EIM_DA10__GPIO_1_10,
	MX50_PAD_EIM_DA11__GPIO_1_11,
	MX50_PAD_EIM_DA12__GPIO_1_12,
	MX50_PAD_EIM_DA13__GPIO_1_13,
	MX50_PAD_EIM_DA14__GPIO_1_14,
	MX50_PAD_EIM_DA15__GPIO_1_15,
	MX50_PAD_EIM_CS2__GPIO_1_16,
	MX50_PAD_EIM_CS1__GPIO_1_17,
	MX50_PAD_EIM_CS0__GPIO_1_18,
	MX50_PAD_EIM_EB0__GPIO_1_19,
	MX50_PAD_EIM_EB1__GPIO_1_20,
	MX50_PAD_EIM_WAIT__GPIO_1_21,
	MX50_PAD_EIM_BCLK__GPIO_1_22,
	MX50_PAD_EIM_RDY__GPIO_1_23,
	MX50_PAD_EIM_OE__GPIO_1_24,
	MX50_PAD_EIM_RW__GPIO_1_25,
	MX50_PAD_EIM_LBA__GPIO_1_26,
	MX50_PAD_EIM_CRE__GPIO_1_27,

	/* NVCC_NANDF pads */
	MX50_PAD_DISP_D8__GPIO_2_8,
	MX50_PAD_DISP_D9__GPIO_2_9,
	MX50_PAD_DISP_D10__GPIO_2_10,
	MX50_PAD_DISP_D11__GPIO_2_11,
	MX50_PAD_DISP_D12__GPIO_2_12,
	MX50_PAD_DISP_D13__GPIO_2_13,
	MX50_PAD_DISP_D14__GPIO_2_14,
	MX50_PAD_DISP_D15__GPIO_2_15,

	MX50_PAD_SD3_CMD__GPIO_5_18,
	MX50_PAD_SD3_CLK__GPIO_5_19,
	MX50_PAD_SD3_D0__GPIO_5_20,
	MX50_PAD_SD3_D1__GPIO_5_21,
	MX50_PAD_SD3_D2__GPIO_5_22,
	MX50_PAD_SD3_D3__GPIO_5_23,
	MX50_PAD_SD3_D4__GPIO_5_24,
	MX50_PAD_SD3_D5__GPIO_5_25,
	MX50_PAD_SD3_D6__GPIO_5_26,
	MX50_PAD_SD3_D7__GPIO_5_27,
	MX50_PAD_SD3_WP__GPIO_5_28,

	/* NVCC_LCD pads */
	MX50_PAD_DISP_D0__GPIO_2_0,
	MX50_PAD_DISP_D1__GPIO_2_1,
	MX50_PAD_DISP_D2__GPIO_2_2,
	MX50_PAD_DISP_D3__GPIO_2_3,
	MX50_PAD_DISP_D4__GPIO_2_4,
	MX50_PAD_DISP_D5__GPIO_2_5,
	MX50_PAD_DISP_D6__GPIO_2_6,
	MX50_PAD_DISP_D7__GPIO_2_7,
	MX50_PAD_DISP_WR__GPIO_2_16,
	MX50_PAD_DISP_RS__GPIO_2_17,
	MX50_PAD_DISP_BUSY__GPIO_2_18,
	//MX50_PAD_DISP_RD__GPIO_2_19,
	MX50_PAD_DISP_RESET__GPIO_2_20,
	MX50_PAD_DISP_CS__GPIO_2_21,

	/* CSPI pads */
	MX50_PAD_CSPI_SCLK__GPIO_4_8,
	MX50_PAD_CSPI_MOSI__GPIO_4_9,
	MX50_PAD_CSPI_MISO__GPIO_4_10,
	MX50_PAD_CSPI_SS0__GPIO_4_11,

	/*NVCC_MISC pins as GPIO */
       /*can't comment below:wen
         2013 -04 -16*/
	MX50_PAD_I2C1_SCL__GPIO_6_18,
	MX50_PAD_I2C1_SDA__GPIO_6_19,
	MX50_PAD_I2C2_SCL__GPIO_6_20,
	MX50_PAD_I2C2_SDA__GPIO_6_21,
	MX50_PAD_I2C3_SCL__GPIO_6_22,
	MX50_PAD_I2C3_SDA__GPIO_6_23,

	/* NVCC_MISC_PWM_USB_OTG pins */
	MX50_PAD_PWM1__GPIO_6_24,
//	MX50_PAD_PWM2__GPIO_6_25,
	MX50_PAD_EPITO__GPIO_6_27,
	MX50_PAD_WDOG__GPIO_6_28,

	/* FEC related. */
	MX50_PAD_EPDC_D10__GPIO_3_10,
	MX50_PAD_SSI_RXC__GPIO_6_5,
	MX50_PAD_SSI_RXFS__GPIO_6_4,
};

static iomux_v3_cfg_t suspend_exit_pads[ARRAY_SIZE(suspend_enter_pads)];

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
	//.num_wp = 3,/*copy from patch:hang_in_suspend_patch*/
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

static struct dvfs_wp dvfs_core_setpoint[] = {
	{33, 13, 33, 10, 10, 0x08}, /* 800MHz*/
	{28, 8, 33, 10, 10, 0x08},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x08},   /* 160MHz*/
	{28, 8, 33, 20, 30, 0x08},   /*160MHz, AHB 133MHz, LPAPM mode*/
	{29, 0, 33, 20, 10, 0x08},}; /* 160MHz, AHB 24MHz */

/* working point(wp): 0 - 800MHz; 1 - 400MHz, 2 - 160MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1200000,},/*1050000*/
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 400000000,
	 .cpu_podf = 1,
	 .cpu_voltage = 1200000,},/*1050000*/
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 160000000,
	 .cpu_podf = 4,
	 .cpu_voltage = 1200000,}, /*850000*/
};

static struct dvfs_wp *mx50_rdp_get_dvfs_core_table(int *wp)
{
	*wp = ARRAY_SIZE(dvfs_core_setpoint);
	return dvfs_core_setpoint;
}

static struct cpu_wp *mx50_rdp_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

static void mx50_rdp_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};


/* workaround for cspi chipselect pin may not keep correct level when idle */
static void mx50_rdp_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	switch (cspi_mode) {
	case 1:
		break;
	case 2:
		break;
	case 3:
		switch (chipselect) {
		case 0x1:
			{
			iomux_v3_cfg_t cspi_ss0 = MX50_PAD_CSPI_SS0__CSPI_SS0;
			//iomux_v3_cfg_t cspi_cs1 = MX50_PAD_ECSPI1_MOSI__GPIO_4_13;

			/* pull up/down deassert it */
			mxc_iomux_v3_setup_pad(cspi_ss0);
			//mxc_iomux_v3_setup_pad(cspi_cs1);

			//gpio_request(CSPI_CS1, "cspi-cs1");
			//gpio_direction_input(CSPI_CS1);
			}
			break;
		case 0x2:
			{
			//iomux_v3_cfg_t cspi_ss1 = MX50_PAD_ECSPI1_MOSI__CSPI_SS1;
			iomux_v3_cfg_t cspi_ss0 = MX50_PAD_CSPI_SS0__GPIO_4_11;

			/*disable other ss */
			//mxc_iomux_v3_setup_pad(cspi_ss1);
			mxc_iomux_v3_setup_pad(cspi_ss0);

			/* pull up/down deassert it */
			//gpio_request(CSPI_CS2, "cspi-cs2");
			//gpio_direction_input(CSPI_CS2);
			}
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}
}

static void mx50_rdp_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		break;
	case 2:
		break;
	case 3:
		switch (chipselect) {
		case 0x1:
			//gpio_free(CSPI_CS1);
			break;
		case 0x2:
			//gpio_free(CSPI_CS2);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx50_rdp_gpio_spi_chipselect_active,
	.chipselect_inactive = mx50_rdp_gpio_spi_chipselect_inactive,
};

static struct mxc_spi_master mxcspi3_data = {
	.maxchipselect = 4,
	.spi_version = 7,
	.chipselect_active = mx50_rdp_gpio_spi_chipselect_active,
	.chipselect_inactive = mx50_rdp_gpio_spi_chipselect_inactive,
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
static struct regulator_init_data pm_sensor_lm75a_init_data[] =
{
	{
		.constraints = {           
			.name = "DISPLAY",
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name = "VCOM",
			.min_uV = mV_to_uV(-4325),
			.max_uV = mV_to_uV(-500),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name = "V3P3",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	{
		.constraints = {
			.name = "TMST",
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		},
	},
};
#endif


/* Fixed voltage regulator DCDC_3V15 */
static struct regulator_consumer_supply fixed_volt_reg_consumers[] = {
	{
		/* sgtl5000 */
		.supply = "VDDIO",
		.dev_name = "1-000a",
	},
};

static struct regulator_init_data fixed_volt_reg_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(fixed_volt_reg_consumers),
	.consumer_supplies = fixed_volt_reg_consumers,
};

static struct fixed_voltage_config fixed_volt_reg_pdata = {
	.supply_name = "DCDC_3V15",
	.microvolts = 3150000,
	.init_data = &fixed_volt_reg_init_data,
	.gpio = -EINVAL,
};
#if defined(CONFIG_E_INK_EDPC)
static int epdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for EPDC pins - used during power up/down */
	ret |= gpio_request(EPDC_D0, "epdc_d0");
	ret |= gpio_request(EPDC_D1, "epdc_d1");
	ret |= gpio_request(EPDC_D2, "epdc_d2");
	ret |= gpio_request(EPDC_D3, "epdc_d3");
	ret |= gpio_request(EPDC_D4, "epdc_d4");
	ret |= gpio_request(EPDC_D5, "epdc_d5");
	ret |= gpio_request(EPDC_D6, "epdc_d6");
	ret |= gpio_request(EPDC_D7, "epdc_d7");
	ret |= gpio_request(EPDC_D8, "epdc_d8");
	ret |= gpio_request(EPDC_D9, "epdc_d9");
	ret |= gpio_request(EPDC_D10, "epdc_d10");
	ret |= gpio_request(EPDC_D11, "epdc_d11");
	ret |= gpio_request(EPDC_D12, "epdc_d12");
	ret |= gpio_request(EPDC_D13, "epdc_d13");
	ret |= gpio_request(EPDC_D14, "epdc_d14");
	ret |= gpio_request(EPDC_D15, "epdc_d15");

	ret |= gpio_request(EPDC_GDCLK, "epdc_gdclk");
	ret |= gpio_request(EPDC_GDSP, "epdc_gdsp");
	ret |= gpio_request(EPDC_GDOE, "epdc_gdoe");
	ret |= gpio_request(EPDC_GDRL, "epdc_gdrl");
	ret |= gpio_request(EPDC_SDCLK, "epdc_sdclk");
	ret |= gpio_request(EPDC_SDOE, "epdc_sdoe");
	ret |= gpio_request(EPDC_SDLE, "epdc_sdle");
	ret |= gpio_request(EPDC_SDSHR, "epdc_sdshr");
	ret |= gpio_request(EPDC_BDR0, "epdc_bdr0");
	ret |= gpio_request(EPDC_BDR1, "epdc_bdr1");
	ret |= gpio_request(EPDC_SDCE0, "epdc_sdce0");
	//ret |= gpio_request(EPDC_SDCE1, "epdc_sdce1");
	ret |= gpio_request(EPDC_SDCE2, "epdc_sdce2");

	return ret;
}

static void epdc_put_pins(void)
{
	gpio_free(EPDC_D0);
	gpio_free(EPDC_D1);
	gpio_free(EPDC_D2);
	gpio_free(EPDC_D3);
	gpio_free(EPDC_D4);
	gpio_free(EPDC_D5);
	gpio_free(EPDC_D6);
	gpio_free(EPDC_D7);

	gpio_free(EPDC_D8);
	gpio_free(EPDC_D9);
	gpio_free(EPDC_D10);
	gpio_free(EPDC_D11);
	gpio_free(EPDC_D12);
	gpio_free(EPDC_D13);
	gpio_free(EPDC_D14);
	gpio_free(EPDC_D15);

	gpio_free(EPDC_GDCLK);
	gpio_free(EPDC_GDSP);
	gpio_free(EPDC_GDOE);
	gpio_free(EPDC_GDRL);
	gpio_free(EPDC_SDCLK);
	gpio_free(EPDC_SDOE);
	gpio_free(EPDC_SDLE);
	gpio_free(EPDC_SDSHR);

	gpio_free(EPDC_BDR0);
	gpio_free(EPDC_BDR1);
	gpio_free(EPDC_SDCE0);
	//gpio_free(EPDC_SDCE1);
	gpio_free(EPDC_SDCE2);
}

static iomux_v3_cfg_t mx50_epdc_pads_enabled[] = {
	MX50_PAD_EPDC_D0__EPDC_D0,
	MX50_PAD_EPDC_D1__EPDC_D1,
	MX50_PAD_EPDC_D2__EPDC_D2,
	MX50_PAD_EPDC_D3__EPDC_D3,
	MX50_PAD_EPDC_D4__EPDC_D4,
	MX50_PAD_EPDC_D5__EPDC_D5,
	MX50_PAD_EPDC_D6__EPDC_D6,
	MX50_PAD_EPDC_D7__EPDC_D7,
        MX50_PAD_EPDC_D8__EPDC_D8,
        MX50_PAD_EPDC_D9__EPDC_D9,
        MX50_PAD_EPDC_D10__EPDC_D10,
        MX50_PAD_EPDC_D11__EPDC_D11,
        MX50_PAD_EPDC_D12__EPDC_D12,
        MX50_PAD_EPDC_D13__EPDC_D13,
        MX50_PAD_EPDC_D14__EPDC_D14,
        MX50_PAD_EPDC_D15__EPDC_D15,
	MX50_PAD_EPDC_GDCLK__EPDC_GDCLK,
	MX50_PAD_EPDC_GDSP__EPDC_GDSP,
	MX50_PAD_EPDC_GDOE__EPDC_GDOE,
	MX50_PAD_EPDC_GDRL__EPDC_GDRL,
	MX50_PAD_EPDC_SDCLK__EPDC_SDCLK,
	MX50_PAD_EPDC_SDOE__EPDC_SDOE,
	MX50_PAD_EPDC_SDLE__EPDC_SDLE,
	MX50_PAD_EPDC_SDSHR__EPDC_SDSHR,
	MX50_PAD_EPDC_BDR0__EPDC_BDR0,
	MX50_PAD_EPDC_BDR1__EPDC_BDR1,
	MX50_PAD_EPDC_SDCE0__EPDC_SDCE0,
	//MX50_PAD_EPDC_SDCE1__EPDC_SDCE1,
	MX50_PAD_EPDC_SDCE2__EPDC_SDCE2,
};

static iomux_v3_cfg_t mx50_epdc_pads_disabled[] = {
	MX50_PAD_EPDC_D0__GPIO_3_0,
	MX50_PAD_EPDC_D1__GPIO_3_1,
	MX50_PAD_EPDC_D2__GPIO_3_2,
	MX50_PAD_EPDC_D3__GPIO_3_3,
	MX50_PAD_EPDC_D4__GPIO_3_4,
	MX50_PAD_EPDC_D5__GPIO_3_5,
	MX50_PAD_EPDC_D6__GPIO_3_6,
	MX50_PAD_EPDC_D7__GPIO_3_7,
	MX50_PAD_EPDC_D8__GPIO_3_8,
	MX50_PAD_EPDC_D9__GPIO_3_9,
	MX50_PAD_EPDC_D10__GPIO_3_10,
	MX50_PAD_EPDC_D11__GPIO_3_11,
	MX50_PAD_EPDC_D12__GPIO_3_12,
	MX50_PAD_EPDC_D13__GPIO_3_13,
	MX50_PAD_EPDC_D14__GPIO_3_14,
	MX50_PAD_EPDC_D15__GPIO_3_15,
	MX50_PAD_EPDC_GDCLK__GPIO_3_16,
	MX50_PAD_EPDC_GDSP__GPIO_3_17,
	MX50_PAD_EPDC_GDOE__GPIO_3_18,
	MX50_PAD_EPDC_GDRL__GPIO_3_19,
	MX50_PAD_EPDC_SDCLK__GPIO_3_20,
	MX50_PAD_EPDC_SDOE__GPIO_3_23,
	MX50_PAD_EPDC_SDLE__GPIO_3_24,
	MX50_PAD_EPDC_SDSHR__GPIO_3_26,
	MX50_PAD_EPDC_BDR0__GPIO_4_23,
	MX50_PAD_EPDC_BDR1__GPIO_4_24,
	MX50_PAD_EPDC_SDCE0__GPIO_4_25,
	//MX50_PAD_EPDC_SDCE1__GPIO_4_26,
	MX50_PAD_EPDC_SDCE2__GPIO_4_27,
};

static void epdc_enable_pins(void)
{
	/* Configure MUX settings to enable EPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx50_epdc_pads_enabled, \
				ARRAY_SIZE(mx50_epdc_pads_enabled));

	gpio_direction_input(EPDC_D0);
	gpio_direction_input(EPDC_D1);
	gpio_direction_input(EPDC_D2);
	gpio_direction_input(EPDC_D3);
	gpio_direction_input(EPDC_D4);
	gpio_direction_input(EPDC_D5);
	gpio_direction_input(EPDC_D6);
	gpio_direction_input(EPDC_D7);
	gpio_direction_input(EPDC_D8);
	gpio_direction_input(EPDC_D9);
	gpio_direction_input(EPDC_D10);
	gpio_direction_input(EPDC_D11);
	gpio_direction_input(EPDC_D12);
	gpio_direction_input(EPDC_D13);
	gpio_direction_input(EPDC_D14);
	gpio_direction_input(EPDC_D15);
	gpio_direction_input(EPDC_GDCLK);
	gpio_direction_input(EPDC_GDSP);
	gpio_direction_input(EPDC_GDOE);
	gpio_direction_input(EPDC_GDRL);
	gpio_direction_input(EPDC_SDCLK);
	gpio_direction_input(EPDC_SDOE);
	gpio_direction_input(EPDC_SDLE);
	gpio_direction_input(EPDC_SDSHR);
	gpio_direction_input(EPDC_BDR0);
	gpio_direction_input(EPDC_BDR1);
	gpio_direction_input(EPDC_SDCE0);
	//gpio_direction_input(EPDC_SDCE1);
	gpio_direction_input(EPDC_SDCE2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx50_epdc_pads_disabled, \
				ARRAY_SIZE(mx50_epdc_pads_disabled));

	gpio_direction_output(EPDC_D0, 0);
	gpio_direction_output(EPDC_D1, 0);
	gpio_direction_output(EPDC_D2, 0);
	gpio_direction_output(EPDC_D3, 0);
	gpio_direction_output(EPDC_D4, 0);
	gpio_direction_output(EPDC_D5, 0);
	gpio_direction_output(EPDC_D6, 0);
	gpio_direction_output(EPDC_D7, 0);
	gpio_direction_output(EPDC_D8, 0);
	gpio_direction_output(EPDC_D9, 0);
	gpio_direction_output(EPDC_D10, 0);
	gpio_direction_output(EPDC_D11, 0);
	gpio_direction_output(EPDC_D12, 0);
	gpio_direction_output(EPDC_D13, 0);
	gpio_direction_output(EPDC_D14, 0);
	gpio_direction_output(EPDC_D15, 0);
	gpio_direction_output(EPDC_GDCLK, 0);
	gpio_direction_output(EPDC_GDSP, 0);
	gpio_direction_output(EPDC_GDOE, 0);
	gpio_direction_output(EPDC_GDRL, 0);
	gpio_direction_output(EPDC_SDCLK, 0);
	gpio_direction_output(EPDC_SDOE, 0);
	gpio_direction_output(EPDC_SDLE, 0);
	gpio_direction_output(EPDC_SDSHR, 0);
	gpio_direction_output(EPDC_BDR0, 0);
	gpio_direction_output(EPDC_BDR1, 0);
	gpio_direction_output(EPDC_SDCE0, 0);
	//gpio_direction_output(EPDC_SDCE1, 0);
	gpio_direction_output(EPDC_SDCE2, 0);
}


////default use 
static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock =32000000,/*default:30000000 ,32000000==16Mhz*/
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode ec080sc1_mode = {
	.name = "E80_V250",
	.refresh = 85,
	.xres = 1800,
	.yres = 800,
	.pixclock =75000000 ,/*68571429*/
	.left_margin = 8,
	.right_margin = 122,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 12,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

/* 16-bit bus interface, 8 pixel per line clock data transfer mode */
static struct fb_videomode e097oc5_8ppc_mode = {
  .name = "ED097OC5_8PPC",
  	.refresh = 25,
  	.xres = 2400,
  	.yres = 1650,
  	.pixclock = 68571429,
  	.left_margin = 8,
  	.right_margin = 380,
  	.upper_margin = 4,
  	.lower_margin = 60,
  	.hsync_len = 8,
  	.vsync_len = 4,
  	.sync = 0,
  	.vmode = FB_VMODE_NONINTERLACED,
  	.flag = 0,
  	};
  /* 8-bit bus interface, 4 pixel per line clock data transfer mode */
  static struct fb_videomode e097oc5_4ppc_mode = {
  .name = "ED097OC5_4PPC",
  	.refresh = 25,
  	.xres = 2400,
  	.yres = 1650,
  	.pixclock = 68571429,
  	.left_margin = 8,
  	.right_margin = 380,
  	.upper_margin = 4,
  	.lower_margin = 60,
  	.hsync_len = 8,
  	.vsync_len = 4,
  	.sync = 0,
  	.vmode = FB_VMODE_NONINTERLACED,
  	.flag = 0,
  	};

  /* 16-bit bus interface, 8 pixel per line clock data transfer mode */
  static struct fb_videomode e097oc6_8ppc_mode = {
  .name = "ED097OC6_8PPC",
  	.refresh = 60,
  	.xres = 1600,
  	.yres = 1200,
  	.pixclock = 80000000,
  	.left_margin = 8,
  	.right_margin = 280,
  	.upper_margin = 4,
  	.lower_margin = 7,
  	.hsync_len = 12,
  	.vsync_len = 1,
  	.sync = 0,
  	.vmode = FB_VMODE_NONINTERLACED,
  	.flag = 0,
  	};
  /* 8-bit bus interface, 4 pixel per line clock data transfer mode */
  static struct fb_videomode e097oc6_4ppc_mode = {
  .name = "ED097OC6_4PPC",
  	.refresh = 60,
  	.xres = 1600,
  	.yres = 1200,
  	.pixclock = 80000000,
  	.left_margin = 4,
  	.right_margin = 140,
  	.upper_margin = 4,
  	.lower_margin = 7,
  	.hsync_len = 6,
  	.vsync_len = 1,
  	.sync = 0,
  	.vmode = FB_VMODE_NONINTERLACED,
  	.flag = 0,
  	};

static struct fb_videomode e60_null_mode = {
	.name = "E60_NULL",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	};
static struct fb_videomode e97_null_mode = {
        .name = "E97_NULL",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 127,
	.upper_margin = 4,
	.lower_margin = 9,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	};

static struct mxc_epdc_fb_mode panel_modes[] = {
	{
		&e60_v220_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		465,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		9,	/* gdclk_offs */
		1,	/* num_ce */
	},
	{
		&ec080sc1_mode,
		4,	/* vscan_holdoff */
		40,	/*10, sdoed_width */
		80,	/* sdoed_delay */
		40,	/* sdoez_width */
		80,	/* sdoez_delay */
		876,	/* gdclk_hp_offs */
		23,	/* gdsp_offs */
		0,	/* gdoe_offs */
		15,	/* gdclk_offs */
		1,	/* num_ce */
	}, 
	{
 	      &e097oc5_8ppc_mode,
		2,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		1284,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		1,	/* gdclk_offs */
		2,	/* num_ce */
		},
	{
	      &e097oc5_4ppc_mode,
		2,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		1269,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		1,	/* gdclk_offs */
		2,	/* num_ce */
		},
	{
		&e097oc6_8ppc_mode,
		2,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		1022,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		1,	/* gdclk_offs */
		2,	/* num_ce */
		},
	{
		&e097oc6_4ppc_mode,
		2,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		1022,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		1,	/* gdclk_offs */
		2,	/* num_ce */
		},
	{
		&e60_null_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		465,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		9,	/* gdclk_offs */
		1,	/* num_ce */
	},	
	{
		&e97_null_mode,
		8,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		622,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		1,	/* gdclk_offs */
		3,	/* num_ce */
	},
};

static struct mxc_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
	.get_pins = epdc_get_pins,
	.put_pins = epdc_put_pins,
	.enable_pins = epdc_enable_pins,
	.disable_pins = epdc_disable_pins,
};
#endif

#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
static struct platform_device pm_sensor_lm75a_device = {
	.name = "pm_sensor_lm75a",
	.id = 0,
};
//static struct platform_device pm_lm75a_utils_device = {
	//.name = "pm_lm75a_utils",
	//.id = 0,
//};

static struct pm_sensor_platform_data  pm_sensor_lm75a_pdata   __initdata = {
	//.gpio_pmic_pwrgood=EPDC_PWRSTAT;
	//.gpio_pmic_vcom_ctrl=EPDC_VCOM;
	//.gpio_pmic_wakeup=EPDC_PWRCTRL0;
	//.gpio_pmic_intr=EPDC_PMIC_INT;
	//.gpio_pmic_powerup=EPDC_PWRCTRL0;
	.regulator_init = pm_sensor_lm75a_init_data,
	.init =pm_sensor_lm75a_init,
};

static int pm_sensor_lm75a_init(struct pm_sensor *pm_sensor)
{
	struct pm_sensor_platform_data  *pdata = &pm_sensor_lm75a_pdata;
	int i, ret;

	pm_sensor->pwr_seq0 = pdata->pwr_seq0;
	pm_sensor->pwr_seq1 = pdata->pwr_seq1;
	pm_sensor->pwr_seq2 = pdata->pwr_seq2;
	pm_sensor->upseq0 = pdata->upseq0;
	pm_sensor->upseq1 = pdata->upseq1;
	pm_sensor->dwnseq0 = pdata->dwnseq0;
	pm_sensor->dwnseq1 = pdata->dwnseq1;

	pm_sensor->max_wait = (3+3+3+6); /* values from upseq1 */

	pm_sensor->gpio_pmic_pwrgood = pdata->gpio_pmic_pwrgood;
	pm_sensor->gpio_pmic_vcom_ctrl = pdata->gpio_pmic_vcom_ctrl;
	pm_sensor->gpio_pmic_wakeup = pdata->gpio_pmic_wakeup;
	pm_sensor->gpio_pmic_powerup = pdata->gpio_pmic_powerup; 
	pm_sensor->vcom_setup = false;
	pm_sensor->init_done = false;

	for (i = 0; i < PM_SENSOR_LM75A_NUM_REGULATORS; i++) {
		ret = pm_sensor_register_regulator(pm_sensor, i,
			&pdata->regulator_init[i]);
		if (ret != 0) {
			printk(KERN_ERR"TPS6518x regulator init failed: %d\n",
				ret);
			return ret;
		}
	}

	//pm_sensor_register_regulator(pm_sensor,4,&vddgp);
	
	regulator_has_full_constraints();

	return 0;
}

#endif

static struct imxi2c_platform_data mxci2c_data = {
       .bitrate = 50000,//100000
};





static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
#if defined(CONFIG_MXC_TP_ELAN)
	{ 
		.type = "elan-touch",
		.addr = 0x10,
		.irq = gpio_to_irq(TOUCH_PANEL_IRQ), 
	},
#endif
#if defined(CONFIG_MXC_GT813)
	{
		.type= "gt813",
		.addr=0x5d,
		//.irq = gpio_to_irq(GT813_TOUCH_PANEL_IRQ), 
	},
#endif
};
    
   

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
#if defined(CONFIG_SND_SOC_SGTL5000)
	 {
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
#endif
#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
	{
           I2C_BOARD_INFO("pm_sensor_lm75a", 0x48), 
           .platform_data = &pm_sensor_lm75a_pdata,
	},
 #endif 
 #if defined(CONFIG_MXC_ENCRYPT)
	{
		//encrypt
		.type = "encrypt",
		.addr=0x3d,
	},
#endif

 };
    
static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
  #if defined(CONFIG_ATA5701_TOUCH)
	{
	  .type="ata5701",
	  .addr = 0x33,
	 .irq = gpio_to_irq(MX50_RDP_ATA5701_IRQ), 
	}
#endif
#if defined(CONFIG_CY8C_TOUCH)
	{
	  .type="cy8c",
	  .addr = 0x24,
	 .irq = gpio_to_irq(MX50_RDP_CY8C_IRQ), 
	}
#endif

};

 
static struct mtd_partition mxc_dataflash_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x000100000,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},
};

static struct flash_platform_data mxc_spi_flash_data[] = {
	{
	 .name = "mxc_dataflash",
	 .parts = mxc_dataflash_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_dataflash_partitions),
	 .type = "at45db321d",},
	 {
	 .name = "m25p80",
	 .parts = mxc_dataflash_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_dataflash_partitions),
	 .type = "m25p32",}
};


static struct spi_board_info mxc_dataflash_device[] __initdata = {
	{
	 .modalias = "mxc_dataflash",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 3,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[0],},
};

static struct spi_board_info m25pxx_dataflash_device[] __initdata = {
	{
	 .modalias = "m25p80",
	 .max_speed_hz = 20000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 3,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[1],},
};

static int mx508_rdp_sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;
  
	//if (to_platform_device(dev)->id == 0)
	//	rc = gpio_get_value(MX508_RDP_GPIO4_19_SD1_WP);
	//else if (to_platform_device(dev)->id == 1)
	//	rc = gpio_get_value(SD2_WP);
	//else if (to_platform_device(dev)->id == 2)
	//	rc = 0;
	//printk("%s %s %d rc=%d  \n",__FILE__,__func__,__LINE__,rc);

	return rc;
}

static unsigned int mx508_rdp_sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;
	
	if (to_platform_device(dev)->id == 0){
		ret = gpio_get_value(MX508_RDP_GPIO1_27_SD1_CD);
		//printk("%s,ret=%d\n",__func__,ret);
		if(ret==0)/*SD Card is in*/
		{
		   mx50_rdp_sdcard_control(true);
		   msleep(100);
		}else{/*SD Card is out*/
		  mx50_rdp_sdcard_control(false);
		   msleep(100);
		}
	}
	//else if (to_platform_device(dev)->id == 1)
	//	ret = gpio_get_value(SD2_CD);
	//else if (to_platform_device(dev)->id == 2)
	//	ret = 1;
	//printk("%s %s %d ret=%d \n",__FILE__,__func__,__LINE__,ret);
	
	return ret;
}

static struct mxc_mmc_platform_data mx508_rdp_mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	//.max_clk = 50000000,
	.max_clk = 25000000,
	.card_inserted_state = 0,
	.status = mx508_rdp_sdhc_get_card_det_status,
	.wp_status = mx508_rdp_sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

  /*
 * NOTE: Due to possible timing issue, it is not recommended to use usdhc
 * with DDR mode enabled. Instead, we use esdhc for DDR mode by default.
 */
static struct mxc_mmc_platform_data mx508_rdp_mmc3_data = {
    .ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
        | MMC_VDD_31_32,
    .caps = MMC_CAP_4_BIT_DATA |MMC_CAP_8_BIT_DATA /*| MMC_CAP_DATA_DDR*/,
    //.caps = MMC_CAP_4_BIT_DATA |MMC_CAP_8_BIT_DATA | MMC_CAP_DATA_DDR ,
    .min_clk = 400000,
    .max_clk = 40000000,
    //..max_clk = 10000000,
    //.clk_always_on = 1,
   // .dll_override_en = 1,
    .dll_delay_cells = 0,/*inand :0*/
    .card_inserted_state = 1,
    .status = mx508_rdp_sdhc_get_card_det_status,
    .wp_status = mx508_rdp_sdhc_write_protect,
    .clock_mmc = "esdhc_clk",
};

 #if defined(CONFIG_SND_SOC_SGTL5000)
 int mxc_sgtl5000_init(void);

static int mxc_sgtl5000_amp_enable(int enable)
{
	//gpio_set_value(MX50_RDP_SPEAKER_CONTROL, !enable);
	return 0;
}

static int mxc_sgtl5000_clock_enable(int enable)
{
	//gpio_set_value(MX50_RDP_SGTL5000_EN, enable);
	//mxc_sgtl5000_init();
	return 0;
}

static int headphone_det_status(void)
{
 	return (gpio_get_value(MX50_RDP_HEADPHONE_INSERT_DETECT) == 0);
 }
static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.init = mxc_sgtl5000_init,
	.hp_irq = gpio_to_irq(MX50_RDP_HEADPHONE_INSERT_DETECT),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.clock_enable = mxc_sgtl5000_clock_enable,
	//.sysclk = 12288000,
	//.sysclk = 12000000,
	.ext_ram_rx = 1,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

int mxc_sgtl5000_init(void)
{

	struct clk *ssi_ext1;
	int rate;

	
	ssi_ext1 = clk_get(NULL, "ssi_ext1_clk");
	
	if (IS_ERR(ssi_ext1))
	{
		printk("%s %s %d clock get fail \n",__FILE__,__func__,__LINE__); 
		return -1;
	}

	rate = clk_round_rate(ssi_ext1, 12000000);
	
	if (rate < 8000000 || rate > 27000000) {
		printk(KERN_ERR "Error: SGTL5000 mclk freq %d out of range!\n",
			rate);
		clk_put(ssi_ext1);
		return -1;
	}

	clk_set_rate(ssi_ext1, rate);
	clk_enable(ssi_ext1);
	
	sgtl5000_data.sysclk = rate;
  
	return 0;
}
 

#endif

static struct mxc_pwm_platform_data mx50_rdp_mxc_pwm2_data = {
	.pwmo_invert = 1,
};

static struct platform_pwm_backlight_data mx50_rdp_mxc_pwm2_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 2000000,
	//.max_brightness = 31,
	//.dft_brightness = 16,
	//.pwm_period_ns = 1000000,	
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.can_disable = 1, 					\
}

  

  static struct gpio_keys_button mx50_rdp_buttons[] = {
	//GPIO_BUTTON(MX50_RDP_ECSPI1_SS0_GPIO4_15_SYSTEM_POWER, KEY_POWER, 1, "power", 1),
	GPIO_BUTTON(MX50_RDP_UART3_TXD_GPIO6_14_HOME, KEY_MENU, 1, "menu", 0),
	GPIO_BUTTON(MX50_RDP_DISP_RS_GP2_17_HOME, KEY_DOWN, 1, "down", 0),
	GPIO_BUTTON(MX50_RDP_DISP_DAT15_GPIO2_15_MENU, KEY_UP, 1, "up", 0),
	GPIO_BUTTON(MX50_RDP_DISP_RS_GP2_12_DOWN, KEY_BACK, 1, "back", 0),
	
};

 
static struct gpio_keys_platform_data mx50_rdp_button_data = {
	.buttons	= mx50_rdp_buttons,
	.nbuttons	= ARRAY_SIZE(mx50_rdp_buttons),
};

static struct platform_device mx50_rdp_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &mx50_rdp_button_data,
	}
};

static void __init mx50_rdp_add_device_buttons(void)
{
	platform_device_register(&mx50_rdp_button_device);
}
#else
static void __init mx50_rdp_add_device_buttons(void) {}
#endif

#if defined(CONFIG_MXC_PMIC_MC13892)
static void mxc_register_powerkey(pwrkey_callback pk_cb)
{
	pmic_event_callback_t power_key_event;

	power_key_event.param = (void *)1;
	power_key_event.func = (void *)pk_cb;
	pmic_event_subscribe(EVENT_PWRONI, power_key_event);
	power_key_event.param = (void *)3;
	pmic_event_subscribe(EVENT_PWRON3I, power_key_event);
 }

static int mxc_pwrkey_getstatus(int id)
{
	int sense, off = 3;

	pmic_read_reg(REG_INT_SENSE1, &sense, 0xffffffff);
	//printk("%s,%d id =%d,sense=0x%x ,",__func__,__LINE__,id,sense);
	switch (id) {
	case 2:
		off = 4;
		break;
	case 3:
		off = 2;
		break;
	}
	//printk("off=%d (sense & (1 << off))=%d\n",off,(sense & (1 << off)));

	if (sense & (1 << off))
		return 0;

	return 1;
}

static struct power_key_platform_data mx50_rdp_pwrkey_data = {
	.key_value = KEY_POWER,
	.register_pwrkey = mxc_register_powerkey,
	.get_key_status = mxc_pwrkey_getstatus,
};
#endif

#if defined (CONFIG_LEDS_GPIO_PLATFORM)
static struct gpio_led mx50_rdp_leds[] = {
	{	/* "led0", yellow */
		.name			= "led0",
		.gpio			= MX50_RDP_GPIO_LEDS_CONTROL,
		.active_low		= 1,
		.default_trigger	= "gpio",
		.default_state        = 1,
	},
};
static struct gpio_led_platform_data mx50_rdp_led_data;
static struct platform_device mx50_rdp_leds_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev.platform_data	= &mx50_rdp_led_data,
};

#define MX50_RDP_PERIOD_LED_OFF     5000 /* ms */
#define MX50_RDP_PERIOD_LED_ON       100/* ms */
static int mx50_rdp_gpio_blink_set(unsigned gpio, int state,
	unsigned long *delay_on, unsigned long *delay_off)
{

	if (delay_on && delay_off && !*delay_on && !*delay_off)
	{
	    *delay_on =MX50_RDP_PERIOD_LED_ON ;
	    *delay_off =MX50_RDP_PERIOD_LED_OFF ;
	}
	printk("%s,%d,state=%d   \n",__func__,__LINE__,state); 

        ////state //
        ///0:open the light
        ///2:close the light
	switch(state) {
	case 0:
		gpio_direction_output(MX50_RDP_GPIO_LEDS_CONTROL,0);
		gpio_set_value(gpio, 0);
		break;
	case 2:
		gpio_direction_output(MX50_RDP_GPIO_LEDS_CONTROL,1);
		gpio_set_value(gpio, 1);
		break;
	default:
		gpio_direction_output(MX50_RDP_GPIO_LEDS_CONTROL,1);
		gpio_set_value(gpio, 1);
		break;
	}
	return 1;  //0
}

static void __init mx50_rdp_add_led_gpio_device(void)
{
	int i;

	mx50_rdp_led_data.leds = mx50_rdp_leds;
	mx50_rdp_led_data.num_leds = ARRAY_SIZE(mx50_rdp_leds);
	mx50_rdp_led_data.gpio_blink_set = mx50_rdp_gpio_blink_set,
		
	platform_device_register(&mx50_rdp_leds_device);
}
#else
static void __init mx50_rdp_add_led_gpio_device(void) {}
#endif 
 #if  defined(CONFIG_USB_DETECT_CHECK)
/*return the hardware switch state.*/
 //int get_bootup_type(void)
 //{
 //   return gpio_get_value(MX50_RDP_DCDC_INSERT_CHARGE_DETECT);
 //}
static int mx50_rdp_get_5vOrUSB_state(void)
{
	return gpio_get_value(MX50_RDP_DCDC_INSERT_CHARGE_DETECT);
}


static struct mxc_usb_detect_data mxc_5vOrUsb_detect_data = {
	.usb_irq = gpio_to_irq(MX50_RDP_DCDC_INSERT_CHARGE_DETECT),
	.usbdetect_status = mx50_rdp_get_5vOrUSB_state,
};
static struct platform_device mxc_5vOrUsb_detect_device = {
	.name = "usb-detect-check",
};
#endif
 
 


//wen in suspend state,we need set some gpio as high impedance state
static int mx50_rdp_set_gpio_high_impedance_state(void)
{
	//unsigned short rc = 0;
	///1.MMA67660
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	///3 encrypt ic 
#if defined(CONFIG_MXC_ENCRYPT_GPIO)
	gpio_request(MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA, "encrypt-dat");
	gpio_direction_input(MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA);

	gpio_request(MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL, "encrypt-clk");
	gpio_direction_input(MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL);

#endif


  
#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
   //-15v
	gpio_request(LM75A_EPDC_PWRCTRL0, "EDPC-POWER");
	gpio_direction_input(LM75A_EPDC_PWRCTRL0);
//+15v
	gpio_request(LM75A_EPDC_PWRCTRL1, "EDPC-POWER");
	gpio_direction_input(LM75A_EPDC_PWRCTRL1);
//+22v
	gpio_request(LM75A_EPDC_PWRCTRL2, "EDPC-POWER");
	gpio_direction_input(LM75A_EPDC_PWRCTRL2);
//vcom
	gpio_request(LM75A_EPDC_VCOM, "EDPC-POWER");
	gpio_direction_input(LM75A_EPDC_VCOM);
#endif

	return 0;
}

//wen in resume state,we need send the gpio in normal state
static int mx50_rdp_set_gpio_normal_state(void)
{

#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
   //-15v
	gpio_request(LM75A_EPDC_PWRCTRL0, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_PWRCTRL0, 0);
//+15v
	gpio_request(LM75A_EPDC_PWRCTRL1, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_PWRCTRL1, 0);
//+22v
	gpio_request(LM75A_EPDC_PWRCTRL2, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_PWRCTRL2,0);
//vcom
	gpio_request(LM75A_EPDC_VCOM, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_VCOM, 0);
#endif


	///4 encrypt ic 
#if defined(CONFIG_MXC_ENCRYPT_GPIO)
	gpio_request(MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA, "encrypt-dat");
	gpio_direction_output(MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA, 0);

	gpio_request(MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL, "encrypt-clk");
	gpio_direction_output(MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL, 0);

#endif

	return 0;
}

int mx50_rdp_gpio_suspend(void)
{
	mx50_rdp_set_gpio_high_impedance_state();

	printk("%s %s %d iXrzSuspendMode=%d\n",__FILE__,__func__,__LINE__,iXrzSuspendMode); 
	//mx50_rdp_light_control(false);

       ///only in normal suspend mode need close the power
       if(iXrzSuspendMode == 0){
	   ///in order to make the backligt display use setting
	   mx50_rdp_backlight_control(false);

	   mx50_rdp_tp_power_control(false);
       	}
	return 0;
}

int mx50_rdp_gpio_resume(void)
{	
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
       if(iXrzSuspendMode == 0){
	   mx50_rdp_tp_power_control(true);
	  //mx50_rdp_light_control(true);
           mx50_rdp_backlight_control(true);
       	}
	mx50_rdp_set_gpio_normal_state();

	return 0;
}

static int __initdata enable_w1 = { 0 };
static int __init w1_setup(char *__unused)
{
	enable_w1 = 1;
	return cpu_is_mx50();
}

__setup("w1", w1_setup);

static struct android_pmem_platform_data android_pmem_data = {
	.name = "pmem_adsp",
	.size = SZ_8M,
};

static struct android_pmem_platform_data android_pmem_gpu_data = {
	.name = "pmem_gpu",
	.size = SZ_64M,//SZ_32M,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_all[] = {
	"rndis",
	"usb_mass_storage",
	"adb"
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0c01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c02,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0c10,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
};

static struct usb_mass_storage_platform_data mass_storage_data = {
	//.nluns		= 3,
	.nluns		= 2,
	.vendor		= "xrz",
	.product	= "eReader",
	.release	= 0x0100,
};

static struct usb_ether_platform_data rndis_data = {
	.vendorID	= 0x15a2,
	.vendorDescr	= "xrz",
};

static struct android_usb_platform_data android_usb_data = {
	.vendor_id      = 0x15a2,
	.product_id     = 0x0c01,
	.version        = 0x0100,
	.product_name   = "eBook",
	.manufacturer_name = "xrz",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static int __initdata enable_keypad = {0};
static int __init keypad_setup(char *__unused)
{
	enable_keypad = 1;
	return cpu_is_mx50();
}

__setup("keypad", keypad_setup);

static struct mxs_dma_plat_data dma_apbh_data = {
	.chan_base = MXS_DMA_CHANNEL_AHB_APBH,
	.chan_num = MXS_MAX_DMA_CHANNELS,
};




static void fec_gpio_iomux_init()
{
	//iomux_v3_cfg_t iomux_setting;

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__);

	//if (board_is_mx50_rd3())
	//{
	//	printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	//	iomux_setting = (MX50_PAD_SD2_CD__GPIO_5_17 & \
	//		~MUX_PAD_CTRL_MASK) | \
	//		MUX_PAD_CTRL(PAD_CTL_PKE | PAD_CTL_DSE_HIGH);
	//	mxc_iomux_v3_setup_pad(iomux_setting);
	//}
	//#if 1
	//else
	//{
	//	printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	//	iomux_setting = (MX50_PAD_I2C3_SDA__GPIO_6_23 & \
	//		~MUX_PAD_CTRL_MASK) | \
	//		MUX_PAD_CTRL(PAD_CTL_PKE | PAD_CTL_DSE_HIGH);
	//
	//	/* Enable the Pull/keeper */
	//	mxc_iomux_v3_setup_pad(iomux_setting);
	//}
	//#endif 
	///gpio_request(FEC_EN, "fec-en");
	//if (board_is_mx50_rd3())
	//	gpio_direction_output(FEC_EN, 1);
	//else
	//	gpio_direction_output(FEC_EN, 0);
	//gpio_request(FEC_RESET_B, "fec-reset_b");
	//gpio_direction_output(FEC_RESET_B, 0);
	//udelay(500);
	//gpio_set_value(FEC_RESET_B, 1);

}

static void fec_gpio_iomux_deinit()
{
	//iomux_v3_cfg_t iomux_setting;

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__);

	//if (board_is_mx50_rd3())
	//{
	//	printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	//	iomux_setting = (MX50_PAD_SD2_CD__GPIO_5_17 & \
	//		~MUX_PAD_CTRL_MASK) | MUX_PAD_CTRL(0x4);
	//	mxc_iomux_v3_setup_pad(iomux_setting);
	//}
	//#if 0
	//else
	//{
	//	iomux_setting = (MX50_PAD_I2C3_SDA__GPIO_6_23 & \
	//		~MUX_PAD_CTRL_MASK) | MUX_PAD_CTRL(0x4);
	//	mxc_iomux_v3_setup_pad(iomux_setting);
	//}
	//#endif 
	//mxc_iomux_v3_setup_pad(iomux_setting);
	//gpio_request(FEC_EN, "fec-en");
	//gpio_direction_input(FEC_EN);
	//gpio_request(FEC_RESET_B, "fec-reset_b");
	//gpio_direction_input(FEC_RESET_B);

	mx50_rdp_gpio_suspend();
}

static void mx50_suspend_enter()
{
	iomux_v3_cfg_t *p = suspend_enter_pads;
	int i;

	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	//iomux_v3_cfg_t iomux_setting =
	//(MX50_PAD_ECSPI2_SCLK__GPIO_4_16 &
	//~MUX_PAD_CTRL_MASK) | MUX_PAD_CTRL(0x84);
	/* Clear the SELF_BIAS bit and power down
	* the band-gap.
	*/
	//__raw_writel(MXC_ANADIG_REF_SELFBIAS_OFF,
	//	apll_base + MXC_ANADIG_MISC_CLR);
	//__raw_writel(MXC_ANADIG_REF_PWD,
	//	apll_base + MXC_ANADIG_MISC_SET);

	//if (board_is_mx50_rd3()) {
	/* Enable the Pull/keeper */
	//mxc_iomux_v3_setup_pad(iomux_setting);
	//gpio_request(DCDC_EN, "dcdc-en");
	//gpio_direction_output(DCDC_EN, 1);
	//}

	/* Set PADCTRL to 0 for all IOMUX. */
	for (i = 0; i < ARRAY_SIZE(suspend_enter_pads); i++) {
		suspend_exit_pads[i] = *p;
		*p &= ~MUX_PAD_CTRL_MASK;
		p++;
	}
	mxc_iomux_v3_get_multiple_pads(suspend_exit_pads,
		ARRAY_SIZE(suspend_exit_pads));
	mxc_iomux_v3_setup_multiple_pads(suspend_enter_pads,
		ARRAY_SIZE(suspend_enter_pads));

	fec_gpio_iomux_deinit();
}

static void mx50_suspend_exit()
{
	//iomux_v3_cfg_t iomux_setting =
	//(MX50_PAD_ECSPI2_SCLK__GPIO_4_16 &
	//~MUX_PAD_CTRL_MASK) | MUX_PAD_CTRL(0x84);
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	mx50_rdp_gpio_resume();
	/* Power Up the band-gap and set the SELFBIAS bit. */
	//__raw_writel(MXC_ANADIG_REF_PWD,
	//	apll_base + MXC_ANADIG_MISC_CLR);
	//udelay(100);
	//__raw_writel(MXC_ANADIG_REF_SELFBIAS_OFF,
	//	apll_base + MXC_ANADIG_MISC_SET);

	//if (board_is_mx50_rd3()) {
	/* Enable the Pull/keeper */
	//mxc_iomux_v3_setup_pad(iomux_setting);
	//gpio_request(DCDC_EN, "dcdc-en");
	//gpio_direction_output(DCDC_EN, 0);
	//}

	mxc_iomux_v3_setup_multiple_pads(suspend_exit_pads,
		ARRAY_SIZE(suspend_exit_pads));
	fec_gpio_iomux_init();
}

static struct mxc_pm_platform_data mx50_pm_data = {
	.suspend_enter = mx50_suspend_enter,
	.suspend_exit = mx50_suspend_exit,
};

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
								   char **cmdline, struct meminfo *mi)
{
	mxc_set_cpu_type(MXC_CPU_MX50);

	get_cpu_wp = mx50_rdp_get_cpu_wp;
	set_num_cpu_wp = mx50_rdp_set_num_cpu_wp;
	get_dvfs_core_wp = mx50_rdp_get_dvfs_core_table;
	num_cpu_wp = ARRAY_SIZE(cpu_wp_auto);
}
/*
*  init the elan touch gpio 
*/
#if defined(CONFIG_MXC_TP_ELAN)
static int elan_touch_gpio_init(void)
{ 
	/*ELAN TP IRQ*/
	gpio_request(TOUCH_PANEL_IRQ, "touch-panel-irq");
	gpio_direction_input(TOUCH_PANEL_IRQ);
	/*ELAN TP RESET*/
	gpio_request(TOUCH_PANEL_RESET, "touch-panel-reset");
	gpio_direction_output(TOUCH_PANEL_RESET, 1);

    return 0;
}
#endif

////init
static int mx50_rdp_tp_power_init(void)
{
      gpio_request(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER, "tp-power");
      gpio_direction_output(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER, 0);

     return 0;
}
/*
   *  TP and ata5701 power control,only use for c036
   *  default is true
   *  1:open
   *  0: close
   */
int  mx50_rdp_tp_power_control(bool bOpen)
{
     printk("%s,bOpen=%d\n",__func__,bOpen);
      if(bOpen){
         //gpio_request(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER, "tp-power");
         //gpio_direction_output(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER, 1);
          gpio_set_value(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER,1);
     }else{
        // gpio_request(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER, "tp-power");
         //gpio_direction_output(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER, 0);
          gpio_set_value(MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER,0);
     	}
	
    return 0;
}

/////SD Card control
///sd card is in,set to false
///sd card is out,set to true
static int mx50_rdp_sdcard_control(bool enable)
{
    if(enable){
         gpio_request(MX508_RDP_GPIO4_19_SD1_WP, "sdhc1-wp");
         gpio_direction_output(MX508_RDP_GPIO4_19_SD1_WP, 0);
     }else{
         gpio_request(MX508_RDP_GPIO4_19_SD1_WP, "sdhc1-wp");
         gpio_direction_output(MX508_RDP_GPIO4_19_SD1_WP, 1);
     	}
		
	 return 0;
}

/////light contrl
///false : open the light
///true : close the light
int  mx50_rdp_light_control(bool bOpen)
{
	if (bOpen) {
		gpio_request(MX50_RDP_GPIO4_4_LIGHT_CONTROL, "light-control");
		gpio_direction_output(MX50_RDP_GPIO4_4_LIGHT_CONTROL, 0);
	} else {
		gpio_request(MX50_RDP_GPIO4_4_LIGHT_CONTROL, "light-control");
		gpio_direction_output(MX50_RDP_GPIO4_4_LIGHT_CONTROL, 1);
	}

	return 0;
}
/////init
static int  mx50_rdp_backlight_init(void)
{
       gpio_request(MX50_RDP_BACKLIGHT_ENABLE_CONTROL, "backlight-ctl");
       gpio_direction_output(MX50_RDP_BACKLIGHT_ENABLE_CONTROL, 1);

    return 0;
}
//////backlight enable or disable
///true: enable
///false: disable
void  mx50_rdp_backlight_control(bool bOpen)
{
     //gpio_request(MX50_RDP_BACKLIGHT_ENABLE_CONTROL, "backlight-ctl");
     if(bOpen){
         //gpio_direction_output(MX50_RDP_BACKLIGHT_ENABLE_CONTROL, 1);
         gpio_set_value(MX50_RDP_BACKLIGHT_ENABLE_CONTROL,1);
     }else{
        // gpio_direction_output(MX50_RDP_BACKLIGHT_ENABLE_CONTROL, 0);
        gpio_set_value(MX50_RDP_BACKLIGHT_ENABLE_CONTROL,0);
     }
}
EXPORT_SYMBOL(mx50_rdp_backlight_control);

///WDOG init
void mx50_rdp_wdog_control(void)
{
        gpio_request(WDOG_B, "wdog");
        gpio_direction_output(WDOG_B, 0);

}
EXPORT_SYMBOL(mx50_rdp_wdog_control);

//////init .set to low
int mx50_rdp_mc13892_charger_gpio_control_init(void)
{
     gpio_request(MX50_RDP_CHARGER_GPIO_CONTROL, "charger-con");
     gpio_direction_output(MX50_RDP_CHARGER_GPIO_CONTROL, 0);

    return 0;
}
//////charger control 
/////GPIO control,when battary voltage < 3.6v,set as high,
/////other set to low.
/////true: set to high
//// false:set to low
/////default :low
int mx50_rdp_mc13892_charger_gpio_control(bool bOpen)
{
      printk("%s,bOpen=%d\n",__func__,bOpen);
      if(bOpen){
         //gpio_request(MX50_RDP_CHARGER_GPIO_CONTROL, "charger-con");
         gpio_direction_output(MX50_RDP_CHARGER_GPIO_CONTROL, 1);
     }else{
         //gpio_request(MX50_RDP_CHARGER_GPIO_CONTROL, "charger-con");
         gpio_direction_output(MX50_RDP_CHARGER_GPIO_CONTROL, 0);
     	}

	 return 0;
}
int mx50_rdp_get_mc13892_charger_gpio_control_state(void)
{
   return gpio_get_value(MX50_RDP_CHARGER_GPIO_CONTROL);
}

//#endif
static void __init mx50_rdp_io_init(void)
{
	//iomux_v3_cfg_t cspi_keeper = (MX50_PAD_ECSPI1_SCLK__GPIO_4_12 & ~MUX_PAD_CTRL_MASK);

	iomux_v3_cfg_t *p = mx50_rdp;
	int i;
	int iVersion=0;

	/* Set PADCTRL to 0 for all IOMUX. */
	for (i = 0; i < ARRAY_SIZE(mx50_rdp); i++) {
		iomux_v3_cfg_t pad_ctl = *p;
		pad_ctl &= ~MUX_PAD_CTRL_MASK;
		mxc_iomux_v3_setup_pad(pad_ctl);
		p++;
	}

	mxc_iomux_v3_setup_multiple_pads(mx50_rdp, \
		ARRAY_SIZE(mx50_rdp));

	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	mx50_rdp_tp_power_init();
	mx50_rdp_tp_power_control(true);

	//mx50_rdp_light_control(true);
	mx50_rdp_backlight_init();

	mx50_rdp_mc13892_charger_gpio_control_init();

	gpio_request(MX508_RDP_GPIO1_27_SD1_CD, "sdhc1-cd");
	gpio_direction_input(MX508_RDP_GPIO1_27_SD1_CD);

	/*
	* PWRUP pin of TPS6518x connects to this PWR0 gpio pin
	*/

#if defined(CONFIG_MFD_PM_SENSOR_LM75A)
	   //-15v
	gpio_request(LM75A_EPDC_PWRCTRL0, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_PWRCTRL0, 0);
	//+15v
	gpio_request(LM75A_EPDC_PWRCTRL1, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_PWRCTRL1, 0);
	//+22v
	gpio_request(LM75A_EPDC_PWRCTRL2, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_PWRCTRL2,0);
	//vcom
	gpio_request(LM75A_EPDC_VCOM, "EDPC-POWER");
	gpio_direction_output(LM75A_EPDC_VCOM, 0);
#endif
	fec_gpio_iomux_init();

#if defined(CONFIG_SND_SOC_SGTL5000)
	/* SGTL5000_OSC_EN */
	//gpio_request(MX50_RDP_SGTL5000_EN, "sgtl5000-osc-en");
	//gpio_direction_output(MX50_RDP_SGTL5000_EN, 1);

         ///////////////////////////////////////////////
	///headphone insert detect
	//////////////////////////////////////////
	gpio_request(MX50_RDP_HEADPHONE_INSERT_DETECT, "hp-det");
	gpio_direction_input(MX50_RDP_HEADPHONE_INSERT_DETECT);

#endif


#if defined(CONFIG_MXC_TP_ELAN)
	elan_touch_gpio_init();
#endif


#if defined(CONFIG_USB_WIFI_RTL8192CU)
	/*usb wifi power enable*/
	gpio_request(USBWIFI_PWER_EN, "usb-wifi-enable");
	gpio_direction_output(USBWIFI_PWER_EN, 0);

	///default we set the wifi power and wifi-3g switch in high 
	///impedance state. ---
	gpio_direction_input(USBWIFI_PWER_EN);

#endif
        //for wifi test
        // msleep(10);
        //xrz_wifi_open();


	///3 encrypt ic ,new version use it.
#if defined(CONFIG_MXC_ENCRYPT_GPIO)
	gpio_request(MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA, "encrypt-dat");
	gpio_direction_output(MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA, 0);

	gpio_request(MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL, "encrypt-clk");
	gpio_direction_output(MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL, 0);

#endif


}
/*!
* Board specific initialization.
*/

static void __init mxc_board_init(void)
{
        int  hardwareversion = 0;
	/* SD card detect irqs */
	mxcsdhc1_device.resource[2].start = gpio_to_irq(MX508_RDP_GPIO1_27_SD1_CD);
	mxcsdhc1_device.resource[2].end = gpio_to_irq(MX508_RDP_GPIO1_27_SD1_CD);


	//mxcsdhc2_device.resource[2].start = gpio_to_irq(SD2_CD);
	//mxcsdhc2_device.resource[2].end = gpio_to_irq(SD2_CD);

	//mxcsdhc3_device.resource[2].start = gpio_to_irq(SD3_CD);
	//mxcsdhc3_device.resource[2].end = gpio_to_irq(SD3_CD);

	mxc_cpu_common_init();
	mx50_rdp_io_init();

	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxcspi3_device, &mxcspi3_data);
	if (board_is_mx50_rd3())
		dvfs_core_data.reg_id = "SW1A";
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	if (board_is_mx50_rd3())
		bus_freq_data.gp_reg_id = "SW1A";
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxs_dma_apbh_device, &dma_apbh_data);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	if (!mxc_fuse_get_gpu_status())
		mxc_register_device(&gpu_device, &gpu_data);
	mxc_register_device(&mxc_pxp_device, NULL);
	mxc_register_device(&mxc_pxp_client_device, NULL);
	mxc_register_device(&mxc_pxp_v4l2, NULL);
	mxc_register_device(&pm_device, &mx50_pm_data);
	//if (enable_keypad)
	//mxc_register_device(&mxc_keypad_device, &keypad_plat_data);
	mxc_register_device(&mxcsdhc3_device, &mx508_rdp_mmc3_data);

	//if (board_is_mx50_rd3())
	//mmc2_data.power_mmc = NULL;
	//mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc1_device, &mx508_rdp_mmc1_data);

	//mxc_register_device(&mxcsdhc3_device, &mx508_rdp_mmc3_data);

	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_fec_device, &fec_data);
	if (board_is_mx50_rd3())
		spi_register_board_info(m25pxx_dataflash_device,
		ARRAY_SIZE(m25pxx_dataflash_device));
	else
		spi_register_board_info(mxc_dataflash_device,
		ARRAY_SIZE(mxc_dataflash_device));

	i2c_register_board_info(0, mxc_i2c0_board_info,
		ARRAY_SIZE(mxc_i2c0_board_info));
	
	i2c_register_board_info(1, mxc_i2c1_board_info,
		ARRAY_SIZE(mxc_i2c1_board_info));
	///don't need mma7660,so comment below
	i2c_register_board_info(2, mxc_i2c2_board_info,
		ARRAY_SIZE(mxc_i2c2_board_info));
	

	//mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device,
		&android_pmem_gpu_data);
	mxc_register_device(&usb_mass_storage_device, &mass_storage_data);
	mxc_register_device(&usb_rndis_device, &rndis_data);
	mxc_register_device(&android_usb_device, &android_usb_data);

#if defined(CONFIG_MFD_PM_SENSOR_LM75A) 
       mxc_register_device(&pm_sensor_lm75a_device, NULL);
#endif
#if defined(CONFIG_E_INK_EDPC)
	mxc_register_device(&epdc_device, &epdc_data);
#endif

         
	mxc_register_device(&mxc_pwm2_device, &mx50_rdp_mxc_pwm2_data);
	mxc_register_device(&mxc_pwm2_backlight_device,
		&mx50_rdp_mxc_pwm2_backlight_data);

///mx50_rdp_backlight_control(true);

	mxc_register_device(&mxs_viim, NULL);
	mxc_register_device(&mxc_rngb_device, NULL);
	mxc_register_device(&dcp_device, NULL);

#if defined(CONFIG_MXC_PMIC_MC13892)	
	mxc_register_device(&mxc_powerkey_device, &mx50_rdp_pwrkey_data);
#endif
	mxc_register_device(&fixed_volt_reg_device, &fixed_volt_reg_pdata);
	//if (mx50_revision() >= IMX_CHIP_REVISION_1_1)
	//mxc_register_device(&mxc_zq_calib_device, NULL);
	mx50_rdp_add_device_buttons();

#if  defined(CONFIG_MXC_PMIC_MC13892)
	mx50_rdp_init_mc13892();
#endif

#if defined(CONFIG_SND_SOC_SGTL5000)	
	sgtl5000_data.ext_ram_clk = clk_get(NULL, "ddr_clk");
	clk_put(sgtl5000_data.ext_ram_clk);

	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);

#endif	

#if  defined(CONFIG_USB_DETECT_CHECK)
         mxc_register_device(&mxc_5vOrUsb_detect_device, &mxc_5vOrUsb_detect_data);
#endif

	mx50_rdp_add_led_gpio_device();

	/* SD3 for boot not need gpmi nand flash*/
	//mxc_register_device(&gpmi_nfc_device, &gpmi_nfc_platform_data);
	mx5_usb_dr_init();
	mx5_usbh1_init();
	mxc_register_device(&mxc_perfmon, &mxc_perfmon_data);
	//mx50_rdp_gpio_init();
	//read_power_register();
}

static void __init mx50_rdp_timer_init(void)
{
	struct clk *uart_clk;

	mx50_clocks_init(32768, 24000000, 22579200);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx50_rdp_timer_init,
};

static void __init fixup_android_board(struct machine_desc *desc, struct tag *tags,
									   char **cmdline, struct meminfo *mi)
{
	int pmem_size = 0;//android_pmem_data.size;
	int pmem_gpu_size = android_pmem_gpu_data.size;
	struct tag *mem_tag = 0;
	int total_mem = 0;

	mxc_set_cpu_type(MXC_CPU_MX50);

	get_cpu_wp = mx50_rdp_get_cpu_wp;
	set_num_cpu_wp = mx50_rdp_set_num_cpu_wp;
	get_dvfs_core_wp = mx50_rdp_get_dvfs_core_table;
	num_cpu_wp = ARRAY_SIZE(cpu_wp_auto);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			break;
		}
	}

	if (mem_tag) {

		android_pmem_data.start = mem_tag->u.mem.start
			+ total_mem - pmem_gpu_size - pmem_size;
		android_pmem_gpu_data.start = android_pmem_data.start
			+ pmem_size;

		mem_tag->u.mem.size =
			android_pmem_data.start - mem_tag->u.mem.start;

	}
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX50_RDP data structure.
 */
MACHINE_START(MX50_RDP, "Freescale MX50 Reference Design Platform")
	/* Maintainer: Freescale Semiconductor, Inc. */
#ifdef CONFIG_ANDROID_PMEM
	.fixup = fixup_android_board,
#else
	.fixup = fixup_mxc_board,
#endif
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END


static int proc_keylock_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	*eof = 1;

	//printk("%s: %d\n", __func__, iXrzSuspendMode);

	return snprintf(page, PAGE_SIZE, "%lu\n", iXrzSuspendMode);
}

static int proc_keylock_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[16];

	if (count > sizeof(buf) -1 )
		return -EINVAL;

	if (!count)
		return 0;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';

	switch (buf[0]) {
		case '0':
			iXrzSuspendMode = 0;
			break;
		default:
			iXrzSuspendMode = 1;
			break;
	}

	//printk("%s: %d\n", __func__, iXrzSuspendMode);

	return count;
}

int __init deepsleep_init(void)
{
	unsigned int i;
	struct proc_dir_entry *dir, *file;

	dir = proc_mkdir("keylock", NULL);
	if (!dir) {
		printk("could not create /proc/keylock\n");
		return -1;
	}

	file = create_proc_entry("lock", S_IRUGO | S_IWUGO, dir);
	if (! file) {
		printk("could not create /proc/keylock/lock\n");
		return -1;
	}

	file->data = NULL;
	file->read_proc = proc_keylock_read;
	file->write_proc = proc_keylock_write;

	return 0;
}

late_initcall(deepsleep_init);
