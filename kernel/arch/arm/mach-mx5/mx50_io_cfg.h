/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#if defined(CONFIG_ARCH_MX50)
#ifndef __MX50_IO_CFG_H__
#define __MX50_IO_CFG_H__




#define MX508_RDP_GPIO4_19_SD1_WP	(3*32 + 19)	/*GPIO_4_19 */
#define MX508_RDP_GPIO1_27_SD1_CD	(0*32 + 27)	/*GPIO_1_27 */

#if defined(CONFIG_E_INK_EDPC)
#define EPDC_D0		(2*32 + 0)	/*GPIO_3_0 */
#define EPDC_D1		(2*32 + 1)	/*GPIO_3_1 */
#define EPDC_D2		(2*32 + 2)	/*GPIO_3_2 */
#define EPDC_D3		(2*32 + 3)	/*GPIO_3_3 */
#define EPDC_D4		(2*32 + 4)	/*GPIO_3_4 */
#define EPDC_D5		(2*32 + 5)	/*GPIO_3_5 */
#define EPDC_D6		(2*32 + 6)	/*GPIO_3_6 */
#define EPDC_D7		(2*32 + 7)	/*GPIO_3_7 */
#define EPDC_D8		(2*32 + 8)	/*GPIO_3_8 */
#define EPDC_D9		(2*32 + 9)	/*GPIO_3_9 */
#define EPDC_D10		(2*32 + 10)/*GPIO_3_10 */
#define EPDC_D11		(2*32 + 11)/*GPIO_3_11 */
#define EPDC_D12		(2*32 + 12)/*GPIO_3_12 */
#define EPDC_D13		(2*32 + 13)/*GPIO_3_13 */
#define EPDC_D14		(2*32 + 14)/*GPIO_3_14 */
#define EPDC_D15		(2*32 + 15)/*GPIO_3_15 */
#define EPDC_GDCLK	(2*32 + 16)	/*GPIO_3_16 */
#define EPDC_GDSP	(2*32 + 17)	/*GPIO_3_17 */
#define EPDC_GDOE	(2*32 + 18)	/*GPIO_3_18 */
#define EPDC_GDRL	(2*32 + 19)	/*GPIO_3_19 */
#define EPDC_SDCLK	(2*32 + 20)	/*GPIO_3_20 */
#define EPDC_SDOEZ	(2*32 + 21)	/*GPIO_3_21 */
#define EPDC_SDOED	(2*32 + 22)	/*GPIO_3_22 */
#define EPDC_SDOE	(2*32 + 23)	/*GPIO_3_23 */
#define EPDC_SDLE	        (2*32 + 24)	/*GPIO_3_24 */
#define EPDC_SDCLKN	(2*32 + 25)	/*GPIO_3_25 */
#define EPDC_SDSHR	(2*32 + 26)	/*GPIO_3_26 */
#define EPDC_PWRCOM	(2*32 + 27)	/*GPIO_3_27 */
#define EPDC_PWRSTAT	(2*32 + 28)	/*GPIO_3_28 */
#define EPDC_PWRCTRL0	(2*32 + 29)	/*GPIO_3_29 */
#define EPDC_PWRCTRL1	(2*32 + 30)	/*GPIO_3_30 */
#define EPDC_PWRCTRL2	(2*32 + 31)	/*GPIO_3_31 */
#define EPDC_PWRCTRL3	(3*32 + 20)	/*GPIO_4_20 */
#define EPDC_BDR0	(3*32 + 23)	/*GPIO_4_23 */
#define EPDC_BDR1	(3*32 + 24)	/*GPIO_4_24 */
#define EPDC_SDCE0	(3*32 + 25)	/*GPIO_4_25 */
#define EPDC_SDCE1	(3*32 + 26)	/*GPIO_4_26 */
#define EPDC_SDCE2	(3*32 + 27)	/*GPIO_4_27 */
#define EPDC_SDCE3	(3*32 + 28)	/*GPIO_4_28 */
#define EPDC_SDCE4	(3*32 + 29)	/*GPIO_4_29 */
#define EPDC_SDCE5	(3*32 + 30)	/*GPIO_4_30 */
#define EPDC_PMIC_WAKE		(5*32 + 16)	/*GPIO_6_16 */
#define EPDC_PMIC_INT		(5*32 + 17)	/*GPIO_6_17 */
#define EPDC_VCOM	(3*32 + 21)	/*GPIO_4_21 */
#define EPDC_PWRSTAT	(2*32 + 28)	/*GPIO_3_28 */
#define ELCDIF_PWR_ON (board_is_mx50_rd3() ? (1*32 + 18) : (1*32 + 21))

#endif


#if  defined(CONFIG_MFD_PM_SENSOR_LM75A)
#define LM75A_EPDC_PWRCTRL0	(2*32 + 29)	/*GPIO_3_29 */
#define LM75A_EPDC_PWRCTRL1	(2*32 + 30)	/*GPIO_3_30 */
#define LM75A_EPDC_PWRCTRL2	(2*32 + 31)	/*GPIO_3_31 */
#define LM75A_EPDC_VCOM	(3*32 + 21)	/*GPIO_4_21 */
#define LM75A_EPDC_PMIC_WAKE		(5*32 + 16)	/*GPIO_6_16 */
#endif

////////////////////////////////////////////////////////////
////////elan touch ic define
///////////////////////////////////////////////////////////
#if defined(CONFIG_MXC_TP_ELAN)
#define TOUCH_PANEL_IRQ 	  (4*32 + 13)/*GPIO5_13*/
#define TOUCH_PANEL_RESET (4*32 + 14)/*GPIO5_14*/
#define TOUCH_PANEL_SCL     (5*32+18)/*GPIO6_18*/
#define TOUCH_PANEL_SDA    (5*32+19)/*GPIO6_19*/
#endif


////////////////////////////////////////////////////////////
////////gt811 touch ic define
///////////////////////////////////////////////////////////
#if defined(CONFIG_MXC_GT813)
#define GT813_TOUCH_PANEL_IRQ 	  (4*32 + 13)/*GPIO5_13*/
#define GT813_TOUCH_PANEL_RESET (4*32 + 14)/*GPIO5_14*/
#define GT813_TOUCH_PANEL_SCL     (5*32+18)/*GPIO6_18*/
#define GT813_OUCH_PANEL_SDA    (5*32+19)/*GPIO6_19*/
#endif

////////////////////////////////////////////////////////////
////////ata5701 touch key
///////////////////////////////////////////////////////////
#if defined(CONFIG_ATA5701_TOUCH)
#define MX50_RDP_ATA5701_IRQ    (4*32+7)/*GPIO5_7*/
#endif
////////////////////////////////////////////////////////////
////////CY8C21434 touch key
///////////////////////////////////////////////////////////
#if defined(CONFIG_CY8C_TOUCH)
#define MX50_RDP_CY8C_IRQ    (4*32+7)/*GPIO5_7*/
#define  MX50_RDP_CY8C_RESET   (3*32+2)/*GPIO4_2*/
#endif
////////////////////////////////////////////////////////////
////////usb wifi 8192cu defined
///////////////////////////////////////////////////////////
#if defined(CONFIG_USB_WIFI_RTL8192CU)

#define USBWIFI_PWER_EN                          (3*32 + 17)/*GPIO4_17*/
//#define USBWIFI_3G_POWER_CONTROL    (1*32 +19 )/*GPIO2_19*/
//#define USBWIFI_3G_SWAP_ENABLE           (3*32 + 16)/*GPIO4_16*/


#endif



////////////////////////////////////////////////////////////
////////encrypt gpio define
///////////////////////////////////////////////////////////
#if defined(CONFIG_MXC_ENCRYPT_GPIO)
#define MX50_RDP_UART1_CTS_GPIO6_8_ENCRYPT_SDA  (5*32+8)
#define MX50_RDP_EPITO_GPIO6_27_ENCRYPT_SCL    (5*32+27)
#endif


///////////////////////////////////////////////////////////
////////////check usb insert or not 
//////////////////////////////////////////////////////////
#if defined(CONFIG_USB_DETECT_CHECK)
/*DCDC Insert detect */
#define MX50_RDP_DCDC_INSERT_CHARGE_DETECT   (4*32+16)/*GPIO5_16 */
#endif


////////////////////////////////////////////////////////////

////////sgtl5000  gpio define
///////////////////////////////////////////////////////////
#if defined(CONFIG_SND_SOC_SGTL5000)
///sgtl5000 enable or disabled  :H,enabled ,L,disabled.
#define  MX50_RDP_SGTL5000_EN      (5*32+26)/*GPIO6_26*/

///headphone insert detect
#define MX50_RDP_HEADPHONE_INSERT_DETECT   (3*32+15)/*GPIO4_15 */


#endif

////////////////////////////////////////////////////////////
///////////////MC13892 c036 /////////////////////
////////////////////////////////////////////////////////////
#define MX50_RDP_GPIO4_0_TOUCH_TOTAL_POWER   (3*32+0)/*GPIO4_10*/

///light define... if the device open,sign it
#define  MX50_RDP_GPIO4_4_LIGHT_CONTROL     (3*32+4)/*GPIO4_4*/

///backlight enable or disable control
#define MX50_RDP_BACKLIGHT_ENABLE_CONTROL   (1*32+4)/*GPIO2_4*/

//////charger gpio control,when battary voltage < 3.6v,set to high
#define MX50_RDP_CHARGER_GPIO_CONTROL     (3*32+6)/*GPIO4_6*/
////////////////////////////////////////////////////////////
////////G P I O  B U T T ON  DEFINE
///////////////////////////////////////////////////////////

//////home button/////
#define MX50_RDP_UART3_TXD_GPIO6_14_HOME    (5*32+14)/*GPIO6_14 */

#define  MX50_RDP_DISP_RS_GP2_17_HOME   (1*32+17)/*GPIO2_17*/

#define  MX50_RDP_DISP_RS_GP2_12_DOWN   (1*32+12)/*GPIO2_12*/
////// menu  button /////
#define MX50_RDP_DISP_DAT15_GPIO2_15_MENU    (1*32+15)/*GPIO2_15*/

////////////////////////////////////////////////////////////
///////////////GPIO LED CONTROL/////////////////////
////////////////////////////////////////////////////////////
#if defined (CONFIG_LEDS_GPIO_PLATFORM)
#define  MX50_RDP_GPIO_LEDS_CONTROL     (3*32+4)/*GPIO4_4*/
#endif


#define WDOG_B   (5*32+28)/*GPIO6_28*/
 


#endif/* __MX50_IO_CFG_H__ */
#endif/*CONFIG_ARCH_MX50*/
