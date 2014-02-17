/*
 * (C) Copyright 2002-2006
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * To match the U-Boot user interface on ARM platforms to the U-Boot
 * standard (as on PPC platforms), some messages with debug character
 * are removed from the default U-Boot build.
 *
 * Define DEBUG here if you want additional info as shown below
 * printed upon startup:
 *
 * U-Boot code: 00F00000 -> 00F3C774  BSS: -> 00FC3274
 * IRQ Stack: 00ebff7c
 * FIQ Stack: 00ebef7c
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <stdio_dev.h>
#include <timestamp.h>
#include <version.h>
#include <net.h>
#include <serial.h>
#include <nand.h>
#include <onenand_uboot.h>
#include <mmc.h>

#include <asm/io.h>
#include <asm/arch/mx50.h>
#include <asm/arch/mx50_pins.h>
#include <asm/arch/iomux.h>


#ifdef CONFIG_DRIVER_SMC91111
#include "../drivers/net/smc91111.h"
#endif
#ifdef CONFIG_DRIVER_LAN91C96
#include "../drivers/net/lan91c96.h"
#endif

DECLARE_GLOBAL_DATA_PTR;

ulong monitor_flash_len;

#ifdef CONFIG_HAS_DATAFLASH
extern int  AT91F_DataflashInit(void);
extern void dataflash_print_info(void);
#endif

#if defined CONFIG_SPLASH_SCREEN// && defined CONFIG_VIDEO_MX5
//extern void setup_splash_image(void);
extern int setup_splash_img();

extern int setup_lowbattary_splash_img();
#endif

extern int check_recovery_cmd_file(void);
//extern  int check_recovery_cmd_state_file(void);

#ifndef CONFIG_IDENT_STRING
#define CONFIG_IDENT_STRING ""
#endif

const char version_string[] =
	U_BOOT_VERSION" (" U_BOOT_DATE " - " U_BOOT_TIME ")"CONFIG_IDENT_STRING;

#ifdef CONFIG_DRIVER_CS8900
extern void cs8900_get_enetaddr (void);
#endif

#ifdef CONFIG_DRIVER_RTL8019
extern void rtl8019_get_enetaddr (uchar * addr);
#endif

#if defined(CONFIG_HARD_I2C) || \
    defined(CONFIG_SOFT_I2C)
#include <i2c.h>
#endif
#ifdef CONFIG_IMX_CSPI
//#include "mc13892_check_battery.h"
extern void check_battery(void);
#endif
/*
 * Begin and End of memory area for malloc(), and current "brk"
 */
static ulong mem_malloc_start = 0;
static ulong mem_malloc_end = 0;
static ulong mem_malloc_brk = 0;

static
void mem_malloc_init (ulong dest_addr)
{
	mem_malloc_start = dest_addr;
	mem_malloc_end = dest_addr + CONFIG_SYS_MALLOC_LEN;
	mem_malloc_brk = mem_malloc_start;

	memset ((void *) mem_malloc_start, 0,
			mem_malloc_end - mem_malloc_start);
}

void *sbrk (ptrdiff_t increment)
{
	ulong old = mem_malloc_brk;
	ulong new = old + increment;

	if ((new < mem_malloc_start) || (new > mem_malloc_end)) {
		return (NULL);
	}
	mem_malloc_brk = new;

	return ((void *) old);
}

extern int brig_switch;
extern int brig_val;
extern int  brig_switch_val(void);
void frontlight_config(void);
int frontlight_brightness_config(int duty_cycles);
void frontlight_on(void);

/************************************************************************
 * Coloured LED functionality
 ************************************************************************
 * May be supplied by boards if desired
 */
void inline __coloured_LED_init (void) {}
void coloured_LED_init(void)__attribute__((weak, alias("__coloured_LED_init")));
void inline __red_LED_on (void) {}
void red_LED_on(void) __attribute__((weak, alias("__red_LED_on")));
void inline __red_LED_off(void) {}
void red_LED_off(void)       __attribute__((weak, alias("__red_LED_off")));
void inline __green_LED_on(void) {}
void green_LED_on(void) __attribute__((weak, alias("__green_LED_on")));
void inline __green_LED_off(void) {}
void green_LED_off(void) __attribute__((weak, alias("__green_LED_off")));
void inline __yellow_LED_on(void) {}
void yellow_LED_on(void) __attribute__((weak, alias("__yellow_LED_on")));
void inline __yellow_LED_off(void) {}
void yellow_LED_off(void) __attribute__((weak, alias("__yellow_LED_off")));
void inline __blue_LED_on(void) {}
void blue_LED_on(void) __attribute__((weak, alias("__blue_LED_on")));
void inline __blue_LED_off(void) {}
void blue_LED_off(void) __attribute__((weak, alias("__blue_LED_off")));

/************************************************************************
 * Init Utilities							*
 ************************************************************************
 * Some of this code should be moved into the core functions,
 * or dropped completely,
 * but let's get it working (again) first...
 */

#if defined(CONFIG_ARM_DCC) && !defined(CONFIG_BAUDRATE)
#define CONFIG_BAUDRATE 115200
#endif
static int init_baudrate (void)
{
	char tmp[64];	/* long enough for environment variables */
	int i = getenv_r ("baudrate", tmp, sizeof (tmp));
	gd->bd->bi_baudrate = gd->baudrate = (i > 0)
			? (int) simple_strtoul (tmp, NULL, 10)
			: CONFIG_BAUDRATE;

	return (0);
}

static int display_banner (void)
{
	printf ("\n\n%s\n\n", version_string);
	debug ("U-Boot code: %08lX -> %08lX  BSS: -> %08lX\n",
	       _armboot_start, _bss_start, _bss_end);
#ifdef CONFIG_MODEM_SUPPORT
	debug ("Modem Support enabled\n");
#endif
#ifdef CONFIG_USE_IRQ
	debug ("IRQ Stack: %08lx\n", IRQ_STACK_START);
	debug ("FIQ Stack: %08lx\n", FIQ_STACK_START);
#endif

	return (0);
}

/*
 * WARNING: this code looks "cleaner" than the PowerPC version, but
 * has the disadvantage that you either get nothing, or everything.
 * On PowerPC, you might see "DRAM: " before the system hangs - which
 * gives a simple yet clear indication which part of the
 * initialization if failing.
 */
static int display_dram_config (void)
{
	int i;

#ifdef DEBUG
	puts ("RAM Configuration:\n");

	for(i=0; i<CONFIG_NR_DRAM_BANKS; i++) {
		printf ("Bank #%d: %08lx ", i, gd->bd->bi_dram[i].start);
		print_size (gd->bd->bi_dram[i].size, "\n");
	}
#else
	ulong size = 0;

	for (i=0; i<CONFIG_NR_DRAM_BANKS; i++) {
		size += gd->bd->bi_dram[i].size;
	}
	puts("DRAM:  ");
	print_size(size, "\n");
#endif

	return (0);
}

#ifndef CONFIG_SYS_NO_FLASH
static void display_flash_config (ulong size)
{
	puts ("Flash: ");
	print_size (size, "\n");
}
#endif /* CONFIG_SYS_NO_FLASH */

#if defined(CONFIG_HARD_I2C) || defined(CONFIG_SOFT_I2C)
static int init_func_i2c (void)
{
	puts ("I2C:   ");
	i2c_init (CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	puts ("ready\n");
	return (0);
}
#endif

#if defined(CONFIG_CMD_PCI) || defined (CONFIG_PCI)
#include <pci.h>
static int arm_pci_init(void)
{
	pci_init();
	return 0;
}
#endif /* CONFIG_CMD_PCI || CONFIG_PCI */

static int init_power(void)
{
	mx50_rdp_power_button_init();
     return 0;
}
/////////////////////////////////////////////////////
///wen add
///for low battary notice
//0:false; 1:true.
////////////////////////////////////////////////////
static int iLowBattaryValue =0;
////////////////////////////////////////////////////
#define  TOTAL_COUNT   4/*3 */
#define  SHUTDOWN_COUNT   (TOTAL_COUNT-1)
extern int checkDeviceBootReason(void);

#if defined(CONFIG_CMD_SPI)
#include <asm/arch/mx50.h>
#include <asm/arch/mx50_pins.h>
#include <asm/arch/iomux.h>
#include <imx_spi.h>
#include <asm/arch/imx_spi_pmic.h>

extern struct spi_slave *myslave;
extern int get_battery_voltage(void);

#define MC13892_REG_CHARGE 48
#define MC13892_READ_VALUE_NUM   10
#define MC13892_MIN_BATTARY_VALUE  3650 /*3600/*4000*/
#define MC13892_MIN_CHARGING_VALUE  4000 /*1500*/

/*check this funtion is OK*/
static int pmic_get_batt_vol()
{
	int adcValue=0, i=0;
	int adcCurValue=0;
	int adctotalValue =0;
	unsigned int val;

	debug("%s %s %d \n",__FILE__,__func__,__LINE__);
	val = pmic_reg(myslave, 43, 0, 0);/*Register 43, ADC 0 */
	/* enable battery current reading; */
	val |= 0x4;
	pmic_reg(myslave, 43, val, 1);	

	for (i = 0; i < MC13892_READ_VALUE_NUM; i++) {
		val = 0x1 | (0x43<<8)| (0x71<<16);
		pmic_reg(myslave, 44, val, 1);	
		udelay(1000); 

		val = pmic_reg(myslave, 45, 0, 0);/*Register 45, ADC 2 */
		adcValue = (val&0xfff)>>2;/*read  battery voltage value */

		debug("val = 0x%x, adcValue=0x%x, battery voltage=%d mV\n", val, adcValue, (adcValue*4687)/1000);	
		debug("val = 0x%x, adcValue=0x%x, voltage=%d mV\n", val, adcValue, (adcValue*4687)/1024);	

		adcCurValue = (adcValue*4687)/1024;
		debug("i=%d adcCurValue=%d\n",i,adcCurValue);

		adctotalValue = adctotalValue + adcCurValue;
		adcCurValue = 0;
	}

	printf(" (adctotalValue/MC13892_READ_VALUE_NUM) =%d mV \n",(adctotalValue/MC13892_READ_VALUE_NUM));
	return (adctotalValue/MC13892_READ_VALUE_NUM);
}
/*check this funtion is OK*/
static int pmic_get_charging_vol()
{
	int rtn=0;
	int adcValue=0, i=0, adcValue2=0;
	unsigned int val;
	int adcChargingCurValue=0;
	int adcChargingTotalValue =0;

	val = pmic_reg(myslave, 43, 0, 0);
	val |= 0x4;
	pmic_reg(myslave, 43, val, 1);	

	for(i=0;i<MC13892_READ_VALUE_NUM;i++)
	{
		val = 0x1 | (0x43<<8)| (0x71<<16);
		pmic_reg(myslave, 44, val, 1);	
		udelay(1000); 

		val = pmic_reg(myslave, 45, 0, 0);/*Register 45, ADC 2 */
		adcValue2 = (val>>14)&0x1ff; /*read charge voltage value */

		adcChargingCurValue = (adcValue2*235)/20;
		//printf(" i=%d, adcChargingCurValue =%d mV \n",i,adcChargingCurValue);

		adcChargingTotalValue = adcChargingTotalValue + adcChargingCurValue;

		adcChargingCurValue = 0;
	}
	printf(" (adcChargingTotalValue/MC13892_READ_VALUE_NUM) =%d mV \n",(adcChargingTotalValue/MC13892_READ_VALUE_NUM));

	return (adcChargingTotalValue/MC13892_READ_VALUE_NUM);

}
int pmic_get_charger_connect_state(void)
{
	int adcChargingCurValue=0;
	int  bChargerInsertState = 0;/*0:no insert Charger; 1: insert Charger */

	adcChargingCurValue = pmic_get_charging_vol();

	printf("%s %s %d adcChargingCurValue =%d \n",__FILE__,__func__,__LINE__ ,adcChargingCurValue);

	if(adcChargingCurValue > MC13892_MIN_CHARGING_VALUE)
		bChargerInsertState = 1;
	else 
		bChargerInsertState =0;	


	return bChargerInsertState;
}
///// PWRON3//////
///disable the PWRON3
static void pmic_disable_autoreset()
{
	unsigned int val;

	//printf("%s %s %d \n",__FILE__,__func__,__LINE__);

	val = pmic_reg(myslave, 15, 0, 0);

	val&= (~0x8);/*1:enable  reset;0:disable  reset*/

	pmic_reg(myslave, 15, val, 1);	

}
///// PWRON3//////
///enable the PWRON3
static void pmic_enable_autoreset()
{
	unsigned int val;

	//printf("%s %s %d \n",__FILE__,__func__,__LINE__);

	val = pmic_reg(myslave, 15, 0, 0);

	val |= (0x8);/*1:enable  reset;0:disable  reset*/

	pmic_reg(myslave, 15, val, 1);	
}


//static void pmic_shutdown_user()
void pmic_shutdown_user()	
{

	unsigned int val;
	printf("pmic_shutdown_user\n");

	val = pmic_reg(myslave, 13, 0, 0);/*REG_POWER_CTRL0 address 13;  */

	//val |= 0x8;/*SPI command for entering user off modes; it use 5mA current if use 5v Vbus, it use 1mA current if use battery */
	val |= 0x000008;
	debug("pmic_shutdown_user :val2 =%d\n",val);  
	pmic_reg(myslave, 13, val, 1);	
	spi_pmic_free(myslave); 

}

static int myBtnInit(void) 
{
	unsigned int val;
	int icount=0;
	int adcChargingCurValue=0;
	int adcCurValue=0;
	//int  iRecovery=0;
	int iDeviceBootReason = -1;
	int  bChargerInsertState = 0;/*0:no insert Charger; 1: insert Charger */

	debug("%s %s %d \n",__FILE__,__func__,__LINE__);

	pmic_disable_autoreset();

	//mx50_rdp_power_button_init();



	bChargerInsertState = pmic_get_charger_connect_state();

	iDeviceBootReason = checkDeviceBootReason();
    	printf("%s bChargerInsertState= %d , iRecovery= %d, \n",__func__,bChargerInsertState,iDeviceBootReason);

	if ((bChargerInsertState == 0) && (iDeviceBootReason != 2)) {
		icount = 0;

		//////////////////////////////////
		///we need ckeck the battary value,
		///for display open logo or lowbattary logo.
		///////////////////////////////////
		//adcCurValue = pmic_get_batt_vol();
		adcCurValue = get_battery_voltage();		
		debug("%s %s %d adcCurValue = %d \n", __FILE__, __func__, __LINE__, adcCurValue);

		//check the voltage
		if (adcCurValue < MC13892_MIN_BATTARY_VALUE) {
			iLowBattaryValue = 1;
		}
		else {
			iLowBattaryValue = 0;
		}

//		if (iLowBattaryValue == 1)
//			pmic_shutdown_user();
		
#if 0
		// TODO: I don't understand why this code is here
		while (1) {
			val = 0;
			
			/* read register 5 for check whether the power button had been pressed */
			val = pmic_reg(myslave, 5, 0, 0);
			if ((val&(1<<2))==0) {
				icount++;		     
				if (icount > TOTAL_COUNT) {
					debug("%s %s %d icount = %d\n", __FILE__, __func__, __LINE__, icount);
					break;
				}
			}
			else {
				debug("%s %s %d icount = %d\n", __FILE__, "Key up", __LINE__, icount);
				break;
			}
			
			udelay(1000);
		}

		if (icount > 0 && icount < SHUTDOWN_COUNT) {
			debug("%s %s %d icount = %d\n", __FILE__, "pmic_shutdown_user", __LINE__, icount);
			pmic_shutdown_user();
		}
#endif		
		//spi_pmic_free(myslave); 
	}

	return 0;
}

#elif defined(CONFIG_I2C_MXC)
#endif
/*
 * Breathe some life into the board...
 *
 * Initialize a serial port as console, and carry out some hardware
 * tests.
 *
 * The first part of initialization is running from Flash memory;
 * its main purpose is to initialize the RAM so that we
 * can relocate the monitor code to RAM.
 */

/*
 * All attempts to come up with a "common" initialization sequence
 * that works for all boards and architectures failed: some of the
 * requirements are just _too_ different. To get rid of the resulting
 * mess of board dependent #ifdef'ed code we now make the whole
 * initialization sequence configurable to the user.
 *
 * The requirements for any new initalization function is simple: it
 * receives a pointer to the "global data" structure as it's only
 * argument, and returns an integer return code, where 0 means
 * "continue" and != 0 means "fatal error, hang the system".
 */
typedef int (init_fnc_t) (void);

int print_cpuinfo (void);

init_fnc_t *init_sequence[] = {
#if defined(CONFIG_ARCH_CPU_INIT)
	arch_cpu_init,		/* basic arch cpu dependent setup */
#endif
	board_init,		/* basic board dependent setup */
#if defined(CONFIG_USE_IRQ)
	interrupt_init,		/* set up exceptions */
#endif
	timer_init,		/* initialize timer */
	env_init,		/* initialize environment */
	init_baudrate,		/* initialze baudrate settings */
	serial_init,		/* serial communications setup */
	console_init_f,		/* stage 1 init of console */
	display_banner,		/* say that we are here */
#if defined(CONFIG_DISPLAY_CPUINFO)
	print_cpuinfo,		/* display cpu info (and speed) */
#endif
#if defined(CONFIG_DISPLAY_BOARDINFO)
	checkboard,		/* display board info */
#endif
	//init_power,
#if defined(CONFIG_HARD_I2C) || defined(CONFIG_SOFT_I2C)
	init_func_i2c,
#endif
	dram_init,		/* configure available RAM banks */
#if defined(CONFIG_CMD_PCI) || defined (CONFIG_PCI)
	arm_pci_init,
#endif
	display_dram_config,
	NULL,
};

typedef enum {
	FALSE = 0,
	TRUE = 1
} boolean_t;
#if defined CONFIG_SPLASH_SCREEN
	int bmp_not_loaded = 1;
#endif

void start_armboot (void)
{
	init_fnc_t **init_fnc_ptr;
	char *s;
#if defined(CONFIG_VFD) || defined(CONFIG_LCD)
	unsigned long addr;
#endif

	/* Pointer is writable since we allocated a register for it */
	gd = (gd_t*)(_armboot_start - CONFIG_SYS_MALLOC_LEN - sizeof(gd_t));
	/* compiler optimization barrier needed for GCC >= 3.4 */
	__asm__ __volatile__("": : :"memory");

	memset ((void*)gd, 0, sizeof (gd_t));
	gd->bd = (bd_t*)((char*)gd - sizeof(bd_t));
	memset (gd->bd, 0, sizeof (bd_t));

	gd->flags |= GD_FLG_RELOC;

	monitor_flash_len = _bss_start - _armboot_start;

	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}

	/* armboot_start is defined in the board-specific linker script */
	mem_malloc_init (_armboot_start - CONFIG_SYS_MALLOC_LEN);

#ifdef CONFIG_IMX_CSPI
	check_battery();
#endif

#ifndef CONFIG_SYS_NO_FLASH
	/* configure available FLASH banks */
	display_flash_config (flash_init ());
#endif /* CONFIG_SYS_NO_FLASH */

#ifdef CONFIG_VFD
#	ifndef PAGE_SIZE
#	  define PAGE_SIZE 4096
#	endif
	/*
	 * reserve memory for VFD display (always full pages)
	 */
	/* bss_end is defined in the board-specific linker script */
	addr = (_bss_end + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1);
	vfd_setmem (addr);
	gd->fb_base = addr;
#endif /* CONFIG_VFD */

#ifdef CONFIG_LCD
	/* board init may have inited fb_base */
	if (!gd->fb_base) {
#		ifndef PAGE_SIZE
#		  define PAGE_SIZE 4096
#		endif
		/*
		 * reserve memory for LCD display (always full pages)
		 */
		/* bss_end is defined in the board-specific linker script */
		addr = (_bss_end + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1);
		lcd_setmem (addr);
		gd->fb_base = addr;
	}
#endif /* CONFIG_LCD */

#if defined(CONFIG_CMD_NAND)
	puts ("NAND:  ");
	nand_init();		/* go init the NAND */
#endif

#if defined(CONFIG_CMD_ONENAND)
	onenand_init();
#endif

#ifdef CONFIG_HAS_DATAFLASH
	AT91F_DataflashInit();
	dataflash_print_info();
#endif

#ifdef CONFIG_GENERIC_MMC
	puts ("MMC:   ");
	mmc_initialize (gd->bd);
#endif

	/* initialize environment */
	env_relocate ();

#ifdef CONFIG_VFD
	/* must do this after the framebuffer is allocated */
	drv_vfd_init();
#endif /* CONFIG_VFD */

#ifdef CONFIG_SERIAL_MULTI
	serial_initialize();
#endif

#ifdef BOARD_LATE_INIT
	board_late_init ();
#endif
#ifdef BOARD_HOLD_POWER_ON
	myBtnInit();
#endif
	/* IP Address */
	gd->bd->bi_ip_addr = getenv_IPaddr ("ipaddr");

	//show only LOW_BATTERY_LOGO_SPLASH, but first normal boot must clear screen.
	if(iLowBattaryValue == 0 /*&& get_screen_init_flag() == TRUE*/){
		//set_screen_init_flag(FALSE);
#if defined CONFIG_SPLASH_SCREEN
		bmp_not_loaded = setup_splash_img();
#endif
	} else if(iLowBattaryValue ==1)
	{
#if defined CONFIG_SPLASH_SCREEN  
		bmp_not_loaded = setup_lowbattary_splash_img();
#endif
		///logo had display,we need check whether need power off.
		//mx50_rdp_lowBattary_poweroff();
	}

	stdio_init ();	/* get the devices list going. */

	//shutdown after show splash img when low power
	if (iLowBattaryValue == 1) {
		//set_screen_init_flag(TRUE); //need for set clean screen before show
		pmic_shutdown_user();
		udelay(1000 * 1000);
	}
	//Frontlight is off if low power
	if(iLowBattaryValue == 0) {
		// init frontlight
		brig_switch_val();
		if((int)brig_switch == 1){
			frontlight_config();
			frontlight_brightness_config(brig_val);
			udelay(50 * 1000);
			frontlight_on();
		}
	}

	jumptable_init ();

#if defined(CONFIG_API)
	/* Initialize API */
	api_init ();
#endif

	console_init_r ();	/* fully init console as a device */


#if defined(CONFIG_ARCH_MISC_INIT)
	/* miscellaneous arch dependent initialisations */
	arch_misc_init ();
#endif
#if defined(CONFIG_MISC_INIT_R)
	/* miscellaneous platform dependent initialisations */
	misc_init_r ();
#endif

	/* enable exceptions */
	enable_interrupts ();

	/* Perform network card initialisation if necessary */
#ifdef CONFIG_DRIVER_TI_EMAC
	/* XXX: this needs to be moved to board init */
extern void davinci_eth_set_mac_addr (const u_int8_t *addr);
	if (getenv ("ethaddr")) {
		uchar enetaddr[6];
		eth_getenv_enetaddr("ethaddr", enetaddr);
		davinci_eth_set_mac_addr(enetaddr);
	}
#endif

#ifdef CONFIG_DRIVER_CS8900
	/* XXX: this needs to be moved to board init */
	cs8900_get_enetaddr ();
#endif

#if defined(CONFIG_DRIVER_SMC91111) || defined (CONFIG_DRIVER_LAN91C96)
	/* XXX: this needs to be moved to board init */
	if (getenv ("ethaddr")) {
		uchar enetaddr[6];
		eth_getenv_enetaddr("ethaddr", enetaddr);
		smc_set_mac_addr(enetaddr);
	}
#endif /* CONFIG_DRIVER_SMC91111 || CONFIG_DRIVER_LAN91C96 */

#if defined(CONFIG_ENC28J60_ETH) && !defined(CONFIG_ETHADDR)
	extern void enc_set_mac_addr (void);
	enc_set_mac_addr ();
#endif /* CONFIG_ENC28J60_ETH && !CONFIG_ETHADDR*/

	/* Initialize from environment */
	if ((s = getenv ("loadaddr")) != NULL) {
		load_addr = simple_strtoul (s, NULL, 16);
	}
#if defined(CONFIG_CMD_NET)
	if ((s = getenv ("bootfile")) != NULL) {
		copy_filename (BootFile, s, sizeof (BootFile));
	}
#endif


//#ifdef CONFIG_ANDROID_RECOVERY
//	check_recovery_mode();
//#endif

#if defined(CONFIG_CMD_NET)
#if defined(CONFIG_NET_MULTI)
	puts ("Net:   ");
#endif
	eth_initialize(gd->bd);
#if defined(CONFIG_RESET_PHY_R)
	debug ("Reset Ethernet PHY\n");
	reset_phy();
#endif
#endif
	/* main_loop() can return to retry autoboot, if so just run it again. */
	for (;;) {
		main_loop ();
	}

	/* NOTREACHED - no way out of command loop except booting */
}

void hang (void)
{
	puts ("### ERROR ### Please RESET the board ###\n");
	for (;;);
}
