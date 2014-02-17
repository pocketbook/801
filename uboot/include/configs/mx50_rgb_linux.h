/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the MX50-RDP Freescale board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/mx50.h>
#include <asm/mxc_key_defs.h>


 /* High Level Configuration Options */
#define CONFIG_MXC
#define CONFIG_MX50
#define CONFIG_MX50_RDP
/*if we use the  mDDR,need commit below*/
//#define CONFIG_DDR2
#define CONFIG_FLASH_HEADER
#define CONFIG_FLASH_HEADER_OFFSET 0x400

#define CONFIG_SKIP_RELOCATE_UBOOT


#define CONFIG_ARCH_CPU_INIT
#define CONFIG_ARCH_MMU


#define CONFIG_MX50_HCLK_FREQ	24000000
#define CONFIG_SYS_PLL2_FREQ    400
#define CONFIG_SYS_AHB_PODF     2
#define CONFIG_SYS_AXIA_PODF    0
#define CONFIG_SYS_AXIB_PODF    1

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_SYS_64BIT_VSPRINTF

#define BOARD_LATE_INIT
/*
 * Disabled for now due to build problems under Debian and a significant
 * increase in the final file size: 144260 vs. 109536 Bytes.
 */

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_REVISION_TAG		1
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 2 * 1024 * 1024)
/* size in bytes reserved for initial data */
#define CONFIG_SYS_GBL_DATA_SIZE	128

/*
 * Hardware drivers
 */
#define CONFIG_MXC_UART
#define CONFIG_UART_BASE_ADDR	UART1_BASE_ADDR

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

/***********************************************************
 * Command definition
 ***********************************************************/

#include <config_cmd_default.h>

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_NET_RETRY_COUNT  100
#define CONFIG_NET_MULTI      1
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_DNS

#define CONFIG_CMD_MMC
#define CONFIG_CMD_ENV

/*#define CONFIG_CMD */
#define CONFIG_REF_CLK_FREQ    CONFIG_MX50_HCLK_FREQ

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY  	3/*3*/

#define CONFIG_PRIME	"FEC0"

#define CONFIG_LOADADDR		0x70800000	/* loadaddr env var */
#define CONFIG_RD_LOADADDR	(CONFIG_LOADADDR + 0x300000)

#define UBOOT_FOR_LINUX
/////project define/////
#define MMC3_BOOTUP
//#define CONFIG_ZT2083_CHARGEMEASURE
#define BOARD_HOLD_POWER_ON
#define BOARD_HAS_TPS_PM_LM75A
//#define  BOARD_HAS_REGULATOR_TPS6518X
//#define MX50RDP_HARDWARE_VERSION_CONTROL
//#define MX50RDP_SOFRWARE_VERSION_CONTROL
//#define BOARD_HAS_HD_SCREEN_1024X758
#define CONFIG_EPDC_RGB



 ///boot from sd3
#define	CONFIG_EXTRA_ENV_SETTINGS\
		"splashimage=0x70800000\0"\ 
		"bootargs_mmc=setenv bootargs ${bootargs} root=/dev/mmcblk0p1 rootwait rw fl_val=${fl_val} fl_switch=${fl_switch}\0" \
		"bootcmd_mmc=run bootargs_base bootargs_mmc; mmc read 2 ${loadaddr} 0x800 0x2000;bootm\0" \
		"bootcmd=run bootcmd_mmc\0"\
		"bootargs_base=setenv bootargs console=ttymxc0,115200 androidboot.console=ttymxc0 video=mxcepdcfb:E80_V250\0"



#define CONFIG_ARP_TIMEOUT	200UL

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */
#define CONFIG_SYS_PROMPT		"MX50_RDP U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256	/* Console I/O Buffer Size */
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE /* Boot Argument Buffer Size */

#define CONFIG_SYS_MEMTEST_START	0	/* memtest works on */
#define CONFIG_SYS_MEMTEST_END		0x10000

#undef	CONFIG_SYS_CLKS_IN_HZ		/* everything, incl board info, in Hz */

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_SYS_HZ				1000

#define CONFIG_CMDLINE_EDITING	1

#define CONFIG_FEC0_IOBASE	FEC_BASE_ADDR
#define CONFIG_FEC0_PINMUX	-1
#define CONFIG_FEC0_PHY_ADDR	-1
#define CONFIG_FEC0_MIIBASE	-1

#define CONFIG_GET_FEC_MAC_ADDR_FROM_IIM

#define CONFIG_MXC_FEC
#define CONFIG_MII
#define CONFIG_MII_GASKET
#define CONFIG_DISCOVER_PHY

#define CONFIG_SPLASH_SCREEN
/*
 * SPLASH SCREEN Configs
 */

#ifdef CONFIG_SPLASH_SCREEN
	#define CONFIG_LCD
	#undef LCD_TEST_PATTERN	
	#define CONFIG_FB_BASE                          (TEXT_BASE + 0x500000)
	
	#define CONFIG_SYS_CONSOLE_IS_IN_ENV            1
	#define CONFIG_SPLASH_IS_IN_MMC                 1 
	#define CONFIG_LOW_BATTARY_SPLASH_IS_IN_MMC   1

	#define LCD_BPP	LCD_MONOCHROME
	//#define LCD_BPP	LCD_COLOR16
	//#define CONFIG_BMP_16BPP
	//#define CONFIG_BMP_8BPP
	//#define CONFIG_ATMEL_LCD_BGR555
	 //#define CONFIG_SPLASH_SCREEN_ALIGN           1 
	// #define CONFIG_CMD_BMP
	// #define CONFIG_VIDEO_BMP_GZIP



	#define CONFIG_MXC_EPDC				1
	#define CONFIG_WORKING_BUF_ADDR			(TEXT_BASE + 0x100000)
	#define CONFIG_WAVEFORM_BUF_ADDR              (TEXT_BASE + 0x400000)
	
	#define CONFIG_WAVEFORM_FILE_OFFSET	0x800000
	#define CONFIG_WAVEFORM_FILE_SIZE			0x200000
	#define CONFIG_WAVEFORM_FILE_IN_MMC
	
#endif /*pair with #ifdef CONFIG_SPLASH_SCREEN */


/*display the uboot logo offset and size*/
#ifdef CONFIG_SPLASH_IS_IN_MMC
	#define CONFIG_SPLASH_IMG_OFFSET		0xA00000/*0x800000*/
	#define CONFIG_SPLASH_IMG_SIZE				0x200000
#endif /*pair with #ifdef CONFIG_SPLASH_IS_IN_MMC */

 /*display the low battary  logo offset and size*/
#ifdef CONFIG_LOW_BATTARY_SPLASH_IS_IN_MMC   
	#define CONFIG_LOW_BATTARY_SPLASH_IMG_OFFSET		0xB00000
	#define CONFIG_LOW_BATTARY_SPLASH_IMG_SIZE			0x100000
#endif

/*  software version control offset and buffer size */
#if defined(MX50RDP_SOFRWARE_VERSION_CONTROL)
	#define CONFIG_SOFTWARE_VERSION_OFFSET		0xC00000
	#define CONFIG_SOFTWARE_BUFFER_SIZE           2048
#endif

/*
 * DDR ZQ calibration
 */
/*#define CONFIG_ZQ_CALIB*/

/*
 * I2C Configs
 */
//#define CONFIG_CMD_I2C          1

#ifdef CONFIG_CMD_I2C
	#define CONFIG_HARD_I2C         1
	#define CONFIG_I2C_MXC          1
	#define CONFIG_SYS_I2C_PORT             I2C2_BASE_ADDR
	#define CONFIG_SYS_I2C_SPEED            100000
	#define CONFIG_SYS_I2C_SLAVE            0xfe
#endif


/*
 * SPI Configs
 */

#define CONFIG_CMD_SPI
 #ifdef CONFIG_CMD_SPI
        #define CONFIG_FSL_SF		1
        #define CONFIG_CMD_SF
        #define CONFIG_SPI_FLASH_IMX_ATMEL	1
        #define CONFIG_SPI_FLASH_CS	1
        #define CONFIG_IMX_CSPI
        #define IMX_CSPI_VER_0_7        1
        #define MAX_SPI_BYTES		(8 * 4)
        #define CONFIG_IMX_SPI_PMIC
        #define CONFIG_IMX_SPI_PMIC_CS 0
#endif

/*
 * MMC Configs
 */
#ifdef CONFIG_CMD_MMC
	#define CONFIG_MMC				1
	#define CONFIG_GENERIC_MMC
	#define CONFIG_IMX_MMC
	#define CONFIG_SYS_FSL_ESDHC_NUM        3
	#define CONFIG_SYS_FSL_ESDHC_ADDR       0
	#define CONFIG_SYS_MMC_ENV_DEV  0
	#define CONFIG_DOS_PARTITION	1
	#define CONFIG_CMD_FAT		1
	#define CONFIG_CMD_EXT2		1

	/* detect whether ESDHC1, ESDHC2, or ESDHC3 is boot device */
	#define CONFIG_DYNAMIC_MMC_DEVNO

	#define CONFIG_BOOT_PARTITION_ACCESS

	/*
	  *  Boot up from emmc,we need comment below
	  *  DDR detect and DDR mode
	  *  wen add. 2012-03-21
	  */
	//#define CONFIG_EMMC_DDR_PORT_DETECT
	//#define CONFIG_EMMC_DDR_MODE

	/* Indicate to esdhc driver which ports support 8-bit data */
	#define CONFIG_MMC_8BIT_PORTS		0x6   /* SD2 and SD3 */

	/* Uncomment the following define to enable uSDHC instead
	 * of eSDHC on SD3 port for SDR mode. since eSDHC timing on MX50
	 * is borderline for SDR mode. DDR mode will be disabled when this
	 * define is enabled since the uSDHC timing on MX50 is borderline
	 * for DDR mode. */

	/*#define CONFIG_MX50_ENABLE_USDHC_SDR	1*/
#endif

/*
 * GPMI Nand Configs
 */
//#define CONFIG_CMD_NAND

#ifdef CONFIG_CMD_NAND
	#define CONFIG_NAND_GPMI
	#define CONFIG_GPMI_NFC_SWAP_BLOCK_MARK
	#define CONFIG_GPMI_NFC_V2

	#define CONFIG_GPMI_REG_BASE	GPMI_BASE_ADDR
	#define CONFIG_BCH_REG_BASE	BCH_BASE_ADDR

	#define NAND_MAX_CHIPS		8
	#define CONFIG_SYS_NAND_BASE		0x40000000
	#define CONFIG_SYS_MAX_NAND_DEVICE	1
#endif

/*
 * APBH DMA Configs
 */
#define CONFIG_APBH_DMA

#ifdef CONFIG_APBH_DMA
	#define CONFIG_APBH_DMA_V2
	#define CONFIG_MXS_DMA_REG_BASE	ABPHDMA_BASE_ADDR
#endif

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128 * 1024)	/* regular stack */

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1
#define PHYS_SDRAM_1		CSD0_BASE_ADDR
#define PHYS_SDRAM_1_SIZE	(256 * 1024 * 1024)
#define iomem_valid_addr(addr, size) \
	(addr >= PHYS_SDRAM_1 && addr <= (PHYS_SDRAM_1 + PHYS_SDRAM_1_SIZE))

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CONFIG_SYS_NO_FLASH

/* Monitor at beginning of flash */
#define CONFIG_FSL_ENV_IN_MMC

#define CONFIG_ENV_SECT_SIZE    (128 * 1024)
#define CONFIG_ENV_SIZE         CONFIG_ENV_SECT_SIZE

#if defined(CONFIG_FSL_ENV_IN_NAND)
	#define CONFIG_ENV_IS_IN_NAND 1
	#define CONFIG_ENV_OFFSET	  0x100000
#elif defined(CONFIG_FSL_ENV_IN_MMC)
	#define CONFIG_ENV_IS_IN_MMC	1
	#define CONFIG_ENV_OFFSET	   (768 * 1024)
#elif defined(CONFIG_FSL_ENV_IN_SF)
	#define CONFIG_ENV_IS_IN_SPI_FLASH	1
	#define CONFIG_ENV_SPI_CS		1
	#define CONFIG_ENV_OFFSET       (768 * 1024)
#else
	#define CONFIG_ENV_IS_NOWHERE	1
#endif



#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS

//#define CONFIG_ANDROID_SYSTEM_PARTITION_MMC 2
//#define CONFIG_ANDROID_RECOVERY_PARTITION_MMC 4
//#define CONFIG_ANDROID_CACHE_PARTITION_MMC 6

#endif				/* __CONFIG_H */

