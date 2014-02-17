/*
 * Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *
 * (C) Copyright 2009-2011 Freescale Semiconductor, Inc.
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

#include <common.h>
#include <asm/io.h>
#include <asm/arch/mx50.h>
#include <asm/arch/mx50_pins.h>
#include <asm/arch/iomux.h>
#include <asm/errno.h>

#ifdef CONFIG_IMX_CSPI
#include <imx_spi.h>
#include <asm/arch/imx_spi_pmic.h>

#include "mc13892_check_battery.h"

struct spi_slave *myslave;

#endif

#if CONFIG_I2C_MXC
#include <i2c.h>
#endif

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

#ifdef CONFIG_ARCH_MMU
#include <asm/mmu.h>
#include <asm/arch/mmu.h>
#endif

#ifdef CONFIG_CMD_CLOCK
#include <asm/clock.h>
#endif

#ifdef CONFIG_MXC_EPDC
#include <lcd.h>
#endif

#ifdef CONFIG_ANDROID_RECOVERY
#include "../common/recovery.h"
#include <part.h>
#include <ext2fs.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <ubi_uboot.h>
#include <jffs2/load_kernel.h>
#endif

DECLARE_GLOBAL_DATA_PTR;
#ifdef CONFIG_LCD
short lcd_cmap[256];
#endif

static u32 system_rev;
static enum boot_device boot_dev;
//extern int iLowBattaryValue;
//extern void zt2083_power_off(void);
int mx50_rdp_power_keep_init(void);
static inline void setup_boot_device(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4 ;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;

	switch (bt_mem_ctl) {
	case 0x0:
		if (bt_mem_type)
			boot_dev = ONE_NAND_BOOT;
		else
			boot_dev = WEIM_NOR_BOOT;
		break;
	case 0x2:
		if (bt_mem_type)
			boot_dev = SATA_BOOT;
		else
			boot_dev = PATA_BOOT;
		break;
	case 0x3:
		if (bt_mem_type)
			boot_dev = SPI_NOR_BOOT;
		else
			boot_dev = I2C_BOOT;
		break;
	case 0x4:
	case 0x5:
		boot_dev = SD_BOOT;
		break;
	case 0x6:
	case 0x7:
		boot_dev = MMC_BOOT;
		break;
	case 0x8 ... 0xf:
		boot_dev = NAND_BOOT;
		break;
	default:
		boot_dev = UNKNOWN_BOOT;
		break;
	}
}

enum boot_device get_boot_device(void)
{
	return boot_dev;
}

u32 get_board_rev(void)
{
	return system_rev;
}

static inline void setup_soc_rev(void)
{
	int reg = __REG(ROM_SI_REV);

	switch (reg) {
	case 0x10:
		system_rev = 0x50000 | CHIP_REV_1_0;
		break;
	case 0x11:
		system_rev = 0x50000 | CHIP_REV_1_1_1;
		break;
	default:
		system_rev = 0x50000 | CHIP_REV_1_1_1;
	}
}

static inline void set_board_rev(int rev)
{
	system_rev |= (rev & 0xF) << 8;
}

static inline void setup_board_rev(void)
{
#if defined(CONFIG_MX50_RD3)
	set_board_rev(0x3);
#endif
}

static inline void setup_arch_id(void)
{
#if defined(CONFIG_MX50_RDP) || defined(CONFIG_MX50_RD3)
	gd->bd->bi_arch_number = MACH_TYPE_MX50_RDP;
#elif defined(CONFIG_MX50_ARM2)
	gd->bd->bi_arch_number = MACH_TYPE_MX50_ARM2;
#else
#	error "Unsupported board!"
#endif
}

inline int is_soc_rev(int rev)
{
	return (system_rev & 0xFF) - rev;
}

#ifdef CONFIG_ARCH_MMU
void board_mmu_init(void)
{
	unsigned long ttb_base = PHYS_SDRAM_1 + 0x4000;
	unsigned long i;

	/*
	* Set the TTB register
	*/
	asm volatile ("mcr  p15,0,%0,c2,c0,0" : : "r"(ttb_base) /*:*/);

	/*
	* Set the Domain Access Control Register
	*/
	i = ARM_ACCESS_DACR_DEFAULT;
	asm volatile ("mcr  p15,0,%0,c3,c0,0" : : "r"(i) /*:*/);

	/*
	* First clear all TT entries - ie Set them to Faulting
	*/
	memset((void *)ttb_base, 0, ARM_FIRST_LEVEL_PAGE_TABLE_SIZE);
	/* Actual   Virtual  Size   Attributes          Function */
	/* Base     Base     MB     cached? buffered?  access permissions */
	/* xxx00000 xxx00000 */
	X_ARM_MMU_SECTION(0x000, 0x000, 0x10,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* ROM, 16M */
	X_ARM_MMU_SECTION(0x070, 0x070, 0x010,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* IRAM */
	X_ARM_MMU_SECTION(0x100, 0x100, 0x040,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* SATA */
	X_ARM_MMU_SECTION(0x180, 0x180, 0x100,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* IPUv3M */
	X_ARM_MMU_SECTION(0x200, 0x200, 0x200,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* GPU */
	X_ARM_MMU_SECTION(0x400, 0x400, 0x300,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* periperals */
	X_ARM_MMU_SECTION(0x700, 0x700, 0x400,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* CSD0 1G */
	X_ARM_MMU_SECTION(0x700, 0xB00, 0x400,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* CSD0 1G */
	X_ARM_MMU_SECTION(0xF00, 0xF00, 0x100,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* CS1 EIM control*/
	X_ARM_MMU_SECTION(0xF80, 0xF80, 0x001,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* iRam */

	/* Workaround for arm errata #709718 */
	/* Setup PRRR so device is always mapped to non-shared */
	asm volatile ("mrc p15, 0, %0, c10, c2, 0" : "=r"(i) : /*:*/);
	i &= (~(3 << 0x10));
	asm volatile ("mcr p15, 0, %0, c10, c2, 0" : : "r"(i) /*:*/);

	/* Enable MMU */
	MMU_ON();
}
#endif

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
	return 0;
}
int mx50_rdp_recovery_button_init(void)
{
	unsigned int reg;

	//printf("%s %s %d\n",__FILE__,__func__,__LINE__);
	
         /*init the recovery mode button gpio */
  	mxc_request_iomux(MX50_PIN_UART3_TXD, IOMUX_CONFIG_ALT1);
	/* Set as input */
	reg = readl(GPIO6_BASE_ADDR + 0x4);
	//reg |= (1 << 14);
	reg &= (~(1 << 14));
	writel(reg, GPIO6_BASE_ADDR + 0x4);

      return 0;
}
//int read_power_register(void)
//{
//   int value=0;
 //  value = readl(IRAM_BASE_ADDR+0x
//}
int mx50_rdp_vcom_button_init(void)
{
	unsigned int reg;

	
         /*init the vcom voltage mode button gpio */
  	mxc_request_iomux(MX50_PIN_KEY_COL1, IOMUX_CONFIG_ALT1);
	/* Set as input */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	//reg |= (1 << 14);
	reg &= (~(1 << 2));
	writel(reg, GPIO4_BASE_ADDR + 0x4);

      return 0;
}
int setup_vcom_voltage(void)
{
     unsigned int reg;
     static int icount = 0;

     ///init the three voltage button
     //gpio3_29,gpio3_30,gpio3_31
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL0,IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO3_BASE_ADDR + 0x4);  
	reg |= (1 << 29);
	writel(reg, GPIO3_BASE_ADDR + 0x4);

	////now we need set as high
      /*set as high*/
	reg = readl(GPIO3_BASE_ADDR + 0x0);  
	reg|=(1<<29);
	writel(reg, GPIO3_BASE_ADDR + 0x0);

         ////gpio3_30
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL1,IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO3_BASE_ADDR + 0x4);  
	reg |= (1 << 30);
	writel(reg, GPIO3_BASE_ADDR + 0x4);

      /*set as high*/
	reg = readl(GPIO3_BASE_ADDR + 0x0);  
	reg|=(1<<30);
	writel(reg, GPIO3_BASE_ADDR + 0x0);

       ////gpio3_31
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL2,IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO3_BASE_ADDR + 0x4);  
	reg |= (1 << 31);
	writel(reg, GPIO3_BASE_ADDR + 0x4);

	////now we need set as high
      /*set as high*/
	reg = readl(GPIO3_BASE_ADDR + 0x0);  
	reg|=(1<<31);
	writel(reg, GPIO3_BASE_ADDR + 0x0);	


         ///vcom0  gpio4_21
	mxc_request_iomux(MX50_PIN_EPDC_VCOM0,IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO4_BASE_ADDR + 0x4);  
	reg |= (1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

      /*set as high*/
	reg = readl(GPIO4_BASE_ADDR + 0x0);  
	reg|=(1<<21);
	writel(reg, GPIO4_BASE_ADDR + 0x0);
	 //now enter while(1)

	 while(1){
	   udelay(10000*10000);
	 }
	 return 0;
}

static void setup_uart(void)
{
	unsigned int reg;

#if defined(CONFIG_MX50_RD3)
	/* UART3 TXD */
	mxc_request_iomux(MX50_PIN_UART3_TXD, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX50_PIN_UART3_TXD, 0x1E4);
	/* Enable UART1 */
	reg = readl(GPIO6_BASE_ADDR + 0x0);
	reg |= (1 << 14);
	writel(reg, GPIO6_BASE_ADDR + 0x0);

	reg = readl(GPIO6_BASE_ADDR + 0x4);
	reg |= (1 << 14);
	writel(reg, GPIO6_BASE_ADDR + 0x4);
#endif
	/* UART1 RXD */
	mxc_request_iomux(MX50_PIN_UART1_RXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_UART1_RXD, 0x1E4);
	mxc_iomux_set_input(MUX_IN_UART1_IPP_UART_RXD_MUX_SELECT_INPUT, 0x1);

	/* UART1 TXD */
	mxc_request_iomux(MX50_PIN_UART1_TXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_UART1_TXD, 0x1E4);
}

#ifdef CONFIG_I2C_MXC
static void setup_i2c(unsigned int module_base)
{
	switch (module_base) {
	case I2C1_BASE_ADDR:
		/* i2c1 SDA */
		mxc_request_iomux(MX50_PIN_I2C1_SDA,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX50_PIN_I2C1_SDA, PAD_CTL_SRE_FAST |
				PAD_CTL_ODE_OPENDRAIN_ENABLE |
				PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				PAD_CTL_HYS_ENABLE);
		/* i2c1 SCL */
		mxc_request_iomux(MX50_PIN_I2C1_SCL,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX50_PIN_I2C1_SCL, PAD_CTL_SRE_FAST |
				PAD_CTL_ODE_OPENDRAIN_ENABLE |
				PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				PAD_CTL_HYS_ENABLE);
		break;
	case I2C2_BASE_ADDR:
		/* i2c2 SDA */
		mxc_request_iomux(MX50_PIN_I2C2_SDA,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX50_PIN_I2C2_SDA,
				PAD_CTL_SRE_FAST |
				PAD_CTL_ODE_OPENDRAIN_ENABLE |
				PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				PAD_CTL_HYS_ENABLE);

		/* i2c2 SCL */
		mxc_request_iomux(MX50_PIN_I2C2_SCL,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX50_PIN_I2C2_SCL,
				PAD_CTL_SRE_FAST |
				PAD_CTL_ODE_OPENDRAIN_ENABLE |
				PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				PAD_CTL_HYS_ENABLE);
		break;
	case I2C3_BASE_ADDR:
		/* i2c3 SDA */
		mxc_request_iomux(MX50_PIN_I2C3_SDA,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX50_PIN_I2C3_SDA,
				PAD_CTL_SRE_FAST |
				PAD_CTL_ODE_OPENDRAIN_ENABLE |
				PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				PAD_CTL_HYS_ENABLE);

		/* i2c3 SCL */
		mxc_request_iomux(MX50_PIN_I2C3_SCL,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX50_PIN_I2C3_SCL,
				PAD_CTL_SRE_FAST |
				PAD_CTL_ODE_OPENDRAIN_ENABLE |
				PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
				PAD_CTL_HYS_ENABLE);
		break;
	default:
		printf("Invalid I2C base: 0x%x\n", module_base);
		break;
	}
}

#endif

#ifdef CONFIG_IMX_CSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	switch (dev->slave.cs) {
	case 0:
		/* PMIC */
		dev->base = CSPI3_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_HIGH;
		dev->ss = 0;
		dev->fifo_sz = 32;
		dev->us_delay = 0;
		break;
	case 1:
		/* SPI-NOR */
		dev->base = CSPI3_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 1;
		dev->fifo_sz = 32;
		dev->us_delay = 0;
		break;
	default:
		printf("Invalid Bus ID!\n");
	}

	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev)
{

	switch (dev->base) {
	case CSPI3_BASE_ADDR:
		mxc_request_iomux(MX50_PIN_CSPI_MOSI, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX50_PIN_CSPI_MOSI, 0x4);

		mxc_request_iomux(MX50_PIN_CSPI_MISO, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX50_PIN_CSPI_MISO, 0x4);

		if (dev->ss == 0) {
			/* de-select SS1 of instance: cspi */
			mxc_request_iomux(MX50_PIN_ECSPI1_MOSI,
						IOMUX_CONFIG_ALT1);

			mxc_request_iomux(MX50_PIN_CSPI_SS0, IOMUX_CONFIG_ALT0);
			mxc_iomux_set_pad(MX50_PIN_CSPI_SS0, 0xE4);
		} else if (dev->ss == 1) {
			/* de-select SS0 of instance: cspi */
			mxc_request_iomux(MX50_PIN_CSPI_SS0, IOMUX_CONFIG_ALT1);

			mxc_request_iomux(MX50_PIN_ECSPI1_MOSI,
						IOMUX_CONFIG_ALT2);
			mxc_iomux_set_pad(MX50_PIN_ECSPI1_MOSI, 0xE4);
			mxc_iomux_set_input(
			MUX_IN_CSPI_IPP_IND_SS1_B_SELECT_INPUT, 0x1);
		}

		mxc_request_iomux(MX50_PIN_CSPI_SCLK, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX50_PIN_CSPI_SCLK, 0x4);
		break;
	case CSPI2_BASE_ADDR:
	case CSPI1_BASE_ADDR:
		/* ecspi1-2 fall through */
		break;
	default:
		break;
	}
}
#endif

#ifdef CONFIG_NAND_GPMI
void setup_gpmi_nand(void)
{
	u32 src_sbmr = readl(SRC_BASE_ADDR + 0x4);

	/* Fix for gpmi gatelevel issue */
	mxc_iomux_set_pad(MX50_PIN_SD3_CLK, 0x00e4);

	/* RESETN,WRN,RDN,DATA0~7 Signals iomux*/
	/* Check if 1.8v NAND is to be supported */
	if ((src_sbmr & 0x00000004) >> 2)
		*(u32 *)(IOMUXC_BASE_ADDR + PAD_GRP_START + 0x58) = (0x1 << 13);

	/* RESETN */
	mxc_request_iomux(MX50_PIN_SD3_WP, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_WP, PAD_CTL_DRV_HIGH);

	/* WRN */
	mxc_request_iomux(MX50_PIN_SD3_CMD, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_CMD, PAD_CTL_DRV_HIGH);

	/* RDN */
	mxc_request_iomux(MX50_PIN_SD3_CLK, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_CLK, PAD_CTL_DRV_HIGH);

	/* D0 */
	mxc_request_iomux(MX50_PIN_SD3_D4, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D4, PAD_CTL_DRV_HIGH);

	/* D1 */
	mxc_request_iomux(MX50_PIN_SD3_D5, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D5, PAD_CTL_DRV_HIGH);

	/* D2 */
	mxc_request_iomux(MX50_PIN_SD3_D6, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D6, PAD_CTL_DRV_HIGH);

	/* D3 */
	mxc_request_iomux(MX50_PIN_SD3_D7, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D7, PAD_CTL_DRV_HIGH);

	/* D4 */
	mxc_request_iomux(MX50_PIN_SD3_D0, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D0, PAD_CTL_DRV_HIGH);

	/* D5 */
	mxc_request_iomux(MX50_PIN_SD3_D1, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D1, PAD_CTL_DRV_HIGH);

	/* D6 */
	mxc_request_iomux(MX50_PIN_SD3_D2, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D2, PAD_CTL_DRV_HIGH);

	/* D7 */
	mxc_request_iomux(MX50_PIN_SD3_D3, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_SD3_D3, PAD_CTL_DRV_HIGH);

	/*CE0~3,and other four controls signals muxed on KPP*/
	switch ((src_sbmr & 0x00000018) >> 3) {
	case  0:
		/* Muxed on key */
		if ((src_sbmr & 0x00000004) >> 2)
			*(u32 *)(IOMUXC_BASE_ADDR + PAD_GRP_START + 0x20) =
								(0x1 << 13);

		/* CLE */
		mxc_request_iomux(MX50_PIN_KEY_COL0, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_COL0, PAD_CTL_DRV_HIGH);

		/* ALE */
		mxc_request_iomux(MX50_PIN_KEY_ROW0, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_ROW0, PAD_CTL_DRV_HIGH);

		/* READY0 */
		mxc_request_iomux(MX50_PIN_KEY_COL3, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_COL3,
				PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				PAD_CTL_100K_PU);
		mxc_iomux_set_input(
			MUX_IN_RAWNAND_U_GPMI_INPUT_GPMI_RDY0_IN_SELECT_INPUT,
			INPUT_CTL_PATH0);

		/* DQS */
		mxc_request_iomux(MX50_PIN_KEY_ROW3, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_ROW3, PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(
			MUX_IN_RAWNAND_U_GPMI_INPUT_GPMI_DQS_IN_SELECT_INPUT,
			INPUT_CTL_PATH0);

		/* CE0 */
		mxc_request_iomux(MX50_PIN_KEY_COL1, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_COL1, PAD_CTL_DRV_HIGH);

		/* CE1 */
		mxc_request_iomux(MX50_PIN_KEY_ROW1, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_ROW1, PAD_CTL_DRV_HIGH);

		/* CE2 */
		mxc_request_iomux(MX50_PIN_KEY_COL2, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_COL2, PAD_CTL_DRV_HIGH);

		/* CE3 */
		mxc_request_iomux(MX50_PIN_KEY_ROW2, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_KEY_ROW2, PAD_CTL_DRV_HIGH);

		break;
	case 1:
	case 3:
		if ((src_sbmr & 0x00000004) >> 2)
			*(u32 *)(IOMUXC_BASE_ADDR + PAD_GRP_START + 0xc) =
								(0x1 << 13);

		/* CLE */
		mxc_request_iomux(MX50_PIN_EIM_DA8, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA8, PAD_CTL_DRV_HIGH);

		/* ALE */
		mxc_request_iomux(MX50_PIN_EIM_DA9, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA9, PAD_CTL_DRV_HIGH);

		/* READY0 */
		mxc_request_iomux(MX50_PIN_EIM_DA14, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA14,
				PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				PAD_CTL_100K_PU);
		mxc_iomux_set_input(
			MUX_IN_RAWNAND_U_GPMI_INPUT_GPMI_RDY0_IN_SELECT_INPUT,
			INPUT_CTL_PATH2);

		/* DQS */
		mxc_request_iomux(MX50_PIN_EIM_DA15, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA15, PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(
			MUX_IN_RAWNAND_U_GPMI_INPUT_GPMI_DQS_IN_SELECT_INPUT,
			INPUT_CTL_PATH2);

		/* CE0 */
		mxc_request_iomux(MX50_PIN_EIM_DA10, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA10, PAD_CTL_DRV_HIGH);

		/* CE1 */
		mxc_request_iomux(MX50_PIN_EIM_DA11, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA11, PAD_CTL_DRV_HIGH);

		/* CE2 */
		mxc_request_iomux(MX50_PIN_EIM_DA12, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA12, PAD_CTL_DRV_HIGH);

		/* CE3 */
		mxc_request_iomux(MX50_PIN_EIM_DA13, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA13, PAD_CTL_DRV_HIGH);

		break;
	case 2:
		if ((src_sbmr & 0x00000004) >> 2)
			*(u32 *)(IOMUXC_BASE_ADDR + PAD_GRP_START + 0x48) =
								(0x1 << 13);

		/* CLE */
		mxc_request_iomux(MX50_PIN_DISP_D8, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_DISP_D8, PAD_CTL_DRV_HIGH);

		/* ALE */
		mxc_request_iomux(MX50_PIN_DISP_D9, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_DISP_D9, PAD_CTL_DRV_HIGH);

		/* READY0 */
		mxc_request_iomux(MX50_PIN_DISP_D14, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_DISP_D14,
				PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				PAD_CTL_100K_PU);
		mxc_iomux_set_input(
			MUX_IN_RAWNAND_U_GPMI_INPUT_GPMI_RDY0_IN_SELECT_INPUT,
			INPUT_CTL_PATH1);

		/* DQS */
		mxc_request_iomux(MX50_PIN_DISP_D15, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_DISP_D15, PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(
			MUX_IN_RAWNAND_U_GPMI_INPUT_GPMI_DQS_IN_SELECT_INPUT,
			INPUT_CTL_PATH1);

		/* CE0 */
		mxc_request_iomux(MX50_PIN_DISP_D10, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_DISP_D10, PAD_CTL_DRV_HIGH);

		/* CE1 */
		mxc_request_iomux(MX50_PIN_EIM_DA11, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_EIM_DA11, PAD_CTL_DRV_HIGH);

		/* CE2 */
		mxc_request_iomux(MX50_PIN_DISP_D12, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_DISP_D12, PAD_CTL_DRV_HIGH);

		/* CE3 */
		mxc_request_iomux(MX50_PIN_DISP_D13, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX50_PIN_DISP_D13, PAD_CTL_DRV_HIGH);

		break;
	default:
		break;
	}
}
#endif

#ifdef CONFIG_MXC_FEC

#ifdef CONFIG_GET_FEC_MAC_ADDR_FROM_IIM

#define HW_OCOTP_MACn(n)	(0x00000250 + (n) * 0x10)

int fec_get_mac_addr(unsigned char *mac)
{
	u32 *ocotp_mac_base =
		(u32 *)(OCOTP_CTRL_BASE_ADDR + HW_OCOTP_MACn(0));
	int i;

	for (i = 0; i < 6; ++i, ++ocotp_mac_base)
		mac[6 - 1 - i] = readl(++ocotp_mac_base);

	return 0;
}
#endif

static void setup_fec(void)
{
	volatile unsigned int reg;

#if defined(CONFIG_MX50_RDP)
	/* FEC_EN: gpio6-23 set to 0 to enable FEC */
	mxc_request_iomux(MX50_PIN_I2C3_SDA, IOMUX_CONFIG_ALT1);

	reg = readl(GPIO6_BASE_ADDR + 0x0);
	reg &= ~(1 << 23);
	writel(reg, GPIO6_BASE_ADDR + 0x0);

	reg = readl(GPIO6_BASE_ADDR + 0x4);
	reg |= (1 << 23);
	writel(reg, GPIO6_BASE_ADDR + 0x4);

#elif defined(CONFIG_MX50_RD3)
	/* FEC_EN: gpio4-15 set to 1 to enable FEC */
	mxc_request_iomux(MX50_PIN_ECSPI1_SS0, IOMUX_CONFIG_ALT1);

	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg |= (1 << 15);
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 15);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* DCDC_PWREN(GP4_16) set to 1 to enable DCDC_3V15 */
	mxc_request_iomux(MX50_PIN_ECSPI2_SCLK, IOMUX_CONFIG_ALT1);
	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg |= (1 << 16);
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 16);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* Isolate EIM signals and boot configuration signals. - GPIO6_11 to 1*/
	mxc_request_iomux(MX50_PIN_UART2_RXD, IOMUX_CONFIG_ALT1);
	reg = readl(GPIO6_BASE_ADDR + 0x0);
	reg |= (1 << 11);
	writel(reg, GPIO6_BASE_ADDR + 0x0);

	reg = readl(GPIO6_BASE_ADDR + 0x4);
	reg |= (1 << 11);
	writel(reg, GPIO6_BASE_ADDR + 0x4);
#endif

	/*FEC_MDIO*/
	mxc_request_iomux(MX50_PIN_SSI_RXC, IOMUX_CONFIG_ALT6);
	mxc_iomux_set_pad(MX50_PIN_SSI_RXC, 0xC);
	mxc_iomux_set_input(MUX_IN_FEC_FEC_MDI_SELECT_INPUT, 0x1);

	/*FEC_MDC*/
	mxc_request_iomux(MX50_PIN_SSI_RXFS, IOMUX_CONFIG_ALT6);
	mxc_iomux_set_pad(MX50_PIN_SSI_RXFS, 0x004);

	/* FEC RXD1 */
	mxc_request_iomux(MX50_PIN_DISP_D3, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D3, 0x0);
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RDATA_1_SELECT_INPUT, 0x0);

	/* FEC RXD0 */
	mxc_request_iomux(MX50_PIN_DISP_D4, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D4, 0x0);
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RDATA_0_SELECT_INPUT, 0x0);

	 /* FEC TXD1 */
	mxc_request_iomux(MX50_PIN_DISP_D6, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D6, 0x004);

	/* FEC TXD0 */
	mxc_request_iomux(MX50_PIN_DISP_D7, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D7, 0x004);

	/* FEC TX_EN */
	mxc_request_iomux(MX50_PIN_DISP_D5, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D5, 0x004);

	/* FEC TX_CLK */
	mxc_request_iomux(MX50_PIN_DISP_D0, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D0, 0x0);
	mxc_iomux_set_input(MUX_IN_FEC_FEC_TX_CLK_SELECT_INPUT, 0x0);

	/* FEC RX_ER */
	mxc_request_iomux(MX50_PIN_DISP_D1, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D1, 0x0);
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RX_ER_SELECT_INPUT, 0);

	/* FEC CRS */
	mxc_request_iomux(MX50_PIN_DISP_D2, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX50_PIN_DISP_D2, 0x0);
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RX_DV_SELECT_INPUT, 0);

#if defined(CONFIG_MX50_RDP) || defined(CONFIG_MX50_RD3)
	/* FEC_RESET_B: gpio4-12 */
	mxc_request_iomux(MX50_PIN_ECSPI1_SCLK, IOMUX_CONFIG_ALT1);

	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg &= ~(1 << 12);
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 12);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	udelay(500);

	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg |= (1 << 12);
	writel(reg, GPIO4_BASE_ADDR + 0x0);
#elif defined(CONFIG_MX50_ARM2)
	/* phy reset: gpio4-6 */
	mxc_request_iomux(MX50_PIN_KEY_COL3, IOMUX_CONFIG_ALT1);

	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg &= ~0x40;
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= 0x40;
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	udelay(500);

	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg |= 0x40;
	writel(reg, GPIO4_BASE_ADDR + 0x0);
#else
#	error "Unsupported board!"
#endif
}
#endif

#ifdef CONFIG_CMD_MMC

struct fsl_esdhc_cfg esdhc_cfg[3] = {
	{MMC_SDHC1_BASE_ADDR, 1, 1},
	{MMC_SDHC2_BASE_ADDR, 1, 1},
	{MMC_SDHC3_BASE_ADDR, 1, 1},
};


#ifdef CONFIG_DYNAMIC_MMC_DEVNO
int get_mmc_env_devno(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	int mmc_devno = 0;

	 //UBOOT_DEBUG_FUNC();

	switch (soc_sbmr & 0x00300000) {
	default:
	case 0x0:
		mmc_devno = 0;
		break;
	case 0x00100000:
		mmc_devno = 1;
		break;
	case 0x00200000:
		mmc_devno = 2;
		break;
	}

	return mmc_devno;
}
#endif

#ifdef CONFIG_EMMC_DDR_PORT_DETECT
int detect_mmc_emmc_ddr_port(struct fsl_esdhc_cfg *cfg)
{
	return (MMC_SDHC3_BASE_ADDR == cfg->esdhc_base) ? 1 : 0;
}
#endif

/* The following function enables uSDHC instead of eSDHC
 * on SD3 port for SDR mode since eSDHC timing on MX50
 * is borderline for SDR mode. DDR mode will be disabled when this
 * define is enabled since the uSDHC timing on MX50 is borderline
 * for DDR mode. */
#ifdef CONFIG_MX50_ENABLE_USDHC_SDR
void enable_usdhc()
{
	/* Bring DIGCTL block out of reset and ungate clock */
	writel(0xC0000000, DIGCTL_BASE_ADDR + 0x8);
	/* Set bit 0 to select uSDHC */
	writel(1, DIGCTL_BASE_ADDR + 0x4);
}
#endif

int esdhc_gpio_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;
	for (index = 0; index < CONFIG_SYS_FSL_ESDHC_NUM;
		++index) {
		switch (index) {
		case 0:
			mxc_request_iomux(MX50_PIN_SD1_CMD, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD1_CLK, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD1_D0,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD1_D1,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD1_D2,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD1_D3,  IOMUX_CONFIG_ALT0);

			mxc_iomux_set_pad(MX50_PIN_SD1_CMD, 0x1E4);
			mxc_iomux_set_pad(MX50_PIN_SD1_CLK, 0xD4);
			mxc_iomux_set_pad(MX50_PIN_SD1_D0,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD1_D1,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD1_D2,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD1_D3,  0x1D4);

			break;
		case 1:
			mxc_request_iomux(MX50_PIN_SD2_CMD, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_CLK, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D0,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D1,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D2,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D3,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D4,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D5,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D6,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD2_D7,  IOMUX_CONFIG_ALT0);

			mxc_iomux_set_pad(MX50_PIN_SD2_CMD, 0x14);
			mxc_iomux_set_pad(MX50_PIN_SD2_CLK, 0xD4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D0,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D1,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D2,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D3,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D4,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D5,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D6,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD2_D7,  0x1D4);

			break;
		case 2:
#ifndef CONFIG_NAND_GPMI
			mxc_request_iomux(MX50_PIN_SD3_CMD, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_CLK, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D0,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D1,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D2,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D3,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D4,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D5,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D6,  IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX50_PIN_SD3_D7,  IOMUX_CONFIG_ALT0);

			mxc_iomux_set_pad(MX50_PIN_SD3_CMD, 0x1E4);
			mxc_iomux_set_pad(MX50_PIN_SD3_CLK, 0xD4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D0,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D1,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D2,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D3,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D4,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D5,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D6,  0x1D4);
			mxc_iomux_set_pad(MX50_PIN_SD3_D7,  0x1D4);
#endif
			break;
		default:
			printf("Warning: you configured more ESDHC controller"
				"(%d) as supported by the board(2)\n",
				CONFIG_SYS_FSL_ESDHC_NUM);
			return status;
			break;
		}
		//UBOOT_DEBUG_VALUE(index);
		status |= fsl_esdhc_initialize(bis, &esdhc_cfg[index]);
	}

	return status;
}

int board_mmc_init(bd_t *bis)
{
	//UBOOT_DEBUG_FUNC();
	if (!esdhc_gpio_init(bis))
		return 0;
	else
		return -1;
}

#endif

#ifdef CONFIG_MXC_EPDC
#ifdef CONFIG_SPLASH_SCREEN
int setup_splash_img()
{
#ifdef CONFIG_SPLASH_IS_IN_MMC
	int mmc_dev = get_mmc_env_devno();
	ulong offset = CONFIG_SPLASH_IMG_OFFSET;
	ulong size = CONFIG_SPLASH_IMG_SIZE;
	ulong addr = 0;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;


	  s = getenv("splashimage");
	
	printf("setup_splash_img s = %s \n",s);

	if (NULL == s) {
		puts("env splashimage not found!\n");
		return -1;
	}
	addr = simple_strtoul(s, NULL, 16);

	//printf("setup_splash_img s = %s \n");
        //printf("%s %s %d\n",__FILE__,__func__,__LINE__);

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}
        //printf("%s %s %d\n",__FILE__,__func__,__LINE__);
	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return  -1;
	}
	//printf("%s %s %d\n",__FILE__,__func__,__LINE__);

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
					blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	//printf("%s %s %d\n",__FILE__,__func__,__LINE__);

	return (n == blk_cnt) ? 0 : -1;
#endif
}
#endif

int setup_lowbattary_splash_img()
{
#ifdef CONFIG_LOW_BATTARY_SPLASH_IS_IN_MMC
	int mmc_dev = get_mmc_env_devno();
	ulong offset = CONFIG_LOW_BATTARY_SPLASH_IMG_OFFSET;
	ulong size = CONFIG_LOW_BATTARY_SPLASH_IMG_SIZE;
	ulong addr = 0;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	s = getenv("splashimage");
	
	printf("setup_splash_img s = %s \n",s);

	if (NULL == s) {
		puts("env splashimage not found!\n");
		return -1;
	}
	addr = simple_strtoul(s, NULL, 16);


	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}
	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return  -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
					blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);


	return (n == blk_cnt) ? 0 : -1;
#endif
}

vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 1800,
	.vl_row =800 ,
	.vl_pixclock = 75000000 /*68571429*/,
	.vl_left_margin = 8,
	.vl_right_margin = 122,
	.vl_upper_margin = 4,
	.vl_lower_margin = 8,
	.vl_hsync = 12,
	.vl_vsync = 1,
	.vl_sync = 0,
	.vl_mode = 0,
	.vl_flag = 0,
	.vl_bpix = 3,
	//.vl_bpix = 4,
	cmap : (void *)lcd_cmap,
};

static void setup_epdc_power()
{
	unsigned int reg;

	/* Setup epdc voltage */
	
	/* EPDC_PWRCTRL0/GP3_29  control -15V H:open L:close*/
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL0, IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg |= (1 << 29);
	writel(reg, GPIO3_BASE_ADDR + 0x4);

	/* EPDC_PWRCTRL1/GP3_30  control  +15V H:open L:close*/
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL1, IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg |= (1 << 30);
	writel(reg, GPIO3_BASE_ADDR + 0x4);

	/* EPDC_PWRCTRL2/GP3_31  control  +22V H:open L:close*/
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL2, IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg |= (1 << 31);
	writel(reg, GPIO3_BASE_ADDR + 0x4);

	/* EPDC VCOM0 - GPIO4[21] for VCOM control */
	mxc_request_iomux(MX50_PIN_EPDC_VCOM0, IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* EPDC BORDER - GPIO4[23] for BORDER control */
	mxc_request_iomux(MX50_PIN_EPDC_BDR0, IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 23);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* EPDC BORDER - GPIO4[24] for BORDER control */
	mxc_request_iomux(MX50_PIN_EPDC_BDR1, IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 24);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/////edpc power control, GPIO6_9 wen add.
	//mxc_request_iomux(MX50_PIN_UART1_RTS,IOMUX_CONFIG_ALT1);
	/* Set as output */
	//reg = readl(GPIO6_BASE_ADDR + 0x4);  
	//reg |= (1 << 9);
	//writel(reg, GPIO6_BASE_ADDR + 0x4);

}
void epdc_power_on()
{
	unsigned int reg;



	/* EPDC_PWRCTRL0/GP3_29  control -15V H:open L:close*/
	reg = readl(GPIO3_BASE_ADDR + 0x0);
	reg |= (1 << 29);
	writel(reg, GPIO3_BASE_ADDR + 0x0);

	/* EPDC_PWRCTRL1/GP3_30  control  +15V H:open L:close*/
	reg = readl(GPIO3_BASE_ADDR + 0x0);
	reg |= (1 << 30);
	writel(reg, GPIO3_BASE_ADDR + 0x0);

	/* EPDC_PWRCTRL2/GP3_31  control  +22V H:open L:close*/
	reg = readl(GPIO3_BASE_ADDR + 0x0);
	reg |= (1 << 31);
	writel(reg, GPIO3_BASE_ADDR + 0x0);

	/* EPDC VCOM0 - GPIO4[21] for VCOM control */
	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg |= (1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	/* EPDC BORDER - GPIO4[23] for BORDER control */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 23);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* EPDC BORDER - GPIO4[24] for BORDER control */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 24);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	udelay(1000);
}

void  epdc_power_off()
{
	unsigned int reg;


	/* EPDC BORDER - GPIO4[24] for BORDER control */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg &= (~(1 << 24));
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* EPDC BORDER - GPIO4[23] for BORDER control */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg &= (~(1 << 23));
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* Disable VCOM */
	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg&=(~(1<<21));
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	/* EPDC_PWRCTRL2/GP3_31  control  +22V H:open L:close*/
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg&=(~(1<<31));
	writel(reg, GPIO3_BASE_ADDR + 0x4);

	/* EPDC_PWRCTRL1/GP3_30  control  +15V H:open L:close*/
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg&=(~(1<<30));
	writel(reg, GPIO3_BASE_ADDR + 0x4);

	/* EPDC_PWRCTRL0/GP3_29  control -15V H:open L:close*/
	reg = readl(GPIO3_BASE_ADDR + 0x4);
	reg&=(~(1<<29));
	writel(reg, GPIO3_BASE_ADDR + 0x4);
}

/*
 * Function returns true if waveforms is loaded in memory and it is correct.
 */
int check_loaded_waveform(void) {

	int i, lastoffset=0, wfcount=0;
	ulong *idata = (ulong *)CONFIG_WAVEFORM_BUF_ADDR;
	ulong size = CONFIG_WAVEFORM_FILE_SIZE;

	for (i=0; i<16; i++) {
		if ((idata[i * 2] & 3) != 0 || idata[i * 2] > size) {
			printf("[%s] wrong sequence address wfdata[%d]=0x%x :: (%d)\n",__func__, i, idata[i*2],(idata[i * 2] > size));
			wfcount = -1;
			break;
		}
		if (idata[i * 2 + 1] != 0) {
			printf("[%s] unexpected value wfdata[%d]=0x%x\n",__func__, i*2+1, idata[i*2+1]);
			wfcount = -1;
			break;
		}
		if (idata[i * 2] == 0) continue;
		if (idata[i * 2] < lastoffset) break;
		wfcount++;
		lastoffset = idata[i * 2];
	}
	if (wfcount < 3) {
		printf("[%s] has not found waveform\n",__func__);
		return 0;
	}

	//printf("[%s] has found %d count wf\n",__func__,wfcount);
	return 1;
}

int setup_waveform_file()
{
#ifdef CONFIG_WAVEFORM_FILE_IN_MMC
	int mmc_dev = get_mmc_env_devno();
	ulong offset = CONFIG_WAVEFORM_FILE_OFFSET;
	ulong size = CONFIG_WAVEFORM_FILE_SIZE;
	ulong addr = CONFIG_WAVEFORM_BUF_ADDR;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
		blk_cnt, (u_char *)addr);
	flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#else
	return -1;
#endif
}

static void setup_epdc()
{
	unsigned int reg;

	/* epdc iomux settings */
	mxc_request_iomux(MX50_PIN_EPDC_D0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D4, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D5, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D6, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D7, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D8, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D9, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D10, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D11, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D12, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D13, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D14, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_D15, IOMUX_CONFIG_ALT0);

	mxc_request_iomux(MX50_PIN_EPDC_GDCLK, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_GDSP, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_GDOE, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_GDRL, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_SDCLK, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_SDOE, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_SDLE, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_SDSHR, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_BDR0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_SDCE0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_SDCE1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_EPDC_SDCE2, IOMUX_CONFIG_ALT0);

	/*** epdc Maxim PMIC settings ***/
	/* pwr ctrl*/
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL0, IOMUX_CONFIG_ALT1);
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL1, IOMUX_CONFIG_ALT1);
	mxc_request_iomux(MX50_PIN_EPDC_PWRCTRL2, IOMUX_CONFIG_ALT1);

	///* EPDC PWRSTAT - GPIO3[28] for PWR_GOOD status */
	//mxc_request_iomux(MX50_PIN_EPDC_PWRSTAT, IOMUX_CONFIG_ALT1);

	/* EPDC VCOM0 - GPIO4[21] for VCOM control */
	mxc_request_iomux(MX50_PIN_EPDC_VCOM0, IOMUX_CONFIG_ALT1);

	mxc_request_iomux(MX50_PIN_EPDC_BDR0, IOMUX_CONFIG_ALT1);

	mxc_request_iomux(MX50_PIN_EPDC_BDR1, IOMUX_CONFIG_ALT1);

	///* UART4 TXD - GPIO6[16] for EPD PMIC WAKEUP */
	//mxc_request_iomux(MX50_PIN_UART4_TXD, IOMUX_CONFIG_ALT1);

	/*** Set pixel clock rates for EPDC ***/

	/* EPDC AXI clk and EPDC PIX clk from PLL1 */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CLKSEQ_BYPASS);
	reg &= ~(0x3 << 4);
	reg |= (0x2 << 4) | (0x2 << 12);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CLKSEQ_BYPASS);

	/* EPDC AXI clk enable and set to 200MHz (800/4) */
	reg = readl(CCM_BASE_ADDR + 0xA8);
	reg &= ~((0x3 << 30) | 0x3F);
	reg |= (0x2 << 30) | 0x4;
	writel(reg, CCM_BASE_ADDR + 0xA8);

	/* EPDC PIX clk enable and set to 72MHz (800/11) */
	reg = readl(CCM_BASE_ADDR + 0xA0);
	reg &= ~((0x3 << 30) | (0x3 << 12) | 0x3F);
	reg |= (0x2 << 30) | (0x1 << 12) | 11;
	writel(reg, CCM_BASE_ADDR + 0xA0);

	panel_info.epdc_data.working_buf_addr = CONFIG_WORKING_BUF_ADDR;
	//Be sure, there are enough memmory
	memset(panel_info.epdc_data.working_buf_addr, 0x00, 2 * panel_info.vl_col * panel_info.vl_row);

	panel_info.epdc_data.waveform_buf_addr = CONFIG_WAVEFORM_BUF_ADDR;


	panel_info.epdc_data.wv_modes.mode_init = 0;
	panel_info.epdc_data.wv_modes.mode_du = 1;
	panel_info.epdc_data.wv_modes.mode_gc4 = 3;
	panel_info.epdc_data.wv_modes.mode_gc8 = 2;
	panel_info.epdc_data.wv_modes.mode_gc16 = 2;
	panel_info.epdc_data.wv_modes.mode_gc32 = 2;

	setup_epdc_power();

	/* Assign fb_base */
	gd->fb_base = CONFIG_FB_BASE;
}
#endif


#ifdef CONFIG_IMX_CSPI
int mx50_rdp_charger_gpio_control(int iopen)
{
     unsigned int reg;

	printf("%s %s %d bOpen=%d \n",__FILE__,__func__,__LINE__,iopen);	

     //gpio4_6
	mxc_request_iomux(MX50_PIN_KEY_COL3,IOMUX_CONFIG_ALT1);
	/* Set as output */
	reg = readl(GPIO4_BASE_ADDR + 0x4);  
	reg |= (1 << 6);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	if(1==iopen){
	      /*open the charger gpio,set as high*/
		reg = readl(GPIO4_BASE_ADDR + 0x0);  
		reg|=(1<<6);
		writel(reg, GPIO4_BASE_ADDR + 0x0);
	}else if(0 == iopen){
	       /*close the charger gpio,set as low*/
		reg = readl(GPIO4_BASE_ADDR + 0x0);  
		reg&=(~(1<<6));
		writel(reg, GPIO4_BASE_ADDR + 0x0);
	}


  return 0;
}

static void setup_power(void)
{
	//struct spi_slave *slave;
	unsigned int val;

	puts("PMIC Mode: SPI\n");

	/* Enable VGEN1 to enable ethernet */
	myslave = spi_pmic_probe();

#if defined(CONFIG_MX50_RD3)
	/* Set global reset time to 0s*/
	val = pmic_reg(myslave, MC13892_REG_POWER_CTL2, 0, 0);
	val &= ~(0x300);
	pmic_reg(myslave, MC13892_REG_POWER_CTL2, val, 1);
#else
	val = pmic_reg(myslave, MC13892_REG_SETTING_0, 0, 0);
	val |= 0x3;
	pmic_reg(myslave, MC13892_REG_SETTING_0, val, 1);

	val = pmic_reg(myslave, MC13892_REG_MODE_0, 0, 0);
	val |= 0x1;
	pmic_reg(myslave, MC13892_REG_MODE_0, val, 1);

	/* Enable VCAM   */
	val = pmic_reg(myslave, MC13892_REG_MODE_1, 0, 0);
	val |= 0x40;
	pmic_reg(myslave, MC13892_REG_MODE_1, val, 1);

	/* Set core voltage to 1.1V */
	val = pmic_reg(myslave, MC13892_REG_24, 0, 0);
	//val = (val & (~0x1f)) | 0x14;//1.1v
	val = (val & (~0x1f)) | 0x18;///1.2v
	pmic_reg(myslave, MC13892_REG_24, val, 1);

	/* Setup VCC (SW2) to 1.25 	*/
	val = pmic_reg(myslave, MC13892_REG_25, 0, 0);
	val = (val & (~0x1f)) | 0x1a;
	pmic_reg(myslave, MC13892_REG_25, val, 1);

#endif

	//spi_pmic_free(myslave);wen del
}

void setup_voltage_cpu(void)
{
	/* Currently VDDGP 1.05v
	 * no one tell me we need increase the core
	 * voltage to let CPU run at 800Mhz, not do it
	 */

	/* Raise the core frequency to 800MHz */
	writel(0x0, CCM_BASE_ADDR + CLKCTL_CACRR);

}
#endif
#if defined(CONFIG_MXC_KPD)
int setup_mxc_kpd(void)
{
	mxc_request_iomux(MX50_PIN_KEY_COL0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_KEY_COL1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_KEY_COL2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_KEY_COL3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_KEY_ROW0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_KEY_ROW1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_KEY_ROW2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX50_PIN_KEY_ROW3, IOMUX_CONFIG_ALT0);

	return 0;
}
#endif

int board_init(void)
{
  //int icount =0;
#ifdef CONFIG_MFG
/* MFG firmware need reset usb to avoid host crash firstly */
#define USBCMD 0x140
	int val = readl(OTG_BASE_ADDR + USBCMD);
	val &= ~0x1; /*RS bit*/
	writel(val, OTG_BASE_ADDR + USBCMD);
#endif
	/* boot device */
	setup_boot_device();

	/* soc rev */
	setup_soc_rev();

	/* board rev */
	setup_board_rev();

	/* arch id for linux */
	setup_arch_id();

	/* boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	/* iomux for uart */
	setup_uart();

#ifdef CONFIG_MXC_FEC
	/* iomux for fec */
	setup_fec();
#endif

#ifdef CONFIG_NAND_GPMI
	setup_gpmi_nand();
#endif

#ifdef CONFIG_MXC_EPDC
	setup_epdc();
#endif

	// while(1){
	//  //count++;
	//   udelay(100000);
	//  //f(icount >= 50)
	//    //reak;
	// }

	return 0;
}


int board_late_init(void)
{
#ifdef CONFIG_IMX_CSPI
	setup_power();
#endif
	//wifi_power_init();
	return 0;
}
int checkDeviceBootReason(void)
{
   int iBootReason = -1;
    switch (__REG(SRC_BASE_ADDR + 0x8)) {
	case 0x0001:
		iBootReason =0;
		break;
	case 0x0009:
		iBootReason=2;
		break;
	case 0x0010:
	case 0x0011:
		iBootReason =2;
		break;
	default:
		iBootReason = -1;
		break;
	}   
   return iBootReason;
}
int checkboard(void)
{
#if defined(CONFIG_MX50_RDP)
	printf("Board: MX50 RDP board\n");
#elif defined(CONFIG_MX50_RD3)
	printf("Board: MX50 RD3 board\n");
#elif defined(CONFIG_MX50_ARM2)
	printf("Board: MX50 ARM2 board\n");
#else
#	error "Unsupported board!"
#endif

	printf("Boot Reason: [");

	switch (__REG(SRC_BASE_ADDR + 0x8)) {
	case 0x0001:
		printf("POR");
		break;
	case 0x0009:
		printf("RST");
		break;
	case 0x0010:
	case 0x0011:
		printf("WDOG");
		break;
	default:
		printf("unknown");
	}
	printf("]\n");

	printf("Boot Device: ");
	switch (get_boot_device()) {
	case WEIM_NOR_BOOT:
		printf("NOR\n");
		break;
	case ONE_NAND_BOOT:
		printf("ONE NAND\n");
		break;
	case PATA_BOOT:
		printf("PATA\n");
		break;
	case SATA_BOOT:
		printf("SATA\n");
		break;
	case I2C_BOOT:
		printf("I2C\n");
		break;
	case SPI_NOR_BOOT:
		printf("SPI NOR\n");
		break;
	case SD_BOOT:
		printf("SD\n");
		break;
	case MMC_BOOT:
		printf("MMC\n");
		break;
	case NAND_BOOT:
		printf("NAND\n");
		break;
	case UNKNOWN_BOOT:
	default:
		printf("UNKNOWN\n");
		break;
	}

	return 0;
}

/*Front light data*/
#define PWMCR					0x0   /*Pwm Control Register*/
#define PWMSAR					0X0C  /*Pwm Sample Register*/
#define PWMPR					0X10  /*Pwm Period Register*/
#define PWMSAR_DUTY_CYCLES		31
//#define PWMPR_PERIOD_CYCLES		320
//#define PWMPR_PERIOD_CYCLES		16000
#define PWMPR_PERIOD_CYCLES		32
#define PWMCR_EN				(1 << 0)
#define PWMCR_OFF				(0 << 0)
//#define PWMCR_PRESCALER			( 0x7FF << 4)
#define PWMCR_PRESCALER			(0xFFF << 4)
#define PWMCR_STOPEN			(1 << 25)
#define PWMCR_DOZEEN			(1 << 24)
#define PWMCR_WAITEN			(1 << 23)
#define PWMCR_DBGEN			    (1 << 22)
#define PWMCR_CLK_IPG 			(1 << 16)
#define PWMCR_CLK_IPG_HIGH 		(2 << 16)
#define PWMCR_CLK_IPG_32K		(3 << 16)

void frontlight_on(void)
{
	unsigned int reg;
	/* not GPIO6[9] output 1*/
	/* but GPIO2_4 */
	reg = readl(GPIO2_BASE_ADDR + 0X0);
	reg |= (1 << 4);
	writel(reg, GPIO2_BASE_ADDR + 0X0);
}

void frontlight_off(void)
{
	unsigned int reg;
	/*PWM Disable*/
	reg = readl(PWM2_BASE_ADDR + PWMCR);
	reg &= ~PWMCR_EN;
	reg |= PWMCR_OFF;
	writel(reg,PWM2_BASE_ADDR + PWMCR);
	/* not GPIO6[9] output 0*/
	/* but GPIO2_4 */
	reg = readl(GPIO2_BASE_ADDR + 0X0);
	reg &= ~(1 <<4);
	reg |= (0<<4);
	writel(reg, GPIO2_BASE_ADDR + 0X0);
}

static unsigned int reg_clock;
static unsigned int reg_cr;
static unsigned int reg_sar;
static unsigned int reg_pr;
void frontlight_config(void)
{	
	unsigned int reg;
	mxc_request_iomux(MX50_PIN_DISP_D4, IOMUX_CONFIG_ALT1);
	reg = readl(GPIO2_BASE_ADDR + 0X04);
	reg |= (1 << 4);
	writel(reg, GPIO2_BASE_ADDR + 0X04);
	/*Config PWM CLOCK*/
	mxc_request_iomux(MX50_PIN_PWM2, IOMUX_CONFIG_ALT0);
	reg_clock = readl(CCM_BASE_ADDR + CLKCTL_CCGR2);
	printf("CLKCTL_CCGR2: %#x\n", reg_clock);
	reg_clock &= ~(0x3 << 14);
	reg_clock |= (0x3 << 14);
	printf("CLKCTL_CCGR2: %#x\n", reg_clock);
	writel(reg_clock, CCM_BASE_ADDR + CLKCTL_CCGR2);
	/*Config PWM Control Register*/
	reg_cr = readl(PWM2_BASE_ADDR + PWMCR);
//	reg_cr |= PWMCR_STOPEN | PWMCR_DOZEEN | PWMCR_WAITEN | PWMCR_DBGEN | PWMCR_CLK_IPG;
	reg_cr |= PWMCR_STOPEN | PWMCR_DOZEEN | PWMCR_WAITEN | PWMCR_DBGEN | PWMCR_CLK_IPG_32K;
	reg_cr &= ~PWMCR_PRESCALER;
	writel(reg_cr, PWM2_BASE_ADDR + PWMCR);
}

int  get_screen_init_flag(void)
{

	int mmc_dev = 2; //Nand store
	ulong offset = 30722 * 512;
	ulong size = 512;
	ulong addr = 0x70800000;
	uint blk_start, blk_cnt, n;
	int screen_init_flag = 0;

	struct mmc *mmc = find_mmc_device(mmc_dev);

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

		blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
		blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
		n = mmc->block_dev.block_read(mmc_dev, blk_start,
						blk_cnt, (u_char *)addr);
		flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);
//		printf("addr 's value is %x n is %d\n", addr, n);

		screen_init_flag =  *((int *)addr);
//		printf("==================\n");

		return screen_init_flag;
}

void set_screen_init_flag(int f)
{

	int mmc_dev = 2; //Nand store
	ulong offset = 30722 * 512;
	ulong size = 512;
	ulong addr = 0x70800000;
	uint blk_start, blk_cnt, n;

	struct mmc *mmc = find_mmc_device(mmc_dev);

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

		blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
		blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
		mmc->block_dev.block_read(mmc_dev, blk_start,
						blk_cnt, (u_char *)addr);
		flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);

//		printf("addr 's value is %x n is %d\n", addr, n);
		*((int *)addr) = f;
		mmc->block_dev.block_write(mmc_dev, blk_start,
								blk_cnt, (u_char *)addr);
//		printf("==================\n");

		return 0;
}


int brig_switch;
int brig_val;

int  brig_switch_val(void)
{

	int mmc_dev = 2; //Nand store
	ulong offset = 30721 * 512;
	ulong size = 512;
	ulong addr = 0x70800000;
	uint blk_start, blk_cnt, n;
	int h_val,l_val,m_val;
	char *fl_buffer = NULL;
	char *switch_buffer = NULL;
	int brightness;

	struct mmc *mmc = find_mmc_device(mmc_dev);

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

		blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
		blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
		n = mmc->block_dev.block_read(mmc_dev, blk_start,
						blk_cnt, (u_char *)addr);
		flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);
//		printf("addr 's value is %x n is %d\n", addr, n);

		brig_val =  *((int *)addr);
//		printf("==================\n");
		brig_switch = *(((int *)(addr))+1);

//		printf("%x, %x\n", brig_val, brig_switch);
		//no value in the FL address
		if(brig_val == 0 || brig_switch == 0)/*we store it with ASCII format*/
			goto off_fl;
		//wrong value in the FL address
//		if((brig_val & 0x00ff) > 63 || ((brig_val & 0xff000000) >> 24) != 0)
//			goto off_fl;

		if(((brig_val & 0xff000000)>> 24) == 0x0a){
				l_val = (((brig_val & 0xff0000)>>16) - 0x30);
				m_val = ((brig_val & 0xff00)>>8) - 0x30;
				h_val = ((brig_val & 0x00ff) - 0x30);

				brig_val = l_val + m_val*10 + h_val*100;
//			printf("l_val %d, h_val%d\n", l_val, h_val);
		}
		else if(((brig_val & 0xff0000) >> 16) == 0x0a){
				l_val = ((brig_val & 0xff00)>>8) - 0x30;
				h_val = ((brig_val & 0x00ff) - 0x30);
				brig_val = l_val + h_val*10;
			}
			else{
				brig_val = (brig_val & 0x00ff) - 0x30;
			}
			
		brig_switch = (brig_switch & 0x00ff) - 0x30;
		brightness = brig_val;

		if(brig_val > 255 || brig_val < 0)
			goto off_fl;

		printf("frontlight brightness is 0x%x,brightness switch is 0x%x\n", brig_val, brig_switch);
		goto out;

off_fl:
		brig_switch = 0;
		brightness = 0;
		printf("Due to wrong value frontlight light will keep off\n");
out:
		fl_buffer = (char *)malloc(2);
		sprintf(fl_buffer, "%d", brightness);
		printf("fl_buffer: %s\n",fl_buffer);

		switch_buffer = (char *)malloc(1);
		sprintf(switch_buffer, "%d", brig_switch);
		printf("switch_buffer: %s\n",switch_buffer);

		setenv("fl_val",fl_buffer);
		setenv("fl_switch",switch_buffer);

		free(fl_buffer);
		free(switch_buffer);
		return 0;

}


int frontlight_brightness_config(int duty_cycles)
{
	unsigned int reg;
	printf("FL duty cycles %d->", duty_cycles);
	duty_cycles = 255 - duty_cycles;
	duty_cycles = duty_cycles * PWMPR_PERIOD_CYCLES / 255; // 0-255
//	duty_cycles = PWMPR_PERIOD_CYCLES / 8;
	printf("%d\n", duty_cycles);

	/*Config PWM Sample Register*/
	reg_sar = readl(PWM2_BASE_ADDR + PWMSAR);
	reg_sar &= 0;
	reg_sar |= duty_cycles;
	printf("brightness duty cycles is %d\n", duty_cycles);
	if (duty_cycles > PWMPR_PERIOD_CYCLES)
	{
		printf("Wrong value,it must be [0	%d]\n",PWMPR_PERIOD_CYCLES);
		return -1;
	}
	writel(reg_sar, PWM2_BASE_ADDR + PWMSAR);
	/*Setup PWM Period Register*/
	writel(PWMPR_PERIOD_CYCLES - 2, PWM2_BASE_ADDR + PWMPR);
	reg_pr = readl(PWM2_BASE_ADDR + PWMPR);

	/*PWM Enable*/
	reg = readl(PWM2_BASE_ADDR + PWMCR);
	reg |= PWMCR_EN;
	writel(reg,PWM2_BASE_ADDR + PWMCR);
	return 0;
}
