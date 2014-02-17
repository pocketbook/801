/*
 * Freescale SSP Register Definitions
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * This file is created by xml file. Don't Edit it.
 *
 * Xml Revision: 4.0
 * Template revision: 26195
 */

#ifndef __ARCH_ARM___SSP_H
#define __ARCH_ARM___SSP_H

#ifndef BF
#define BF(value, field) (((value) << BP_##field) & BM_##field)
#endif

#define HW_SSP_CTRL0	(0x00000000)
#define HW_SSP_CTRL0_SET	(0x00000004)
#define HW_SSP_CTRL0_CLR	(0x00000008)
#define HW_SSP_CTRL0_TOG	(0x0000000c)

#define BM_SSP_CTRL0_SFTRST	0x80000000
#define BM_SSP_CTRL0_CLKGATE	0x40000000
#define BM_SSP_CTRL0_RUN	0x20000000
#define BM_SSP_CTRL0_SDIO_IRQ_CHECK	0x10000000
#define BM_SSP_CTRL0_LOCK_CS	0x08000000
#define BM_SSP_CTRL0_IGNORE_CRC	0x04000000
#define BM_SSP_CTRL0_READ	0x02000000
#define BM_SSP_CTRL0_DATA_XFER	0x01000000
#define BP_SSP_CTRL0_BUS_WIDTH	22
#define BM_SSP_CTRL0_BUS_WIDTH	0x00C00000
#define BF_SSP_CTRL0_BUS_WIDTH(v)  \
		(((v) << 22) & BM_SSP_CTRL0_BUS_WIDTH)
#define BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT   0x0
#define BV_SSP_CTRL0_BUS_WIDTH__FOUR_BIT  0x1
#define BV_SSP_CTRL0_BUS_WIDTH__EIGHT_BIT 0x2
#define BM_SSP_CTRL0_WAIT_FOR_IRQ	0x00200000
#define BM_SSP_CTRL0_WAIT_FOR_CMD	0x00100000
#define BM_SSP_CTRL0_LONG_RESP	0x00080000
#define BM_SSP_CTRL0_CHECK_RESP	0x00040000
#define BM_SSP_CTRL0_GET_RESP	0x00020000
#define BM_SSP_CTRL0_ENABLE	0x00010000
#define BP_SSP_CTRL0_RSVD0	0
#define BM_SSP_CTRL0_RSVD0	0x0000FFFF
#define BF_SSP_CTRL0_RSVD0(v)  \
		(((v) << 0) & BM_SSP_CTRL0_RSVD0)

#define HW_SSP_CMD0	(0x00000010)
#define HW_SSP_CMD0_SET	(0x00000014)
#define HW_SSP_CMD0_CLR	(0x00000018)
#define HW_SSP_CMD0_TOG	(0x0000001c)

#define BP_SSP_CMD0_RSVD0	27
#define BM_SSP_CMD0_RSVD0	0xF8000000
#define BF_SSP_CMD0_RSVD0(v) \
		(((v) << 27) & BM_SSP_CMD0_RSVD0)
#define BM_SSP_CMD0_SOFT_TERMINATE	0x04000000
#define BM_SSP_CMD0_DBL_DATA_RATE_EN	0x02000000
#define BM_SSP_CMD0_PRIM_BOOT_OP_EN	0x01000000
#define BM_SSP_CMD0_BOOT_ACK_EN	0x00800000
#define BM_SSP_CMD0_SLOW_CLKING_EN	0x00400000
#define BM_SSP_CMD0_CONT_CLKING_EN	0x00200000
#define BM_SSP_CMD0_APPEND_8CYC	0x00100000
#define BP_SSP_CMD0_RSVD1	8
#define BM_SSP_CMD0_RSVD1	0x000FFF00
#define BF_SSP_CMD0_RSVD1(v)  \
		(((v) << 8) & BM_SSP_CMD0_RSVD1)
#define BP_SSP_CMD0_CMD	0
#define BM_SSP_CMD0_CMD	0x000000FF
#define BF_SSP_CMD0_CMD(v)  \
		(((v) << 0) & BM_SSP_CMD0_CMD)
#define BV_SSP_CMD0_CMD__MMC_GO_IDLE_STATE        0x00
#define BV_SSP_CMD0_CMD__MMC_SEND_OP_COND         0x01
#define BV_SSP_CMD0_CMD__MMC_ALL_SEND_CID         0x02
#define BV_SSP_CMD0_CMD__MMC_SET_RELATIVE_ADDR    0x03
#define BV_SSP_CMD0_CMD__MMC_SET_DSR              0x04
#define BV_SSP_CMD0_CMD__MMC_RESERVED_5           0x05
#define BV_SSP_CMD0_CMD__MMC_SWITCH               0x06
#define BV_SSP_CMD0_CMD__MMC_SELECT_DESELECT_CARD 0x07
#define BV_SSP_CMD0_CMD__MMC_SEND_EXT_CSD         0x08
#define BV_SSP_CMD0_CMD__MMC_SEND_CSD             0x09
#define BV_SSP_CMD0_CMD__MMC_SEND_CID             0x0A
#define BV_SSP_CMD0_CMD__MMC_READ_DAT_UNTIL_STOP  0x0B
#define BV_SSP_CMD0_CMD__MMC_STOP_TRANSMISSION    0x0C
#define BV_SSP_CMD0_CMD__MMC_SEND_STATUS          0x0D
#define BV_SSP_CMD0_CMD__MMC_BUSTEST_R            0x0E
#define BV_SSP_CMD0_CMD__MMC_GO_INACTIVE_STATE    0x0F
#define BV_SSP_CMD0_CMD__MMC_SET_BLOCKLEN         0x10
#define BV_SSP_CMD0_CMD__MMC_READ_SINGLE_BLOCK    0x11
#define BV_SSP_CMD0_CMD__MMC_READ_MULTIPLE_BLOCK  0x12
#define BV_SSP_CMD0_CMD__MMC_BUSTEST_W            0x13
#define BV_SSP_CMD0_CMD__MMC_WRITE_DAT_UNTIL_STOP 0x14
#define BV_SSP_CMD0_CMD__MMC_SET_BLOCK_COUNT      0x17
#define BV_SSP_CMD0_CMD__MMC_WRITE_BLOCK          0x18
#define BV_SSP_CMD0_CMD__MMC_WRITE_MULTIPLE_BLOCK 0x19
#define BV_SSP_CMD0_CMD__MMC_PROGRAM_CID          0x1A
#define BV_SSP_CMD0_CMD__MMC_PROGRAM_CSD          0x1B
#define BV_SSP_CMD0_CMD__MMC_SET_WRITE_PROT       0x1C
#define BV_SSP_CMD0_CMD__MMC_CLR_WRITE_PROT       0x1D
#define BV_SSP_CMD0_CMD__MMC_SEND_WRITE_PROT      0x1E
#define BV_SSP_CMD0_CMD__MMC_ERASE_GROUP_START    0x23
#define BV_SSP_CMD0_CMD__MMC_ERASE_GROUP_END      0x24
#define BV_SSP_CMD0_CMD__MMC_ERASE                0x26
#define BV_SSP_CMD0_CMD__MMC_FAST_IO              0x27
#define BV_SSP_CMD0_CMD__MMC_GO_IRQ_STATE         0x28
#define BV_SSP_CMD0_CMD__MMC_LOCK_UNLOCK          0x2A
#define BV_SSP_CMD0_CMD__MMC_APP_CMD              0x37
#define BV_SSP_CMD0_CMD__MMC_GEN_CMD              0x38
#define BV_SSP_CMD0_CMD__SD_GO_IDLE_STATE         0x00
#define BV_SSP_CMD0_CMD__SD_ALL_SEND_CID          0x02
#define BV_SSP_CMD0_CMD__SD_SEND_RELATIVE_ADDR    0x03
#define BV_SSP_CMD0_CMD__SD_SET_DSR               0x04
#define BV_SSP_CMD0_CMD__SD_IO_SEND_OP_COND       0x05
#define BV_SSP_CMD0_CMD__SD_SELECT_DESELECT_CARD  0x07
#define BV_SSP_CMD0_CMD__SD_SEND_CSD              0x09
#define BV_SSP_CMD0_CMD__SD_SEND_CID              0x0A
#define BV_SSP_CMD0_CMD__SD_STOP_TRANSMISSION     0x0C
#define BV_SSP_CMD0_CMD__SD_SEND_STATUS           0x0D
#define BV_SSP_CMD0_CMD__SD_GO_INACTIVE_STATE     0x0F
#define BV_SSP_CMD0_CMD__SD_SET_BLOCKLEN          0x10
#define BV_SSP_CMD0_CMD__SD_READ_SINGLE_BLOCK     0x11
#define BV_SSP_CMD0_CMD__SD_READ_MULTIPLE_BLOCK   0x12
#define BV_SSP_CMD0_CMD__SD_WRITE_BLOCK           0x18
#define BV_SSP_CMD0_CMD__SD_WRITE_MULTIPLE_BLOCK  0x19
#define BV_SSP_CMD0_CMD__SD_PROGRAM_CSD           0x1B
#define BV_SSP_CMD0_CMD__SD_SET_WRITE_PROT        0x1C
#define BV_SSP_CMD0_CMD__SD_CLR_WRITE_PROT        0x1D
#define BV_SSP_CMD0_CMD__SD_SEND_WRITE_PROT       0x1E
#define BV_SSP_CMD0_CMD__SD_ERASE_WR_BLK_START    0x20
#define BV_SSP_CMD0_CMD__SD_ERASE_WR_BLK_END      0x21
#define BV_SSP_CMD0_CMD__SD_ERASE_GROUP_START     0x23
#define BV_SSP_CMD0_CMD__SD_ERASE_GROUP_END       0x24
#define BV_SSP_CMD0_CMD__SD_ERASE                 0x26
#define BV_SSP_CMD0_CMD__SD_LOCK_UNLOCK           0x2A
#define BV_SSP_CMD0_CMD__SD_IO_RW_DIRECT          0x34
#define BV_SSP_CMD0_CMD__SD_IO_RW_EXTENDED        0x35
#define BV_SSP_CMD0_CMD__SD_APP_CMD               0x37
#define BV_SSP_CMD0_CMD__SD_GEN_CMD               0x38

#define HW_SSP_CMD1	(0x00000020)

#define BP_SSP_CMD1_CMD_ARG	0
#define BM_SSP_CMD1_CMD_ARG	0xFFFFFFFF
#define BF_SSP_CMD1_CMD_ARG(v)	(v)

#define HW_SSP_XFER_SIZE	(0x00000030)

#define BP_SSP_XFER_SIZE_XFER_COUNT	0
#define BM_SSP_XFER_SIZE_XFER_COUNT	0xFFFFFFFF
#define BF_SSP_XFER_SIZE_XFER_COUNT(v)	(v)

#define HW_SSP_BLOCK_SIZE	(0x00000040)

#define BP_SSP_BLOCK_SIZE_RSVD0	28
#define BM_SSP_BLOCK_SIZE_RSVD0	0xF0000000
#define BF_SSP_BLOCK_SIZE_RSVD0(v) \
		(((v) << 28) & BM_SSP_BLOCK_SIZE_RSVD0)
#define BP_SSP_BLOCK_SIZE_BLOCK_COUNT	4
#define BM_SSP_BLOCK_SIZE_BLOCK_COUNT	0x0FFFFFF0
#define BF_SSP_BLOCK_SIZE_BLOCK_COUNT(v)  \
		(((v) << 4) & BM_SSP_BLOCK_SIZE_BLOCK_COUNT)
#define BP_SSP_BLOCK_SIZE_BLOCK_SIZE	0
#define BM_SSP_BLOCK_SIZE_BLOCK_SIZE	0x0000000F
#define BF_SSP_BLOCK_SIZE_BLOCK_SIZE(v)  \
		(((v) << 0) & BM_SSP_BLOCK_SIZE_BLOCK_SIZE)

#define HW_SSP_COMPREF	(0x00000050)

#define BP_SSP_COMPREF_REFERENCE	0
#define BM_SSP_COMPREF_REFERENCE	0xFFFFFFFF
#define BF_SSP_COMPREF_REFERENCE(v)	(v)

#define HW_SSP_COMPMASK	(0x00000060)

#define BP_SSP_COMPMASK_MASK	0
#define BM_SSP_COMPMASK_MASK	0xFFFFFFFF
#define BF_SSP_COMPMASK_MASK(v)	(v)

#define HW_SSP_TIMING	(0x00000070)

#define BP_SSP_TIMING_TIMEOUT	16
#define BM_SSP_TIMING_TIMEOUT	0xFFFF0000
#define BF_SSP_TIMING_TIMEOUT(v) \
		(((v) << 16) & BM_SSP_TIMING_TIMEOUT)
#define BP_SSP_TIMING_CLOCK_DIVIDE	8
#define BM_SSP_TIMING_CLOCK_DIVIDE	0x0000FF00
#define BF_SSP_TIMING_CLOCK_DIVIDE(v)  \
		(((v) << 8) & BM_SSP_TIMING_CLOCK_DIVIDE)
#define BP_SSP_TIMING_CLOCK_RATE	0
#define BM_SSP_TIMING_CLOCK_RATE	0x000000FF
#define BF_SSP_TIMING_CLOCK_RATE(v)  \
		(((v) << 0) & BM_SSP_TIMING_CLOCK_RATE)

#define HW_SSP_CTRL1	(0x00000080)
#define HW_SSP_CTRL1_SET	(0x00000084)
#define HW_SSP_CTRL1_CLR	(0x00000088)
#define HW_SSP_CTRL1_TOG	(0x0000008c)

#define BM_SSP_CTRL1_SDIO_IRQ	0x80000000
#define BM_SSP_CTRL1_SDIO_IRQ_EN	0x40000000
#define BM_SSP_CTRL1_RESP_ERR_IRQ	0x20000000
#define BM_SSP_CTRL1_RESP_ERR_IRQ_EN	0x10000000
#define BM_SSP_CTRL1_RESP_TIMEOUT_IRQ	0x08000000
#define BM_SSP_CTRL1_RESP_TIMEOUT_IRQ_EN	0x04000000
#define BM_SSP_CTRL1_DATA_TIMEOUT_IRQ	0x02000000
#define BM_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN	0x01000000
#define BM_SSP_CTRL1_DATA_CRC_IRQ	0x00800000
#define BM_SSP_CTRL1_DATA_CRC_IRQ_EN	0x00400000
#define BM_SSP_CTRL1_FIFO_UNDERRUN_IRQ	0x00200000
#define BM_SSP_CTRL1_FIFO_UNDERRUN_EN	0x00100000
#define BM_SSP_CTRL1_CEATA_CCS_ERR_IRQ	0x00080000
#define BM_SSP_CTRL1_CEATA_CCS_ERR_IRQ_EN	0x00040000
#define BM_SSP_CTRL1_RECV_TIMEOUT_IRQ	0x00020000
#define BM_SSP_CTRL1_RECV_TIMEOUT_IRQ_EN	0x00010000
#define BM_SSP_CTRL1_FIFO_OVERRUN_IRQ	0x00008000
#define BM_SSP_CTRL1_FIFO_OVERRUN_IRQ_EN	0x00004000
#define BM_SSP_CTRL1_DMA_ENABLE	0x00002000
#define BM_SSP_CTRL1_CEATA_CCS_ERR_EN	0x00001000
#define BM_SSP_CTRL1_SLAVE_OUT_DISABLE	0x00000800
#define BM_SSP_CTRL1_PHASE	0x00000400
#define BM_SSP_CTRL1_POLARITY	0x00000200
#define BM_SSP_CTRL1_SLAVE_MODE	0x00000100
#define BP_SSP_CTRL1_WORD_LENGTH	4
#define BM_SSP_CTRL1_WORD_LENGTH	0x000000F0
#define BF_SSP_CTRL1_WORD_LENGTH(v)  \
		(((v) << 4) & BM_SSP_CTRL1_WORD_LENGTH)
#define BV_SSP_CTRL1_WORD_LENGTH__RESERVED0    0x0
#define BV_SSP_CTRL1_WORD_LENGTH__RESERVED1    0x1
#define BV_SSP_CTRL1_WORD_LENGTH__RESERVED2    0x2
#define BV_SSP_CTRL1_WORD_LENGTH__FOUR_BITS    0x3
#define BV_SSP_CTRL1_WORD_LENGTH__EIGHT_BITS   0x7
#define BV_SSP_CTRL1_WORD_LENGTH__SIXTEEN_BITS 0xF
#define BP_SSP_CTRL1_SSP_MODE	0
#define BM_SSP_CTRL1_SSP_MODE	0x0000000F
#define BF_SSP_CTRL1_SSP_MODE(v)  \
		(((v) << 0) & BM_SSP_CTRL1_SSP_MODE)
#define BV_SSP_CTRL1_SSP_MODE__SPI    0x0
#define BV_SSP_CTRL1_SSP_MODE__SSI    0x1
#define BV_SSP_CTRL1_SSP_MODE__SD_MMC 0x3
#define BV_SSP_CTRL1_SSP_MODE__MS     0x4

#define HW_SSP_DATA	(0x00000090)

#define BP_SSP_DATA_DATA	0
#define BM_SSP_DATA_DATA	0xFFFFFFFF
#define BF_SSP_DATA_DATA(v)	(v)

#define HW_SSP_SDRESP0	(0x000000a0)

#define BP_SSP_SDRESP0_RESP0	0
#define BM_SSP_SDRESP0_RESP0	0xFFFFFFFF
#define BF_SSP_SDRESP0_RESP0(v)	(v)

#define HW_SSP_SDRESP1	(0x000000b0)

#define BP_SSP_SDRESP1_RESP1	0
#define BM_SSP_SDRESP1_RESP1	0xFFFFFFFF
#define BF_SSP_SDRESP1_RESP1(v)	(v)

#define HW_SSP_SDRESP2	(0x000000c0)

#define BP_SSP_SDRESP2_RESP2	0
#define BM_SSP_SDRESP2_RESP2	0xFFFFFFFF
#define BF_SSP_SDRESP2_RESP2(v)	(v)

#define HW_SSP_SDRESP3	(0x000000d0)

#define BP_SSP_SDRESP3_RESP3	0
#define BM_SSP_SDRESP3_RESP3	0xFFFFFFFF
#define BF_SSP_SDRESP3_RESP3(v)	(v)

#define HW_SSP_DDR_CTRL	(0x000000e0)

#define BP_SSP_DDR_CTRL_DMA_BURST_TYPE	30
#define BM_SSP_DDR_CTRL_DMA_BURST_TYPE	0xC0000000
#define BF_SSP_DDR_CTRL_DMA_BURST_TYPE(v) \
		(((v) << 30) & BM_SSP_DDR_CTRL_DMA_BURST_TYPE)
#define BP_SSP_DDR_CTRL_RSVD0	2
#define BM_SSP_DDR_CTRL_RSVD0	0x3FFFFFFC
#define BF_SSP_DDR_CTRL_RSVD0(v)  \
		(((v) << 2) & BM_SSP_DDR_CTRL_RSVD0)
#define BM_SSP_DDR_CTRL_NIBBLE_POS	0x00000002
#define BM_SSP_DDR_CTRL_TXCLK_DELAY_TYPE	0x00000001

#define HW_SSP_DLL_CTRL	(0x000000f0)

#define BP_SSP_DLL_CTRL_REF_UPDATE_INT	28
#define BM_SSP_DLL_CTRL_REF_UPDATE_INT	0xF0000000
#define BF_SSP_DLL_CTRL_REF_UPDATE_INT(v) \
		(((v) << 28) & BM_SSP_DLL_CTRL_REF_UPDATE_INT)
#define BP_SSP_DLL_CTRL_SLV_UPDATE_INT	20
#define BM_SSP_DLL_CTRL_SLV_UPDATE_INT	0x0FF00000
#define BF_SSP_DLL_CTRL_SLV_UPDATE_INT(v)  \
		(((v) << 20) & BM_SSP_DLL_CTRL_SLV_UPDATE_INT)
#define BP_SSP_DLL_CTRL_RSVD1	16
#define BM_SSP_DLL_CTRL_RSVD1	0x000F0000
#define BF_SSP_DLL_CTRL_RSVD1(v)  \
		(((v) << 16) & BM_SSP_DLL_CTRL_RSVD1)
#define BP_SSP_DLL_CTRL_SLV_OVERRIDE_VAL	10
#define BM_SSP_DLL_CTRL_SLV_OVERRIDE_VAL	0x0000FC00
#define BF_SSP_DLL_CTRL_SLV_OVERRIDE_VAL(v)  \
		(((v) << 10) & BM_SSP_DLL_CTRL_SLV_OVERRIDE_VAL)
#define BM_SSP_DLL_CTRL_SLV_OVERRIDE	0x00000200
#define BM_SSP_DLL_CTRL_RSVD0	0x00000100
#define BM_SSP_DLL_CTRL_GATE_UPDATE	0x00000080
#define BP_SSP_DLL_CTRL_SLV_DLY_TARGET	3
#define BM_SSP_DLL_CTRL_SLV_DLY_TARGET	0x00000078
#define BF_SSP_DLL_CTRL_SLV_DLY_TARGET(v)  \
		(((v) << 3) & BM_SSP_DLL_CTRL_SLV_DLY_TARGET)
#define BM_SSP_DLL_CTRL_SLV_FORCE_UPD	0x00000004
#define BM_SSP_DLL_CTRL_RESET	0x00000002
#define BM_SSP_DLL_CTRL_ENABLE	0x00000001

#define HW_SSP_STATUS	(0x00000100)

#define BM_SSP_STATUS_PRESENT	0x80000000
#define BM_SSP_STATUS_MS_PRESENT	0x40000000
#define BM_SSP_STATUS_SD_PRESENT	0x20000000
#define BM_SSP_STATUS_CARD_DETECT	0x10000000
#define BP_SSP_STATUS_RSVD3	23
#define BM_SSP_STATUS_RSVD3	0x0F800000
#define BF_SSP_STATUS_RSVD3(v)  \
		(((v) << 23) & BM_SSP_STATUS_RSVD3)
#define BM_SSP_STATUS_DMABURST	0x00400000
#define BM_SSP_STATUS_DMASENSE	0x00200000
#define BM_SSP_STATUS_DMATERM	0x00100000
#define BM_SSP_STATUS_DMAREQ	0x00080000
#define BM_SSP_STATUS_DMAEND	0x00040000
#define BM_SSP_STATUS_SDIO_IRQ	0x00020000
#define BM_SSP_STATUS_RESP_CRC_ERR	0x00010000
#define BM_SSP_STATUS_RESP_ERR	0x00008000
#define BM_SSP_STATUS_RESP_TIMEOUT	0x00004000
#define BM_SSP_STATUS_DATA_CRC_ERR	0x00002000
#define BM_SSP_STATUS_TIMEOUT	0x00001000
#define BM_SSP_STATUS_RECV_TIMEOUT_STAT	0x00000800
#define BM_SSP_STATUS_CEATA_CCS_ERR	0x00000400
#define BM_SSP_STATUS_FIFO_OVRFLW	0x00000200
#define BM_SSP_STATUS_FIFO_FULL	0x00000100
#define BP_SSP_STATUS_RSVD1	6
#define BM_SSP_STATUS_RSVD1	0x000000C0
#define BF_SSP_STATUS_RSVD1(v)  \
		(((v) << 6) & BM_SSP_STATUS_RSVD1)
#define BM_SSP_STATUS_FIFO_EMPTY	0x00000020
#define BM_SSP_STATUS_FIFO_UNDRFLW	0x00000010
#define BM_SSP_STATUS_CMD_BUSY	0x00000008
#define BM_SSP_STATUS_DATA_BUSY	0x00000004
#define BM_SSP_STATUS_RSVD0	0x00000002
#define BM_SSP_STATUS_BUSY	0x00000001

#define HW_SSP_DLL_STS	(0x00000110)

#define BP_SSP_DLL_STS_RSVD0	14
#define BM_SSP_DLL_STS_RSVD0	0xFFFFC000
#define BF_SSP_DLL_STS_RSVD0(v) \
		(((v) << 14) & BM_SSP_DLL_STS_RSVD0)
#define BP_SSP_DLL_STS_REF_SEL	8
#define BM_SSP_DLL_STS_REF_SEL	0x00003F00
#define BF_SSP_DLL_STS_REF_SEL(v)  \
		(((v) << 8) & BM_SSP_DLL_STS_REF_SEL)
#define BP_SSP_DLL_STS_SLV_SEL	2
#define BM_SSP_DLL_STS_SLV_SEL	0x000000FC
#define BF_SSP_DLL_STS_SLV_SEL(v)  \
		(((v) << 2) & BM_SSP_DLL_STS_SLV_SEL)
#define BM_SSP_DLL_STS_REF_LOCK	0x00000002
#define BM_SSP_DLL_STS_SLV_LOCK	0x00000001

#define HW_SSP_DEBUG	(0x00000120)

#define BP_SSP_DEBUG_DATACRC_ERR	28
#define BM_SSP_DEBUG_DATACRC_ERR	0xF0000000
#define BF_SSP_DEBUG_DATACRC_ERR(v) \
		(((v) << 28) & BM_SSP_DEBUG_DATACRC_ERR)
#define BM_SSP_DEBUG_DATA_STALL	0x08000000
#define BP_SSP_DEBUG_DAT_SM	24
#define BM_SSP_DEBUG_DAT_SM	0x07000000
#define BF_SSP_DEBUG_DAT_SM(v)  \
		(((v) << 24) & BM_SSP_DEBUG_DAT_SM)
#define BV_SSP_DEBUG_DAT_SM__DSM_IDLE 0x0
#define BV_SSP_DEBUG_DAT_SM__DSM_WORD 0x2
#define BV_SSP_DEBUG_DAT_SM__DSM_CRC1 0x3
#define BV_SSP_DEBUG_DAT_SM__DSM_CRC2 0x4
#define BV_SSP_DEBUG_DAT_SM__DSM_END  0x5
#define BP_SSP_DEBUG_MSTK_SM	20
#define BM_SSP_DEBUG_MSTK_SM	0x00F00000
#define BF_SSP_DEBUG_MSTK_SM(v)  \
		(((v) << 20) & BM_SSP_DEBUG_MSTK_SM)
#define BV_SSP_DEBUG_MSTK_SM__MSTK_IDLE  0x0
#define BV_SSP_DEBUG_MSTK_SM__MSTK_CKON  0x1
#define BV_SSP_DEBUG_MSTK_SM__MSTK_BS1   0x2
#define BV_SSP_DEBUG_MSTK_SM__MSTK_TPC   0x3
#define BV_SSP_DEBUG_MSTK_SM__MSTK_BS2   0x4
#define BV_SSP_DEBUG_MSTK_SM__MSTK_HDSHK 0x5
#define BV_SSP_DEBUG_MSTK_SM__MSTK_BS3   0x6
#define BV_SSP_DEBUG_MSTK_SM__MSTK_RW    0x7
#define BV_SSP_DEBUG_MSTK_SM__MSTK_CRC1  0x8
#define BV_SSP_DEBUG_MSTK_SM__MSTK_CRC2  0x9
#define BV_SSP_DEBUG_MSTK_SM__MSTK_BS0   0xA
#define BV_SSP_DEBUG_MSTK_SM__MSTK_END1  0xB
#define BV_SSP_DEBUG_MSTK_SM__MSTK_END2W 0xC
#define BV_SSP_DEBUG_MSTK_SM__MSTK_END2R 0xD
#define BV_SSP_DEBUG_MSTK_SM__MSTK_DONE  0xE
#define BM_SSP_DEBUG_CMD_OE	0x00080000
#define BP_SSP_DEBUG_DMA_SM	16
#define BM_SSP_DEBUG_DMA_SM	0x00070000
#define BF_SSP_DEBUG_DMA_SM(v)  \
		(((v) << 16) & BM_SSP_DEBUG_DMA_SM)
#define BV_SSP_DEBUG_DMA_SM__DMA_IDLE   0x0
#define BV_SSP_DEBUG_DMA_SM__DMA_DMAREQ 0x1
#define BV_SSP_DEBUG_DMA_SM__DMA_DMAACK 0x2
#define BV_SSP_DEBUG_DMA_SM__DMA_STALL  0x3
#define BV_SSP_DEBUG_DMA_SM__DMA_BUSY   0x4
#define BV_SSP_DEBUG_DMA_SM__DMA_DONE   0x5
#define BV_SSP_DEBUG_DMA_SM__DMA_COUNT  0x6
#define BP_SSP_DEBUG_MMC_SM	12
#define BM_SSP_DEBUG_MMC_SM	0x0000F000
#define BF_SSP_DEBUG_MMC_SM(v)  \
		(((v) << 12) & BM_SSP_DEBUG_MMC_SM)
#define BV_SSP_DEBUG_MMC_SM__MMC_IDLE 0x0
#define BV_SSP_DEBUG_MMC_SM__MMC_CMD  0x1
#define BV_SSP_DEBUG_MMC_SM__MMC_TRC  0x2
#define BV_SSP_DEBUG_MMC_SM__MMC_RESP 0x3
#define BV_SSP_DEBUG_MMC_SM__MMC_RPRX 0x4
#define BV_SSP_DEBUG_MMC_SM__MMC_TX   0x5
#define BV_SSP_DEBUG_MMC_SM__MMC_CTOK 0x6
#define BV_SSP_DEBUG_MMC_SM__MMC_RX   0x7
#define BV_SSP_DEBUG_MMC_SM__MMC_CCS  0x8
#define BV_SSP_DEBUG_MMC_SM__MMC_PUP  0x9
#define BV_SSP_DEBUG_MMC_SM__MMC_WAIT 0xA
#define BP_SSP_DEBUG_CMD_SM	10
#define BM_SSP_DEBUG_CMD_SM	0x00000C00
#define BF_SSP_DEBUG_CMD_SM(v)  \
		(((v) << 10) & BM_SSP_DEBUG_CMD_SM)
#define BV_SSP_DEBUG_CMD_SM__CSM_IDLE  0x0
#define BV_SSP_DEBUG_CMD_SM__CSM_INDEX 0x1
#define BV_SSP_DEBUG_CMD_SM__CSM_ARG   0x2
#define BV_SSP_DEBUG_CMD_SM__CSM_CRC   0x3
#define BM_SSP_DEBUG_SSP_CMD	0x00000200
#define BM_SSP_DEBUG_SSP_RESP	0x00000100
#define BP_SSP_DEBUG_SSP_RXD	0
#define BM_SSP_DEBUG_SSP_RXD	0x000000FF
#define BF_SSP_DEBUG_SSP_RXD(v)  \
		(((v) << 0) & BM_SSP_DEBUG_SSP_RXD)

#define HW_SSP_VERSION	(0x00000130)

#define BP_SSP_VERSION_MAJOR	24
#define BM_SSP_VERSION_MAJOR	0xFF000000
#define BF_SSP_VERSION_MAJOR(v) \
		(((v) << 24) & BM_SSP_VERSION_MAJOR)
#define BP_SSP_VERSION_MINOR	16
#define BM_SSP_VERSION_MINOR	0x00FF0000
#define BF_SSP_VERSION_MINOR(v)  \
		(((v) << 16) & BM_SSP_VERSION_MINOR)
#define BP_SSP_VERSION_STEP	0
#define BM_SSP_VERSION_STEP	0x0000FFFF
#define BF_SSP_VERSION_STEP(v)  \
		(((v) << 0) & BM_SSP_VERSION_STEP)
#endif /* __ARCH_ARM___SSP_H */
