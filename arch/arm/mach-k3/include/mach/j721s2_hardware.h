/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * K3: J721S2 SoC definitions, structures etc.
 *
 * (C) Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com/
 */
#ifndef __ASM_ARCH_J721S2_HARDWARE_H
#define __ASM_ARCH_J721S2_HARDWARE_H

#ifndef __ASSEMBLY__
#include <linux/bitops.h>
#endif

#define WKUP_CTRL_MMR0_BASE				0x43000000
#define MCU_CTRL_MMR0_BASE				0x40f00000
#define CTRL_MMR0_BASE					0x00100000

#define CTRLMMR_MAIN_DEVSTAT				(CTRL_MMR0_BASE + 0x30)
#define MAIN_DEVSTAT_BOOT_MODE_B_MASK			BIT(0)
#define MAIN_DEVSTAT_BOOT_MODE_B_SHIFT			0
#define MAIN_DEVSTAT_BKUP_BOOTMODE_MASK			GENMASK(3, 1)
#define MAIN_DEVSTAT_BKUP_BOOTMODE_SHIFT		1
#define MAIN_DEVSTAT_PRIM_BOOTMODE_MMC_PORT_MASK	BIT(6)
#define MAIN_DEVSTAT_PRIM_BOOTMODE_PORT_SHIFT		6
#define MAIN_DEVSTAT_BKUP_MMC_PORT_MASK			BIT(7)
#define MAIN_DEVSTAT_BKUP_MMC_PORT_SHIFT		7

#define CTRLMMR_WKUP_DEVSTAT				(WKUP_CTRL_MMR0_BASE + 0x30)
#define WKUP_DEVSTAT_PRIMARY_BOOTMODE_MASK		GENMASK(5, 3)
#define WKUP_DEVSTAT_PRIMARY_BOOTMODE_SHIFT		3
#define WKUP_DEVSTAT_MCU_OMLY_MASK			BIT(6)
#define WKUP_DEVSTAT_MCU_ONLY_SHIFT			6

/* Cause of the last reset. All defined bits are write-1-to-clear. */
#define CTRLMMR_WKUP_RESET_SRC_STAT			(WKUP_CTRL_MMR0_BASE + 0x50)
#define RESET_SRC_STAT_THERMAL_RST			BIT(24)
#define RESET_SRC_STAT_DBUGSS_RST			BIT(20)
#define RESET_SRC_STAT_COLD_OUT_RST			BIT(19)
#define RESET_SRC_STAT_WARM_OUT_RST			BIT(16)
#define RESET_SRC_STAT_PORZ_PIN				BIT(11)
#define RESET_SRC_STAT_RESET_REQZ_PIN			BIT(9)
#define RESET_SRC_STAT_MCU_RSTZ_PIN			BIT(8)
#define RESET_SRC_STAT_SW_MAIN_POR			BIT(3)
#define RESET_SRC_STAT_SW_MAIN_WARMRST			BIT(1)
#define RESET_SRC_STAT_SW_MCU_WARMRST			BIT(0)

/* ROM HANDOFF Structure location */
#define ROM_EXTENDED_BOOT_DATA_INFO			0x41cfdb00

/* MCU SCRATCHPAD usage */
#define TI_SRAM_SCRATCH_BOARD_EEPROM_START	CONFIG_SYS_K3_MCU_SCRATCHPAD_BASE

/*
 * R5 SPL is the only stage that still sees CTRLMMR_WKUP_RESET_SRC_STAT, and it
 * has to clear the latch, so it relays the value through the top of the MCU
 * scratchpad for the later stages. The board EEPROM record sits at the base of
 * the same window. MCU MSRAM is indeterminate after a cold power-on, hence the
 * magic word.
 */
#define TI_SRAM_SCRATCH_RESET_SRC_MAGIC		(CONFIG_SYS_K3_MCU_SCRATCHPAD_BASE + \
						 CONFIG_SYS_K3_MCU_SCRATCHPAD_SIZE - 8)
#define TI_SRAM_SCRATCH_RESET_SRC_STAT		(CONFIG_SYS_K3_MCU_SCRATCHPAD_BASE + \
						 CONFIG_SYS_K3_MCU_SCRATCHPAD_SIZE - 4)
#define TI_SRAM_SCRATCH_RESET_SRC_MAGIC_VAL	0x52535453	/* "RSTS" */

#if defined(CONFIG_SYS_K3_SPL_ATF) && !defined(__ASSEMBLY__)

#define J721S2_DEV_MCU_RTI0			295
#define J721S2_DEV_MCU_RTI1			296

static const u32 put_device_ids[] = {
	J721S2_DEV_MCU_RTI0,
	J721S2_DEV_MCU_RTI1,
};

#endif

#define J721S2_DEV_MCU_ARMSS0_CPU0		284
#define J721S2_DEV_MCU_ARMSS0_CPU1		285

static const u32 put_core_ids[] = {
	J721S2_DEV_MCU_ARMSS0_CPU1,
	J721S2_DEV_MCU_ARMSS0_CPU0,	/* Handle CPU0 after CPU1 */
};

#endif /* __ASM_ARCH_J721S2_HARDWARE_H */
