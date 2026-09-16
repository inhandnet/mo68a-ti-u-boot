// SPDX-License-Identifier: GPL-2.0+
/*
 * Board specific initialization for J721S2 EVM
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com/
 *	David Huang <d-huang@ti.com>
 *
 */

#include <env.h>
#include <fdt_support.h>
#include <generic-phy.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <net.h>
#include <asm/arch/hardware.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <spl.h>
#include <dm.h>
#include <dm/uclass-internal.h>
#include <dm/root.h>
#include <asm/arch/k3-ddr.h>

#include "../common/board_detect.h"
#include "../common/fdt_ops.h"

DECLARE_GLOBAL_DATA_PTR;


/*
 * tps6594_wdog_disable - 直接 MMIO 操作 MCU_I2C0，禁用 TPS6594 看门狗。
 *
 * 背景：ti-sci power-domain 对 A72 SPL 不开放 dev-id 221（MCU_I2C0），
 * 导致 DM probe 失败。此处绕过 DM，直接读写控制器寄存器，
 * 时钟和引脚复用已由 SYSFW/R5 SPL 在移交前配置完毕。
 *
 * 硬件：OMAP I2C IP-V2（ti,j721e-i2c），基地址 0x40b00000。
 * TPS6594 page 4（看门狗），I2C 从地址 0x12，寄存器 WD_THR_CFG = 0x09。
 * WD_EN = bit[6]；清零即禁用看门狗。
 */

/* MCU_I2C0 基地址 */
#define MCU_I2C0_BASE		0x40b00000UL

/* OMAP I2C IP-V2 寄存器偏移 */
#define MCCI2C_SYSC		0x010
#define MCCI2C_IRQSTAT_RAW	0x024	/* 含 BB 位，只读 */
#define MCCI2C_IRQSTAT		0x028	/* 写 1 清零 */
#define MCCI2C_SYSS		0x090
#define MCCI2C_CNT		0x098
#define MCCI2C_DATA		0x09c
#define MCCI2C_CON		0x0a4
#define MCCI2C_OA		0x0a8
#define MCCI2C_SA		0x0ac
#define MCCI2C_PSC		0x0b0
#define MCCI2C_SCLL		0x0b4
#define MCCI2C_SCLH		0x0b8

/* CON 位 */
#define MCCI2C_CON_EN		BIT(15)
#define MCCI2C_CON_MST		BIT(10)
#define MCCI2C_CON_TRX		BIT(9)
#define MCCI2C_CON_STP		BIT(1)
#define MCCI2C_CON_STT		BIT(0)

/* 状态位（IRQSTAT_RAW 与 IRQSTAT 位域相同） */
#define MCCI2C_STAT_BB		BIT(12)
#define MCCI2C_STAT_XRDY	BIT(4)
#define MCCI2C_STAT_RRDY	BIT(3)
#define MCCI2C_STAT_ARDY	BIT(2)
#define MCCI2C_STAT_NACK	BIT(1)
#define MCCI2C_STAT_AL		BIT(0)

#define MCCI2C_SYSS_RDONE	BIT(0)

/* MCU_I2C0 功能时钟：96 MHz（J721S2 默认） */
#define MCCI2C_FCLK_KHZ		96000U
#define MCCI2C_SPEED_KHZ	400U	/* Fast Mode */

/* TPS6594 page 4（看门狗） */
#define TPS6594_WDG_ADDR	0x12
#define TPS6594_WD_THR_CFG	0x09
#define TPS6594_WD_EN		BIT(6)

static inline u16 mci2c_rd(u32 off)
{
	return readw((void __iomem *)(uintptr_t)(MCU_I2C0_BASE + off));
}

static inline void mci2c_wr(u16 val, u32 off)
{
	writew(val, (void __iomem *)(uintptr_t)(MCU_I2C0_BASE + off));
}

/* 等待总线空闲（BB=0），超时约 100 ms */
static int mci2c_wait_bb(void)
{
	int t = 10000;

	mci2c_wr(0xffff, MCCI2C_IRQSTAT);
	while ((mci2c_rd(MCCI2C_IRQSTAT_RAW) & MCCI2C_STAT_BB) && --t)
		udelay(10);
	mci2c_wr(0xffff, MCCI2C_IRQSTAT);
	return t ? 0 : -1;
}

/* 等待事务事件（XRDY/RRDY/ARDY/NACK/AL），超时约 100 ms */
static u16 mci2c_wait_ev(void)
{
	int t = 10000;
	u16 s;

	do {
		udelay(10);
		s = mci2c_rd(MCCI2C_IRQSTAT_RAW);
	} while (!(s & (MCCI2C_STAT_XRDY | MCCI2C_STAT_RRDY | MCCI2C_STAT_ARDY |
			MCCI2C_STAT_NACK | MCCI2C_STAT_AL)) && --t);

	if (!t) {
		mci2c_wr(0xffff, MCCI2C_IRQSTAT);
		return 0;
	}
	return s;
}

/* 软复位并配置 400 kHz */
static int mci2c_init(void)
{
	u32 scl;
	int t = 1000;

	if (mci2c_rd(MCCI2C_CON) & MCCI2C_CON_EN) {
		mci2c_wr(0, MCCI2C_CON);
		udelay(50000);
	}
	mci2c_wr(0x2, MCCI2C_SYSC);	/* SRST */
	udelay(1000);
	mci2c_wr(MCCI2C_CON_EN, MCCI2C_CON);
	while (!(mci2c_rd(MCCI2C_SYSS) & MCCI2C_SYSS_RDONE) && --t)
		udelay(1000);
	if (!t)
		return -1;

	/*
	 * 400 kHz：内部时钟 = 9.6 MHz，PSC = 9
	 * scl = 9600/400 = 24；SCLL = 24-8-7 = 9；SCLH = 8-5 = 3
	 */
	scl = (MCCI2C_FCLK_KHZ / 10) / MCCI2C_SPEED_KHZ;
	mci2c_wr(0, MCCI2C_CON);
	mci2c_wr(MCCI2C_FCLK_KHZ / (MCCI2C_FCLK_KHZ / 10) - 1, MCCI2C_PSC);
	mci2c_wr(scl - scl / 3 - 7, MCCI2C_SCLL);
	mci2c_wr(scl / 3 - 5,       MCCI2C_SCLH);
	mci2c_wr(MCCI2C_CON_EN, MCCI2C_CON);
	udelay(1000);
	mci2c_wr(0xffff, MCCI2C_IRQSTAT);
	mci2c_wr(0, MCCI2C_OA);

	return mci2c_wait_bb();
}

/* 从从设备 slave 的寄存器 reg 读取 1 字节 */
static int mci2c_read1(u8 slave, u8 reg, u8 *val)
{
	u16 s;

	if (mci2c_wait_bb())
		return -1;

	/* 地址阶段：写寄存器偏移 */
	mci2c_wr(1, MCCI2C_CNT);
	mci2c_wr(slave, MCCI2C_SA);
	mci2c_wr(MCCI2C_CON_EN | MCCI2C_CON_MST | MCCI2C_CON_STT |
		 MCCI2C_CON_STP | MCCI2C_CON_TRX, MCCI2C_CON);

	s = mci2c_wait_ev();
	if (!s || (s & MCCI2C_STAT_NACK))
		return -1;
	if (s & MCCI2C_STAT_XRDY) {
		mci2c_wr(reg, MCCI2C_DATA);
		mci2c_wr(MCCI2C_STAT_XRDY, MCCI2C_IRQSTAT);
	}
	s = mci2c_wait_ev();
	if (!s || (s & MCCI2C_STAT_NACK)) {
		mci2c_wr(0xffff, MCCI2C_IRQSTAT);
		return -1;
	}
	if (s & MCCI2C_STAT_ARDY)
		mci2c_wr(MCCI2C_STAT_ARDY, MCCI2C_IRQSTAT);
	mci2c_wr(0xffff, MCCI2C_IRQSTAT);

	if (mci2c_wait_bb())
		return -1;

	/* 数据阶段：读 1 字节 */
	mci2c_wr(1, MCCI2C_CNT);
	mci2c_wr(slave, MCCI2C_SA);
	mci2c_wr(MCCI2C_CON_EN | MCCI2C_CON_MST | MCCI2C_CON_STT |
		 MCCI2C_CON_STP, MCCI2C_CON);			/* 无 TRX = 读 */

	s = mci2c_wait_ev();
	if (!s || (s & MCCI2C_STAT_NACK))
		return -1;
	if (!(s & MCCI2C_STAT_RRDY)) {
		mci2c_wr(0xffff, MCCI2C_IRQSTAT);
		return -1;
	}
	*val = mci2c_rd(MCCI2C_DATA) & 0xff;
	mci2c_wr(MCCI2C_STAT_RRDY, MCCI2C_IRQSTAT);

	s = mci2c_wait_ev();
	if (s & MCCI2C_STAT_ARDY)
		mci2c_wr(MCCI2C_STAT_ARDY, MCCI2C_IRQSTAT);
	mci2c_wr(0xffff, MCCI2C_IRQSTAT);
	return 0;
}

/* 向从设备 slave 的寄存器 reg 写入 1 字节 */
static int mci2c_write1(u8 slave, u8 reg, u8 val)
{
	u16 s;
	int phase = 0;	/* 0 = 发寄存器地址，1 = 发数据 */

	if (mci2c_wait_bb())
		return -1;

	mci2c_wr(2, MCCI2C_CNT);	/* 寄存器地址 + 数据，共 2 字节 */
	mci2c_wr(slave, MCCI2C_SA);
	mci2c_wr(MCCI2C_CON_EN | MCCI2C_CON_MST | MCCI2C_CON_STT |
		 MCCI2C_CON_STP | MCCI2C_CON_TRX, MCCI2C_CON);

	while (1) {
		s = mci2c_wait_ev();
		if (!s || (s & (MCCI2C_STAT_NACK | MCCI2C_STAT_AL))) {
			mci2c_wr(0xffff, MCCI2C_IRQSTAT);
			return -1;
		}
		if (s & MCCI2C_STAT_XRDY) {
			mci2c_wr(phase == 0 ? reg : val, MCCI2C_DATA);
			mci2c_wr(MCCI2C_STAT_XRDY, MCCI2C_IRQSTAT);
			phase++;
		}
		if (s & MCCI2C_STAT_ARDY) {
			mci2c_wr(MCCI2C_STAT_ARDY, MCCI2C_IRQSTAT);
			break;
		}
	}
	mci2c_wr(0xffff, MCCI2C_IRQSTAT);
	return 0;
}

static int tps6594_wdog_disable(void)
{
	u8 val;
	int ret;

	ret = mci2c_init();
	if (ret) {
		pr_err("%s: MCU_I2C0 init failed\n", __func__);
		return ret;
	}

	ret = mci2c_read1(TPS6594_WDG_ADDR, TPS6594_WD_THR_CFG, &val);
	if (ret) {
		pr_err("%s: read WD_THR_CFG failed\n", __func__);
		return ret;
	}

	val &= ~TPS6594_WD_EN;

	ret = mci2c_write1(TPS6594_WDG_ADDR, TPS6594_WD_THR_CFG, val);
	if (ret) {
		pr_err("%s: write WD_THR_CFG failed\n", __func__);
		return ret;
	}

	printf("TPS6594 watchdog disabled (WD_THR_CFG=0x%02x)\n", val);
	return 0;
}

int board_init(void)
{
	return 0;
}

phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
#ifdef CONFIG_PHYS_64BIT
	/* Limit RAM used by U-Boot to the DDR low region */
	if (gd->ram_top > 0x100000000)
		return 0x100000000;
#endif

	return gd->ram_top;
}

/* Enables the spi-nand dts node, if onboard mux is set to spinand */
static void __maybe_unused detect_enable_spinand(void *blob)
{
	if (IS_ENABLED(CONFIG_DM_GPIO) && IS_ENABLED(CONFIG_OF_LIBFDT)) {
		struct gpio_desc desc = {0};
		char *ospi_mux_sel_gpio = "6";
		int nand_offset, nor_offset;

		if (dm_gpio_lookup_name(ospi_mux_sel_gpio, &desc))
			return;

		if (dm_gpio_request(&desc, ospi_mux_sel_gpio))
			return;

		if (dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN))
			return;

		nand_offset = fdt_node_offset_by_compatible(blob, -1, "spi-nand");
		if (nand_offset < 0)
			return;

		nor_offset = fdt_node_offset_by_compatible(blob,
							   fdt_parent_offset(blob, nand_offset),
							   "jedec,spi-nor");

		if (dm_gpio_get_value(&desc)) {
			fdt_status_okay(blob, nand_offset);
			fdt_del_node(blob, nor_offset);
		} else {
			fdt_del_node(blob, nand_offset);
		}
	}
}

#if defined(CONFIG_XPL_BUILD)
void spl_perform_fixups(struct spl_image_info *spl_image)
{
	if (IS_ENABLED(CONFIG_K3_DDRSS)) {
		if (IS_ENABLED(CONFIG_K3_INLINE_ECC))
			fixup_ddr_driver_for_ecc(spl_image);
	} else {
		fixup_memory_node(spl_image);
	}
	detect_enable_spinand(spl_image->fdt_addr);
}
#endif

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
/*
 * Publish the reset source that R5 SPL latched for us. The register itself is
 * gone by now (SPL has to clear the latch), so the value comes from the MCU
 * scratchpad relay. Local extension, hence the vendor prefix: userspace reads
 * it back as /proc/device-tree/chosen/inhand,reset-source.
 */
static void fixup_reset_source(void *blob)
{
	u32 stat;
	int chosen;

	if (readl(TI_SRAM_SCRATCH_RESET_SRC_MAGIC) !=
	    TI_SRAM_SCRATCH_RESET_SRC_MAGIC_VAL)
		return;

	stat = readl(TI_SRAM_SCRATCH_RESET_SRC_STAT);

	chosen = fdt_find_or_add_subnode(blob, 0, "chosen");
	if (chosen < 0) {
		printf("reset-source: no /chosen node: %s\n",
		       fdt_strerror(chosen));
		return;
	}
	fdt_setprop_u32(blob, chosen, "inhand,reset-source", stat);

	env_set_hex("reset_source", stat);
}

int ft_board_setup(void *blob, struct bd_info *bd)
{

	detect_enable_spinand(blob);
	fixup_reset_source(blob);

	return 0;
}
#endif

#ifdef CONFIG_TI_I2C_BOARD_DETECT
/*
 * Functions specific to EVM and SK designs of J721S2/AM68 family.
 */

#define board_is_j721s2_som()	board_ti_k3_is("J721S2X-PM1-SOM")

#define board_is_am68_sk_som() board_ti_k3_is("AM68-SK-SOM")

int do_board_detect(void)
{
	int ret;

	if (board_ti_was_eeprom_read())
		return 0;

	ret = ti_i2c_eeprom_am6_get_base(CONFIG_EEPROM_BUS_ADDRESS,
					 CONFIG_EEPROM_CHIP_ADDRESS);
	if (ret) {
		printf("EEPROM not available at 0x%02x, trying to read at 0x%02x\n",
		       CONFIG_EEPROM_CHIP_ADDRESS, CONFIG_EEPROM_CHIP_ADDRESS + 1);
		ret = ti_i2c_eeprom_am6_get_base(CONFIG_EEPROM_BUS_ADDRESS,
						 CONFIG_EEPROM_CHIP_ADDRESS + 1);
		if (ret)
			pr_err("Reading on-board EEPROM at 0x%02x failed %d\n",
				CONFIG_EEPROM_CHIP_ADDRESS + 1, ret);
	}

	return ret;
}

int checkboard(void)
{
	struct ti_am6_eeprom *ep = TI_AM6_EEPROM_DATA;

	if (do_board_detect())
		/* EEPROM not populated */
		printf("Board: %s rev %s\n", "J721S2X-PM1-SOM", "E1");
	else
		printf("Board: %s rev %s\n", ep->name, ep->version);

	return 0;
}

static struct ti_fdt_map ti_j721s2_evm_fdt_map[] = {
	{"j721s2", "ti/k3-j721s2-common-proc-board.dtb"},
	{"am68-sk", "ti/k3-am68-sk-base-board.dtb"},
	{ /* Sentinel. */ }
};

static void setup_board_eeprom_env(void)
{
	char *name = "j721s2";

	if (do_board_detect())
		goto invalid_eeprom;

	if (board_is_j721s2_som())
		name = "j721s2";
	else if (board_is_am68_sk_som())
		name = "am68-sk";
	else
		printf("Unidentified board claims %s in eeprom header\n",
		       board_ti_get_name());

invalid_eeprom:
	set_board_info_env_am6(name);
	ti_set_fdt_env(name, ti_j721s2_evm_fdt_map);
}

static void setup_serial(void)
{
	struct ti_am6_eeprom *ep = TI_AM6_EEPROM_DATA;
	unsigned long board_serial;
	char *endp;
	char serial_string[17] = { 0 };

	if (env_get("serial#"))
		return;

	board_serial = simple_strtoul(ep->serial, &endp, 16);
	if (*endp != '\0') {
		pr_err("Error: Can't set serial# to %s\n", ep->serial);
		return;
	}

	snprintf(serial_string, sizeof(serial_string), "%016lx", board_serial);
	env_set("serial#", serial_string);
}

/*
 * Declaration of daughtercards to probe. Note that when adding more
 * cards they should be grouped by the 'i2c_addr' field to allow for a
 * more efficient probing process.
 */
static const struct {
	u8 i2c_addr;		/* I2C address of card EEPROM */
	char *card_name;	/* EEPROM-programmed card name */
	char *dtbo_name;	/* Device tree overlay to apply */
	u8 eth_offset;		/* ethXaddr MAC address index offset */
} ext_cards[] = {
	{
		0x52,
		"J7X-GESI-EXP",
		"k3-j721s2-gesi-exp-board.dtbo",
		1,		/* Start populating from eth1addr */
	},
};

#define DAUGHTER_CARD_NO_OF_MAC_ADDR	5
static bool daughter_card_detect_flags[ARRAY_SIZE(ext_cards)];

static int probe_daughtercards(void)
{
	char mac_addr[DAUGHTER_CARD_NO_OF_MAC_ADDR][TI_EEPROM_HDR_ETH_ALEN];
	bool eeprom_read_success;
	struct ti_am6_eeprom ep;
	u8 previous_i2c_addr;
	u8 mac_addr_cnt;
	int i;
	int ret;

	/* Mark previous I2C address variable as not populated */
	previous_i2c_addr = 0xff;

	/* No EEPROM data was read yet */
	eeprom_read_success = false;

	/* Iterate through list of daughtercards */
	for (i = 0; i < ARRAY_SIZE(ext_cards); i++) {
		/* Obtain card-specific I2C address */
		u8 i2c_addr = ext_cards[i].i2c_addr;

		/* Read card EEPROM if not already read previously */
		if (i2c_addr != previous_i2c_addr) {
			/* Store I2C address so we can avoid reading twice */
			previous_i2c_addr = i2c_addr;

			/* Get and parse the daughter card EEPROM record */
			ret = ti_i2c_eeprom_am6_get(CONFIG_EEPROM_BUS_ADDRESS,
						    i2c_addr,
						    &ep,
						    (char **)mac_addr,
						    DAUGHTER_CARD_NO_OF_MAC_ADDR,
						    &mac_addr_cnt);
			if (ret) {
				debug("%s: No daughtercard EEPROM at 0x%02x found %d\n",
				      __func__, i2c_addr, ret);
				eeprom_read_success = false;
				/* Skip to the next daughtercard to probe */
				continue;
			}

			/* EEPROM read successful, okay to further process. */
			eeprom_read_success = true;
		}

		/* Only continue processing if EEPROM data was read */
		if (!eeprom_read_success)
			continue;

		/* Only process the parsed data if we found a match */
		if (strncmp(ep.name, ext_cards[i].card_name, sizeof(ep.name)))
			continue;

		printf("Detected: %s rev %s\n", ep.name, ep.version);
		daughter_card_detect_flags[i] = true;

		if (!IS_ENABLED(CONFIG_XPL_BUILD)) {
			int j;
			/*
			 * Populate any MAC addresses from daughtercard into the U-Boot
			 * environment, starting with a card-specific offset so we can
			 * have multiple ext_cards contribute to the MAC pool in a well-
			 * defined manner.
			 */
			for (j = 0; j < mac_addr_cnt; j++) {
				if (!is_valid_ethaddr((u8 *)mac_addr[j]))
					continue;

				eth_env_set_enetaddr_by_index("eth", ext_cards[i].eth_offset + j,
							      (uchar *)mac_addr[j]);
			}
		}
	}

	if (!IS_ENABLED(CONFIG_XPL_BUILD)) {
		char name_overlays[1024] = { 0 };

		for (i = 0; i < ARRAY_SIZE(ext_cards); i++) {
			if (!daughter_card_detect_flags[i])
				continue;

			/* Skip if no overlays are to be added */
			if (!strlen(ext_cards[i].dtbo_name))
				continue;

			/*
			 * Make sure we are not running out of buffer space by checking
			 * if we can fit the new overlay, a trailing space to be used
			 * as a separator, plus the terminating zero.
			 */
			if (strlen(name_overlays) + strlen(ext_cards[i].dtbo_name) + 2 >
			    sizeof(name_overlays))
				return -ENOMEM;

			/* Append to our list of overlays */
			strcat(name_overlays, ext_cards[i].dtbo_name);
			strcat(name_overlays, " ");
		}

		/* Apply device tree overlay(s) to the U-Boot environment, if any */
		if (strlen(name_overlays))
			return env_set("name_overlays", name_overlays);
	}

	return 0;
}
#endif

/*
 * This function chooses the right dtb based on the board name read from
 * EEPROM if the EEPROM is programmed. Also, by default the boot chooses
 * the EVM DTB if there is no EEPROM is programmed or not detected.
 */
#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	bool eeprom_read = board_ti_was_eeprom_read();

	if (!eeprom_read || board_is_j721s2_som()) {
		if (!strcmp(name, "k3-j721s2-common-proc-board"))
			return 0;
	} else if (!eeprom_read || board_is_am68_sk_som()) {
		if (!strcmp(name, "k3-am68-sk-base-board"))
			return 0;
	}

	return -1;
}
#endif

int board_late_init(void)
{
	if (IS_ENABLED(CONFIG_TI_I2C_BOARD_DETECT)) {
		setup_board_eeprom_env();
		setup_serial();
		probe_daughtercards();
	}

	return 0;
}

ofnode cadence_qspi_get_subnode(struct udevice *dev)
{
	if (IS_ENABLED(CONFIG_SPL_BUILD) &&
	    IS_ENABLED(CONFIG_TARGET_J721S2_R5_EVM)) {
		if (spl_boot_device() == BOOT_DEVICE_SPINAND)
			return ofnode_by_compatible(dev_ofnode(dev), "spi-nand");
	}

	return dev_read_first_subnode(dev);
}

void spl_board_init(void)
{
	struct udevice *dev;
	int ret;

	/* wucl debug: disable watchdog
	 * ① 看门狗禁用必须最先执行
	 *    - 任何时刻都可能超时，越早禁用越安全
	 *    - ESM 初始化之前完成，避免 ESM 触发时看门狗干扰
	 */
	if (tps6594_wdog_disable())
		printf("WARNING: TPS6594 watchdog disable failed!\n");

	if (IS_ENABLED(CONFIG_ESM_K3)) {
		const char * const esms[] = {"esm@700000", "esm@40800000", "esm@42080000"};

		for (int i = 0; i < ARRAY_SIZE(esms); ++i) {
			ret = uclass_get_device_by_name(UCLASS_MISC, esms[i],
							&dev);
			if (ret) {
				printf("MISC init for %s failed: %d\n", esms[i], ret);
				break;
			}
		}
	}

	if (IS_ENABLED(CONFIG_ESM_PMIC) && ret == 0) {
		ret = uclass_get_device_by_driver(UCLASS_MISC,
						  DM_DRIVER_GET(pmic_esm),
						  &dev);
		if (ret)
			printf("ESM PMIC init failed: %d\n", ret);
	}
}
