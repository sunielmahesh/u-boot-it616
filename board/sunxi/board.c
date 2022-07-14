// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2012-2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 * (C) Copyright 2013 Luke Kenneth Casson Leighton <lkcl@lkcl.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some board init for the Allwinner A10-evb board.
 */

#include <common.h>
#include <clock_legacy.h>
#include <dm.h>
#include <env.h>
#include <hang.h>
#include <i2c.h>
#include <image.h>
#include <init.h>
#include <led.h>
#include <log.h>
#include <mmc.h>
#include <axp_pmic.h>
#include <generic-phy.h>
#include <phy-sun4i-usb.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#include <asm/arch/display.h>
#include <asm/arch/dram.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc.h>
#include <asm/arch/prcm.h>
#include <asm/arch/pmic_bus.h>
#include <asm/arch/spl.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <linux/delay.h>
#include <u-boot/crc.h>
#ifndef CONFIG_ARM64
#include <asm/armv7.h>
#endif
#include <asm/gpio.h>
#include <asm/io.h>
#include <u-boot/crc.h>
#include <env_internal.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <nand.h>
#include <net.h>
#include <spl.h>
#include <sy8106a.h>
#include <asm/setup.h>
#include <status_led.h>

DECLARE_GLOBAL_DATA_PTR;

#define IT6161_MIPI_RX 0x6C
#define IT6161_HDMI_TX 0x4C

#define INPUT_CLOCK_DELAY  0x01
#define F_MODE_RGB444  0
#define F_MODE_YUV422 1
#define F_MODE_YUV444 2
#define F_MODE_CLRMOD_MASK 3

#define REG_TX_SW_RST       0x04
#define B_TX_ENTEST    (1<<7)
#define B_TX_REF_RST_HDMITX (1<<5)
#define B_TX_AREF_RST (1<<4)
#define B_HDMITX_VID_RST (1<<3)
#define B_HDMITX_AUD_RST (1<<2)
#define B_TX_HDMI_RST (1<<1)
#define B_TX_HDCP_RST_HDMITX (1<<0)

#define REG_TX_AVIINFO_DB1 0x58

#define REG_TX_AFE_DRV_CTRL 0x61
#define B_TX_AFE_DRV_RST    (1<<4)

#define REG_TX_INPUT_MODE  0x70
#define O_TX_INCLKDLY   0
#define M_TX_INCLKDLY   3
#define B_TX_INDDR          (1<<2)
#define B_TX_SYNCEMB    (1<<3)
#define B_TX_2X656CLK   (1<<4)
#define B_TX_PCLKDIV2  (1<<5)
#define M_TX_INCOLMOD   (3<<6)
#define B_TX_IN_RGB    0
#define B_TX_IN_YUV422 (1<<6)
#define B_TX_IN_YUV444 (2<<6)

#define T_MODE_CCIR656 (1<<0)
#define T_MODE_SYNCEMB (1<<1)
#define T_MODE_INDDR   (1<<2)
#define T_MODE_PCLKDIV2 (1<<3)

#define REG_TX_CSC_CTRL    0x72
#define M_TX_CSC_SEL       3
#define B_TX_EN_DITHER      (1<<7)
#define B_TX_EN_UDFILTER    (1<<6)
#define B_TX_DNFREE_GO      (1<<5)

#define REG_TX_HDMI_MODE   0xC0
#define B_TX_HDMI_MODE 1
#define B_TX_DVI_MODE  0

#define REG_TX_GCP     0xC1
#define B_TX_SETAVMUTE        (1<<0)

#define REG_TX_PKT_GENERAL_CTRL    0xC6
#define B_TX_ENABLE_PKT    1
#define B_TX_REPEAT_PKT    (1<<1)

typedef enum {
        PCLK_LOW = 0,
        PCLK_MEDIUM,
        PCLK_HIGH
} VIDEOPCLKLEVEL;

typedef struct structRegSetEntry {
        u8 offset ;
        u8 invAndMask ;
        u8 OrMask ;
} RegSetEntry;

const RegSetEntry HDMITX_Init_Table[] = {
        {0x0F, 0x40, 0x00},
        {0x62, 0x08, 0x00},
        {0x64, 0x04, 0x00},
        {0x01, 0x00, 0x00},
        {0x04, 0x20, 0x20},
        {0x04, 0x1D, 0x1D},
        {0x01, 0x00, 0x00},
        {0x0F, 0x01, 0x00},
        {0xA9, 0x80, 0x80},
        {0xBF, 0x80, 0x80},
        {0xF8, 0xFF, 0xC3},
        {0xF8, 0xFF, 0xA5},
        {0xF4, 0x0C, 0x00},
        {0xF3, 0x02, 0x00},
        {0xF8, 0xFF, 0xFF},
        {0x5A, 0x0C, 0x0C},
        {0xD1, 0x0A, 0x02},
        {0x5D, 0x04, 0x04},
        {0x65, 0x03, 0x00},
        {0x71, 0xF9, 0x18},
        {0xCF, 0xFF, 0x00},
        {0xd1, 0x02, 0x00},
        {0x59, 0xD0, 0x40},
        {0xE1, 0x20, 0x20},
        {0xF5, 0x40, 0x00},
        {0x05, 0xC0, 0x40},
        {0x0C, 0xFF, 0xFF},
        {0x0D, 0xFF, 0xFF},
        {0x0E, 0x03, 0x03},
        {0x0C, 0xFF, 0x00},
        {0x0D, 0xFF, 0x00},
        {0x0E, 0x02, 0x00},
        {0x20, 0x01, 0x00},
        {0, 0, 0}
};

const RegSetEntry HDMITX_DefaultVideo_Table[] = {
        {0x72, 0xff, 0x00},
        {0x70, 0xff, 0x00},
        {0x0F, 0x01, 0x00},
        {0x72, 0xFF, 0x03},
        {0x73, 0xFF, 0x00},
        {0x74, 0xFF, 0x80},
        {0x75, 0xFF, 0x00},
        {0x76, 0xFF, 0x00},
        {0x77, 0xFF, 0x08},
        {0x78, 0xFF, 0x53},
        {0x79, 0xFF, 0x3C},
        {0x7A, 0xFF, 0x89},
        {0x7B, 0xFF, 0x3E},
        {0x7C, 0xFF, 0x00},
        {0x7D, 0xFF, 0x08},
        {0x7E, 0xFF, 0x51},
        {0x7F, 0xFF, 0x0C},
        {0x80, 0xFF, 0x00},
        {0x81, 0xFF, 0x00},
        {0x82, 0xFF, 0x00},
        {0x83, 0xFF, 0x08},
        {0x84, 0xFF, 0x00},
        {0x85, 0xFF, 0x00},
        {0x86, 0xFF, 0x87},
        {0x87, 0xFF, 0x0E},
        {0x88, 0xF0, 0x00},
        {0x04, 0x08, 0x00},
        {0, 0, 0}
};

const RegSetEntry HDMITX_SetHDMI_Table[] = {
        {0xC0, 0x01, 0x01},
        {0xC1, 0x03, 0x03},
        {0xC6, 0x03, 0x03},
        {0, 0, 0}
};

const RegSetEntry HDMITX_SetDVI_Table[] = {
        {0x0F, 0x01, 0x01},
        {0x58, 0xFF, 0x00},
        {0x0F, 0x01, 0x00},
        {0xC0, 0x01, 0x00},
        {0xC1, 0x03, 0x02},
        {0xC6, 0x03, 0x00},
        {0, 0, 0}
};

const RegSetEntry HDMITX_DefaultAVIInfo_Table[] = {
        {0x0F, 0x01, 0x01},
        {0x58, 0xFF, 0x10},
        {0x59, 0xFF, 0x08},
        {0x5A, 0xFF, 0x00},
        {0x5B, 0xFF, 0x00},
        {0x5C, 0xFF, 0x00},
        {0x5D, 0xFF, 0x57},
        {0x5E, 0xFF, 0x00},
        {0x5F, 0xFF, 0x00},
        {0x60, 0xFF, 0x00},
        {0x61, 0xFF, 0x00},
        {0x62, 0xFF, 0x00},
        {0x63, 0xFF, 0x00},
        {0x64, 0xFF, 0x00},
        {0x65, 0xFF, 0x00},
        {0x0F, 0x01, 0x00},
        {0xCD, 0x03, 0x03},
        {0, 0, 0}
};

static inline int Switch_HDMITX_Bank(struct udevice *it6161, int x);

void i2c_init_board(void)
{
#ifdef CONFIG_I2C0_ENABLE
#if defined(CONFIG_MACH_SUN4I) || \
    defined(CONFIG_MACH_SUN5I) || \
    defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(0), SUN4I_GPB_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(1), SUN4I_GPB_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(14), SUN6I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(15), SUN6I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN8I_V3S)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(6), SUN8I_V3S_GPB_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(7), SUN8I_V3S_GPB_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(2), SUN8I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(3), SUN8I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN50I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(0), SUN50I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(1), SUN50I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#endif
#endif

#ifdef CONFIG_I2C1_ENABLE
#if defined(CONFIG_MACH_SUN4I) || \
    defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(18), SUN4I_GPB_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(19), SUN4I_GPB_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN5I)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(15), SUN5I_GPB_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(16), SUN5I_GPB_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(16), SUN6I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(17), SUN6I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(4), SUN8I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(5), SUN8I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN50I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(2), SUN50I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(3), SUN50I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#endif
#endif

#ifdef CONFIG_R_I2C_ENABLE
#ifdef CONFIG_MACH_SUN50I
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(8), SUN50I_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(9), SUN50I_GPL_R_TWI);
#elif CONFIG_MACH_SUN50I_H616
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN50I_H616_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN50I_H616_GPL_R_TWI);
#else
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN8I_H3_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN8I_H3_GPL_R_TWI);
#endif
#endif
}

/*
 * Try to use the environment from the boot source first.
 * For MMC, this means a FAT partition on the boot device (SD or eMMC).
 * If the raw MMC environment is also enabled, this is tried next.
 * When booting from NAND we try UBI first, then NAND directly.
 * SPI flash falls back to FAT (on SD card).
 */
enum env_location env_get_location(enum env_operation op, int prio)
{
	if (prio > 1)
		return ENVL_UNKNOWN;

	/* NOWHERE is exclusive, no other option can be defined. */
	if (IS_ENABLED(CONFIG_ENV_IS_NOWHERE))
		return ENVL_NOWHERE;

	switch (sunxi_get_boot_device()) {
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
			return ENVL_MMC;
		break;
	case BOOT_DEVICE_NAND:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_NAND))
			return ENVL_NAND;
		break;
	case BOOT_DEVICE_SPI:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_SPI_FLASH))
			return ENVL_SPI_FLASH;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		break;
	case BOOT_DEVICE_BOARD:
		break;
	default:
		break;
	}

	/*
	 * If we come here for the first time, we *must* return a valid
	 * environment location other than ENVL_UNKNOWN, or the setup sequence
	 * in board_f() will silently hang. This is arguably a bug in
	 * env_init(), but for now pick one environment for which we know for
	 * sure to have a driver for. For all defconfigs this is either FAT
	 * or UBI, or NOWHERE, which is already handled above.
	 */
	if (prio == 0) {
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
	}

	return ENVL_UNKNOWN;
}

static int it6161_write(struct udevice *dev, uint addr,
                      uint8_t value)
{
        int err;

        err = dm_i2c_write(dev, addr, &value, 1);

        return err;
}

static int it6161_read(struct udevice *dev, unsigned int addr)
{
        uint8_t valb;
        int err;

        err = dm_i2c_read(dev, addr, &valb, 1);
        if (err)
                return err;

        return valb;
}

static int it6161_set_bits(struct udevice *dev, uint addr,
                         uint mask, uint value)
{
        uint8_t valb;
        int err;

        err = dm_i2c_read(dev, addr, &valb, 1);
        if (err)
                return err;

        valb &= ~mask;
        valb |= value;

        err = dm_i2c_write(dev, addr, &valb, 1);

        return err;
}

static void hdmitx_FireAFE(struct udevice *it6161)
{
        Switch_HDMITX_Bank(it6161, 0);
        it6161_write(it6161, REG_TX_AFE_DRV_CTRL,0);
}

static inline int Switch_HDMITX_Bank(struct udevice *it6161, int x)
{
        return it6161_set_bits(it6161, 0x0F, 1, (x)&1);
}

static void setHDMITX_AVMute(struct udevice *it6161, u8 bEnable)
{
        Switch_HDMITX_Bank(it6161, 0);
        it6161_set_bits(it6161, REG_TX_GCP,B_TX_SETAVMUTE, bEnable?B_TX_SETAVMUTE:0);
        it6161_write(it6161, REG_TX_PKT_GENERAL_CTRL, B_TX_ENABLE_PKT|B_TX_REPEAT_PKT);
}

static void hdmitx_SetupAFE(struct udevice *it6161, VIDEOPCLKLEVEL level)
{
        printf("%s: pclk:%s", __func__, level ? "high" : "low");
        it6161_write(it6161, REG_TX_AFE_DRV_CTRL,B_TX_AFE_DRV_RST);
        switch (level) {
        case PCLK_HIGH:
                it6161_set_bits(it6161, 0x62, 0x90, 0x80);
                it6161_set_bits(it6161, 0x64, 0x89, 0x80);
                it6161_set_bits(it6161, 0x68, 0x10, 0x00);
                it6161_set_bits(it6161, 0x66, 0x80, 0x80);
                break ;
        default:
                it6161_set_bits(it6161, 0x62, 0x90, 0x10);
                it6161_set_bits(it6161, 0x64, 0x89, 0x09);
                it6161_set_bits(it6161, 0x68, 0x10, 0x10);
                break ;
        }
        it6161_set_bits(it6161, REG_TX_SW_RST,B_TX_REF_RST_HDMITX|B_HDMITX_VID_RST,0);
        it6161_write(it6161, REG_TX_AFE_DRV_CTRL,0);
}

static void hdmitx_SetCSCScale(struct udevice *it6161, u8 bInputMode,u8 bOutputMode)
{
        u8 ucData;

        it6161_set_bits(it6161, 0xF, 0x10, 0x10);
        ucData = it6161_read(it6161, REG_TX_CSC_CTRL) & ~(M_TX_CSC_SEL|B_TX_DNFREE_GO|B_TX_EN_DITHER|B_TX_EN_UDFILTER);
        it6161_write(it6161, REG_TX_CSC_CTRL,ucData);
}

static void hdmitx_SetInputMode(struct udevice *it6161, u8 InputColorMode,u8 bInputSignalType)
{
        u8 ucData ;

        ucData = it6161_read(it6161, REG_TX_INPUT_MODE);
        ucData &= ~(M_TX_INCOLMOD|B_TX_2X656CLK|B_TX_SYNCEMB|B_TX_INDDR|B_TX_PCLKDIV2);
        ucData |= INPUT_CLOCK_DELAY ;

        switch(InputColorMode & F_MODE_CLRMOD_MASK) {
        case F_MODE_YUV422:
                ucData |= B_TX_IN_YUV422 ;
                break ;
        case F_MODE_YUV444:
                ucData |= B_TX_IN_YUV444 ;
                break ;
        case F_MODE_RGB444:
        default:
                ucData |= B_TX_IN_RGB ;
                break ;
        }
        if(bInputSignalType & T_MODE_PCLKDIV2)
                ucData |= B_TX_PCLKDIV2 ;

        if(bInputSignalType & T_MODE_CCIR656)
                ucData |= B_TX_2X656CLK ;

        if(bInputSignalType & T_MODE_SYNCEMB)
                ucData |= B_TX_SYNCEMB ;

        if(bInputSignalType & T_MODE_INDDR)
                ucData |= B_TX_INDDR ;

        it6161_write(it6161, REG_TX_INPUT_MODE,ucData);
}

static void hdmitx_loadregsetting(struct udevice *it6161, const RegSetEntry table[])
{
    int i ;

    for( i = 0 ;  ; i++ )
    {
        if( table[i].offset == 0 && table[i].invAndMask == 0 && table[i].OrMask == 0 )
        {
            return ;
        }
        else if( table[i].invAndMask == 0 && table[i].OrMask == 0 )
        {
            printf("delay(%d)\n",(int)table[i].offset);
            mdelay(table[i].offset);
        }
        else if( table[i].invAndMask == 0xFF )
        {
            printf("it6161_write(it6161, %02x,%02x)\n", (int)table[i].offset, (int)table[i].OrMask);
            it6161_write(it6161, table[i].offset, table[i].OrMask);
        }
        else
        {
            printf("it6161_set_bits(it6161, %02x,%02x,%02x)\n", (int)table[i].offset, (int)table[i].invAndMask, (int)table[i].OrMask);
            it6161_set_bits(it6161, table[i].offset, table[i].invAndMask, table[i].OrMask);
        }
    }
}

static bool HDMITX_EnableVideoOutput(struct udevice *it6161, VIDEOPCLKLEVEL level,u8 inputColorMode,u8 outputColorMode,u8 bHDMI)
{

    it6161_write(it6161, REG_TX_SW_RST, B_HDMITX_VID_RST|B_HDMITX_AUD_RST|B_TX_AREF_RST|B_TX_HDCP_RST_HDMITX);

    Switch_HDMITX_Bank(it6161, 1);
    it6161_write(it6161, REG_TX_AVIINFO_DB1, 0x00);
    Switch_HDMITX_Bank(it6161, 0);

    if(bHDMI)
    {
        setHDMITX_AVMute(it6161, true);
    }
    hdmitx_SetInputMode(it6161, inputColorMode, 0);

    hdmitx_SetCSCScale(it6161, inputColorMode, outputColorMode);

    if(bHDMI)
    {
        it6161_write(it6161, REG_TX_HDMI_MODE, B_TX_HDMI_MODE);
    }
    else
    {
        it6161_write(it6161, REG_TX_HDMI_MODE, B_TX_DVI_MODE);
    }

    hdmitx_SetupAFE(it6161, level);
    it6161_write(it6161, REG_TX_SW_RST, B_HDMITX_AUD_RST|B_TX_AREF_RST|B_TX_HDCP_RST_HDMITX);

    hdmitx_FireAFE(it6161);
        return true;
}

static void mipirx_reset(struct udevice *it6161)
{
        it6161_set_bits(it6161, 0x10, 0x0F, 0x0F);
        mdelay(1);
        it6161_set_bits(it6161, 0x10, 0x0F, 0x00);
        it6161_set_bits(it6161, 0x05, 0x08, 0x08);
        mdelay(1);
        it6161_set_bits(it6161, 0x05, 0x08, 0x00);
        it6161_set_bits(it6161, 0x0d, 0x02, 0x00);
        it6161_set_bits(it6161, 0x0C, 0x0F, 0x03);
        it6161_set_bits(it6161, 0x11, 0x3F, 0x01);
        it6161_set_bits(it6161, 0x12, 0x03, 0x00);
        it6161_set_bits(it6161, 0x18, 0xf7, (4<<4) | 0x03);
        it6161_set_bits(it6161, 0x19, 0xf3, (12<<4) | 0x03);
        it6161_set_bits(it6161, 0x20, 0xf7, 0x03);
        it6161_set_bits(it6161, 0x21, 0x07, 0);

        it6161_set_bits(it6161, 0x44, 0x3a, 0x32);
        it6161_set_bits(it6161, 0x4B, 0x1f, 0x01);
        it6161_write(it6161, 0x4C, 4);
        it6161_set_bits(it6161, 0x4D, 0x01, (4>>8) & 0x01);
        it6161_set_bits(it6161, 0x4E, 0x0C, 0x00);
        it6161_set_bits(it6161, 0x4F, 0x03, 0x01);
        it6161_write(it6161, 0x27, 0x3E);
        it6161_set_bits(it6161, 0x70, 0x01, 0x01);
        it6161_write(it6161, 0x72, 4);
        it6161_write(it6161, 0x73, 3);
        it6161_set_bits(it6161, 0x80, 0x40, 0x00);
        it6161_write(it6161, 0x21, 0x00);
        it6161_set_bits(it6161, 0x84, 0x70, 0x00);
        it6161_set_bits(it6161, 0xA0, 0x01, 1);
        it6161_set_bits(it6161, 0x80, 0x1F, 0x02);
        it6161_set_bits(it6161, 0x70, 0x01, 1);
        it6161_set_bits(it6161, 0x05, 0x02, 0x02);
        it6161_write(it6161, 0xA1, 0x00);
        it6161_write(it6161, 0xA2, 0x00);
        it6161_write(it6161, 0xA3, 0x08);
        it6161_write(it6161, 0xA5, 0x04);
        it6161_set_bits(it6161, 0x31, 0x80, 0x00);
        it6161_set_bits(it6161, 0x33, 0x80, 0x00);
        it6161_set_bits(it6161, 0x35, 0x80, 0x00);
        it6161_set_bits(it6161, 0x37, 0x80, 0x00);
        it6161_set_bits(it6161, 0x39, 0x80, 0x00);
        it6161_set_bits(it6161, 0x3A, 0x80, 0x00);
        it6161_set_bits(it6161, 0x3C, 0x80, 0x00);
        it6161_set_bits(it6161, 0x3E, 0x80, 0x00);
        it6161_set_bits(it6161, 0x41, 0x80, 0x00);
        it6161_set_bits(it6161, 0x43, 0x80, 0x00);
}

static void cal_rclk(struct udevice *it6161)
{
        u8 i;
        int t10usint;
        u32 sum, RxRCLK;

        sum = 0;
        for(i=0; i<5; i++)
        {
                it6161_set_bits(it6161, 0x94, 0x80, 0x80);
                mdelay(100);
                it6161_set_bits(it6161, 0x94, 0x80, 0x00);

                RxRCLK = it6161_read(it6161, 0x97);
                RxRCLK <<= 8;
                RxRCLK += it6161_read(it6161, 0x96);
                RxRCLK <<=8;
                RxRCLK += it6161_read(it6161, 0x95);
                sum += RxRCLK;

        }
        sum /= 5;
        printf("\n");

        RxRCLK = sum/100;
        t10usint = RxRCLK/100;
        printf("RxRCLK = %d,%03d,%03d\n",(sum*10)/1000000,((sum*10)%1000000)/1000,((sum*10)%100));
        printf("T10usInt=0x%03X\n", (int)t10usint);
        it6161_write(it6161, 0x91, t10usint&0xFF);
}

static void init_mipi_rx(struct udevice *it6161)
{
        printf("Init MIPIRX\n");
        mipirx_reset(it6161);
        cal_rclk(it6161);

        it6161_set_bits(it6161, 0x05, 0x03, 0x00);
        it6161_set_bits(it6161, 0x08, 0x10, 0x10);
}

static void set_ppara(struct udevice *it6161)
{
        it6161_set_bits(it6161, 0x05, 0x04, 0x04);
        it6161_set_bits(it6161, 0x05, 0x04, 0x00);
        it6161_set_bits(it6161, 0x09, 0x10, 0x10);
}

static void it6161_init(void)
{
        struct udevice *bus, *hdmi_tx, *mipi_rx;
        int i2c_bus = 1, ret;
        u8 val;

        printf("start %s", __func__);
        ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
        if (ret) {
                printf("%s: No bus %d\n", __func__, i2c_bus);
                return;
        }

        ret = dm_i2c_probe(bus, IT6161_MIPI_RX, 0, &mipi_rx);
        if (ret) {
                printf("return val it6161: %d\n", ret);
                printf("%s: Can't find device id=0x%x, on bus %d\n",
                        __func__, IT6161_MIPI_RX, i2c_bus);
                return;
        }

        ret = dm_i2c_probe(bus, IT6161_HDMI_TX, 0, &hdmi_tx);
        if (ret) {
                printf("return val hdmi: %d\n", ret);
                printf("%s: Can't find device id=0x%x, on bus %d\n",
                        __func__, IT6161_HDMI_TX, i2c_bus);
                return;
        }

        init_mipi_rx(mipi_rx);
        hdmitx_loadregsetting(hdmi_tx, HDMITX_Init_Table);
        hdmitx_loadregsetting(hdmi_tx, HDMITX_DefaultVideo_Table);
        hdmitx_loadregsetting(hdmi_tx, HDMITX_SetHDMI_Table);
        hdmitx_loadregsetting(hdmi_tx, HDMITX_DefaultAVIInfo_Table);
        set_ppara(mipi_rx);
        HDMITX_EnableVideoOutput(hdmi_tx, 0, 0, 0, 0);
        setHDMITX_AVMute(hdmi_tx, false);
}

#ifdef CONFIG_DM_MMC
static void mmc_pinmux_setup(int sdc);
#endif

#ifdef CONFIG_LED_GPIO
#define DLDO1_VOLT             3300

static int renew_dlpc_gpio(void)
{
	struct udevice *dev;
	int ret;

       ret = led_get_by_label("renew-e:proj-on", &dev);
       if (ret) {
               printf("failed to probe 'renew-e:proj-on' lable\n");
               return ret;
       }

       ret = led_set_state(dev, LEDST_ON);
       if (ret) {
               printf("failed to set PROJ_ON GPIO\n");
               return ret;
       }

       ret = axp_set_dldo(1, DLDO1_VOLT);
       if (ret) {
               printf("failed to set 3.3V to DLDO1\n");
               return ret;
       }

       mdelay(10);

       ret = led_get_by_label("renew-e:reset", &dev);
       if (ret) {
               printf("failed to probe 'renew-e:reset' lable\n");
               return ret;
       }

       ret = led_set_state(dev, LEDST_ON);
       if (ret) {
               printf("failed to set reset GPIO\n");
               return ret;
       }

       mdelay(500);

       return 0;
}

static int renew_it6161_gpio(void)
{
       struct udevice *dev;
        int ret;

       ret = led_get_by_label("renew:it6161-rstn", &dev);
       if (ret) {
               printf("failed to probe 'renew:it6161-rstn' lable\n");
               return ret;
       }

       ret = led_set_state(dev, LEDST_ON);
        if (ret) {
                printf("failed to set it6161 rstn GPIO\n");
                return ret;
        }

       ret = axp_set_eldo(1, 1200);
        if (ret) {
                printf("failed to set 1.2V to ELDO1\n");
                return ret;
        }

        mdelay(500);
       return 0;
}
#endif

/* add board specific code here */
int board_init(void)
{
	__maybe_unused int id_pfr1, ret, satapwr_pin, macpwr_pin;

	gd->bd->bi_boot_params = (PHYS_SDRAM_0 + 0x100);

#if !defined(CONFIG_ARM64) && !defined(CONFIG_MACH_SUNIV)
	asm volatile("mrc p15, 0, %0, c0, c1, 1" : "=r"(id_pfr1));
	debug("id_pfr1: 0x%08x\n", id_pfr1);
	/* Generic Timer Extension available? */
	if ((id_pfr1 >> CPUID_ARM_GENTIMER_SHIFT) & 0xf) {
		uint32_t freq;

		debug("Setting CNTFRQ\n");

		/*
		 * CNTFRQ is a secure register, so we will crash if we try to
		 * write this from the non-secure world (read is OK, though).
		 * In case some bootcode has already set the correct value,
		 * we avoid the risk of writing to it.
		 */
		asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r"(freq));
		if (freq != CONFIG_COUNTER_FREQUENCY) {
			debug("arch timer frequency is %d Hz, should be %d, fixing ...\n",
			      freq, CONFIG_COUNTER_FREQUENCY);
#ifdef CONFIG_NON_SECURE
			printf("arch timer frequency is wrong, but cannot adjust it\n");
#else
			asm volatile("mcr p15, 0, %0, c14, c0, 0"
				     : : "r"(CONFIG_COUNTER_FREQUENCY));
#endif
		}
	}
#endif /* !CONFIG_ARM64 && !CONFIG_MACH_SUNIV */

	ret = axp_gpio_init();
	if (ret)
		return ret;

	/* strcmp() would look better, but doesn't get optimised away. */
	if (CONFIG_SATAPWR[0]) {
		satapwr_pin = sunxi_name_to_gpio(CONFIG_SATAPWR);
		if (satapwr_pin >= 0) {
			gpio_request(satapwr_pin, "satapwr");
			gpio_direction_output(satapwr_pin, 1);

			/*
			 * Give the attached SATA device time to power-up
			 * to avoid link timeouts
			 */
			mdelay(500);
		}
	}

	if (CONFIG_MACPWR[0]) {
		macpwr_pin = sunxi_name_to_gpio(CONFIG_MACPWR);
		if (macpwr_pin >= 0) {
			gpio_request(macpwr_pin, "macpwr");
			gpio_direction_output(macpwr_pin, 1);
		}
	}

#if CONFIG_IS_ENABLED(DM_I2C)
	/*
	 * Temporary workaround for enabling I2C clocks until proper sunxi DM
	 * clk, reset and pinctrl drivers land.
	 */
	i2c_init_board();
#endif

/* enable DM PMIC in u-boot */
	pmic_bus_init();

#if CONFIG_LED_GPIO
	renew_dlpc_gpio();
	renew_it6161_gpio();
#endif

	it6161_init();

	eth_init_board();

	return 0;
}

/*
 * On older SoCs the SPL is actually at address zero, so using NULL as
 * an error value does not work.
 */
#define INVALID_SPL_HEADER ((void *)~0UL)

static struct boot_file_head * get_spl_header(uint8_t req_version)
{
	struct boot_file_head *spl = (void *)(ulong)SPL_ADDR;
	uint8_t spl_header_version = spl->spl_signature[3];

	/* Is there really the SPL header (still) there? */
	if (memcmp(spl->spl_signature, SPL_SIGNATURE, 3) != 0)
		return INVALID_SPL_HEADER;

	if (spl_header_version < req_version) {
		printf("sunxi SPL version mismatch: expected %u, got %u\n",
		       req_version, spl_header_version);
		return INVALID_SPL_HEADER;
	}

	return spl;
}

static const char *get_spl_dt_name(void)
{
	struct boot_file_head *spl = get_spl_header(SPL_DT_HEADER_VERSION);

	/* Check if there is a DT name stored in the SPL header. */
	if (spl != INVALID_SPL_HEADER && spl->dt_name_offset)
		return (char *)spl + spl->dt_name_offset;

	return NULL;
}

int dram_init(void)
{
	struct boot_file_head *spl = get_spl_header(SPL_DRAM_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		gd->ram_size = get_ram_size((long *)PHYS_SDRAM_0,
					    PHYS_SDRAM_0_SIZE);
	else
		gd->ram_size = (phys_addr_t)spl->dram_size << 20;

	if (gd->ram_size > CONFIG_SUNXI_DRAM_MAX_SIZE)
		gd->ram_size = CONFIG_SUNXI_DRAM_MAX_SIZE;

	return 0;
}

#if defined(CONFIG_NAND_SUNXI)
static void nand_pinmux_setup(void)
{
	unsigned int pin;

	for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(19); pin++)
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_NAND);

#if defined CONFIG_MACH_SUN4I || defined CONFIG_MACH_SUN7I
	for (pin = SUNXI_GPC(20); pin <= SUNXI_GPC(22); pin++)
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_NAND);
#endif
	/* sun4i / sun7i do have a PC23, but it is not used for nand,
	 * only sun7i has a PC24 */
#ifdef CONFIG_MACH_SUN7I
	sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_NAND);
#endif
}

static void nand_clock_setup(void)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	setbits_le32(&ccm->ahb_gate0, (CLK_GATE_OPEN << AHB_GATE_OFFSET_NAND0));
#if defined CONFIG_MACH_SUN6I || defined CONFIG_MACH_SUN8I || \
    defined CONFIG_MACH_SUN9I || defined CONFIG_MACH_SUN50I
	setbits_le32(&ccm->ahb_reset0_cfg, (1 << AHB_GATE_OFFSET_NAND0));
#endif
	setbits_le32(&ccm->nand0_clk_cfg, CCM_NAND_CTRL_ENABLE | AHB_DIV_1);
}

void board_nand_init(void)
{
	nand_pinmux_setup();
	nand_clock_setup();
#ifndef CONFIG_SPL_BUILD
	sunxi_nand_init();
#endif
}
#endif

#ifdef CONFIG_MMC
static void mmc_pinmux_setup(int sdc)
{
	unsigned int pin;

	switch (sdc) {
	case 0:
		/* SDC0: PF0-PF5 */
		for (pin = SUNXI_GPF(0); pin <= SUNXI_GPF(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPF_SDC0);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
		break;

	case 1:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
		if (IS_ENABLED(CONFIG_MMC1_PINS_PH)) {
			/* SDC1: PH22-PH-27 */
			for (pin = SUNXI_GPH(22); pin <= SUNXI_GPH(27); pin++) {
				sunxi_gpio_set_cfgpin(pin, SUN4I_GPH_SDC1);
				sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
				sunxi_gpio_set_drv(pin, 2);
			}
		} else {
			/* SDC1: PG0-PG5 */
			for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
				sunxi_gpio_set_cfgpin(pin, SUN4I_GPG_SDC1);
				sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
				sunxi_gpio_set_drv(pin, 2);
			}
		}
#elif defined(CONFIG_MACH_SUN5I)
		/* SDC1: PG3-PG8 */
		for (pin = SUNXI_GPG(3); pin <= SUNXI_GPG(8); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN5I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC1: PG0-PG5 */
		for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN6I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN8I)
		/* SDC1: PG0-PG5 */
		for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN8I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#endif
		break;

	case 2:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I)
		/* SDC2: PC6-PC11 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(11); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN5I)
		/* SDC2: PC6-PC15 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC2: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_SDC2);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#elif defined(CONFIG_MACH_SUN8I_R40)
		/* SDC2: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_SDC2);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#elif defined(CONFIG_MACH_SUN8I) || defined(CONFIG_MACH_SUN50I)
		/* SDC2: PC5-PC6, PC8-PC16 */
		for (pin = SUNXI_GPC(5); pin <= SUNXI_GPC(6); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		for (pin = SUNXI_GPC(8); pin <= SUNXI_GPC(16); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN50I_H6)
		/* SDC2: PC4-PC14 */
		for (pin = SUNXI_GPC(4); pin <= SUNXI_GPC(14); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN50I_H616)
		/* SDC2: PC0-PC1, PC5-PC6, PC8-PC11, PC13-PC16 */
		for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(16); pin++) {
			if (pin > SUNXI_GPC(1) && pin < SUNXI_GPC(5))
				continue;
			if (pin == SUNXI_GPC(7) || pin == SUNXI_GPC(12))
				continue;
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 3);
		}
#elif defined(CONFIG_MACH_SUN9I)
		/* SDC2: PC6-PC16 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(16); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#else
		puts("ERROR: No pinmux setup defined for MMC2!\n");
#endif
		break;

	case 3:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
		/* SDC3: PI4-PI9 */
		for (pin = SUNXI_GPI(4); pin <= SUNXI_GPI(9); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPI_SDC3);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC3: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN6I_GPC_SDC3);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUN6I_GPC_SDC3);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#endif
		break;

	default:
		printf("sunxi: invalid MMC slot %d for pinmux setup\n", sdc);
		break;
	}
}

int board_mmc_init(struct bd_info *bis)
{
	if (!IS_ENABLED(CONFIG_UART0_PORT_F)) {
		mmc_pinmux_setup(CONFIG_MMC_SUNXI_SLOT);
		if (!sunxi_mmc_init(CONFIG_MMC_SUNXI_SLOT))
			return -1;
	}

	if (CONFIG_MMC_SUNXI_SLOT_EXTRA != -1) {
		mmc_pinmux_setup(CONFIG_MMC_SUNXI_SLOT_EXTRA);
		if (!sunxi_mmc_init(CONFIG_MMC_SUNXI_SLOT_EXTRA))
			return -1;
	}

	return 0;
}

#if CONFIG_MMC_SUNXI_SLOT_EXTRA != -1
int mmc_get_env_dev(void)
{
	switch (sunxi_get_boot_device()) {
	case BOOT_DEVICE_MMC1:
		return 0;
	case BOOT_DEVICE_MMC2:
		return 1;
	default:
		return CONFIG_SYS_MMC_ENV_DEV;
	}
}
#endif
#endif

#ifdef CONFIG_SPL_BUILD

static void sunxi_spl_store_dram_size(phys_addr_t dram_size)
{
	struct boot_file_head *spl = get_spl_header(SPL_DT_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	/* Promote the header version for U-Boot proper, if needed. */
	if (spl->spl_signature[3] < SPL_DRAM_HEADER_VERSION)
		spl->spl_signature[3] = SPL_DRAM_HEADER_VERSION;

	spl->dram_size = dram_size >> 20;
}

void sunxi_board_init(void)
{
	int power_failed = 0;

#ifdef CONFIG_LED_STATUS
	if (IS_ENABLED(CONFIG_SPL_DRIVERS_MISC))
		status_led_init();
#endif

#ifdef CONFIG_SY8106A_POWER
	power_failed = sy8106a_set_vout1(CONFIG_SY8106A_VOUT1_VOLT);
#endif

#if defined CONFIG_AXP152_POWER || defined CONFIG_AXP209_POWER || \
	defined CONFIG_AXP221_POWER || defined CONFIG_AXP305_POWER || \
	defined CONFIG_AXP809_POWER || defined CONFIG_AXP818_POWER
	power_failed = axp_init();

	if (IS_ENABLED(CONFIG_AXP_DISABLE_BOOT_ON_POWERON) && !power_failed) {
		u8 boot_reason;

		pmic_bus_read(AXP_POWER_STATUS, &boot_reason);
		if (boot_reason & AXP_POWER_STATUS_ALDO_IN) {
			printf("Power on by plug-in, shutting down.\n");
			pmic_bus_write(0x32, BIT(7));
		}
	}

#if defined CONFIG_AXP221_POWER || defined CONFIG_AXP809_POWER || \
	defined CONFIG_AXP818_POWER
	power_failed |= axp_set_dcdc1(CONFIG_AXP_DCDC1_VOLT);
#endif
#if !defined(CONFIG_AXP305_POWER)
	power_failed |= axp_set_dcdc2(CONFIG_AXP_DCDC2_VOLT);
	power_failed |= axp_set_dcdc3(CONFIG_AXP_DCDC3_VOLT);
#endif
#if !defined(CONFIG_AXP209_POWER) && !defined(CONFIG_AXP818_POWER)
	power_failed |= axp_set_dcdc4(CONFIG_AXP_DCDC4_VOLT);
#endif
#if defined CONFIG_AXP221_POWER || defined CONFIG_AXP809_POWER || \
	defined CONFIG_AXP818_POWER
	power_failed |= axp_set_dcdc5(CONFIG_AXP_DCDC5_VOLT);
#endif

#if defined CONFIG_AXP221_POWER || defined CONFIG_AXP809_POWER || \
	defined CONFIG_AXP818_POWER
	power_failed |= axp_set_aldo1(CONFIG_AXP_ALDO1_VOLT);
#endif
#if !defined(CONFIG_AXP305_POWER)
	power_failed |= axp_set_aldo2(CONFIG_AXP_ALDO2_VOLT);
#endif
#if !defined(CONFIG_AXP152_POWER) && !defined(CONFIG_AXP305_POWER)
	power_failed |= axp_set_aldo3(CONFIG_AXP_ALDO3_VOLT);
#endif
#ifdef CONFIG_AXP209_POWER
	power_failed |= axp_set_aldo4(CONFIG_AXP_ALDO4_VOLT);
#endif

#if defined(CONFIG_AXP221_POWER) || defined(CONFIG_AXP809_POWER) || \
	defined(CONFIG_AXP818_POWER)
	power_failed |= axp_set_dldo(1, CONFIG_AXP_DLDO1_VOLT);
	power_failed |= axp_set_dldo(2, CONFIG_AXP_DLDO2_VOLT);
#if !defined CONFIG_AXP809_POWER
	power_failed |= axp_set_dldo(3, CONFIG_AXP_DLDO3_VOLT);
	power_failed |= axp_set_dldo(4, CONFIG_AXP_DLDO4_VOLT);
#endif
	power_failed |= axp_set_eldo(1, CONFIG_AXP_ELDO1_VOLT);
	power_failed |= axp_set_eldo(2, CONFIG_AXP_ELDO2_VOLT);
	power_failed |= axp_set_eldo(3, CONFIG_AXP_ELDO3_VOLT);
#endif

#ifdef CONFIG_AXP818_POWER
	power_failed |= axp_set_fldo(1, CONFIG_AXP_FLDO1_VOLT);
	power_failed |= axp_set_fldo(2, CONFIG_AXP_FLDO2_VOLT);
	power_failed |= axp_set_fldo(3, CONFIG_AXP_FLDO3_VOLT);
#endif

#if defined CONFIG_AXP809_POWER || defined CONFIG_AXP818_POWER
	power_failed |= axp_set_sw(IS_ENABLED(CONFIG_AXP_SW_ON));
#endif
#endif
	printf("DRAM:");
	gd->ram_size = sunxi_dram_init();
	printf(" %d MiB\n", (int)(gd->ram_size >> 20));
	if (!gd->ram_size)
		hang();

	sunxi_spl_store_dram_size(gd->ram_size);

	/*
	 * Only clock up the CPU to full speed if we are reasonably
	 * assured it's being powered with suitable core voltage
	 */
	if (!power_failed)
		clock_set_pll1(get_board_sys_clk());
	else
		printf("Failed to set core voltage! Can't set CPU frequency\n");
}
#endif

#ifdef CONFIG_USB_GADGET
int g_dnl_board_usb_cable_connected(void)
{
	struct udevice *dev;
	struct phy phy;
	int ret;

	ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
	if (ret) {
		pr_err("%s: Cannot find USB device\n", __func__);
		return ret;
	}

	ret = generic_phy_get_by_name(dev, "usb", &phy);
	if (ret) {
		pr_err("failed to get %s USB PHY\n", dev->name);
		return ret;
	}

	ret = generic_phy_init(&phy);
	if (ret) {
		pr_debug("failed to init %s USB PHY\n", dev->name);
		return ret;
	}

	return sun4i_usb_phy_vbus_detect(&phy);
}
#endif

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	char *serial_string;
	unsigned long long serial;

	serial_string = env_get("serial#");

	if (serial_string) {
		serial = simple_strtoull(serial_string, NULL, 16);

		serialnr->high = (unsigned int) (serial >> 32);
		serialnr->low = (unsigned int) (serial & 0xffffffff);
	} else {
		serialnr->high = 0;
		serialnr->low = 0;
	}
}
#endif

/*
 * Check the SPL header for the "sunxi" variant. If found: parse values
 * that might have been passed by the loader ("fel" utility), and update
 * the environment accordingly.
 */
static void parse_spl_header(const uint32_t spl_addr)
{
	struct boot_file_head *spl = get_spl_header(SPL_ENV_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	if (!spl->fel_script_address)
		return;

	if (spl->fel_uEnv_length != 0) {
		/*
		 * data is expected in uEnv.txt compatible format, so "env
		 * import -t" the string(s) at fel_script_address right away.
		 */
		himport_r(&env_htab, (char *)(uintptr_t)spl->fel_script_address,
			  spl->fel_uEnv_length, '\n', H_NOCLEAR, 0, 0, NULL);
		return;
	}
	/* otherwise assume .scr format (mkimage-type script) */
	env_set_hex("fel_scriptaddr", spl->fel_script_address);
}

static bool get_unique_sid(unsigned int *sid)
{
	if (sunxi_get_sid(sid) != 0)
		return false;

	if (!sid[0])
		return false;

	/*
	 * The single words 1 - 3 of the SID have quite a few bits
	 * which are the same on many models, so we take a crc32
	 * of all 3 words, to get a more unique value.
	 *
	 * Note we only do this on newer SoCs as we cannot change
	 * the algorithm on older SoCs since those have been using
	 * fixed mac-addresses based on only using word 3 for a
	 * long time and changing a fixed mac-address with an
	 * u-boot update is not good.
	 */
#if !defined(CONFIG_MACH_SUN4I) && !defined(CONFIG_MACH_SUN5I) && \
    !defined(CONFIG_MACH_SUN6I) && !defined(CONFIG_MACH_SUN7I) && \
    !defined(CONFIG_MACH_SUN8I_A23) && !defined(CONFIG_MACH_SUN8I_A33)
	sid[3] = crc32(0, (unsigned char *)&sid[1], 12);
#endif

	/* Ensure the NIC specific bytes of the mac are not all 0 */
	if ((sid[3] & 0xffffff) == 0)
		sid[3] |= 0x800000;

	return true;
}

/*
 * Note this function gets called multiple times.
 * It must not make any changes to env variables which already exist.
 */
static void setup_environment(const void *fdt)
{
	char serial_string[17] = { 0 };
	unsigned int sid[4];
	uint8_t mac_addr[6];
	char ethaddr[16];
	int i;

	if (!get_unique_sid(sid))
		return;

	for (i = 0; i < 4; i++) {
		sprintf(ethaddr, "ethernet%d", i);
		if (!fdt_get_alias(fdt, ethaddr))
			continue;

		if (i == 0)
			strcpy(ethaddr, "ethaddr");
		else
			sprintf(ethaddr, "eth%daddr", i);

		if (env_get(ethaddr))
			continue;

		/* Non OUI / registered MAC address */
		mac_addr[0] = (i << 4) | 0x02;
		mac_addr[1] = (sid[0] >>  0) & 0xff;
		mac_addr[2] = (sid[3] >> 24) & 0xff;
		mac_addr[3] = (sid[3] >> 16) & 0xff;
		mac_addr[4] = (sid[3] >>  8) & 0xff;
		mac_addr[5] = (sid[3] >>  0) & 0xff;

		eth_env_set_enetaddr(ethaddr, mac_addr);
	}

	if (!env_get("serial#")) {
		snprintf(serial_string, sizeof(serial_string),
			"%08x%08x", sid[0], sid[3]);

		env_set("serial#", serial_string);
	}
}

int misc_init_r(void)
{
	const char *spl_dt_name;
	uint boot;

	env_set("fel_booted", NULL);
	env_set("fel_scriptaddr", NULL);
	env_set("mmc_bootdev", NULL);

	boot = sunxi_get_boot_device();
	/* determine if we are running in FEL mode */
	if (boot == BOOT_DEVICE_BOARD) {
		env_set("fel_booted", "1");
		parse_spl_header(SPL_ADDR);
	/* or if we booted from MMC, and which one */
	} else if (boot == BOOT_DEVICE_MMC1) {
		env_set("mmc_bootdev", "0");
	} else if (boot == BOOT_DEVICE_MMC2) {
		env_set("mmc_bootdev", "1");
	}

	/* Set fdtfile to match the FIT configuration chosen in SPL. */
	spl_dt_name = get_spl_dt_name();
	if (spl_dt_name) {
		char *prefix = IS_ENABLED(CONFIG_ARM64) ? "allwinner/" : "";
		char str[64];

		snprintf(str, sizeof(str), "%s%s.dtb", prefix, spl_dt_name);
		env_set("fdtfile", str);
	}

	setup_environment(gd->fdt_blob);

	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_USB_ETHER
	usb_ether_init();
#endif

	return 0;
}

static void bluetooth_dt_fixup(void *blob)
{
	/* Some devices ship with a Bluetooth controller default address.
	 * Set a valid address through the device tree.
	 */
	uchar tmp[ETH_ALEN], bdaddr[ETH_ALEN];
	unsigned int sid[4];
	int i;

	if (!CONFIG_BLUETOOTH_DT_DEVICE_FIXUP[0])
		return;

	if (eth_env_get_enetaddr("bdaddr", tmp)) {
		/* Convert between the binary formats of the corresponding stacks */
		for (i = 0; i < ETH_ALEN; ++i)
			bdaddr[i] = tmp[ETH_ALEN - i - 1];
	} else {
		if (!get_unique_sid(sid))
			return;

		bdaddr[0] = ((sid[3] >>  0) & 0xff) ^ 1;
		bdaddr[1] = (sid[3] >>  8) & 0xff;
		bdaddr[2] = (sid[3] >> 16) & 0xff;
		bdaddr[3] = (sid[3] >> 24) & 0xff;
		bdaddr[4] = (sid[0] >>  0) & 0xff;
		bdaddr[5] = 0x02;
	}

	do_fixup_by_compat(blob, CONFIG_BLUETOOTH_DT_DEVICE_FIXUP,
			   "local-bd-address", bdaddr, ETH_ALEN, 1);
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int __maybe_unused r;

	/*
	 * Call setup_environment and fdt_fixup_ethernet again
	 * in case the boot fdt has ethernet aliases the u-boot
	 * copy does not have.
	 */
	setup_environment(blob);
	fdt_fixup_ethernet(blob);

	bluetooth_dt_fixup(blob);

#ifdef CONFIG_VIDEO_DT_SIMPLEFB
	r = sunxi_simplefb_setup(blob);
	if (r)
		return r;
#endif
	return 0;
}

#ifdef CONFIG_SPL_LOAD_FIT

static void set_spl_dt_name(const char *name)
{
	struct boot_file_head *spl = get_spl_header(SPL_ENV_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	/* Promote the header version for U-Boot proper, if needed. */
	if (spl->spl_signature[3] < SPL_DT_HEADER_VERSION)
		spl->spl_signature[3] = SPL_DT_HEADER_VERSION;

	strcpy((char *)&spl->string_pool, name);
	spl->dt_name_offset = offsetof(struct boot_file_head, string_pool);
}

int board_fit_config_name_match(const char *name)
{
	const char *best_dt_name = get_spl_dt_name();
	int ret;

#ifdef CONFIG_DEFAULT_DEVICE_TREE
	if (best_dt_name == NULL)
		best_dt_name = CONFIG_DEFAULT_DEVICE_TREE;
#endif

	if (best_dt_name == NULL) {
		/* No DT name was provided, so accept the first config. */
		return 0;
	}
#ifdef CONFIG_PINE64_DT_SELECTION
	if (strstr(best_dt_name, "-pine64-plus")) {
		/* Differentiate the Pine A64 boards by their DRAM size. */
		if ((gd->ram_size == 512 * 1024 * 1024))
			best_dt_name = "sun50i-a64-pine64";
	}
#endif
#ifdef CONFIG_PINEPHONE_DT_SELECTION
	if (strstr(best_dt_name, "-pinephone")) {
		/* Differentiate the PinePhone revisions by GPIO inputs. */
		prcm_apb0_enable(PRCM_APB0_GATE_PIO);
		sunxi_gpio_set_pull(SUNXI_GPL(6), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_cfgpin(SUNXI_GPL(6), SUNXI_GPIO_INPUT);
		udelay(100);

		/* PL6 is pulled low by the modem on v1.2. */
		if (gpio_get_value(SUNXI_GPL(6)) == 0)
			best_dt_name = "sun50i-a64-pinephone-1.2";
		else
			best_dt_name = "sun50i-a64-pinephone-1.1";

		sunxi_gpio_set_cfgpin(SUNXI_GPL(6), SUNXI_GPIO_DISABLE);
		sunxi_gpio_set_pull(SUNXI_GPL(6), SUNXI_GPIO_PULL_DISABLE);
		prcm_apb0_disable(PRCM_APB0_GATE_PIO);
	}
#endif

	ret = strcmp(name, best_dt_name);

	/*
	 * If one of the FIT configurations matches the most accurate DT name,
	 * update the SPL header to provide that DT name to U-Boot proper.
	 */
	if (ret == 0)
		set_spl_dt_name(best_dt_name);

	return ret;
}
#endif
