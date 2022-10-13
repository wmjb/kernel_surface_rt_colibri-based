/*
 * arch/arm/mach-tegra/board-colibri_t30.c
 *
 * Copyright (c) 2012-2014 Toradex, Inc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2. See the file COPYING for more details.
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <linux/can/platform/mcp251x.h>
#include <linux/can/platform/sja1000.h>
#include <linux/clk.h>
#include <linux/colibri_usb.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/input.h>
#include <linux/input/fusion_F0710A.h>
#include <linux/io.h>
#include <linux/leds_pwm.h>
#include <linux/lm95245.h>
#include <linux/mfd/stmpe.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/spi-tegra.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>

#include <mach/io_dpd.h>
#include <mach/sdhci.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/tegra_fiq_debugger.h>
#include <mach/thermal.h>
#include <mach/usb_phy.h>
#include <mach/w1.h>

#include <media/soc_camera.h>
#include <media/tegra_v4l2_camera.h>

#include "board-colibri_t30.h"
#include "board.h"
#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"

//from former drivers/mtd/maps/tegra_nor.h
#define TEGRA_GMI_PHYS			0x70009000
#define TEGRA_GMI_BASE			IO_TO_VIRT(TEGRA_GMI_PHYS)
#define TEGRA_SNOR_CONFIG_REG		(TEGRA_GMI_BASE + 0x00)

//from drivers/mtd/maps/tegra_nor.c
#define __BITMASK0(len)			(BIT(len) - 1)
#define REG_FIELD(val, start, len)	(((val) & __BITMASK0(len)) << (start))

#define TEGRA_SNOR_CONFIG_GO		BIT(31)
#define TEGRA_SNOR_CONFIG_SNOR_CS(val)	REG_FIELD((val), 4, 3)

/* Audio */


/* Camera */

#ifdef CONFIG_TEGRA_CAMERA
static struct platform_device tegra_camera = {
	.name	= "tegra_camera",
	.id	= -1,
};


static void tegra_camera_disable(struct nvhost_device *ndev)
{
}

static int tegra_camera_enable(struct nvhost_device *ndev)
{
	return 0;
}
#endif /* CONFIG_TEGRA_CAMERA */


/* Clocks */
static struct tegra_clk_init_table colibri_t30_clk_init_table[] __initdata = {
	/* name		parent		rate		enabled */



	{"apbif",	"clk_m",	12000000,	false},
	{"audio0",	"i2s0_sync",	0,		false},
	{"audio1",	"i2s1_sync",	0,		false},
	{"audio2",	"i2s2_sync",	0,		false},
	{"audio3",	"i2s3_sync",	0,		false},
	{"audio4",	"i2s4_sync",	0,		false},
	{"blink",	"clk_32k",	32768,		true},
	
	{"clk_out_2",	"extern2",	24000000,	false},
	{"d_audio",	"clk_m",	12000000,	false},
	{"dam0",	"clk_m",	12000000,	false},
	{"dam1",	"clk_m",	12000000,	false},
	{"dam2",	"clk_m",	12000000,	false},
	{"extern2",	"clk_m",	24000000,	false},
	{"hda",		"pll_p",	108000000,	false},
	{"hda2codec_2x","pll_p",	48000000,	false},
	{"i2c1",	"pll_p",	3200000,	false},
	{"i2c2",	"pll_p",	3200000,	false},
	{"i2c3",	"pll_p",	3200000,	false},
	{"i2c4",	"pll_p",	3200000,	false},
	{"i2c5",	"pll_p",	3200000,	false},
	{"i2s0",	"pll_a_out0",	0,		false},
	{"i2s1",	"pll_a_out0",	0,		false},
	{"i2s2",	"pll_a_out0",	0,		false},
	{"i2s3",	"pll_a_out0",	0,		false},
	{"i2s4",	"pll_a_out0",	0,		false},
	{"nor",		"pll_p",	86500000,	true},
	{"pll_a",	NULL,		564480000,	true},
	{"pll_m",	NULL,		0,		false},
	{"pwm",		"pll_p",	5100000,	false},
	{"spdif_out",	"pll_a_out0",	0,		false},
	{"vi",		"pll_p",	0,		false},
	{NULL,		NULL,		0,		0},

};



static struct tegra_i2c_platform_data colibri_t30_i2c1_platform_data = {
	.adapter_nr	= 0,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {400000, 0},
	.bus_count	= 1,
	.scl_gpio	= {I2C_SCL, 0},
	.sda_gpio	= {I2C_SDA, 0},
	.slave_addr	= 0x00FC,
};

/* GEN2_I2C: unused */

/* DDC_CLOCK/DATA on X3 pin 15/16 (e.g. display EDID) */
static struct tegra_i2c_platform_data colibri_t30_i2c4_platform_data = {
	.adapter_nr	= 3,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {10000, 10000},
	.bus_count	= 1,
	.scl_gpio	= {DDC_SCL, 0},
	.sda_gpio	= {DDC_SDA, 0},
	.slave_addr	= 0x00FC,
};


static struct tegra_i2c_platform_data colibri_t30_i2c5_platform_data = {
	.adapter_nr	= 4,
	.arb_recovery	= arb_lost_recovery,
	.bus_clk_rate	= {400000, 0},
	.bus_count	= 1,
	.scl_gpio	= {PWR_I2C_SCL, 0},
	.sda_gpio	= {PWR_I2C_SDA, 0},
};

static void __init colibri_t30_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &colibri_t30_i2c1_platform_data;
	tegra_i2c_device4.dev.platform_data = &colibri_t30_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &colibri_t30_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device5);

}

/* MMC/SD */

#ifndef COLIBRI_T30_SDMMC4B
static struct tegra_sdhci_platform_data colibri_t30_emmc_platform_data = {
	.cd_gpio	= -1,
	.ddr_clk_limit	= 41000000,
	.is_8bit	= 1,
	.mmc_data = {
		.built_in = 1,
		.ocr_mask = MMC_OCR_1V8_MASK,

	},
	.power_gpio	= -1,
	.tap_delay	= 0x0f,
	.wp_gpio	= -1,
};
#endif /* COLIBRI_T30_SDMMC4B */

static struct tegra_sdhci_platform_data colibri_t30_sdcard_platform_data = {
	.cd_gpio	= MMC_CD,
	.ddr_clk_limit	= 41000000,
	.is_8bit	= 0,
	.power_gpio	= -1,
	.tap_delay	= 0x0f,
	.wp_gpio	= -1,
	.no_1v8		= 1,
};

static void __init colibri_t30_sdhci_init(void)
{
	/* register eMMC first */
	tegra_sdhci_device4.dev.platform_data =
#ifdef COLIBRI_T30_SDMMC4B
			&colibri_t30_sdcard_platform_data;
#else
			&colibri_t30_emmc_platform_data;
#endif
	platform_device_register(&tegra_sdhci_device4);

#ifndef COLIBRI_T30_SDMMC4B
	tegra_sdhci_device2.dev.platform_data =
			&colibri_t30_sdcard_platform_data;
	platform_device_register(&tegra_sdhci_device2);
#endif
}

/* PWM LEDs */
static struct led_pwm tegra_leds_pwm[] = {
	{
		.name		= "PWM<B>",
		.pwm_id		= 1,
		.max_brightness	= 255,
		.pwm_period_ns	= 19600,
	},
	{
		.name		= "PWM<C>",
		.pwm_id		= 2,
		.max_brightness	= 255,
		.pwm_period_ns	= 19600,
	},
	{
		.name		= "PWM<D>",
		.pwm_id		= 3,
		.max_brightness	= 255,
		.pwm_period_ns	= 19600,
	},
};

static struct led_pwm_platform_data tegra_leds_pwm_data = {
	.num_leds	= ARRAY_SIZE(tegra_leds_pwm),
	.leds		= tegra_leds_pwm,
};

static struct platform_device tegra_led_pwm_device = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev = {
		.platform_data = &tegra_leds_pwm_data,
	},
};

/* RTC */

#ifdef CONFIG_RTC_DRV_TEGRA
static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start	= TEGRA_RTC_BASE,
		.end	= TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_RTC,
		.end	= INT_RTC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name		= "tegra_rtc",
	.id		= -1,
	.resource	= tegra_rtc_resources,
	.num_resources	= ARRAY_SIZE(tegra_rtc_resources),
};
#endif /* CONFIG_RTC_DRV_TEGRA */

/* SPI */
/*
#if defined(CONFIG_SPI_TEGRA) && defined(CONFIG_SPI_SPIDEV)
static struct tegra_spi_device_controller_data spidev_controller_data = {
	.cs_hold_clk_count	= 1,
	.cs_setup_clk_count	= 1,
	.is_hw_based_cs		= 1,
};

static struct spi_board_info tegra_spi_devices[] __initdata = {
	{
		.bus_num		= 0,		
#if !defined(CONFIG_CAN_MCP251X) && !defined(CONFIG_CAN_MCP251X_MODULE)
		.chip_select		= 0,
#else 
		.chip_select		= 1,
#endif
		.controller_data	= &spidev_controller_data,
		.irq			= 0,
		.max_speed_hz		= 50000000,
		.modalias		= "spidev",
		.mode			= SPI_MODE_0,
		.platform_data		= NULL,
	},
};

static void __init colibri_t30_register_spidev(void)
{
	spi_register_board_info(tegra_spi_devices,
				ARRAY_SIZE(tegra_spi_devices));
}

#else 
#define colibri_t30_register_spidev() do {} while (0)
#endif 
*/
static struct platform_device *colibri_t30_spi_devices[] __initdata = {
	&tegra_spi_device1,
};

static struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else /* !CONFIG_TEGRA_PLLM_RESTRICTED */
	[1] = {.name = "clk_m"},
#endif /* !CONFIG_TEGRA_PLLM_RESTRICTED */
};

static struct tegra_spi_platform_data colibri_t30_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= 16 * 1024,
	.is_clkon_always	= false,
	.max_rate		= 408000000,
};

static void __init colibri_t30_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	colibri_t30_spi_pdata.parent_clk_list = spi_parent_clk;
	colibri_t30_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device1.dev.platform_data = &colibri_t30_spi_pdata;
	platform_add_devices(colibri_t30_spi_devices,
				ARRAY_SIZE(colibri_t30_spi_devices));
}


/* UART */

static struct platform_device *colibri_t30_uart_devices[] __initdata = {
	&tegra_uarta_device, /* Colibri UART_A (formerly FFUART) */
	&tegra_uartd_device, /* Colibri UART_B (formerly BTUART) */
	&tegra_uartb_device, /* Colibri UART_C (formerly STDUART) */
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data colibri_t30_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
	}

	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		colibri_t30_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		colibri_t30_uart_devices[2] = &debug_uartb_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		colibri_t30_uart_devices[1] = &debug_uartd_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA",
			debug_port_id);
		colibri_t30_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
}

static void __init colibri_t30_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	colibri_t30_uart_pdata.parent_clk_list = uart_parent_clk;
	colibri_t30_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &colibri_t30_uart_pdata;
	tegra_uartb_device.dev.platform_data = &colibri_t30_uart_pdata;
	tegra_uartd_device.dev.platform_data = &colibri_t30_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(colibri_t30_uart_devices,
				ARRAY_SIZE(colibri_t30_uart_devices));
}

/* USB */

//TODO: overcurrent

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_DEVICE,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= true,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 8,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.dev = {
		.charging_supported		= false,
		.remote_wakeup_supported	= false,
		.vbus_gpio			= -1,
		.vbus_pmu_irq			= 0,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= true,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 15,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.host = {
		.hot_plug			= true,
		.power_off_on_suspend		= false,
		.remote_wakeup_supported	= true,
		.vbus_gpio			= -1,
		.vbus_reg			= NULL,
	},
};

static void ehci2_utmi_platform_post_phy_on(void)
{
	/* enable VBUS */
	gpio_set_value(LAN_V_BUS, 1);

	/* reset */
	gpio_set_value(LAN_RESET, 0);

	udelay(5);

	/* unreset */
	gpio_set_value(LAN_RESET, 1);
}

static void ehci2_utmi_platform_pre_phy_off(void)
{
	/* disable VBUS */
	gpio_set_value(LAN_V_BUS, 0);
}

static struct tegra_usb_phy_platform_ops ehci2_utmi_plat_ops = {
	.post_phy_on = ehci2_utmi_platform_post_phy_on,
	.pre_phy_off = ehci2_utmi_platform_pre_phy_off,
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.ops		= &ehci2_utmi_plat_ops,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= false,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 15,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.host = {
		.hot_plug			= false,
		.power_off_on_suspend		= true,
		.remote_wakeup_supported	= true,
		.vbus_gpio			= -1,
		.vbus_reg			= NULL,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.has_hostpc	= true,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.phy_intf	= TEGRA_USB_PHY_INTF_UTMI,
	.port_otg	= false,
	.u_cfg.utmi = {
		.elastic_limit		= 16,
		.hssync_start_delay	= 0,
		.idle_wait_delay	= 17,
		.term_range_adj		= 6,
		.xcvr_lsfslew		= 2,
		.xcvr_lsrslew		= 2,
		.xcvr_setup		= 8,
		.xcvr_setup_offset	= 0,
		.xcvr_use_fuses		= 1,
	},
	.u_data.host = {
		.hot_plug			= true,
		.power_off_on_suspend		= false,
		.remote_wakeup_supported	= true,
		.vbus_gpio			= USBH_PEN,
		.vbus_gpio_inverted		= 1,
		.vbus_reg			= NULL,
	},
};

#ifndef CONFIG_USB_TEGRA_OTG
static struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name,
				     tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
					    tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask = tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_usb_platform_data),
				GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci1_utmi_pdata,
	       sizeof(struct tegra_usb_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host controller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static struct colibri_otg_platform_data colibri_otg_pdata = {
	.cable_detect_gpio	= USBC_DET,
	.host_register		= &tegra_usb_otg_host_register,
	.host_unregister	= &tegra_usb_otg_host_unregister,
};
#else /* !CONFIG_USB_TEGRA_OTG */
static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};
#endif /* !CONFIG_USB_TEGRA_OTG */

#ifndef CONFIG_USB_TEGRA_OTG
struct platform_device colibri_otg_device = {
	.name	= "colibri-otg",
	.id	= -1,
	.dev = {
		.platform_data = &colibri_otg_pdata,
	},
};
#endif /* !CONFIG_USB_TEGRA_OTG */

static void colibri_t30_usb_init(void)
{
	//gpio_request(LAN_V_BUS, "LAN_V_BUS");
	//gpio_direction_output(LAN_V_BUS, 0);
	//gpio_export(LAN_V_BUS, false);

	//gpio_request(LAN_RESET, "LAN_RESET");
	//gpio_direction_output(LAN_RESET, 0);
	//gpio_export(LAN_RESET, false);

	/* OTG should be the first to be registered
	   EHCI instance 0: USB1_DP/N -> USBC_P/N */
#ifndef CONFIG_USB_TEGRA_OTG
	platform_device_register(&colibri_otg_device);
#else /* !CONFIG_USB_TEGRA_OTG */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
#endif /* !CONFIG_USB_TEGRA_OTG */

	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);

	/* EHCI instance 1: USB2_DP/N -> AX88772B */
	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
	platform_device_register(&tegra_ehci2_device);

	/* EHCI instance 2: USB3_DP/N -> USBH_P/N */
	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);
}

/* W1, aka OWR, aka OneWire */

#ifdef CONFIG_W1_MASTER_TEGRA
struct tegra_w1_timings colibri_t30_w1_timings = {
		.tsu		= 1,
		.trelease	= 0xf,
		.trdv		= 0xf,
		.tlow0		= 0x3c,
		.tlow1		= 1,
		.tslot		= 0x77,

		.tpdl		= 0x78,
		.tpdh		= 0x1e,
		.trstl		= 0x1df,
		.trsth		= 0x1df,
		.rdsclk		= 0x7,
		.psclk		= 0x50,
};

struct tegra_w1_platform_data colibri_t30_w1_platform_data = {
	.clk_id		= "tegra_w1",
	.timings	= &colibri_t30_w1_timings,
};
#endif /* CONFIG_W1_MASTER_TEGRA */

static struct platform_device *colibri_t30_devices[] __initdata = {
	&tegra_pmu_device,
#if defined(CONFIG_RTC_DRV_TEGRA)
	&tegra_rtc_device,
#endif
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#ifdef CONFIG_TEGRA_CAMERA
	&tegra_camera,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device2,
	&tegra_pcm_device,

	&tegra_hda_device,
	&tegra_cec_device,

	&tegra_led_pwm_device,
	&tegra_pwfm1_device,
	&tegra_pwfm2_device,
	&tegra_pwfm3_device,
#ifdef CONFIG_W1_MASTER_TEGRA
	&tegra_w1_device,
#endif
};

static void __init colibri_t30_init(void)
{
	
	tegra_clk_init_from_table(colibri_t30_clk_init_table);
	colibri_t30_pinmux_init();

	colibri_t30_i2c_init();
	colibri_t30_spi_init();
	colibri_t30_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	colibri_t30_edp_init();
#endif
	colibri_t30_uart_init();
#ifdef CONFIG_W1_MASTER_TEGRA
	tegra_w1_device.dev.platform_data = &colibri_t30_w1_platform_data;
#endif
	platform_add_devices(colibri_t30_devices, ARRAY_SIZE(colibri_t30_devices));
	tegra_ram_console_debug_init();

	//tegra_io_dpd_init();

	colibri_t30_sdhci_init();
	colibri_t30_regulator_init();
	colibri_t30_suspend_init();
	colibri_t30_panel_init();
//	colibri_t30_sensors_init();
	colibri_t30_emc_init();
//	colibri_t30_register_spidev();



	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);

}

static void __init colibri_t30_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* Support 1920X1080 32bpp,double buffered on HDMI*/
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

static const char *colibri_t30_dt_board_compat[] = {
	"toradex,colibri_t30",
	NULL
};

#ifdef CONFIG_ANDROID
MACHINE_START(COLIBRI_T30, "cardhu")
#else
MACHINE_START(COLIBRI_T30, "Toradex Colibri T30")
#endif
	.boot_params	= 0x80000100,
	.dt_compat	= colibri_t30_dt_board_compat,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.init_machine	= colibri_t30_init,
	.map_io		= tegra_map_common_io,
	.reserve	= colibri_t30_reserve,
	.timer		= &tegra_timer,
MACHINE_END
