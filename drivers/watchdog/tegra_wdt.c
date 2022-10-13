/*
 * drivers/watchdog/tegra_wdt.c
 *
 * watchdog driver for NVIDIA tegra internal watchdog
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 * Copyright (c) 2013, Toradex AG
 *
 * based on drivers/watchdog/softdog.c and drivers/watchdog/omap_wdt.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>
#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
#include <mach/irqs.h>
#endif

/* minimum and maximum watchdog trigger periods, in seconds */
#define MIN_WDT_PERIOD	5
#define MAX_WDT_PERIOD	1000

/* Assign Timer 7 to Timer 10 for WDT0 to WDT3, respectively */
#define TMR_SRC_START	7
/*
 * For spinlock lockup detection to work, the heartbeat should be 2*lockup
 * for cases where the spinlock disabled irqs.
 * Must be between MIN_WDT_PERIOD and MAX_WDT_PERIOD
 */
#define WDT_DEFAULT_TIME 60

static int heartbeat = WDT_DEFAULT_TIME;
static bool nowayout = WATCHDOG_NOWAYOUT;

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat period in seconds (default="
		__MODULE_STRING(WDT_DEFAULT_TIME));

#ifdef CONFIG_WATCHDOG_NOWAYOUT
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
#endif

enum tegra_wdt_status {
	WDT_DISABLED = 1 << 0,
	WDT_ENABLED = 1 << 1,
	WDT_KERNEL_HEARTBEAT = 1 << 2,
};

struct tegra_wdt {
	struct miscdevice	miscdev;
	struct notifier_block	notifier;
	struct resource		*res_src;
	struct resource		*res_wdt;
	struct resource		*res_int_base;
	unsigned long		users;
	void __iomem		*wdt_source;
	void __iomem		*wdt_timer;
	void __iomem		*int_base;
	int			tmrsrc;
	int			timeout;
	int			status;
	int			way_out_ok;
	struct tasklet_struct	tasklet;
	int			irq;
	int			irq_counter;
};

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)

#define TIMER_PTV		0x0
 #define TIMER_EN		(1 << 31)
 #define TIMER_PERIODIC		(1 << 30)
#define TIMER_PCR		0x4
 #define TIMER_PCR_INTR		(1 << 30)
#define WDT_EN			(1 << 5)
#define WDT_SEL_TMR1		(0 << 4)
#define WDT_SYS_RST		(1 << 2)

static void tegra_wdt_enable(struct tegra_wdt *wdt)
{
	u32 val;

	/* since the watchdog reset occurs when a second interrupt
	 * is asserted before the first is processed, program the
	 * timer period to one-half of the watchdog period */
	val = wdt->timeout * 1000000ul / 2;
	val |= (TIMER_EN | TIMER_PERIODIC);
	writel(val, wdt->wdt_timer + TIMER_PTV);

	val = WDT_EN | WDT_SEL_TMR1 | WDT_SYS_RST;
	writel(val, wdt->wdt_source);
}

static void tegra_wdt_disable(struct tegra_wdt *wdt)
{
	writel(0, wdt->wdt_source);
	writel(0, wdt->wdt_timer + TIMER_PTV);
}

static inline void tegra_wdt_ping(struct tegra_wdt *wdt)
{
	/* Reset timer */
	tegra_wdt_enable(wdt);

	/* Reenable IRQ in case an interrupt already happend */
	if (wdt->irq_counter) {
		writel(TIMER_PCR_INTR, wdt->wdt_timer + TIMER_PCR);
		wdt->irq_counter = 0;
		enable_irq(wdt->irq);
	}

	return;
}

static irqreturn_t tegra_wdt_interrupt(int irq, void *dev_id)
{
	struct tegra_wdt *wdt = dev_id;

	wdt->irq_counter++;

	if (wdt->status & WDT_KERNEL_HEARTBEAT) {
		tegra_wdt_ping(wdt);
	} else {
		/* If not in heartbeat mode, disable IRQs to avoid IRQ storm
		 * We don't acknowledge the interrupt here since the user
		 * did not reload the timer in time. Upon next interrupt
		 * the system will reset...
		 */
		tasklet_schedule(&wdt->tasklet);
		disable_irq_nosync(irq);
	}

	return IRQ_HANDLED;
}

static void tegra_wdt_do_tasklet(unsigned long data)
{
	struct tegra_wdt *wdt = (struct tegra_wdt *)data;
	dev_info(wdt->miscdev.parent, "Watchdog interrupt received, system "
		       "will reset soon if no ping arrives\n");
}

#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)

#define TIMER_PTV			0
 #define TIMER_EN			(1 << 31)
 #define TIMER_PERIODIC			(1 << 30)
#define TIMER_PCR			0x4
 #define TIMER_PCR_INTR			(1 << 30)
#define WDT_CFG				(0)
 #define WDT_CFG_PERIOD			(1 << 4)
 #define WDT_CFG_INT_EN			(1 << 12)
 #define WDT_CFG_FIQ_INT_EN		(1 << 13)
 #define WDT_CFG_SYS_RST_EN		(1 << 14)
 #define WDT_CFG_PMC2CAR_RST_EN		(1 << 15)
#define WDT_STATUS			(4)
 #define WDT_INTR_STAT			(1 << 1)
 #define WDT_STATUS_EXPIR_COUNTER	(3 << 12)
#define WDT_CMD				(8)
 #define WDT_CMD_START_COUNTER		(1 << 0)
 #define WDT_CMD_DISABLE_COUNTER	(1 << 1)
#define WDT_UNLOCK			(0xC)
 #define WDT_UNLOCK_PATTERN		(0xC45A << 0)
#define ICTLR_IEP_CLASS			0x2C
#define MAX_NR_CPU_WDT			0x4

struct tegra_wdt *tegra_wdt[MAX_NR_CPU_WDT];

static inline void tegra_wdt_ping(struct tegra_wdt *wdt)
{
	writel(WDT_CMD_START_COUNTER, wdt->wdt_source + WDT_CMD);
}

#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
static void tegra_wdt_int_priority(struct tegra_wdt *wdt)
{
	unsigned val = 0;

	if (!wdt->int_base)
		return;
	val = readl(wdt->int_base + ICTLR_IEP_CLASS);
	val &= ~(1 << (INT_WDT_CPU & 31));
	writel(val, wdt->int_base + ICTLR_IEP_CLASS);
}
#endif

static void tegra_wdt_enable(struct tegra_wdt *wdt)
{
	u32 val;

	writel(TIMER_PCR_INTR, wdt->wdt_timer + TIMER_PCR);
	val = (wdt->timeout * 1000000ul) / 4;
	val |= (TIMER_EN | TIMER_PERIODIC);
	writel(val, wdt->wdt_timer + TIMER_PTV);

	writel(WDT_CMD_START_COUNTER, wdt->wdt_source + WDT_CMD);

	/* Interrupt handler is not required for user space since
	 * a warning in a fourth of time don't make sense. Then
	 * the interrupt line is also shared, so it can't be disabled
	 * if one watchdog is about to expire... (interrupt storm)
	 * SYS_RST_EN doesnt work as there is no external reset
	 * from Tegra.
	 */
	val = wdt->tmrsrc | WDT_CFG_PERIOD | /* WDT_CFG_INT_EN |  */
		/*WDT_CFG_SYS_RST_EN |*/ WDT_CFG_PMC2CAR_RST_EN;
#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
	val |= WDT_CFG_FIQ_INT_EN;
#endif
	writel(val, wdt->wdt_source + WDT_CFG);
}

static void tegra_wdt_disable(struct tegra_wdt *wdt)
{
	writel(WDT_UNLOCK_PATTERN, wdt->wdt_source + WDT_UNLOCK);
	writel(WDT_CMD_DISABLE_COUNTER, wdt->wdt_source + WDT_CMD);

	writel(0, wdt->wdt_timer + TIMER_PTV);
}

static void tegra_wdt_interrupt_instance(struct tegra_wdt *wdt)
{
	WARN_ON_ONCE(!(wdt->status & WDT_KERNEL_HEARTBEAT));

	if (wdt->status & WDT_KERNEL_HEARTBEAT) {
		tegra_wdt_ping(wdt);
	}
}

static irqreturn_t tegra_wdt_interrupt(int irq, void *dev_id)
{
	struct tegra_wdt *wdt = NULL;
	unsigned i, status;

	for (i = 0; i < MAX_NR_CPU_WDT; i++) {
		wdt = tegra_wdt[i];
		if (wdt == NULL)
			continue;
		status = readl(wdt->wdt_source + WDT_STATUS);
		if ((wdt->status & WDT_ENABLED) &&
			(status & WDT_INTR_STAT))
		{
			tegra_wdt_interrupt_instance(wdt);
		}
	}

	return IRQ_HANDLED;
}
#endif

static int tegra_wdt_notify(struct notifier_block *this,
			    unsigned long code, void *dev)
{
	struct tegra_wdt *wdt = container_of(this, struct tegra_wdt, notifier);

	if (code == SYS_DOWN || code == SYS_HALT)
		tegra_wdt_disable(wdt);
	return NOTIFY_DONE;
}

static int tegra_wdt_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct tegra_wdt *wdt = container_of(mdev, struct tegra_wdt,
					     miscdev);

	if (test_and_set_bit(1, &wdt->users))
		return -EBUSY;

	/* Reset no way out, we need a new magic again */
	wdt->way_out_ok = 0;

	wdt->status |= WDT_ENABLED;
	wdt->timeout = heartbeat;
	tegra_wdt_enable(wdt);
	file->private_data = wdt;
	return nonseekable_open(inode, file);
}

static int tegra_wdt_release(struct inode *inode, struct file *file)
{
	struct tegra_wdt *wdt = file->private_data;

	if (wdt->status & WDT_ENABLED && !nowayout) {
		if (wdt->way_out_ok) {
			tegra_wdt_disable(wdt);
			wdt->status = WDT_DISABLED;
		} else {
			dev_info(wdt->miscdev.parent, "No Magic Close "
					"received, watchdog not disabled!\n");
		}
	} else if (nowayout) {
		dev_info(wdt->miscdev.parent, "No way out is "
				"enabled, watchdog not disabled!\n");
	}
	clear_bit(1, &wdt->users);
	return 0;
}

static long tegra_wdt_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct tegra_wdt *wdt = file->private_data;
	static DEFINE_SPINLOCK(lock);
	int new_timeout;
	int option;
	static const struct watchdog_info ident = {
		.identity = "Tegra Watchdog",
		.options = WDIOF_SETTIMEOUT,
		.firmware_version = 0,
	};

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info __user *)arg, &ident,
				    sizeof(ident));
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, (int __user *)arg);

	case WDIOC_KEEPALIVE:
		spin_lock(&lock);
		tegra_wdt_ping(wdt);
		spin_unlock(&lock);
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_timeout, (int __user *)arg))
			return -EFAULT;
		spin_lock(&lock);
		tegra_wdt_disable(wdt);
		wdt->timeout = clamp(new_timeout, MIN_WDT_PERIOD, MAX_WDT_PERIOD);
		tegra_wdt_enable(wdt);
		spin_unlock(&lock);
	case WDIOC_GETTIMEOUT:
		return put_user(wdt->timeout, (int __user *)arg);

	case WDIOC_SETOPTIONS:
		if (get_user(option, (int __user *)arg))
			return -EFAULT;
		spin_lock(&lock);
		if (option & WDIOS_DISABLECARD && !nowayout) {
			wdt->status &= ~WDT_ENABLED;
			wdt->status |= WDT_DISABLED;
			tegra_wdt_disable(wdt);
		} else if (option & WDIOS_ENABLECARD) {
			tegra_wdt_enable(wdt);
			wdt->status |= WDT_ENABLED;
			wdt->status &= ~WDT_DISABLED;
		} else {
			spin_unlock(&lock);
			return -EINVAL;
		}
		spin_unlock(&lock);
		return 0;
	}
	return -ENOTTY;
}

static ssize_t tegra_wdt_write(struct file *file, const char __user *data,
			       size_t len, loff_t *ppos)
{
	struct tegra_wdt *wdt = file->private_data;

	/* check if way-out char was written as last data */
	char c;
	if(len) {
		tegra_wdt_ping(wdt);

		if(get_user(c, data + len - 1))
			return -EFAULT;
		else
			wdt->way_out_ok = (('V' == c) ? 1 : 0);
	}

	return len;
}

static const struct file_operations tegra_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= tegra_wdt_write,
	.unlocked_ioctl	= tegra_wdt_ioctl,
	.open		= tegra_wdt_open,
	.release	= tegra_wdt_release,
};

static int tegra_wdt_probe(struct platform_device *pdev)
{
	struct resource *res_src, *res_wdt, *res_irq;
	struct resource	*res_int_base = NULL;
	struct tegra_wdt *wdt;
	int ret = 0;
#ifdef CONFIG_TEGRA_WATCHDOG_ENABLE_HEARTBEAT
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	int is_heartbeat_wdt = 1;
#else
	int is_heartbeat_wdt = !(pdev->id);
#endif
#endif

	if (pdev->id < -1 && pdev->id > 3) {
		dev_err(&pdev->dev, "only IDs 3:0 supported\n");
		return -ENODEV;
	}

	res_src = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res_wdt = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!res_src || !res_wdt || (!pdev->id && !res_irq)) {
		dev_err(&pdev->dev, "incorrect resources\n");
		return -ENOENT;
	}

#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
	res_int_base = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!pdev->id && !res_int_base) {
		dev_err(&pdev->dev, "FIQ_DBG: INT base not defined\n");
		return -ENOENT;
	}
#endif

	if (pdev->id == -1 && !res_irq) {
		dev_err(&pdev->dev, "incorrect irq\n");
		return -ENOENT;
	}

	wdt = kzalloc(sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	wdt->miscdev.parent = &pdev->dev;
	if (pdev->id == -1) {
		wdt->miscdev.minor = WATCHDOG_MINOR;
		wdt->miscdev.name = "watchdog";
	} else {
		wdt->miscdev.minor = MISC_DYNAMIC_MINOR;
		if (pdev->id == 0)
			wdt->miscdev.name = "watchdog0";
		else if (pdev->id == 1)
			wdt->miscdev.name = "watchdog1";
		else if (pdev->id == 2)
			wdt->miscdev.name = "watchdog2";
		else if (pdev->id == 3)
			wdt->miscdev.name = "watchdog3";
	}
	wdt->miscdev.fops = &tegra_wdt_fops;

	wdt->notifier.notifier_call = tegra_wdt_notify;

	res_src = request_mem_region(res_src->start, resource_size(res_src),
				     pdev->name);
	res_wdt = request_mem_region(res_wdt->start, resource_size(res_wdt),
				     pdev->name);

	if (!res_src || !res_wdt) {
		dev_err(&pdev->dev, "unable to request memory resources\n");
		ret = -EBUSY;
		goto fail;
	}

	wdt->wdt_source = ioremap(res_src->start, resource_size(res_src));
	wdt->wdt_timer = ioremap(res_wdt->start, resource_size(res_wdt));
	/* tmrsrc will be used to set WDT_CFG */
	wdt->tmrsrc = (TMR_SRC_START + pdev->id) % 10;
	if (!wdt->wdt_source || !wdt->wdt_timer) {
		dev_err(&pdev->dev, "unable to map registers\n");
		ret = -ENOMEM;
		goto fail;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	/* Watchdog of Tegra 3 are not at the reset controller regs */
	if (readl(wdt->wdt_source) & BIT(12))
		dev_info(&pdev->dev, "last reset due to watchdog timeout\n");
#endif

	tegra_wdt_disable(wdt);
	writel(TIMER_PCR_INTR, wdt->wdt_timer + TIMER_PCR);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	writel(WDT_CMD_START_COUNTER, wdt->wdt_source + WDT_CMD);
#endif

	if (res_irq != NULL) {
#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
		/* FIQ debugger enables FIQ priority for INT_WDT_CPU.
		 * But that will disable IRQ on WDT expiration.
		 * Reset the priority back to IRQ on INT_WDT_CPU so
		 * that tegra_wdt_interrupt gets its chance to restart the
		 * counter before expiration.
		 */
		res_int_base = request_mem_region(res_int_base->start,
						  resource_size(res_int_base),
						  pdev->name);
		if (!res_int_base)
			goto fail;
		wdt->int_base = ioremap(res_int_base->start,
					resource_size(res_int_base));
		if (!wdt->int_base)
			goto fail;
		tegra_wdt_int_priority(wdt);
#endif
		wdt->irq = -1;
		ret = request_irq(res_irq->start, tegra_wdt_interrupt,
				  IRQF_DISABLED, dev_name(&pdev->dev), wdt);
		if (ret) {
			dev_err(&pdev->dev, "unable to configure IRQ\n");
			goto fail;
		}
		wdt->irq = res_irq->start;
	}

	wdt->res_src = res_src;
	wdt->res_wdt = res_wdt;
	wdt->res_int_base = res_int_base;
	wdt->status = WDT_DISABLED;

	ret = register_reboot_notifier(&wdt->notifier);
	if (ret) {
		dev_err(&pdev->dev, "cannot register reboot notifier\n");
		goto fail;
	}

	ret = misc_register(&wdt->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register misc device\n");
		unregister_reboot_notifier(&wdt->notifier);
		goto fail;
	}

	platform_set_drvdata(pdev, wdt);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	tasklet_init(&wdt->tasklet, tegra_wdt_do_tasklet, (unsigned long)wdt);
#endif

#ifdef CONFIG_TEGRA_WATCHDOG_ENABLE_HEARTBEAT
	/* Enable first watchdog during probe */
	if (is_heartbeat_wdt) {
		wdt->status = WDT_ENABLED | WDT_KERNEL_HEARTBEAT;
		wdt->timeout = heartbeat;
		set_bit(1, &wdt->users);
		tegra_wdt_enable(wdt);
		pr_info("WDT kernel heartbeat enabled on probe\n");
	}
#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	tegra_wdt[pdev->id] = wdt;
#endif

	dev_info(&pdev->dev, "%s done\n", __func__);
	return 0;
fail:
	if (wdt->irq != -1)
		free_irq(wdt->irq, wdt);
	if (wdt->wdt_source)
		iounmap(wdt->wdt_source);
	if (wdt->wdt_timer)
		iounmap(wdt->wdt_timer);
	if (wdt->int_base)
		iounmap(wdt->int_base);
	if (res_src)
		release_mem_region(res_src->start, resource_size(res_src));
	if (res_wdt)
		release_mem_region(res_wdt->start, resource_size(res_wdt));
	if (res_int_base)
		release_mem_region(res_int_base->start,
					resource_size(res_int_base));
	kfree(wdt);
	return ret;
}

static int tegra_wdt_remove(struct platform_device *pdev)
{
	struct tegra_wdt *wdt = platform_get_drvdata(pdev);

	tegra_wdt_disable(wdt);

	unregister_reboot_notifier(&wdt->notifier);
	misc_deregister(&wdt->miscdev);
	if (wdt->irq != -1)
		free_irq(wdt->irq, wdt);
	iounmap(wdt->wdt_source);
	iounmap(wdt->wdt_timer);
	if (wdt->int_base)
		iounmap(wdt->int_base);
	release_mem_region(wdt->res_src->start, resource_size(wdt->res_src));
	release_mem_region(wdt->res_wdt->start, resource_size(wdt->res_wdt));
	if (wdt->res_int_base)
		release_mem_region(wdt->res_int_base->start,
					resource_size(wdt->res_int_base));
	kfree(wdt);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_wdt *wdt = platform_get_drvdata(pdev);

	tegra_wdt_disable(wdt);
	return 0;
}

static int tegra_wdt_resume(struct platform_device *pdev)
{
	struct tegra_wdt *wdt = platform_get_drvdata(pdev);

	if (wdt->status & WDT_ENABLED)
		tegra_wdt_enable(wdt);

	return 0;
}
#endif

static struct platform_driver tegra_wdt_driver = {
	.probe		= tegra_wdt_probe,
	.remove		= __devexit_p(tegra_wdt_remove),
#ifdef CONFIG_PM
	.suspend	= tegra_wdt_suspend,
	.resume		= tegra_wdt_resume,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tegra_wdt",
	},
};

static int __init tegra_wdt_init(void)
{
	return platform_driver_register(&tegra_wdt_driver);
}

static void __exit tegra_wdt_exit(void)
{
	platform_driver_unregister(&tegra_wdt_driver);
}

module_init(tegra_wdt_init);
module_exit(tegra_wdt_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Tegra Watchdog Driver");

MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:tegra_wdt");
