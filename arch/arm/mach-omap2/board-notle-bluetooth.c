/*
 * Bluetooth Broadcomm  and low power control via GPIO
 *
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <asm/mach-types.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/wakelock.h>
#include <plat/omap-serial.h>
#include <plat/serial.h>

#include "board-notle.h"

#define BT_REG_GPIO GPIO_WL_BT_REG_ON
#define BT_RESET_GPIO notle_get_gpio(GPIO_BT_RST_N_INDEX)

#define BT_WAKE_GPIO GPIO_BCM_BT_WAKE
#define BT_HOST_WAKE_GPIO notle_get_gpio(GPIO_BCM_BT_HOST_WAKE_INDEX)

static void set_host_wake_locked(int host_wake);
static void set_bcm_wake_locked(int bcm_wake);

/* rf kill interface to turn on or off BT portion of chip */
static struct rfkill *bt_rfkill;
/* Reference to the 32Khz clock regulator */
static struct regulator *clk32kg_reg;
/* Used by rfkill to switch on and off */
static bool bt_enabled;
/* Cached value of host wake GPIO line from bcm */
static bool host_wake_uart_enabled;
/* Cached value of bcm wake GPIO line to bcm */
static bool bcm_wake_uart_enabled;

static const char *WAKE_LOCK_NAME_BCM_WAKE = "bt_bcm_wake";
static const char *WAKE_LOCK_NAME_HOST_WAKE = "bt_host_wake";

/* Time after idle uart before releasing wakelock */
static const int UART_TIMEOUT_SEC = 1;
/* Wakelock release timeout after deassertion HOST_WAKE */
static const long HOST_WAKE_TIMEOUT = HZ/2;

struct bcm_bt_lpm {
	int bcm_wake;
	int host_wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	/* Lock controlled by host */
	struct wake_lock bcm_wake_lock;
	/* Lock controlled by bcm */
	struct wake_lock host_wake_lock;

	struct platform_device *pdev;
} bt_lpm;

/* rfkill_ops callback. Turn transmitter off when blocked is true */
static int bcm4330_bt_rfkill_set_power(void *data, bool blocked)
{
	if (!blocked) {
		/* Turn on bluetooth */
		if (clk32kg_reg && !bt_enabled)
			regulator_enable(clk32kg_reg);
		gpio_set_value(BT_REG_GPIO, 1);
		gpio_set_value(BT_RESET_GPIO, 1);

	} else {
		/* Turn off bluetooth chip */
		gpio_set_value(BT_RESET_GPIO, 0);
		/* Chip won't toggle host_wake after reset.  Make sure
		 we don't hold the wake_lock until chip wakes up again. */
		set_host_wake_locked(0);
		gpio_set_value(BT_REG_GPIO, 0);
		if (clk32kg_reg && bt_enabled)
			regulator_disable(clk32kg_reg);
	}

	bt_enabled = !blocked;

	return 0;
}

static const struct rfkill_ops bcm4330_bt_rfkill_ops = {
	.set_block = bcm4330_bt_rfkill_set_power,
};

/* Drives the GPIO from the host to the bcm chip
 * to request that BT chip remain active or not.
 * Called by the periodic timer to keep the BT chip
 * enabled as traffic is occurring on the UART.
 */
static void set_bcm_wake_locked(int bcm_wake)
{
	if (bt_lpm.bcm_wake == bcm_wake)
		return;
	bt_lpm.bcm_wake = bcm_wake;

	if (bcm_wake) {
		wake_lock(&bt_lpm.bcm_wake_lock);
		if (!bcm_wake_uart_enabled)
			omap_serial_ext_uart_enable(1);
	}

	gpio_set_value(BT_WAKE_GPIO, bcm_wake);

	if (!bcm_wake && bcm_wake_uart_enabled)
		omap_serial_ext_uart_disable(1);
	bcm_wake_uart_enabled = bcm_wake;

	if (!bcm_wake)
		wake_unlock(&bt_lpm.bcm_wake_lock);
}

/*
 * Called after a the bt_lpm timer pops indicating there has
 * been no traffic on the UART for the specified period of time.
 * De-assert the GPIO wake line to the bcm indicating that
 * the bcm is free to suspend if necessary.
 */
static enum hrtimer_restart enter_lpm(struct hrtimer *timer) {
	unsigned long flags;
	struct bcm_bt_lpm *p =
		container_of(timer, struct bcm_bt_lpm, enter_lpm_timer);

	spin_lock_irqsave(&p->uport->lock, flags);
	set_bcm_wake_locked(0);
	spin_unlock_irqrestore(&p->uport->lock, flags);

	return HRTIMER_NORESTART;
}

/*
 * This gets called every time a uart sequence starts up
 * from the uart module which already has interrupts disabled.
 * Cancel the existing timer, if any, and start a new timer.
 */
void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport) {
	int rc;

	/* Save away the uart port for hrtimer callback */
	bt_lpm.uport = uport;

	rc = hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);
	if (rc == -1) {
		dev_warn(&bt_lpm.pdev->dev, "%s timer executing unable to cancel\n",
		         __func__);
	}

	set_bcm_wake_locked(1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
	              HRTIMER_MODE_REL);
}
EXPORT_SYMBOL(bcm_bt_lpm_exit_lpm_locked);

/*
 * Enable UART and acquire wakelock to service communication
 * with BT chip.  Release wakelock when done.
 */
static void set_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;
	bt_lpm.host_wake = host_wake;

	if (host_wake) {
		wake_lock(&bt_lpm.host_wake_lock);
		if (!host_wake_uart_enabled)
			omap_serial_ext_uart_enable(1);
	} else {
		if (host_wake_uart_enabled)
			omap_serial_ext_uart_disable(1);
	}
	host_wake_uart_enabled = host_wake;

	/* Take a timed wakelock, so that upper layers can take it. */
	if (!host_wake)
		wake_lock_timeout(&bt_lpm.host_wake_lock, HOST_WAKE_TIMEOUT);
}

/*
 * Interrupt service routine for when the BT chip
 * toggles GPIO indicating requests or releases need
 * for servicing. */
static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;
	unsigned long flags;

	host_wake = gpio_get_value(BT_HOST_WAKE_GPIO);
	/* Invert the interrupt type to catch the next transition */
	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (!bt_lpm.uport) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	set_host_wake_locked(host_wake);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return IRQ_HANDLED;
}

static int bcm_bt_lpm_init(struct platform_device *pdev)
{
	int rc = 0;
	int irq;
	irq = gpio_to_irq(BT_HOST_WAKE_GPIO);

	rc = gpio_request(BT_WAKE_GPIO, "bcm4330_wake_gpio");
	if (unlikely(rc)) {
		return rc;
	}

	rc = gpio_request(BT_HOST_WAKE_GPIO, "bcm4330_host_wake_gpio");
	if (unlikely(rc)) {
		gpio_free(BT_WAKE_GPIO);
		return rc;
	}

	gpio_direction_output(BT_WAKE_GPIO, 0);
	gpio_direction_input(BT_HOST_WAKE_GPIO);

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(UART_TIMEOUT_SEC, 0);
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	/* Set these to initial values forcing evaluation */
	bt_lpm.host_wake = -1;
	bt_lpm.bcm_wake = -1;

	rc = request_irq(irq, host_wake_isr, IRQF_TRIGGER_HIGH,
	                 "bt_host_wake", NULL);
	if (rc) {
		gpio_free(BT_WAKE_GPIO);
		gpio_free(BT_HOST_WAKE_GPIO);
		return rc;
	}

	rc = irq_set_irq_wake(irq, 1);
	if (rc) {
		free_irq(irq, NULL);
		gpio_free(BT_WAKE_GPIO);
		gpio_free(BT_HOST_WAKE_GPIO);
		return rc;
	}

	wake_lock_init(&bt_lpm.bcm_wake_lock, WAKE_LOCK_SUSPEND,
	               WAKE_LOCK_NAME_BCM_WAKE);
	wake_lock_init(&bt_lpm.host_wake_lock, WAKE_LOCK_SUSPEND,
	               WAKE_LOCK_NAME_HOST_WAKE);

	bt_lpm.pdev = pdev;
	return rc;
}

static int bcm4330_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;

	rc = gpio_request(BT_RESET_GPIO, "bcm4330_nreset_gpip");
	if (unlikely(rc)) {
		return rc;
	}

	rc = gpio_request(BT_REG_GPIO, "bcm4330_nshutdown_gpio");
	if (unlikely(rc)) {
		goto exit0;
	}
	gpio_direction_output(BT_REG_GPIO, 1);
	gpio_direction_output(BT_RESET_GPIO, 1);

	clk32kg_reg = regulator_get(0, "clk32kg");
	if (IS_ERR(clk32kg_reg)) {
		pr_err("clk32kg_reg not found! err: %d\n", (int)clk32kg_reg);
		clk32kg_reg = NULL;
	}

	bt_rfkill = rfkill_alloc("bcm4330 Bluetooth", &pdev->dev,
	                         RFKILL_TYPE_BLUETOOTH, &bcm4330_bt_rfkill_ops,
	                         NULL);

	if (unlikely(!bt_rfkill)) {
		rc = -ENOMEM;
		goto exit1;
	}

	rc = rfkill_register(bt_rfkill);
	if (unlikely(rc)) {
		goto exit2;
	}

	/* TODO Check if these can be removed or moved */
	rfkill_set_states(bt_rfkill, true, false);
	bcm4330_bt_rfkill_set_power(NULL, true);

	rc = bcm_bt_lpm_init(pdev);
	if (unlikely(rc)) {
		goto exit3;
	}

	return rc;

exit3:;
      rfkill_unregister(bt_rfkill);
exit2:;
      rfkill_destroy(bt_rfkill);
exit1:;
      regulator_put(clk32kg_reg);
      gpio_free(BT_REG_GPIO);
exit0:;
      gpio_free(BT_RESET_GPIO);
      return rc;
}

static int bcm4330_bluetooth_remove(struct platform_device *pdev)
{
	int irq;

	irq = gpio_to_irq(BT_HOST_WAKE_GPIO);
	free_irq(irq, NULL);

	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	regulator_put(clk32kg_reg);
	gpio_free(BT_REG_GPIO);
	gpio_free(BT_RESET_GPIO);
	gpio_free(BT_WAKE_GPIO);
	gpio_free(BT_HOST_WAKE_GPIO);

	wake_lock_destroy(&bt_lpm.bcm_wake_lock);
	wake_lock_destroy(&bt_lpm.host_wake_lock);
	return 0;
}

static struct platform_driver bcm4330_bluetooth_platform_driver = {
	.probe = bcm4330_bluetooth_probe,
	.remove = bcm4330_bluetooth_remove,
	.driver = {
		.name = "bcm4330_bluetooth",
		.owner = THIS_MODULE,
	},
};

static int __init bcm4330_bluetooth_init(void)
{
	bt_enabled = false;
	return platform_driver_register(&bcm4330_bluetooth_platform_driver);
}

static void __exit bcm4330_bluetooth_exit(void)
{
	platform_driver_unregister(&bcm4330_bluetooth_platform_driver);
}

module_init(bcm4330_bluetooth_init);
module_exit(bcm4330_bluetooth_exit);

MODULE_ALIAS("platform:bcm4330");
MODULE_DESCRIPTION("bcm4330_bluetooth");
MODULE_AUTHOR("Jaikumar Ganesh <jaikumar@google.com>");
MODULE_LICENSE("GPL");
