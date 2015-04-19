/*
 * drivers/serial/msm_serial.c - driver for msm7k serial device and console
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Acknowledgements:
 * This file is based on msm_serial.c, originally
 * Written by Robert Love <rlove@google.com>  */

#define pr_fmt(fmt) "%s: " fmt, __func__

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/platform_data/qcom-serial_hs_lite.h>
#include <linux/msm-bus.h>
#include "msm_serial_hs_hwreg.h"
#include <linux/htc_cir.h>
#include <linux/regulator/consumer.h>


#define D(x...) pr_info("[CIR] " x)
#define E(x...) pr_err("[CIR][err] " x)

#define HTC_IRDA_FUNC 1
enum uart_core_type {
	LEGACY_HSUART,
	GSBI_HSUART,
	BLSP_HSUART,
};

enum uart_func_mode {
	UART_TWO_WIRE, 
	UART_FOUR_WIRE,
};

struct msm_hsl_port {
	struct uart_port	uart;
	char			name[16];
	struct clk		*clk;
	struct clk		*pclk;
	struct dentry		*loopback_dir;
#ifdef HTC_IRDA_FUNC
	struct dentry		*irda_config; 
#endif
	unsigned int		imr;
	unsigned int		*uart_csr_code;
	unsigned int            *gsbi_mapbase;
	unsigned int            *mapped_gsbi;
	unsigned int            old_snap_state;
	unsigned long		ver_id;
	int			tx_timeout;
	struct mutex		clk_mutex;
	enum uart_core_type	uart_type;
	enum uart_func_mode	func_mode;
	struct wake_lock	port_open_wake_lock;
	int			clk_enable_count;
	u32			bus_perf_client;
	
	struct msm_bus_scale_pdata *bus_scale_table;
	bool use_pinctrl;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	struct pinctrl_state *gpio_state_disable;
	bool cir_dynamic_disable;
#endif
	int (*cir_set_path)(int);
	int (*cir_reset)(void);
	int (*cir_power)(int); 
	uint32_t rst_pin;
	uint32_t cir_sir_switch;
	uint32_t cir_learn_en;
	uint32_t cir_ls_en;
	struct class *irda_class;
	struct device *irda_dev;
	struct class *cir_class;
	struct device *cir_dev;
	struct regulator *cir_regulator;
};

static struct msm_hsl_port *htc_cir_port;
static int cir_enable_flg;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
static int enable_uart_flag = 0xFF;
#endif

#define UARTDM_VERSION_11_13	0
#define UARTDM_VERSION_14	1

#define UART_TO_MSM(uart_port)	((struct msm_hsl_port *) uart_port)
#define is_console(port)	((port)->cons && \
				(port)->cons->index == (port)->line)

static const unsigned int regmap[][UARTDM_LAST] = {
	[UARTDM_VERSION_11_13] = {
		[UARTDM_MR1] = UARTDM_MR1_ADDR,
		[UARTDM_MR2] = UARTDM_MR2_ADDR,
		[UARTDM_IMR] = UARTDM_IMR_ADDR,
		[UARTDM_SR] = UARTDM_SR_ADDR,
		[UARTDM_CR] = UARTDM_CR_ADDR,
		[UARTDM_CSR] = UARTDM_CSR_ADDR,
		[UARTDM_IPR] = UARTDM_IPR_ADDR,
		[UARTDM_ISR] = UARTDM_ISR_ADDR,
		[UARTDM_RX_TOTAL_SNAP] = UARTDM_RX_TOTAL_SNAP_ADDR,
		[UARTDM_TFWR] = UARTDM_TFWR_ADDR,
		[UARTDM_RFWR] = UARTDM_RFWR_ADDR,
		[UARTDM_RF] = UARTDM_RF_ADDR,
		[UARTDM_TF] = UARTDM_TF_ADDR,
		[UARTDM_MISR] = UARTDM_MISR_ADDR,
		[UARTDM_DMRX] = UARTDM_DMRX_ADDR,
		[UARTDM_NCF_TX] = UARTDM_NCF_TX_ADDR,
		[UARTDM_DMEN] = UARTDM_DMEN_ADDR,
		[UARTDM_TXFS] = UARTDM_TXFS_ADDR,
		[UARTDM_RXFS] = UARTDM_RXFS_ADDR,
	},
	[UARTDM_VERSION_14] = {
		[UARTDM_MR1] = 0x0,
		[UARTDM_MR2] = 0x4,
		[UARTDM_IMR] = 0xb0,
		[UARTDM_SR] = 0xa4,
		[UARTDM_CR] = 0xa8,
		[UARTDM_CSR] = 0xa0,
		[UARTDM_IPR] = 0x18,
		[UARTDM_ISR] = 0xb4,
		[UARTDM_RX_TOTAL_SNAP] = 0xbc,
		[UARTDM_TFWR] = 0x1c,
		[UARTDM_RFWR] = 0x20,
		[UARTDM_RF] = 0x140,
		[UARTDM_TF] = 0x100,
		[UARTDM_MISR] = 0xac,
		[UARTDM_DMRX] = 0x34,
		[UARTDM_NCF_TX] = 0x40,
		[UARTDM_DMEN] = 0x3c,
		[UARTDM_TXFS] = 0x4c,
		[UARTDM_RXFS] = 0x50,
	},
};

static struct of_device_id msm_hsl_match_table[] = {
	{	.compatible = "CIR",
	},
	{}
};

#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
static int get_console_state(struct uart_port *port);
#else
static inline int get_console_state(struct uart_port *port) { return -ENODEV; };
#endif

static struct dentry *debug_base;
static inline void wait_for_xmitr(struct uart_port *port, int bits);
static inline void msm_hsl_write(struct uart_port *port,
				 unsigned int val, unsigned int off)
{
	__iowmb();
	__raw_writel_no_log((__force __u32)cpu_to_le32(val),
		port->membase + off);
}
static inline unsigned int msm_hsl_read(struct uart_port *port,
		     unsigned int off)
{
	unsigned int v = le32_to_cpu((__force __le32)__raw_readl_no_log(
		port->membase + off));
	__iormb();
	return v;
}

static unsigned int msm_serial_hsl_has_gsbi(struct uart_port *port)
{
	D("%s: cir uart port[%d] uart_type=%d\n", __func__, (port)->line, UART_TO_MSM(port)->uart_type);
	return (UART_TO_MSM(port)->uart_type == GSBI_HSUART);
}

static int cir_ls_ctrl(int enable)
{
	int ret = 0;
	struct msm_hsl_port *msm_cir_port = htc_cir_port;

	D("%s (): %d\n", __func__, enable);
	if (enable) {
		gpio_direction_output(msm_cir_port->cir_ls_en, 1);
		if (!IS_ERR_OR_NULL(msm_cir_port->pinctrl)) {
			pr_debug("%s(): Using Pinctrl", __func__);
			msm_cir_port->use_pinctrl = true;
			ret = pinctrl_select_state(msm_cir_port->pinctrl,
				msm_cir_port->gpio_state_active);
			if (ret)
				pr_err("%s(): Failed to pinctrl set_state_active",
					__func__);
			return ret;
		}
	} else {
		gpio_direction_output(msm_cir_port->cir_ls_en, 0);
		if (!IS_ERR_OR_NULL(msm_cir_port->pinctrl)) {
			pr_debug("%s(): Using Pinctrl", __func__);
			msm_cir_port->use_pinctrl = true;
			ret = pinctrl_select_state(msm_cir_port->pinctrl,
				msm_cir_port->gpio_state_suspend);
			if (ret)
				pr_err("%s(): Failed to pinctrl set_state_suspend",
					__func__);
			return ret;
		}
	}

	return ret;
}

static void set_gsbi_uart_func_mode(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	unsigned int set_gsbi_uart_mode = GSBI_PROTOCOL_I2C_UART;
	unsigned int cur_gsbi_uart_mode;

	if (msm_hsl_port->func_mode == UART_FOUR_WIRE)
		set_gsbi_uart_mode = GSBI_PROTOCOL_UART;

	if (msm_hsl_port->pclk)
		clk_prepare_enable(msm_hsl_port->pclk);

	
	cur_gsbi_uart_mode = ioread32(msm_hsl_port->mapped_gsbi +
					GSBI_CONTROL_ADDR);
	if ((cur_gsbi_uart_mode & set_gsbi_uart_mode) != set_gsbi_uart_mode)
		iowrite32(set_gsbi_uart_mode,
			msm_hsl_port->mapped_gsbi + GSBI_CONTROL_ADDR);

	D("%s: cir line[%d] GSBI_CONTROL_ADDR-%x\n", __func__,
		port->line, ioread32(msm_hsl_port->mapped_gsbi + GSBI_CONTROL_ADDR));
	D("%s () GSBI_CONTROL_ADDR:port->line %d, ir\n", __func__, port->line);
	if (msm_hsl_port->pclk)
		clk_disable_unprepare(msm_hsl_port->pclk);
}

static int msm_hsl_config_uart_tx_rx_gpios(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;
	int ret;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	if (!IS_ERR_OR_NULL(msm_hsl_port->pinctrl)) {
		pr_debug("%s(): Using Pinctrl", __func__);
		msm_hsl_port->use_pinctrl = true;
		ret = pinctrl_select_state(msm_hsl_port->pinctrl,
				msm_hsl_port->gpio_state_active);
		if (ret)
			pr_err("%s(): Failed to pinctrl set_state",
				__func__);
		return ret;
	} else if (pdata) {
		ret = gpio_request(pdata->uart_tx_gpio,
				"UART_TX_GPIO");
		if (unlikely(ret)) {
			pr_err("gpio request failed for:%d\n",
					pdata->uart_tx_gpio);
			goto exit_uart_config;
		}

		ret = gpio_request(pdata->uart_rx_gpio, "UART_RX_GPIO");
		if (unlikely(ret)) {
			pr_err("gpio request failed for:%d\n",
					pdata->uart_rx_gpio);
			gpio_free(pdata->uart_rx_gpio);
			goto exit_uart_config;
		}
	} else {
		pr_err("Pdata is NULL.\n");
		ret = -EINVAL;
	}

exit_uart_config:
	return ret;
}

static void msm_hsl_unconfig_uart_tx_rx_gpios(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;

	if (pdata) {
		gpio_free(pdata->uart_tx_gpio);
		gpio_free(pdata->uart_rx_gpio);
	} else {
		pr_err("Error:Pdata is NULL.\n");
	}
}

static int msm_hsl_config_uart_hwflow_gpios(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
				pdev->dev.platform_data;
	int ret = -EINVAL;

	if (pdata) {
		ret = gpio_request(pdata->uart_cts_gpio,
					"UART_CTS_GPIO");
		if (unlikely(ret)) {
			pr_err("gpio request failed for:%d\n",
					pdata->uart_cts_gpio);
			goto exit_config_uart;
		}

		ret = gpio_request(pdata->uart_rfr_gpio,
					"UART_RFR_GPIO");
		if (unlikely(ret)) {
			pr_err("gpio request failed for:%d\n",
				pdata->uart_rfr_gpio);
			gpio_free(pdata->uart_cts_gpio);
			goto exit_config_uart;
		}
	} else {
		pr_err("Error: Pdata is NULL.\n");
	}

exit_config_uart:
	return ret;
}

static void msm_hsl_unconfig_uart_hwflow_gpios(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;

	if (pdata) {
		gpio_free(pdata->uart_cts_gpio);
		gpio_free(pdata->uart_rfr_gpio);
	} else {
		pr_err("Error: Pdata is NULL.\n");
	}

}

static int msm_hsl_config_uart_gpios(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	int ret;

	
	ret = msm_hsl_config_uart_tx_rx_gpios(port);
	if (!ret) {
		if (msm_hsl_port->func_mode == UART_FOUR_WIRE) {
			
			ret = msm_hsl_config_uart_hwflow_gpios(port);
			if (ret)
				msm_hsl_unconfig_uart_tx_rx_gpios(port);
		}
	} else {
		msm_hsl_unconfig_uart_tx_rx_gpios(port);
	}

	return ret;
}

static void msm_hsl_unconfig_uart_gpios(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_unconfig_uart_tx_rx_gpios(port);
	if (msm_hsl_port->func_mode == UART_FOUR_WIRE)
		msm_hsl_unconfig_uart_hwflow_gpios(port);
}

static void msm_hsl_get_pinctrl_configs(struct uart_port *port)
{
	struct pinctrl_state *set_state;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_port->pinctrl = devm_pinctrl_get(port->dev);
	if (IS_ERR_OR_NULL(msm_hsl_port->pinctrl)) {
		pr_debug("%s(): Pinctrl not defined", __func__);
	} else {
		pr_info("%s(): Using Pinctrl", __func__);
		msm_hsl_port->use_pinctrl = true;

		set_state = pinctrl_lookup_state(msm_hsl_port->pinctrl,
						PINCTRL_STATE_DEFAULT);
		if (IS_ERR_OR_NULL(set_state)) {
			dev_err(port->dev,
				"pinctrl lookup failed for default state");
			goto pinctrl_fail;
		}

		pr_info("%s(): Pinctrl state active %p\n", __func__,
			set_state);
		msm_hsl_port->gpio_state_active = set_state;

		set_state = pinctrl_lookup_state(msm_hsl_port->pinctrl,
						PINCTRL_STATE_SLEEP);
		if (IS_ERR_OR_NULL(set_state)) {
			dev_err(port->dev,
				"pinctrl lookup failed for sleep state");
			goto pinctrl_fail;
		}

		pr_info("%s(): Pinctrl state sleep %p\n", __func__,
			set_state);
		msm_hsl_port->gpio_state_suspend = set_state;

#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
		if (msm_hsl_port->cir_dynamic_disable) {
			set_state = pinctrl_lookup_state(msm_hsl_port->pinctrl,
							"disable");
			if (IS_ERR_OR_NULL(set_state)) {
				dev_err(port->dev,
					"pinctrl lookup failed for disable state");
				goto pinctrl_fail;
			}

			pr_info("%s(): Pinctrl state disable %p\n", __func__,
				set_state);
			msm_hsl_port->gpio_state_disable = set_state;
		}
#endif
		return;
	}
pinctrl_fail:
	msm_hsl_port->pinctrl = NULL;
	return;
}

static int get_line(struct platform_device *pdev)
{
	struct msm_hsl_port *msm_hsl_port = platform_get_drvdata(pdev);
	return msm_hsl_port->uart.line;
}

static int bus_vote(uint32_t client, int vector)
{
	int ret = 0;

	if (!client)
		return ret;

	pr_debug("Voting for bus scaling:%d\n", vector);

	ret = msm_bus_scale_client_update_request(client, vector);
	if (ret)
		pr_err("Failed to request bus bw vector %d\n", vector);

	return ret;
}

static int clk_en(struct uart_port *port, int enable)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	int ret = 0;

	if (enable) {

		msm_hsl_port->clk_enable_count++;
		ret = bus_vote(msm_hsl_port->bus_perf_client,
				!!msm_hsl_port->clk_enable_count);
		if (ret)
			goto err;
		ret = clk_prepare_enable(msm_hsl_port->clk);
		if (ret)
			goto err_bus;
		if (msm_hsl_port->pclk) {
			ret = clk_prepare_enable(msm_hsl_port->pclk);
			if (ret)
				goto err_clk_disable;
		}
	} else {

		msm_hsl_port->clk_enable_count--;
		clk_disable_unprepare(msm_hsl_port->clk);
		if (msm_hsl_port->pclk)
			clk_disable_unprepare(msm_hsl_port->pclk);
		ret = bus_vote(msm_hsl_port->bus_perf_client,
				!!msm_hsl_port->clk_enable_count);
	}

	return ret;

err_clk_disable:
	clk_disable_unprepare(msm_hsl_port->clk);
err_bus:
	bus_vote(msm_hsl_port->bus_perf_client,
			!!(msm_hsl_port->clk_enable_count - 1));
err:
	msm_hsl_port->clk_enable_count--;
	return ret;
}
static int msm_hsl_loopback_enable_set(void *data, u64 val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned int vid;
	unsigned long flags;
	int ret = 0;

	ret = clk_set_rate(msm_hsl_port->clk, port->uartclk);
	if (!ret) {
		clk_en(port, 1);
	} else {
		pr_err("Error: setting uartclk rate as %u\n",
						port->uartclk);
		return -EINVAL;
	}

	vid = msm_hsl_port->ver_id;
	if (val) {
		spin_lock_irqsave(&port->lock, flags);
		ret = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
		ret |= UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, ret, regmap[vid][UARTDM_MR2]);
		spin_unlock_irqrestore(&port->lock, flags);
	} else {
		spin_lock_irqsave(&port->lock, flags);
		ret = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
		ret &= ~UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, ret, regmap[vid][UARTDM_MR2]);
		spin_unlock_irqrestore(&port->lock, flags);
	}

	clk_en(port, 0);
	return 0;
}
static int msm_hsl_loopback_enable_get(void *data, u64 *val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned long flags;
	int ret = 0;

	ret = clk_set_rate(msm_hsl_port->clk, port->uartclk);
	if (!ret) {
		clk_en(port, 1);
	} else {
		pr_err("Error setting uartclk rate as %u\n",
						port->uartclk);
		return -EINVAL;
	}

	spin_lock_irqsave(&port->lock, flags);
	ret = msm_hsl_read(port, regmap[msm_hsl_port->ver_id][UARTDM_MR2]);
	spin_unlock_irqrestore(&port->lock, flags);
	clk_en(port, 0);

	*val = (ret & UARTDM_MR2_LOOP_MODE_BMSK) ? 1 : 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(loopback_enable_fops, msm_hsl_loopback_enable_get,
			msm_hsl_loopback_enable_set, "%llu\n");

unsigned int force_baud_1;
#ifdef HTC_IRDA_FUNC
static int msm_hsl_irda_enable_set(void *data, u64 val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned long flags;
	int ret = 0;

	if (val == 96) {
		force_baud_1 = 96;
		E("%s(): change baud = 9600\n", __func__);
		return 0;
	} else if (val == 1152) {
		E("%s(): change baud = 115200\n", __func__);
		force_baud_1 = 1152;
		return 0;
	} else if (val == 1153) {
		E("%s():no  change baud \n", __func__);
		force_baud_1 = 0;
		return 0;
	}

	ret = clk_set_rate(msm_hsl_port->clk, 7372800);
	if (!ret) {
		clk_en(port, 1);
		E("%s(): irda Clock enabled for line(%d)\n", __func__, port->line);
	} else {
		E("%s(): Error: Setting the clock rate\n", __func__);
		return -EINVAL;
	}

	if (val) {
		E("%s(): irda turn on IRDA\n", __func__);
		spin_lock_irqsave(&port->lock, flags);
		if (msm_cir_port->cir_learn_en)
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		gpio_direction_output(msm_cir_port->rst_pin, 0);
		msleep(10);

		if (msm_cir_port->cir_sir_switch)
			gpio_direction_output(msm_cir_port->cir_sir_switch, 1);
		ret = 3;
		ret |= (int)val;
		msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);

		if (msm_hsl_port->cir_set_path)
			msm_hsl_port->cir_set_path(PATH_IRDA);
		else
			E("no irda enable callback function");

	} else {
		E("%s(): irda turn off IRDA \n", __func__);
		spin_lock_irqsave(&port->lock, flags);
		if (msm_cir_port->cir_sir_switch)
			gpio_direction_output(msm_cir_port->cir_sir_switch, 0);
		if (msm_cir_port->cir_learn_en)
			gpio_direction_output(msm_cir_port->cir_learn_en, 0);
		gpio_direction_input(msm_cir_port->rst_pin);
		ret = 0;
		msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);

		if (msm_hsl_port->cir_set_path)
			msm_hsl_port->cir_set_path(PATH_NONE);
		else
			E("no irda enable callback function");
	}


	clk_en(port, 0);
	E("%s(): irda Clock enabled for line(%d)\n", __func__, port->line);
	return 0;
}
static int msm_hsl_irda_enable_get(void *data, u64 *val)
{
	E("%s: warning! disallow read register UARTDM_IRDA\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(msm_serial_irda_fops, msm_hsl_irda_enable_get,
			msm_hsl_irda_enable_set, "%llu\n");
#endif

static void msm_hsl_debugfs_init(struct msm_hsl_port *msm_uport,
								int id)
{
	char node_name[15];

	snprintf(node_name, sizeof(node_name), "loopback.%d", id);
	msm_uport->loopback_dir = debugfs_create_file(node_name,
					S_IRUGO | S_IWUSR,
					debug_base,
					msm_uport,
					&loopback_enable_fops);

	if (IS_ERR_OR_NULL(msm_uport->loopback_dir))
		pr_err("Cannot create loopback.%d debug entry", id);

#ifdef HTC_IRDA_FUNC
	snprintf(node_name, sizeof(node_name), "irda.%d", id);
	msm_uport->irda_config = debugfs_create_file(node_name,
					S_IRUGO | S_IWUSR,
					debug_base,
					msm_uport,
					&msm_serial_irda_fops);

	if (IS_ERR_OR_NULL(msm_uport->irda_config))
		E("%s(): Cannot create irda.%d debug entry",
							__func__, id);
#endif
}
static void msm_hsl_stop_tx_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	unsigned int vid = UART_TO_MSM(port)->ver_id;
	int count = 0;

	msm_hsl_port->imr &= ~UARTDM_ISR_TXLEV_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);

	if (msm_cir_port->cir_learn_en && cir_enable_flg == PATH_IRDA) {
		if (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_TXEMT_BMSK)) {
			while (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_TXEMT_BMSK)) {
				udelay(1);
				touch_nmi_watchdog();
				cpu_relax();
				
				if (++count > (1000000000 / 115200) * 6) {
					E("Wait too long for Tx end\n");
					break;
				}
			}
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		} else {
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		}
	}
}

static void msm_hsl_start_tx_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;

	if (port->suspended) {
		pr_err("%s: System is in Suspend state\n", __func__);
		return;
	}
	msm_hsl_port->imr |= UARTDM_ISR_TXLEV_BMSK;

	if (msm_cir_port->cir_learn_en && cir_enable_flg == PATH_IRDA) {
		gpio_direction_output(msm_cir_port->cir_learn_en, 0);
	}

	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);
}

static void msm_hsl_stop_rx_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_port->imr &= ~(UARTDM_ISR_RXLEV_BMSK |
			       UARTDM_ISR_RXSTALE_BMSK);
	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);
}

static void msm_hsl_enable_ms_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_port->imr |= UARTDM_ISR_DELTA_CTS_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);
}

static void handle_rx(struct uart_port *port, unsigned int misr)
{
	struct tty_struct *tty = port->state->port.tty;
	unsigned int vid;
	unsigned int sr;
	int count = 0;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	vid = msm_hsl_port->ver_id;
	if ((msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_OVERRUN_BMSK)) {
		port->icount.overrun++;
		tty_insert_flip_char(tty->port, 0, TTY_OVERRUN);
		msm_hsl_write(port, RESET_ERROR_STATUS,
			regmap[vid][UARTDM_CR]);
	}

	if (misr & UARTDM_ISR_RXSTALE_BMSK) {
		count = msm_hsl_read(port,
			regmap[vid][UARTDM_RX_TOTAL_SNAP]) -
			msm_hsl_port->old_snap_state;
		msm_hsl_port->old_snap_state = 0;
	} else {
		count = 4 * (msm_hsl_read(port, regmap[vid][UARTDM_RFWR]));
		msm_hsl_port->old_snap_state += count;
	}

	
	while (count > 0) {
		unsigned int c;
		char flag = TTY_NORMAL;

		sr = msm_hsl_read(port, regmap[vid][UARTDM_SR]);
		if ((sr & UARTDM_SR_RXRDY_BMSK) == 0) {
			msm_hsl_port->old_snap_state -= count;
			break;
		}
		c = msm_hsl_read(port, regmap[vid][UARTDM_RF]);
		if (sr & UARTDM_SR_RX_BREAK_BMSK) {
			port->icount.brk++;
			if (uart_handle_break(port))
				continue;
		} else if (sr & UARTDM_SR_PAR_FRAME_BMSK) {
			port->icount.frame++;
		} else {
			port->icount.rx++;
		}

		
		sr &= port->read_status_mask;
		if (sr & UARTDM_SR_RX_BREAK_BMSK)
			flag = TTY_BREAK;
		else if (sr & UARTDM_SR_PAR_FRAME_BMSK)
			flag = TTY_FRAME;

		
		
		tty_insert_flip_string(tty->port, (char *) &c,
				       (count > 4) ? 4 : count);
		count -= 4;
	}

	tty_flip_buffer_push(tty->port);
}

static void handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	int sent_tx;
	int tx_count;
	int x;
	unsigned int tf_pointer = 0;
	unsigned int vid;

	vid = UART_TO_MSM(port)->ver_id;
	tx_count = uart_circ_chars_pending(xmit);

	if (tx_count > (UART_XMIT_SIZE - xmit->tail))
		tx_count = UART_XMIT_SIZE - xmit->tail;
	if (tx_count >= port->fifosize)
		tx_count = port->fifosize;

	
	if (port->x_char) {
		wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
		msm_hsl_write(port, tx_count + 1, regmap[vid][UARTDM_NCF_TX]);
		msm_hsl_read(port, regmap[vid][UARTDM_NCF_TX]);
		msm_hsl_write(port, port->x_char, regmap[vid][UARTDM_TF]);
		port->icount.tx++;
		port->x_char = 0;
	} else if (tx_count) {
		wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
		msm_hsl_write(port, tx_count, regmap[vid][UARTDM_NCF_TX]);
		msm_hsl_read(port, regmap[vid][UARTDM_NCF_TX]);
	}
	if (!tx_count) {
		msm_hsl_stop_tx_cir(port);
		return;
	}

	while (tf_pointer < tx_count)  {
		if (unlikely(!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
			       UARTDM_SR_TXRDY_BMSK)))
			continue;
		switch (tx_count - tf_pointer) {
		case 1: {
			x = xmit->buf[xmit->tail];
			port->icount.tx++;
			break;
		}
		case 2: {
			x = xmit->buf[xmit->tail]
				| xmit->buf[xmit->tail+1] << 8;
			port->icount.tx += 2;
			break;
		}
		case 3: {
			x = xmit->buf[xmit->tail]
				| xmit->buf[xmit->tail+1] << 8
				| xmit->buf[xmit->tail + 2] << 16;
			port->icount.tx += 3;
			break;
		}
		default: {
			x = *((int *)&(xmit->buf[xmit->tail]));
			port->icount.tx += 4;
			break;
		}
		}
		msm_hsl_write(port, x, regmap[vid][UARTDM_TF]);
		xmit->tail = ((tx_count - tf_pointer < 4) ?
			      (tx_count - tf_pointer + xmit->tail) :
			      (xmit->tail + 4)) & (UART_XMIT_SIZE - 1);
		tf_pointer += 4;
		sent_tx = 1;
	}

	if (uart_circ_empty(xmit))
		msm_hsl_stop_tx_cir(port);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

}

static void handle_delta_cts(struct uart_port *port)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	msm_hsl_write(port, RESET_CTS, regmap[vid][UARTDM_CR]);
	port->icount.cts++;
	wake_up_interruptible(&port->state->port.delta_msr_wait);
}

static irqreturn_t msm_hsl_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	unsigned int vid;
	unsigned int misr;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	vid = msm_hsl_port->ver_id;
	misr = msm_hsl_read(port, regmap[vid][UARTDM_MISR]);
	
	msm_hsl_write(port, 0, regmap[vid][UARTDM_IMR]);

	if (misr & (UARTDM_ISR_RXSTALE_BMSK | UARTDM_ISR_RXLEV_BMSK)) {
		handle_rx(port, misr);
		if (misr & (UARTDM_ISR_RXSTALE_BMSK))
			msm_hsl_write(port, RESET_STALE_INT,
					regmap[vid][UARTDM_CR]);
		msm_hsl_write(port, 6500, regmap[vid][UARTDM_DMRX]);
		msm_hsl_write(port, STALE_EVENT_ENABLE, regmap[vid][UARTDM_CR]);
	}
	if (misr & UARTDM_ISR_TXLEV_BMSK)
		handle_tx(port);

	if (misr & UARTDM_ISR_DELTA_CTS_BMSK)
		handle_delta_cts(port);

	
	msm_hsl_write(port, msm_hsl_port->imr, regmap[vid][UARTDM_IMR]);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static unsigned int msm_hsl_tx_empty_cir(struct uart_port *port)
{
	unsigned int ret;
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	ret = (msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
	       UARTDM_SR_TXEMT_BMSK) ? TIOCSER_TEMT : 0;
	return ret;
}

static void msm_hsl_reset(struct uart_port *port)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	
	msm_hsl_write(port, RESET_RX, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_TX, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_ERROR_STATUS, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_BREAK_INT, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_CTS, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RFR_LOW, regmap[vid][UARTDM_CR]);
}

static unsigned int msm_hsl_get_mctrl_cir(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR | TIOCM_RTS;
}

static void msm_hsl_set_mctrl_cir(struct uart_port *port, unsigned int mctrl)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;
	unsigned int mr;
	unsigned int loop_mode;

	mr = msm_hsl_read(port, regmap[vid][UARTDM_MR1]);

	if (!(mctrl & TIOCM_RTS)) {
		mr &= ~UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hsl_write(port, mr, regmap[vid][UARTDM_MR1]);
		msm_hsl_write(port, RFR_HIGH, regmap[vid][UARTDM_CR]);
	} else {
		mr |= UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hsl_write(port, mr, regmap[vid][UARTDM_MR1]);
	}

	loop_mode = TIOCM_LOOP & mctrl;
	if (loop_mode) {
		mr = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
		mr |= UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, mr, regmap[vid][UARTDM_MR2]);

		
		msm_hsl_reset(port);

		
		msm_hsl_write(port, UARTDM_CR_RX_EN_BMSK
		      | UARTDM_CR_TX_EN_BMSK, regmap[vid][UARTDM_CR]);
	}
}

static void msm_hsl_break_ctl_cir(struct uart_port *port, int break_ctl)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	if (break_ctl)
		msm_hsl_write(port, START_BREAK, regmap[vid][UARTDM_CR]);
	else
		msm_hsl_write(port, STOP_BREAK, regmap[vid][UARTDM_CR]);
}

static void msm_hsl_set_baud_rate(struct uart_port *port,
						unsigned int baud)
{
	unsigned int baud_code, rxstale, watermark;
	unsigned int data;
	unsigned int vid;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	
		
		if (force_baud_1 == 96)
			baud = 9600;
		else if (force_baud_1 == 1152)
			baud = 115200;
		
	
	D("%s ()baud %d:port->line %d, ir\n", __func__, baud, port->line);

	switch (baud) {
	case 300:
		baud_code = UARTDM_CSR_75;
		rxstale = 1;
		break;
	case 600:
		baud_code = UARTDM_CSR_150;
		rxstale = 1;
		break;
	case 1200:
		baud_code = UARTDM_CSR_300;
		rxstale = 1;
		break;
	case 2400:
		baud_code = UARTDM_CSR_600;
		rxstale = 1;
		break;
	case 4800:
		baud_code = UARTDM_CSR_1200;
		rxstale = 1;
		break;
	case 9600:
		baud_code = UARTDM_CSR_2400;
		rxstale = 2;
		break;
	case 14400:
		baud_code = UARTDM_CSR_3600;
		rxstale = 3;
		break;
	case 19200:
		baud_code = UARTDM_CSR_4800;
		rxstale = 4;
		break;
	case 28800:
		baud_code = UARTDM_CSR_7200;
		rxstale = 6;
		break;
	case 38400:
		baud_code = UARTDM_CSR_9600;
		rxstale = 8;
		break;
	case 57600:
		baud_code = UARTDM_CSR_14400;
		rxstale = 16;
		break;
	case 115200:
		baud_code = UARTDM_CSR_28800;
		rxstale = 31;
		break;
	case 230400:
		baud_code = UARTDM_CSR_57600;
		rxstale = 31;
		break;
	case 460800:
		baud_code = UARTDM_CSR_115200;
		rxstale = 31;
		break;
	case 4000000:
	case 3686400:
	case 3200000:
	case 3500000:
	case 3000000:
	case 2500000:
	case 1500000:
	case 1152000:
	case 1000000:
	case 921600:
		baud_code = 0xff;
		rxstale = 31;
		break;
	default: 
		baud_code = UARTDM_CSR_28800;
		rxstale = 31;
		break;
	}

	vid = msm_hsl_port->ver_id;
	msm_hsl_write(port, baud_code, regmap[vid][UARTDM_CSR]);

	mb();

	if (baud > 460800)
		port->uartclk = baud * 16;
	else
		port->uartclk = 7372800;

	if (clk_set_rate(msm_hsl_port->clk, port->uartclk)) {
		pr_err("Error: setting uartclk rate %u\n", port->uartclk);
		WARN_ON(1);
		return;
	}

	
	msm_hsl_port->tx_timeout = (1000000000 / baud) * 6;

	
	watermark = UARTDM_IPR_STALE_LSB_BMSK & rxstale;
	watermark |= UARTDM_IPR_STALE_TIMEOUT_MSB_BMSK & (rxstale << 2);
	msm_hsl_write(port, watermark, regmap[vid][UARTDM_IPR]);

	watermark = (port->fifosize * 3) / 4;
	msm_hsl_write(port, watermark, regmap[vid][UARTDM_RFWR]);

	
	msm_hsl_write(port, 0, regmap[vid][UARTDM_TFWR]);

	msm_hsl_write(port, CR_PROTECTION_EN, regmap[vid][UARTDM_CR]);
	msm_hsl_reset(port);

	data = UARTDM_CR_TX_EN_BMSK;
	data |= UARTDM_CR_RX_EN_BMSK;
	
	msm_hsl_write(port, data, regmap[vid][UARTDM_CR]);

	msm_hsl_write(port, RESET_STALE_INT, regmap[vid][UARTDM_CR]);
	
	msm_hsl_port->imr = UARTDM_ISR_RXSTALE_BMSK
		| UARTDM_ISR_DELTA_CTS_BMSK | UARTDM_ISR_RXLEV_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr, regmap[vid][UARTDM_IMR]);
	msm_hsl_write(port, 6500, regmap[vid][UARTDM_DMRX]);
	msm_hsl_write(port, STALE_EVENT_ENABLE, regmap[vid][UARTDM_CR]);
}

static void msm_hsl_init_clock(struct uart_port *port)
{
	D("%s ()ok:port->line %d, ir\n", __func__, port->line);
	clk_en(port, 1);
}

static void msm_hsl_deinit_clock(struct uart_port *port)
{
	D("%s ()ok:port->line %d, ir\n", __func__, port->line);
	clk_en(port, 0);
}

static int msm_hsl_startup_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;
	unsigned int data, rfr_level;
	unsigned int vid;
	int ret;
	unsigned long flags;

	snprintf(msm_hsl_port->name, sizeof(msm_hsl_port->name),
		 "msm_serial_hsl%d", port->line);

	D("%s () :port->line %d, ir\n", __func__, port->line);
	if (!(is_console(port)) || (!port->cons) ||
		(port->cons && (!(port->cons->flags & CON_ENABLED)))) {

		if (msm_serial_hsl_has_gsbi(port))
			set_gsbi_uart_func_mode(port);

		if (pdata && pdata->use_pm)
			wake_lock(&msm_hsl_port->port_open_wake_lock);

		if (pdata && pdata->config_gpio) {
			ret = msm_hsl_config_uart_gpios(port);
			if (ret) {
				msm_hsl_unconfig_uart_gpios(port);
				goto release_wakelock;
			}
		}
	}

	if (likely(port->fifosize > 48))
		rfr_level = port->fifosize - 16;
	else
		rfr_level = port->fifosize;

	spin_lock_irqsave(&port->lock, flags);

	vid = msm_hsl_port->ver_id;
	
	data = msm_hsl_read(port, regmap[vid][UARTDM_MR1]);
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK;
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK;
	data |= UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK & (rfr_level << 2);
	data |= UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK & rfr_level;
	msm_hsl_write(port, data, regmap[vid][UARTDM_MR1]);
	spin_unlock_irqrestore(&port->lock, flags);

	ret = request_irq(port->irq, msm_hsl_irq, IRQF_TRIGGER_HIGH,
			  msm_hsl_port->name, port);
	if (unlikely(ret)) {
		pr_err("failed to request_irq\n");
		msm_hsl_unconfig_uart_gpios(port);
		goto release_wakelock;
	}

	return ret;

release_wakelock:
	if (pdata && pdata->use_pm)
		wake_unlock(&msm_hsl_port->port_open_wake_lock);

	return ret;
}

static void msm_hsl_shutdown_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;

	msm_hsl_port->imr = 0;
	
	msm_hsl_write(port, 0, regmap[msm_hsl_port->ver_id][UARTDM_IMR]);

	free_irq(port->irq, port);

	if (!(is_console(port)) || (!port->cons) ||
		(port->cons && (!(port->cons->flags & CON_ENABLED)))) {
		
		if (pdata && pdata->config_gpio)
			msm_hsl_unconfig_uart_gpios(port);

		if (pdata && pdata->use_pm)
			wake_unlock(&msm_hsl_port->port_open_wake_lock);
	}
}

static void msm_hsl_set_termios_cir(struct uart_port *port,
				struct ktermios *termios,
				struct ktermios *old)
{
	unsigned int baud, mr;
	unsigned int vid;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	if (!termios->c_cflag)
		return;

	mutex_lock(&msm_hsl_port->clk_mutex);

	baud = uart_get_baud_rate(port, termios, old, 200, 4000000);

	if (baud == 200)
		baud = 3200000;

	msm_hsl_set_baud_rate(port, baud);

	vid = UART_TO_MSM(port)->ver_id;
	
	mr = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
	mr &= ~UARTDM_MR2_PARITY_MODE_BMSK;
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			mr |= ODD_PARITY;
		else if (termios->c_cflag & CMSPAR)
			mr |= SPACE_PARITY;
		else
			mr |= EVEN_PARITY;
	}

	
	mr &= ~UARTDM_MR2_BITS_PER_CHAR_BMSK;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		mr |= FIVE_BPC;
		break;
	case CS6:
		mr |= SIX_BPC;
		break;
	case CS7:
		mr |= SEVEN_BPC;
		break;
	case CS8:
	default:
		mr |= EIGHT_BPC;
		break;
	}

	
	mr &= ~(STOP_BIT_ONE | STOP_BIT_TWO);
	if (termios->c_cflag & CSTOPB)
		mr |= STOP_BIT_TWO;
	else
		mr |= STOP_BIT_ONE;

	
	msm_hsl_write(port, mr, regmap[vid][UARTDM_MR2]);

	
	mr = msm_hsl_read(port, regmap[vid][UARTDM_MR1]);
	mr &= ~(UARTDM_MR1_CTS_CTL_BMSK | UARTDM_MR1_RX_RDY_CTL_BMSK);
	if (termios->c_cflag & CRTSCTS) {
		mr |= UARTDM_MR1_CTS_CTL_BMSK;
		mr |= UARTDM_MR1_RX_RDY_CTL_BMSK;
	}
	msm_hsl_write(port, mr, regmap[vid][UARTDM_MR1]);

	
	port->read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UARTDM_SR_PAR_FRAME_BMSK;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UARTDM_SR_RX_BREAK_BMSK;

	uart_update_timeout(port, termios->c_cflag, baud);

	mutex_unlock(&msm_hsl_port->clk_mutex);

	D("%s: cir MR is 0x%x\n", __func__, mr);
	D("%s: cir baud is %d\n", __func__, baud);
}

static const char *msm_hsl_type_cir(struct uart_port *port)
{
	return "MSM";
}

static void msm_hsl_release_port_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	resource_size_t size;

	D("%s () :port->line %d, ir\n", __func__, port->line);
	uart_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "uartdm_resource");
	if (!uart_resource) {
		D("%s ()  uart_resource:port->line %d, ir\n", __func__, port->line);
		uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	}
	if (unlikely(!uart_resource))
		return;
	size = uart_resource->end - uart_resource->start + 1;

	release_mem_region(port->mapbase, size);
	iounmap(port->membase);
	port->membase = NULL;

	if (msm_serial_hsl_has_gsbi(port)) {
		D("%s () msm_serial_hsl_has_gsbi :port->line %d, ir\n", __func__, port->line);
		iowrite32(GSBI_PROTOCOL_IDLE, msm_hsl_port->mapped_gsbi +
			  GSBI_CONTROL_ADDR);
		iounmap(msm_hsl_port->mapped_gsbi);
		msm_hsl_port->mapped_gsbi = NULL;
	}
	D("%s () ok :port->line %d, ir\n", __func__, port->line);
}

static int msm_hsl_request_port_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	struct resource *gsbi_resource;
	resource_size_t size;

	uart_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "uartdm_resource");
	if (!uart_resource) {
		D("%s ():uart_resource :port->line %d, ir\n", __func__, port->line);
		uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	}
	if (unlikely(!uart_resource)) {
		pr_err("can't get uartdm resource\n");
		return -ENXIO;
	}
	size = uart_resource->end - uart_resource->start + 1;

	if (unlikely(!request_mem_region(port->mapbase, size,
					 "msm_serial_cir"))) {
		pr_err("can't get mem region for uartdm\n");
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, size);
	if (!port->membase) {
		release_mem_region(port->mapbase, size);
		return -EBUSY;
	}
	D("%s ():uart_resource :port->line %d, memory map for base 0x%llx ir\n", __func__, port->line, port->mapbase);

	if (msm_serial_hsl_has_gsbi(port)) {
		gsbi_resource = platform_get_resource_byname(pdev,
							     IORESOURCE_MEM,
							     "gsbi_resource");
		if (!gsbi_resource)
			gsbi_resource = platform_get_resource(pdev,
						IORESOURCE_MEM, 1);
		if (unlikely(!gsbi_resource)) {
			pr_err("can't get gsbi resource\n");
			return -ENXIO;
		}

		size = gsbi_resource->end - gsbi_resource->start + 1;
		msm_hsl_port->mapped_gsbi = ioremap(gsbi_resource->start,
						    size);
		if (!msm_hsl_port->mapped_gsbi) {
			return -EBUSY;
		}
	}

	D("%s () ok:port->line %d, ir\n", __func__, port->line);

	return 0;
}

static void msm_hsl_config_port_cir(struct uart_port *port, int flags)
{
	D("%s () :port->line %d, ir\n", __func__, port->line);
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_MSM;
		if (msm_hsl_request_port_cir(port)) {
			D("%s () request_port_cir return:port->line %d, ir\n", __func__, port->line);
			return;
		}
	}

	
	if (msm_serial_hsl_has_gsbi(port))
		set_gsbi_uart_func_mode(port);
}

static int msm_hsl_verify_port_cir(struct uart_port *port,
			       struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_MSM))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}

static void msm_hsl_power_cir(struct uart_port *port, unsigned int state,
			  unsigned int oldstate)
{
	int ret;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	unsigned long flags;

	switch (state) {
	case 0:
		ret = clk_set_rate(msm_hsl_port->clk, port->uartclk);
		if (ret)
			pr_err("Error setting UART clock rate to %u\n",
							port->uartclk);
		clk_en(port, 1);
		break;
	case 3:
		if (cir_enable_flg != PATH_CIR) {
			D("%s path is not CIR. flg = %d\n",
						__func__, cir_enable_flg);
			D("%s(): Clear IRDA mode \n", __func__);
			spin_lock_irqsave(&port->lock, flags);
			if (msm_cir_port->cir_sir_switch)
				gpio_direction_output(msm_cir_port->cir_sir_switch, 0);
			if (msm_cir_port->cir_learn_en)
				gpio_direction_output(msm_cir_port->cir_learn_en, 0);
			gpio_direction_input(msm_cir_port->rst_pin);
			ret = 0;
			msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
			spin_unlock_irqrestore(&port->lock, flags);

			cir_enable_flg = PATH_CIR;
			if (msm_hsl_port->cir_set_path)
				msm_hsl_port->cir_set_path(PATH_CIR);
		}
		clk_en(port, 0);
		break;
	default:
		E("%s(): Unknown PM state %d\n", __func__, state);
	}
}

static struct uart_ops msm_hsl_uart_pops = {
	.tx_empty = msm_hsl_tx_empty_cir,
	.set_mctrl = msm_hsl_set_mctrl_cir,
	.get_mctrl = msm_hsl_get_mctrl_cir,
	.stop_tx = msm_hsl_stop_tx_cir,
	.start_tx = msm_hsl_start_tx_cir,
	.stop_rx = msm_hsl_stop_rx_cir,
	.enable_ms = msm_hsl_enable_ms_cir,
	.break_ctl = msm_hsl_break_ctl_cir,
	.startup = msm_hsl_startup_cir,
	.shutdown = msm_hsl_shutdown_cir,
	.set_termios = msm_hsl_set_termios_cir,
	.type = msm_hsl_type_cir,
	.release_port = msm_hsl_release_port_cir,
	.request_port = msm_hsl_request_port_cir,
	.config_port = msm_hsl_config_port_cir,
	.verify_port = msm_hsl_verify_port_cir,
	.pm = msm_hsl_power_cir,
};

static struct msm_hsl_port msm_hsl_uart_ports[] = {
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 0,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 1,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 2,
		},
	},
};

#define UART_NR	ARRAY_SIZE(msm_hsl_uart_ports)

static inline struct uart_port *get_port_from_line(unsigned int line)
{
	return &msm_hsl_uart_ports[line].uart;
}

void wait_for_xmitr(struct uart_port *port, int bits)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	if (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
			UARTDM_SR_TXEMT_BMSK)) {
		while ((msm_hsl_read(port, regmap[vid][UARTDM_ISR]) &
					bits) != bits) {
			udelay(1);
			touch_nmi_watchdog();
			cpu_relax();
		}
		msm_hsl_write(port, CLEAR_TX_READY, regmap[vid][UARTDM_CR]);
	}
	D("%s ():port->line %d, ir\n", __func__, port->line);
}

#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
static void msm_hsl_console_putchar(struct uart_port *port, int ch)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
	msm_hsl_write(port, 1, regmap[vid][UARTDM_NCF_TX]);

	while (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_TXRDY_BMSK)) {
		udelay(1);
		touch_nmi_watchdog();
	}

	msm_hsl_write(port, ch, regmap[vid][UARTDM_TF]);
	D("%s ():port->line %d, ir\n", __func__, port->line);
}

static void msm_hsl_console_write(struct console *co, const char *s,
				  unsigned int count)
{
	struct uart_port *port;
	struct msm_hsl_port *msm_hsl_port;
	unsigned int vid;
	int locked;

	BUG_ON(co->index < 0 || co->index >= UART_NR);

	port = get_port_from_line(co->index);
	msm_hsl_port = UART_TO_MSM(port);
	vid = msm_hsl_port->ver_id;

	
	if (port->sysrq || oops_in_progress)
		locked = spin_trylock(&port->lock);
	else {
		locked = 1;
		spin_lock(&port->lock);
	}
	msm_hsl_write(port, 0, regmap[vid][UARTDM_IMR]);
	uart_console_write(port, s, count, msm_hsl_console_putchar);
	msm_hsl_write(port, msm_hsl_port->imr, regmap[vid][UARTDM_IMR]);
	if (locked == 1)
		spin_unlock(&port->lock);

	D("%s ():port->line %d, ir\n", __func__, port->line);
}

static int msm_hsl_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	unsigned int vid;
	int baud = 0, flow, bits, parity;
	int ret;

	D("%s: ir\n", __func__);

	if (unlikely(co->index >= UART_NR || co->index < 0))
		return -ENXIO;

	port = get_port_from_line(co->index);
	vid = UART_TO_MSM(port)->ver_id;
	D("%s ():port->line %d, ir\n", __func__, port->line);

	if (unlikely(!port->membase))
		return -ENXIO;

	port->cons = co;

	pm_runtime_get_noresume(port->dev);

#ifndef CONFIG_PM_RUNTIME
	msm_hsl_init_clock(port);
#endif
	pm_runtime_resume(port->dev);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	bits = 8;
	parity = 'n';
	flow = 'n';
	msm_hsl_write(port, UARTDM_MR2_BITS_PER_CHAR_8 | STOP_BIT_ONE,
		      regmap[vid][UARTDM_MR2]);	

	if (baud < 300 || baud > 115200)
		baud = 115200;
	msm_hsl_set_baud_rate(port, baud);
	D("%s: cir port[%d] baud=%d\n", __func__, port->line, baud);

	ret = uart_set_options(port, co, baud, parity, bits, flow);
	msm_hsl_reset(port);
	
	msm_hsl_write(port, CR_PROTECTION_EN, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, UARTDM_CR_TX_EN_BMSK, regmap[vid][UARTDM_CR]);

	pr_info("console setup on port #%d\n", port->line);
	D("%s ():port->line %d, ok, ir\n", __func__, port->line);

	return ret;
}

static struct uart_driver msm_hsl_uart_driver;

static struct console msm_hsl_console = {
	.name = "ttyHSL",
	.write = msm_hsl_console_write,
	.device = uart_console_device,
	.setup = msm_hsl_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &msm_hsl_uart_driver,
};

#define MSM_HSL_CONSOLE	(&msm_hsl_console)
static int get_console_state(struct uart_port *port)
{
	if (is_console(port) && (port->cons->flags & CON_ENABLED)) {
		D("%s ()return 1 :port->line %d, ir\n", __func__, port->line);
		return 1;
	}
	else {
		D("%s ()return 0 :port->line %d, ir\n", __func__, port->line);
		return 0;
	}
}

static ssize_t show_msm_console(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int enable;
	struct uart_port *port;

	struct platform_device *pdev = to_platform_device(dev);
	port = get_port_from_line(get_line(pdev));

	enable = get_console_state(port);

	D("%s () :port->line %d, ir\n", __func__, port->line);
	return snprintf(buf, sizeof(enable), "%d\n", enable);
}

static ssize_t set_msm_console(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int enable, cur_state;
	struct uart_port *port;

	struct platform_device *pdev = to_platform_device(dev);
	port = get_port_from_line(get_line(pdev));

	cur_state = get_console_state(port);
	enable = buf[0] - '0';
	D("%s ():port->line %d,enable %d, cur_state %d ir\n", __func__, port->line, enable, cur_state);

	if (enable == cur_state)
		return count;

	switch (enable) {
	case 0:
		D("%s(): Calling stop_console\n", __func__);
		console_stop(port->cons);
		D("%s(): Calling unregister_console\n", __func__);
		unregister_console(port->cons);
		pm_runtime_put_sync(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		msm_hsl_power_cir(port, 3, 1);
		break;
	case 1:
		D("%s(): Calling register_console\n", __func__);
		msm_hsl_power_cir(port, 0, 1);
		pm_runtime_enable(&pdev->dev);
		register_console(port->cons);
		break;
	default:
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(console, S_IWUSR | S_IRUGO, show_msm_console,
						set_msm_console);
#else
#define MSM_HSL_CONSOLE	NULL
#endif

static struct uart_driver msm_hsl_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "msm_serial_cir",
	.dev_name = "ttyHSL",
	.nr = UART_NR,
	.cons = MSM_HSL_CONSOLE,
};

#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
static ssize_t enable_uart_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	D("%s(): %d\n", __func__, enable_uart_flag);
	return 0;
}

const struct tty_operations tty_null_op = {};
static ssize_t enable_uart_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	int ret;

	sscanf(buf, "%d", &ret);
	if (ret == 0 && enable_uart_flag != 0) {
		enable_uart_flag = 0;

		D("%s(): Using Pinctrl\n", __func__);
		msm_cir_port->use_pinctrl = true;
		ret = pinctrl_select_state(msm_cir_port->pinctrl,
			msm_cir_port->gpio_state_disable);
		if (ret)
			pr_err("%s(): Failed to pinctrl set_state_disable, ret=%d",
				__func__, ret);

		tty_set_operations(msm_hsl_uart_driver.tty_driver, &tty_null_op);

		if (msm_cir_port->rst_pin)
			gpio_direction_output(msm_cir_port->rst_pin, 0);
		if (msm_cir_port->cir_ls_en)
			gpio_direction_output(msm_cir_port->cir_ls_en, 0);
		if (msm_cir_port->cir_learn_en)
			gpio_direction_output(msm_cir_port->cir_learn_en, 0);
		if (msm_cir_port->cir_sir_switch)
			gpio_direction_output(msm_cir_port->cir_sir_switch, 0);

		regulator_disable(msm_cir_port->cir_regulator);
		regulator_put(msm_cir_port->cir_regulator);
		D("%s(): disable uart driver\n", __func__);
	}
	return count;
}

static DEVICE_ATTR(enable_uart, 0660, enable_uart_show, enable_uart_store);
#endif

static ssize_t enable_irda_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}
static ssize_t enable_irda_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int irda_en;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (enable_uart_flag == 0)
		return count;
#endif
	sscanf(buf, "%d", &irda_en);
	if (irda_en == 96) {
		force_baud_1 = 96;
		E("%s(): change baud = 9600\n", __func__);
		return count;
	} else if (irda_en == 1152) {
		E("%s(): change baud = 115200\n", __func__);
		force_baud_1 = 1152;
		return count;
	} else if (irda_en == 1153) {
		E("%s():no  change baud \n", __func__);
		force_baud_1 = 0;
		return count;
	}

	return count;
}

static DEVICE_ATTR(enable_irda, 0664, enable_irda_show, enable_irda_store);

static ssize_t enable_cir_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}
static ssize_t enable_cir_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port = &(msm_cir_port->uart);
	unsigned long flags;
	int cir_en, ret = 0;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (enable_uart_flag == 0)
		return count;
#endif
	sscanf(buf, "%d", &cir_en);
	if (cir_en != 1 && cir_en != 3 && cir_en != 0)
		D("%s: parameter invalid. cir_en = %d", __func__, cir_en);

	D("%s: (cir_enable_flg, cir_en) = (%d, %d)\n",
				__func__, cir_enable_flg, cir_en);

	ret = clk_set_rate(msm_cir_port->clk, 7372800);
	if (!ret) {
		clk_en(port, 1);
		D("%s(): irda Clock enabled for line(%d)\n", __func__, port->line);
	} else {
		D("%s(): Error: Setting the clock rate\n", __func__);
		return -EINVAL;
	}

	if (cir_en > 1) {
		D("%s(): Set IRDA mode\n", __func__);
		spin_lock_irqsave(&port->lock, flags);
		if (msm_cir_port->cir_learn_en)
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		gpio_direction_output(msm_cir_port->rst_pin, 0);
		msleep(10);

		if (msm_cir_port->cir_sir_switch)
			gpio_direction_output(msm_cir_port->cir_sir_switch, 1);
		ret = 3;
		ret |= (int)cir_en;
		msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);

		cir_enable_flg = PATH_IRDA;
		if (msm_cir_port->cir_set_path)
			msm_cir_port->cir_set_path(PATH_IRDA);
	}

	clk_en(port, 0);
	return count;
}
static DEVICE_ATTR(enable_cir, 0664, enable_cir_show, enable_cir_store);

static ssize_t enable_learn_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}

static ssize_t enable_learn_store(struct device *dev,
			struct device_attribute *attr, const char *buf,size_t count)
{
	int enable = 0;
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (enable_uart_flag == 0)
		return count;
#endif
	sscanf(buf, "%d", &enable);
	D("%s trigger cir learn, input = %d.\n",__func__, enable);
	if ((enable == 1) && (msm_cir_port->cir_learn_en)){
		gpio_direction_output(msm_cir_port->cir_learn_en, 1);
	} else if ((enable == 0) && (msm_cir_port->cir_learn_en)) {
		gpio_direction_output(msm_cir_port->cir_learn_en,0);
	}
	return count;
}
static DEVICE_ATTR(enable_learn, 0666, enable_learn_show, enable_learn_store);

static ssize_t enable_ls_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}

static ssize_t enable_ls_store(struct device *dev,
			struct device_attribute *attr, const char *buf,size_t count)
{
	int enable = 0;
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (enable_uart_flag == 0)
		return count;
#endif
	sscanf(buf, "%d", &enable);
	D("%s trigger cir ls, input = %d.\n",__func__, enable);
	if ((enable == 1) && (msm_cir_port->cir_ls_en)){
		gpio_direction_output(msm_cir_port->cir_ls_en, 1);
	} else if ((enable == 0) && (msm_cir_port->cir_ls_en)) {
		gpio_direction_output(msm_cir_port->cir_ls_en,0);
	}
	return count;
}
static DEVICE_ATTR(enable_ls, 0644, enable_ls_show, enable_ls_store);

static ssize_t reset_cir_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}
static ssize_t reset_cir_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	int reset;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (enable_uart_flag == 0)
		return count;
#endif
	sscanf(buf, "%d", &reset);
	D("%s trigger cir reset, input = %d.\n",
					__func__, reset);

	if((reset==1) && (msm_cir_port->cir_reset) ) {
		msm_cir_port->cir_reset();
	}
	if (reset == 1) {
		if (msm_cir_port->rst_pin) {
			
			gpio_direction_output(msm_cir_port->rst_pin, 0);
			msleep(2);
			
			gpio_direction_input(msm_cir_port->rst_pin);
			
		}
	}

	D("%s count = %ld.\n", __func__, count);
	return count;
}
static DEVICE_ATTR(reset_cir, 0600, reset_cir_show, reset_cir_store);

static struct cir_platform_data
		*msm_hsl_dt_to_pdata(struct platform_device *pdev)
{
	int ret;
	struct device_node *node = pdev->dev.of_node;
	struct cir_platform_data *pdata;
	struct property *prop;

	D("%s\n", __func__);
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("unable to allocate memory for platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	prop = of_find_property(node, "rst", NULL);
	if (prop) {
		pdata->rst_pin = of_get_named_gpio(node, "rst", 0);
	}
	prop = of_find_property(node, "cir_sir", NULL);
	if (prop) {
		pdata->cir_sir_switch = of_get_named_gpio(node, "cir_sir", 0);
	}
	prop = of_find_property(node, "cir_learn_en", NULL);
	if (prop) {
		pdata->cir_learn_en = of_get_named_gpio(node, "cir_learn_en", 0);
	}

	prop = of_find_property(node, "cir_ls_en", NULL);
	if (prop) {
		pdata->cir_ls_en = of_get_named_gpio(node, "cir_ls_en", 0);
	}

	D("%s: rst_pin=%d, cir_sir_switch=%d, cir_learn_en=%d, cir_ls_en=%d\n",
		__func__, pdata->rst_pin, pdata->cir_sir_switch, pdata->cir_learn_en, pdata->cir_ls_en);
	ret = of_property_read_u32(node, "qcom,config-gpio",
				&pdata->config_gpio);
	if (ret && ret != -EINVAL) {
		pr_err("Error with config_gpio property.\n");
		return ERR_PTR(ret);
	}

	if (pdata->config_gpio) {
		pdata->uart_tx_gpio = of_get_named_gpio(node,
					"qcom,tx-gpio", 0);
		if (pdata->uart_tx_gpio < 0)
				return ERR_PTR(pdata->uart_tx_gpio);

		pdata->uart_rx_gpio = of_get_named_gpio(node,
					"qcom,rx-gpio", 0);
		if (pdata->uart_rx_gpio < 0)
				return ERR_PTR(pdata->uart_rx_gpio);

		
		if (pdata->config_gpio == 4) {
			pdata->uart_cts_gpio = of_get_named_gpio(node,
						"qcom,cts-gpio", 0);
			if (pdata->uart_cts_gpio < 0)
				return ERR_PTR(pdata->uart_cts_gpio);

			pdata->uart_rfr_gpio = of_get_named_gpio(node,
						"qcom,rfr-gpio", 0);
			if (pdata->uart_rfr_gpio < 0)
				return ERR_PTR(pdata->uart_rfr_gpio);
		}
		D("%s: uart_tx_gpio=%d, uart_rx_gpio=%d\n",
			__func__, pdata->uart_tx_gpio, pdata->uart_rx_gpio);
	}

	pdata->use_pm = of_property_read_bool(node, "qcom,use-pm");

	pdata->cir_power_save = of_property_read_bool(node, "qcom,power-save");
	D("%s: cir_power_save=%d\n", __func__, pdata->cir_power_save);

#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	pdata->cir_dynamic_disable = of_property_read_bool(node, "htc,cir-dynamic-disable");
	D("%s: cir_dynamic_disable=%d\n", __func__, pdata->cir_dynamic_disable);
#endif
	return pdata;
}

static int gpio_request_cir(struct msm_hsl_port *msm_cir_port, int value)
{
	int ret = 0;

	D("%s:%d", __func__, value);
	if (value) {
		if (msm_cir_port->rst_pin) {
			ret = gpio_request(msm_cir_port->rst_pin,
					"cir_rst");
			if (unlikely(ret)) {
				pr_err("gpio request failed for:%d\n",
						msm_cir_port->rst_pin);
				goto exit_gpio_config;
			}
			gpio_direction_output(msm_cir_port->rst_pin, 0);
		}

		if (msm_cir_port->cir_sir_switch) {
			ret = gpio_request(msm_cir_port->cir_sir_switch,
					"cir_sir_switch");
			if (unlikely(ret)) {
				pr_err("gpio request failed for:%d\n",
						msm_cir_port->cir_sir_switch);
				goto exit_gpio_config;
			}
			gpio_direction_output(msm_cir_port->cir_sir_switch, 0);
		}

		if (msm_cir_port->cir_learn_en) {
			ret = gpio_request(msm_cir_port->cir_learn_en,
					"cir_learn_en");
			if (unlikely(ret)) {
				pr_err("gpio request failed for:%d\n",
						msm_cir_port->cir_learn_en);
				goto exit_gpio_config;
			}
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		}

		if (msm_cir_port->cir_ls_en) {
			ret = gpio_request(msm_cir_port->cir_ls_en,
					"cir_ls_en");
			if (unlikely(ret)) {
				pr_err("gpio request failed for:%d\n",
						msm_cir_port->cir_ls_en);
				goto exit_gpio_config;
			}
			gpio_direction_output(msm_cir_port->cir_ls_en, 1);
		}

		if (msm_cir_port->rst_pin)
			gpio_direction_input(msm_cir_port->rst_pin);
	} else {
		if (msm_cir_port->rst_pin) {
			gpio_free(msm_cir_port->rst_pin);
		}
		if (msm_cir_port->cir_sir_switch) {
			gpio_free(msm_cir_port->cir_sir_switch);
		}
		if (msm_cir_port->cir_learn_en) {
			gpio_free(msm_cir_port->cir_learn_en);
		}
		if (msm_cir_port->cir_ls_en) {
			gpio_free(msm_cir_port->cir_ls_en);
		}
	}

exit_gpio_config:
	return ret;
}

static atomic_t msm_serial_hsl_next_id = ATOMIC_INIT(0);

static int msm_serial_hsl_probe_cir(struct platform_device *pdev)
{
	struct msm_hsl_port *msm_hsl_port;
	struct resource *uart_resource;
	struct resource *gsbi_resource;
	struct uart_port *port;
	struct cir_platform_data *pdata;
	const struct of_device_id *match;
	u32 line;
	int ret;

	pr_info("[CIR]%s\n",__func__);
	if (pdev->id == -1)
		pdev->id = atomic_inc_return(&msm_serial_hsl_next_id) - 1;

	
	pdata = pdev->dev.platform_data;
	if (pdata)
		line = pdata->line;
	else
		line = pdev->id;

	
	if (pdev->dev.of_node) {
		dev_dbg(&pdev->dev, "device tree enabled\n");
		ret = of_alias_get_id(pdev->dev.of_node, "serial");
		if (ret >= 0)
			line = ret;

		pdata = msm_hsl_dt_to_pdata(pdev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);

		pdev->dev.platform_data = pdata;
	}

	if (unlikely(line < 0 || line >= UART_NR))
		return -ENXIO;

	pr_info("msm_serial_cir: detected port #%d (ttyHSL%d)\n", pdev->id, line);

	port = get_port_from_line(line);
	port->dev = &pdev->dev;
	port->uartclk = 115200;
	msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_port->rst_pin = pdata->rst_pin;
	msm_hsl_port->cir_sir_switch = pdata->cir_sir_switch;
	msm_hsl_port->cir_learn_en = pdata->cir_learn_en;
	msm_hsl_port->cir_ls_en = pdata->cir_ls_en;
	if (pdata->cir_power_save) {
		msm_hsl_port->cir_power = cir_ls_ctrl;
	}
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (pdata->cir_dynamic_disable) {
		msm_hsl_port->cir_dynamic_disable = pdata->cir_dynamic_disable;
	}
#endif

	htc_cir_port = msm_hsl_port;
	gpio_request_cir(htc_cir_port, 1);

	msm_hsl_get_pinctrl_configs(port);

	msm_hsl_port->cir_regulator = regulator_get(msm_hsl_port->uart.dev, "vddcir");
	if (unlikely(IS_ERR(msm_hsl_port->cir_regulator))) {
		ret = PTR_ERR(msm_hsl_port->cir_regulator);
		pr_err("Error getting regulator\n");
		return ret;
	}
	ret = regulator_enable(msm_hsl_port->cir_regulator);
	if (ret < 0) {
		regulator_put(msm_hsl_port->cir_regulator);
		pr_err("regulator_enable failed. rc=%d\n", ret);
		return ret;
	}

	msm_hsl_port->clk = clk_get(&pdev->dev, "core_clk");
	if (unlikely(IS_ERR(msm_hsl_port->clk))) {
		ret = PTR_ERR(msm_hsl_port->clk);
		if (ret != -EPROBE_DEFER)
			pr_err("Error getting clk\n");
		return ret;
	}

	msm_hsl_port->pclk = clk_get(&pdev->dev, "iface_clk");
	if (unlikely(IS_ERR(msm_hsl_port->pclk))) {
		ret = PTR_ERR(msm_hsl_port->pclk);
		if (ret == -EPROBE_DEFER) {
			clk_put(msm_hsl_port->clk);
			return ret;
		} else {
			msm_hsl_port->pclk = NULL;
		}
	}

	
	if (pdata && pdata->config_gpio == 4)
		msm_hsl_port->func_mode = UART_FOUR_WIRE;
	else
		msm_hsl_port->func_mode = UART_TWO_WIRE;

	match = of_match_device(msm_hsl_match_table, &pdev->dev);
	if (!match) {
		msm_hsl_port->ver_id = UARTDM_VERSION_11_13;
	} else {
		msm_hsl_port->ver_id = (unsigned long)match->data;
		msm_hsl_port->uart_type = BLSP_HSUART;

		msm_hsl_port->bus_scale_table = msm_bus_cl_get_pdata(pdev);
		if (!msm_hsl_port->bus_scale_table) {
			pr_err("Bus scaling is disabled\n");
		} else {
			msm_hsl_port->bus_perf_client =
				msm_bus_scale_register_client(
					msm_hsl_port->bus_scale_table);
			if (IS_ERR(&msm_hsl_port->bus_perf_client)) {
				pr_err("Bus client register failed.\n");
				ret = -EINVAL;
				goto err;
			}
		}
	}

	gsbi_resource =	platform_get_resource_byname(pdev,
						     IORESOURCE_MEM,
						     "gsbi_resource");
	if (!gsbi_resource) {
		gsbi_resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		D("%s () gsbi_resourc:port->line %d, ir\n", __func__, port->line);
	}

	if (gsbi_resource) {
pr_info("msm_serial_cir: get gsbi_uart_clk and gsbi_pclk\n");
		msm_hsl_port->uart_type = GSBI_HSUART;
	}
	else {
pr_info("msm_serial_cir: get uartdm_clk\n");
		msm_hsl_port->uart_type = LEGACY_HSUART;
	}


	uart_resource = platform_get_resource_byname(pdev,
						     IORESOURCE_MEM,
						     "uartdm_resource");
	if (!uart_resource)
		uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!uart_resource)) {
		pr_err("getting uartdm_resource failed\n");
		return -ENXIO;
	}
	port->mapbase = uart_resource->start;
pr_info("msm_serial_hsl: port[%d] mapbase:%llx\n", port->line, port->mapbase);

	port->irq = platform_get_irq(pdev, 0);
	if (unlikely((int)port->irq < 0)) {
		pr_err("getting irq failed\n");
		return -ENXIO;
	}

	device_set_wakeup_capable(&pdev->dev, 1);
	platform_set_drvdata(pdev, port);
	pm_runtime_enable(port->dev);
#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
	ret = device_create_file(&pdev->dev, &dev_attr_console);
	D("%s () device_create_file, port->line %d, ir\n", __func__, port->line);
	if (unlikely(ret))
		E("%s() Can't create console attribute\n", __func__);
#endif
	msm_hsl_debugfs_init(msm_hsl_port, get_line(pdev));
	mutex_init(&msm_hsl_port->clk_mutex);
	if (pdata && pdata->use_pm)
		wake_lock_init(&msm_hsl_port->port_open_wake_lock,
				WAKE_LOCK_SUSPEND,
				"msm_serial_hslite_port_open");

	if (msm_hsl_port->pclk) {
		clk_prepare_enable(msm_hsl_port->pclk);
		D("%s () clk_enable, port->line %d, ir\n", __func__, port->line);
	}
	ret = uart_add_one_port(&msm_hsl_uart_driver, port);
	if (msm_hsl_port->pclk) {
		D("%s () clk_disabl, port->line %d, ir\n", __func__, port->line);
		clk_disable_unprepare(msm_hsl_port->pclk);
	}

	D("%s ():port->line %d, ir\n", __func__, port->line);
		msm_hsl_port->irda_class = class_create(THIS_MODULE, "htc_irda");
	if (IS_ERR(msm_hsl_port->irda_class)) {
		ret = PTR_ERR(msm_hsl_port->irda_class);
		msm_hsl_port->irda_class = NULL;
		return -ENXIO;
	}
	msm_hsl_port->irda_dev = device_create(msm_hsl_port->irda_class,
				NULL, 0, "%s", "irda");
	if (unlikely(IS_ERR(msm_hsl_port->irda_dev))) {
		ret = PTR_ERR(msm_hsl_port->irda_dev);
		msm_hsl_port->irda_dev = NULL;
		goto err_create_ls_device;
	}
		
	ret = device_create_file(msm_hsl_port->irda_dev, &dev_attr_enable_irda);
	if (ret)
		goto err_create_ls_device_file;

	msm_hsl_port->cir_class = class_create(THIS_MODULE, "htc_cir");
	if (IS_ERR(msm_hsl_port->cir_class)) {
		ret = PTR_ERR(msm_hsl_port->cir_class);
		msm_hsl_port->cir_class = NULL;
		return -ENXIO;
	}
	msm_hsl_port->cir_dev = device_create(msm_hsl_port->cir_class,
				NULL, 0, "%s", "cir");
	if (unlikely(IS_ERR(msm_hsl_port->cir_dev))) {
		ret = PTR_ERR(msm_hsl_port->cir_dev);
		msm_hsl_port->cir_dev = NULL;
		goto err_create_ls_device;
	}
		
	ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_enable_cir);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_reset_cir);
	if (ret)
		goto err_create_ls_device_file;
	ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_enable_learn);
	if (ret)
		goto err_create_ls_device_file;
	ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_enable_ls);
	if (ret)
		goto err_create_ls_device_file;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (pdata->cir_dynamic_disable) {
		ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_enable_uart);
		if (ret)
			goto err_create_ls_device_file;
	}
#endif
err:
	return ret;

err_create_ls_device_file:
	device_unregister(msm_hsl_port->cir_dev);
err_create_ls_device:
	class_destroy(msm_hsl_port->cir_class);
	return ret;
}

static int msm_serial_hsl_remove_cir(struct platform_device *pdev)
{
	struct msm_hsl_port *msm_hsl_port = platform_get_drvdata(pdev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;
	struct uart_port *port;

	D("%s (): ir\n", __func__);
	port = get_port_from_line(get_line(pdev));
#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
	device_remove_file(&pdev->dev, &dev_attr_console);
#endif
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	if (pdata && pdata->use_pm)
		wake_lock_destroy(&msm_hsl_port->port_open_wake_lock);

	device_set_wakeup_capable(&pdev->dev, 0);
	gpio_request_cir(htc_cir_port, 0);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&msm_hsl_port->clk_mutex);
	uart_remove_one_port(&msm_hsl_uart_driver, port);

	clk_put(msm_hsl_port->pclk);
	clk_put(msm_hsl_port->clk);
	regulator_disable(msm_hsl_port->cir_regulator);
	regulator_put(msm_hsl_port->cir_regulator);
	debugfs_remove(msm_hsl_port->loopback_dir);

	return 0;
}

#ifdef CONFIG_PM
static int msm_serial_hsl_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (enable_uart_flag == 0)
		return 0;
#endif
	port = get_port_from_line(get_line(pdev));

	D("%s ():port->line %d, cir_enable_flg = %d\n",
				__func__, port->line, cir_enable_flg);
	if (port) {

		D("%s ():is_console:port->line %d, ir\n", __func__, port->line);
		if (is_console(port))
			msm_hsl_deinit_clock(port);

		uart_suspend_port(&msm_hsl_uart_driver, port);
		if (device_may_wakeup(dev))
			enable_irq_wake(port->irq);

		if (msm_cir_port->cir_set_path)
			msm_cir_port->cir_set_path(PATH_NONE);
	}

	if (msm_cir_port->cir_power)
		msm_cir_port->cir_power(0);

	return 0;
}

static int msm_serial_hsl_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port;
#if defined(CONFIG_SERIAL_CIR_DYNAMIC_DISABLE)
	if (enable_uart_flag == 0)
		return 0;
#endif
	port = get_port_from_line(get_line(pdev));

	D("%s ():port->line %d, cir_enable_flg = %d\n",
				__func__, port->line, cir_enable_flg);
	if (msm_cir_port->cir_power)
		msm_cir_port->cir_power(1);

	if (port) {
		if (msm_cir_port->cir_set_path)
			msm_cir_port->cir_set_path(cir_enable_flg);

		uart_resume_port(&msm_hsl_uart_driver, port);
		if (device_may_wakeup(dev))
			disable_irq_wake(port->irq);

		if (is_console(port))
			msm_hsl_init_clock(port);
	}

	return 0;
}
#else
#define msm_serial_hsl_suspend NULL
#define msm_serial_hsl_resume NULL
#endif

static int msm_hsl_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(get_line(pdev));

	dev_dbg(dev, "pm_runtime: suspending\n");
	msm_hsl_deinit_clock(port);
	D("%s ():port->line %d, ir\n", __func__, port->line);
	return 0;
}

static int msm_hsl_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(get_line(pdev));

	dev_dbg(dev, "pm_runtime: resuming\n");
	msm_hsl_init_clock(port);
	D("%s ():port->line %d, ir\n", __func__, port->line);
	return 0;
}

static struct dev_pm_ops msm_hsl_dev_pm_ops = {
	.suspend = msm_serial_hsl_suspend,
	.resume = msm_serial_hsl_resume,
	.runtime_suspend = msm_hsl_runtime_suspend,
	.runtime_resume = msm_hsl_runtime_resume,
};

static struct platform_driver msm_hsl_platform_driver = {
	.probe = msm_serial_hsl_probe_cir,
	.remove = msm_serial_hsl_remove_cir,
	.driver = {
		.name = "msm_serial_cir",
		.owner = THIS_MODULE,
		.pm = &msm_hsl_dev_pm_ops,
		.of_match_table = msm_hsl_match_table,
	},
};

static int __init msm_serial_hsl_init_cir(void)
{
	int ret;

	ret = uart_register_driver(&msm_hsl_uart_driver);
	if (unlikely(ret))
		return ret;

	debug_base = debugfs_create_dir("msm_serial_cir", NULL);
	if (IS_ERR_OR_NULL(debug_base))
		E("%s() Cannot create debugfs dir\n", __func__);

	ret = platform_driver_register(&msm_hsl_platform_driver);
	if (unlikely(ret))
		uart_unregister_driver(&msm_hsl_uart_driver);

	D("%s(): driver initialized\n", __func__);

	return ret;
}

static void __exit msm_serial_hsl_exit_cir(void)
{
	debugfs_remove_recursive(debug_base);
#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
	unregister_console(&msm_hsl_console);
#endif
	platform_driver_unregister(&msm_hsl_platform_driver);
	uart_unregister_driver(&msm_hsl_uart_driver);
	D("%s(): \n", __func__);
}

module_init(msm_serial_hsl_init_cir);
module_exit(msm_serial_hsl_exit_cir);

MODULE_DESCRIPTION("Driver for msm HSUART serial device");
MODULE_LICENSE("GPL v2");
