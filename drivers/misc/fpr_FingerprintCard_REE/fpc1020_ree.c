/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/types.h>

#include <linux/ioctl.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <soc/qcom/scm.h>

#include <linux/wakelock.h>
#if 1 //HTC
#include "fpc1020_common.h"
#endif

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250
#define PWR_ON_STEP_SLEEP 100
#define PWR_ON_STEP_RANGE1 100
#define PWR_ON_STEP_RANGE2 900
#define FPC_TTW_HOLD_TIME 1000
#define NUM_PARAMS_REG_ENABLE_SET 2

#define HTC_LDO_CONTROL 1

#if 1   //HTC: REE only needs following pctl_names
static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_irq_active",
};
#else
static const char * const pctl_names[] = {
	"fpc1020_spi_active",
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_cs_low",
	"fpc1020_cs_high",
	"fpc1020_cs_active",
	"fpc1020_irq_active",
};
#endif

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "vdd_ana", 1800000UL, 1800000UL, 6000, },
	{ "vcc_spi", 1800000UL, 1800000UL, 10, },
	{ "vdd_io", 1800000UL, 1800000UL, 6000, },
};

struct fpc1020_data {
	struct device *dev;
	struct spi_device *spi;
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	struct clk *iface_clk;
	struct clk *core_clk;
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
#if HTC_LDO_CONTROL
	struct regulator *regulator_ldo;
	bool use_regulator_ldo;
#endif

	struct wake_lock ttw_wl;
	int irq_gpio;
	int cs0_gpio;
	int cs1_gpio;
	int rst_gpio;
	int qup_id;
	struct mutex lock;
	bool prepared;
	bool wakeup_enabled;
	bool clocks_enabled;
	bool clocks_suspended;

    u32 spi_freq_khz;
	int use_fpc2050;
	struct class           *class;
	struct device          *device;    
	struct cdev            cdev;
	dev_t                  devno;
    
};

/* -------------------------------------------------------------------- */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	const bool target_little_endian = true;
#else
	#err BE target not tested!
	const bool target_little_endian = false;
#endif

static int _fpc1020_reg_access(struct fpc1020_data *fpc1020,
		      fpc1020_reg_access_t *reg_data)
{
	int error = 0;

	u8 temp_buffer[FPC1020_REG_MAX_SIZE];

	struct spi_message msg;

	struct spi_transfer cmd = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)fpc1020->spi_freq_khz * 1000u,
		.tx_buf = &(reg_data->reg),
		.rx_buf = NULL,
		.len    = 1 + FPC1020_REG_ACCESS_DUMMY_BYTES(reg_data->reg),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	struct spi_transfer data = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)fpc1020->spi_freq_khz * 1000u,
		.tx_buf = (reg_data->write)  ? temp_buffer : NULL,
		.rx_buf = (!reg_data->write) ? temp_buffer : NULL,
		.len    = reg_data->reg_size,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	if (reg_data->reg_size > sizeof(temp_buffer)) {
		dev_err(&fpc1020->spi->dev,
			"%s : illegal register size\n",
			__func__);

		error = -ENOMEM;
		goto out;
	}

#ifdef MANUAL_CS
	if (gpio_is_valid(fpc1020->cs_gpio))
		gpio_set_value(fpc1020->cs_gpio, 0);
#endif

	if (reg_data->write) {
		if (target_little_endian) {
			int src = 0;
			int dst = reg_data->reg_size - 1;

			while (src < reg_data->reg_size) {
				temp_buffer[dst] = reg_data->dataptr[src];
				src++;
				dst--;
			}
		} else {
			memcpy(temp_buffer,
				reg_data->dataptr,
				reg_data->reg_size);
		}
	}

	spi_message_init(&msg);
	spi_message_add_tail(&cmd,  &msg);
	spi_message_add_tail(&data, &msg);

	error = spi_sync(fpc1020->spi, &msg);

	if (error)
		dev_err(&fpc1020->spi->dev, "%s : spi_sync failed.\n", __func__);

	if (!reg_data->write) {
		if (target_little_endian) {
			int src = reg_data->reg_size - 1;
			int dst = 0;

			while (dst < reg_data->reg_size) {
				reg_data->dataptr[dst] = temp_buffer[src];
				src--;
				dst++;
			}
		} else {
			memcpy(reg_data->dataptr,
				temp_buffer,
				reg_data->reg_size);
		}
	}

#ifdef MANUAL_CS
	if (gpio_is_valid(fpc1020->cs_gpio))
		gpio_set_value(fpc1020->cs_gpio, 1);
#endif

	//dev_dbg(&fpc1020->spi->dev,
	printk(
		"%s %s 0x%x/%dd (%d bytes) %x %x %x %x : %x %x %x %x\n",
		 __func__,
		(reg_data->write) ? "WRITE" : "READ",
		reg_data->reg,
		reg_data->reg,
		reg_data->reg_size,
		(reg_data->reg_size > 0) ? temp_buffer[0] : 0,
		(reg_data->reg_size > 1) ? temp_buffer[1] : 0,
		(reg_data->reg_size > 2) ? temp_buffer[2] : 0,
		(reg_data->reg_size > 3) ? temp_buffer[3] : 0,
		(reg_data->reg_size > 4) ? temp_buffer[4] : 0,
		(reg_data->reg_size > 5) ? temp_buffer[5] : 0,
		(reg_data->reg_size > 6) ? temp_buffer[6] : 0,
		(reg_data->reg_size > 7) ? temp_buffer[7] : 0);

out:
	return error;
}

static int _fpc1020_check_irq_after_reset(struct fpc1020_data *fpc1020)
{
	int error = 0;
	u8 irq_status;

	fpc1020_reg_access_t reg_clear = {
		.reg = FPC102X_REG_READ_INTERRUPT_WITH_CLEAR,
		.write = false,
		.reg_size = FPC1020_REG_SIZE(
					FPC102X_REG_READ_INTERRUPT_WITH_CLEAR),
		.dataptr = &irq_status
	};

	error = _fpc1020_reg_access(fpc1020, &reg_clear);

	if(error < 0)
		return error;

	if (irq_status != FPC_1020_IRQ_REG_BITS_REBOOT) {
		dev_err(&fpc1020->spi->dev,
			"IRQ register, expected 0x%x, got 0x%x.\n",
			FPC_1020_IRQ_REG_BITS_REBOOT,
			irq_status);

		error = -EIO;
	}

	return (error < 0) ? error : irq_status;
}

static int _fpc1020_read_irq(struct fpc1020_data *fpc1020, bool clear_irq)
{
	int error = 0;
	u8 irq_status;
	fpc1020_reg_access_t reg_read = {
		.reg = FPC102X_REG_READ_INTERRUPT,
		.write = false,
		.reg_size = FPC1020_REG_SIZE(FPC102X_REG_READ_INTERRUPT),
		.dataptr = &irq_status
	};

	fpc1020_reg_access_t reg_clear = {
		.reg = FPC102X_REG_READ_INTERRUPT_WITH_CLEAR,
		.write = false,
		.reg_size = FPC1020_REG_SIZE(
					FPC102X_REG_READ_INTERRUPT_WITH_CLEAR),
		.dataptr = &irq_status
	};

	error = _fpc1020_reg_access(fpc1020,
				(clear_irq) ? &reg_clear : &reg_read);

	if(error < 0)
		return error;

	if (irq_status == FPC_1020_IRQ_REG_BITS_REBOOT) {

		dev_err(&fpc1020->spi->dev,
			"%s: unexpected irq_status = 0x%x\n"
			, __func__, irq_status);

		error = -EIO;
	}

	return (error < 0) ? error : irq_status;
}

static int _fpc1020_cmd(struct fpc1020_data *fpc1020,
			fpc1020_cmd_t cmd)
{
	int error = 0;
	struct spi_message msg;

	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)fpc1020->spi_freq_khz * 1000u,
		.tx_buf = &cmd,
		.rx_buf = NULL,
		.len    = 1,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

#ifdef MANUAL_CS
	if (gpio_is_valid(fpc1020->cs_gpio))
		gpio_set_value(fpc1020->cs_gpio, 0);
#endif

	spi_message_init(&msg);
	spi_message_add_tail(&t,  &msg);

	error = spi_sync(fpc1020->spi, &msg);

	if (error)
		dev_err(&fpc1020->spi->dev, "spi_sync failed.\n");

#ifdef MANUAL_CS
	if (gpio_is_valid(fpc1020->cs_gpio))
		gpio_set_value(fpc1020->cs_gpio, 1);
#endif

    dev_info(&fpc1020->spi->dev, "%s 0x%x/%dd\n", __func__, cmd, cmd);

	return error;
}

int _fpc1020_spi_reset(struct fpc1020_data *fpc1020)
{
    int error = 0;
    int irq_gpio;
    int retry = 10;

    dev_info(&fpc1020->spi->dev, "%s\n", __func__);

    error = _fpc1020_cmd(fpc1020,
        FPC1020_CMD_SOFT_RESET);

    if (error < 0) {
        dev_err(&fpc1020->spi->dev,
            "%s FPC1020_CMD_SOFT_RESET fail, error:%d\n",
            __func__, error);
        goto out;
    }

    while (retry > 0) {
        retry--;

        irq_gpio = gpio_get_value(fpc1020->irq_gpio);
        if (irq_gpio) {
            dev_info(&fpc1020->spi->dev, "%s OK !\n", __func__);
            break;
        } else {
            dev_info(&fpc1020->spi->dev, "%s wait IRQ HIGH ...\n", __func__);
            msleep(1);
        }
    }

out:    
    return error;
}

int _fpc1020_check_hw_id(struct fpc1020_data *fpc1020)
{
    int error = 0;
    u16 hardware_id;
    fpc1020_reg_access_t reg;

    FPC1020_MK_REG_READ(reg, FPC102X_REG_HWID, &hardware_id);
    error = _fpc1020_reg_access(fpc1020, &reg);
    dev_info(&fpc1020->spi->dev, "%s(%d) error:%d hardware_id:0x%x\n", __func__, __LINE__, error, hardware_id);

    return error;
}

static int fpc1020_spi_test(struct fpc1020_data *fpc1020)
{
    int error = 0;

    dev_info(&fpc1020->spi->dev, "%s+\n", __func__);

    _fpc1020_spi_reset(fpc1020);

    error = _fpc1020_check_irq_after_reset(fpc1020);
    if (error < 0) {
        dev_err(&fpc1020->spi->dev, "_fpc1020_check_irq_after_reset failed:%d\n", error);
        goto out;
    }

    error = (gpio_get_value(fpc1020->irq_gpio) != 0) ? -EIO : 0;
    if (error)
        dev_err(&fpc1020->spi->dev, "IRQ pin, not low after clear.\n");

    error = _fpc1020_read_irq(fpc1020, true);
    if (error != 0) {
        dev_err(&fpc1020->spi->dev,
            "IRQ register, expected 0x%x, got 0x%x.\n",
            0,
            (u8)error);

        error = -EIO;
        goto out;
    }

    error = _fpc1020_check_hw_id(fpc1020);
    if (error < 0) {
        dev_err(&fpc1020->spi->dev, "_fpc1020_check_hw_id failed:%d\n", error);
        goto out;
    }

out:

    dev_info(&fpc1020->spi->dev, "%s- error:%d\n", __func__, error);
    return error;
}
/* -------------------------------------------------------------------- */

static int vreg_setup(struct fpc1020_data *fpc1020, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(fpc1020->vreg); i++) {
		const char *n = vreg_conf[i].name;
		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
found:
	vreg = fpc1020->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				dev_err(dev, "Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fpc1020->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			fpc1020->vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

/**
 * Prepare or unprepare the SPI master that we are soon to transfer something
 * over SPI.
 *
 * Please see Linux Kernel manual for SPI master methods for more information.
 *
 * @see Linux SPI master methods
 */
static int spi_set_fabric(struct fpc1020_data *fpc1020, bool active)
{
#if 1 //HTC: REE doesn't need it
    return 0;
#else
	struct spi_master *master = fpc1020->spi->master;
	int rc = active ?
		master->prepare_transfer_hardware(master) :
		master->unprepare_transfer_hardware(master);
	if (rc)
		dev_err(fpc1020->dev, "%s: rc %d\n", __func__, rc);
	else
		dev_dbg(fpc1020->dev, "%s: %d ok\n", __func__, active);
	return rc;
#endif
}

/**
 * Changes ownership of SPI transfers from TEE to REE side or vice versa.
 *
 * SPI transfers can be owned only by one of TEE or REE side at any given time.
 * This can be changed dynamically if needed but of course that needs support
 * from underlaying layers. This function will transfer the ownership from REE
 * to TEE or vice versa.
 *
 * If REE side uses the SPI master when TEE owns the pipe or vice versa the
 * system will most likely crash dump.
 *
 * If available this should be set at boot time to eg. TEE side and not
 * dynamically as that will increase the security of the system. This however
 * implies that there are no other SPI slaves connected that should be handled
 * from REE side.
 *
 * @see SET_PIPE_OWNERSHIP
 */
static int set_pipe_ownership(struct fpc1020_data *fpc1020, bool to_tz)
{
#ifdef SET_PIPE_OWNERSHIP
#err not test
	int rc;
	const u32 TZ_BLSP_MODIFY_OWNERSHIP_ID = 3;
	const u32 TZBSP_APSS_ID = 1;
	const u32 TZBSP_TZ_ID = 3;
	struct scm_desc desc = {
		.arginfo = SCM_ARGS(2),
		.args[0] = fpc1020->qup_id,
		.args[1] = to_tz ? TZBSP_TZ_ID : TZBSP_APSS_ID,
	};

	rc = scm_call2(SCM_SIP_FNID(SCM_SVC_TZ, TZ_BLSP_MODIFY_OWNERSHIP_ID),
		&desc);

	if (rc || desc.ret[0]) {
		dev_err(fpc1020->dev, "%s: scm_call2: responce %llu, rc %d\n",
				__func__, desc.ret[0], rc);
		return -EINVAL;
	}
	dev_dbg(fpc1020->dev, "%s: scm_call2: ok\n", __func__);
#endif
	return 0;
}

static int set_clks(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;
#if 0 //HTC: REE doesn't need it
	mutex_lock(&fpc1020->lock);

	if (enable == fpc1020->clocks_enabled)
		goto out;

	if (enable) {
		rc = clk_set_rate(fpc1020->core_clk,
				fpc1020->spi->max_speed_hz);
		if (rc) {
			dev_err(fpc1020->dev,
					"%s: Error setting clk_rate: %u, %d\n",
					__func__, fpc1020->spi->max_speed_hz,
					rc);
			goto out;
		}
		rc = clk_prepare_enable(fpc1020->core_clk);
		if (rc) {
			dev_err(fpc1020->dev,
					"%s: Error enabling core clk: %d\n",
					__func__, rc);
			goto out;
		}

		rc = clk_prepare_enable(fpc1020->iface_clk);
		if (rc) {
			dev_err(fpc1020->dev,
					"%s: Error enabling iface clk: %d\n",
					__func__, rc);
			clk_disable_unprepare(fpc1020->core_clk);
			goto out;
		}
		dev_dbg(fpc1020->dev, "%s ok. clk rate %u hz\n", __func__,
				fpc1020->spi->max_speed_hz);

		fpc1020->clocks_enabled = true;
	} else {
		clk_disable_unprepare(fpc1020->iface_clk);
		clk_disable_unprepare(fpc1020->core_clk);
		fpc1020->clocks_enabled = false;
	}

out:
	mutex_unlock(&fpc1020->lock);
#endif    

	return rc;
}


static ssize_t clk_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	return set_clks(fpc1020, (*buf == '1')) ? : count;
}

static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;
	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_info(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

/**
 * sysfs node handler to support dynamic change of SPI transfers' ownership
 * between TEE and REE side.
 *
 * An owner in this context is REE or TEE.
 *
 * @see set_pipe_ownership
 * @see SET_PIPE_OWNERSHIP
 */
static ssize_t spi_owner_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;
	bool to_tz;

	if (!strncmp(buf, "tz", strlen("tz")))
		to_tz = true;
	else if (!strncmp(buf, "app", strlen("app")))
		to_tz = false;
	else
		return -EINVAL;

	rc = set_pipe_ownership(fpc1020, to_tz);
	return rc ? rc : count;
}
static DEVICE_ATTR(spi_owner, S_IWUSR, NULL, spi_owner_set);

static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc = select_pin_ctl(fpc1020, buf);
	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

/**
 * Will indicate to the SPI driver that a message is soon to be delivered over
 * it.
 *
 * Exactly what fabric resources are requested is up to the SPI device driver.
 *
 * @see spi_set_fabric
 */
static ssize_t fabric_vote_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc = spi_set_fabric(fpc1020, *buf == '1');
	return rc ? rc : count;
}
static DEVICE_ATTR(fabric_vote, S_IWUSR, NULL, fabric_vote_set);

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc;
	bool enable;

	if (NUM_PARAMS_REG_ENABLE_SET != sscanf(buf, "%15[^,],%c", name, &op))
		return -EINVAL;
	if (op == 'e')
		enable = true;
	else if (op == 'd')
		enable = false;
	else
		return -EINVAL;
	rc = vreg_setup(fpc1020, name, enable);
	return rc ? rc : count;
}
static DEVICE_ATTR(regulator_enable, S_IWUSR, NULL, regulator_enable_set);

static ssize_t spi_bus_lock_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "lock", strlen("lock")))
		spi_bus_lock(fpc1020->spi->master);
	else if (!strncmp(buf, "unlock", strlen("unlock")))
		spi_bus_unlock(fpc1020->spi->master);
	else
		return -EINVAL;
	return count;
}
static DEVICE_ATTR(bus_lock, S_IWUSR, NULL, spi_bus_lock_set);

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;

	int rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);
exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(fpc1020);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);


static ssize_t fpc_spi_test_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int rc = 0;
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    rc = fpc1020_spi_test(fpc1020);
    return rc ? rc : count;
}
static DEVICE_ATTR(fpc_spi_test, S_IWUSR, NULL, fpc_spi_test_set);

/**
 * Will setup clocks, GPIOs, and regulators to correctly initialize the touch
 * sensor to be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, SPI platform clocks, and reset line, all to set
 * the sensor in a correct power on or off state "electrical" wise.
 *
 * @see  spi_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
	int rc;
    struct device *dev = fpc1020->dev;

	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
        dev_info(dev, "%s(%d): fpc1020_spi_setup enable:%d\n", __func__, __LINE__, enable);        
		//spi_bus_lock(fpc1020->spi->master);
		fpc1020->prepared = true;
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");
#if 0
		rc = vreg_setup(fpc1020, "vcc_spi", true);
		if (rc)
			goto exit;

		rc = vreg_setup(fpc1020, "vdd_io", true);
		if (rc)
			goto exit_1;

		rc = vreg_setup(fpc1020, "vdd_ana", true);
		if (rc)
			goto exit_2;
#endif

#if HTC_LDO_CONTROL
        if (fpc1020->use_regulator_ldo) {
            rc = regulator_enable(fpc1020->regulator_ldo);
            if (rc) {
                dev_err(dev, "%s(%d) Fail to enable regulator_ldo regulator\n", __func__, __LINE__);
            }else{
                dev_info(dev, "%s(%d) Enable 1V8 Power Pin\n", __func__, __LINE__);
            }
        }
#endif
		usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE2);

		rc = spi_set_fabric(fpc1020, true);
		if (rc)
			goto exit_3;
        else {
            dev_info(dev, "%s(%d) spi_set_fabric OK\n", __func__, __LINE__);
        }

		(void)select_pin_ctl(fpc1020, "fpc1020_cs_high");
		(void)select_pin_ctl(fpc1020, "fpc1020_reset_active");
		usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE1);
		(void)select_pin_ctl(fpc1020, "fpc1020_cs_active");

		rc = set_pipe_ownership(fpc1020, true);
		if (rc)
			goto exit_4;
	} else if (!enable && fpc1020->prepared) {
        dev_info(dev, "%s(%d): fpc1020_spi_setup enable:%d\n", __func__, __LINE__, enable);
		rc = 0;
		(void)set_pipe_ownership(fpc1020, false);
exit_4:
		(void)spi_set_fabric(fpc1020, false);
exit_3:
		(void)select_pin_ctl(fpc1020, "fpc1020_cs_high");
		(void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");
		usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE2);
#if 0
		(void)vreg_setup(fpc1020, "vdd_ana", false);
exit_2:
		(void)vreg_setup(fpc1020, "vdd_io", false);
exit_1:
		(void)vreg_setup(fpc1020, "vcc_spi", false);
exit:
#endif
		(void)select_pin_ctl(fpc1020, "fpc1020_cs_low");

		fpc1020->prepared = false;
		//spi_bus_unlock(fpc1020->spi->master);
	} else {
		rc = 0;
	}
	mutex_unlock(&fpc1020->lock);
	return rc;
}

static ssize_t spi_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = device_prepare(fpc1020, false);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, spi_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->wakeup_enabled = false;
		smp_wmb();
	} else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUGO, NULL, wakeup_enable_set);


/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
	struct device_attribute *attribute,
	char *buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}


/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
	struct device_attribute *attribute,
	const char *buffer, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	return count;
}

static DEVICE_ATTR(irq, S_IRUGO | S_IWUGO, irq_get, irq_ack);

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_spi_owner.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_fabric_vote.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_bus_lock.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_fpc_spi_test.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;
	dev_info(fpc1020->dev, "%s (%d)\n", __func__, gpio_get_value(fpc1020->irq_gpio));

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();

	if (fpc1020->wakeup_enabled) {
		wake_lock_timeout(&fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
	const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);
	return 0;
}

static int fpc1020_spi_setup(struct fpc1020_data *fpc1020)
{
	int error = 0;

	printk(KERN_INFO "%s\n", __func__);

    dev_info(&fpc1020->spi->dev, "SPI frequency : %d kHz.\n", fpc1020->spi_freq_khz);    

	fpc1020->spi->max_speed_hz = fpc1020->spi_freq_khz * 1000;
	fpc1020->spi->mode = SPI_MODE_0;
	fpc1020->spi->bits_per_word = 8;
	fpc1020->spi->chip_select = 0;

	error = spi_setup(fpc1020->spi);

	if (error) {
		dev_err(&fpc1020->spi->dev, "spi_setup failed\n");
		goto out_err;
	}

#ifdef MANUAL_CS
	if (gpio_is_valid(pdata->cs_gpio)) {

		dev_info(&fpc1020->spi->dev,
			"Assign SPI.CS -> GPIO%d\n",
			pdata->cs_gpio);

		error = gpio_request(pdata->cs_gpio, "fpc1020_cs");
		if (error) {
			dev_err(&fpc1020->spi->dev,
				"gpio_request (cs) failed.\n");

			goto out_err;
		}

		fpc1020->cs_gpio = pdata->cs_gpio;

		error = gpio_direction_output(fpc1020->cs_gpio, 1);
		if (error) {
			dev_err(&fpc1020->spi->dev,
				"gpio_direction_output(cs) failed.\n");
			goto out_err;
		}
	} else {
		error = -EINVAL;
	}
#endif

out_err:
	return error;
}

/* -------------------------------------------------------------------- */
#define FPC1020_CLASS_NAME                      "fpsensor"
static int fpc1020_create_class(struct fpc1020_data *fpc1020)
{
	int error = 0;

	dev_info(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->class = class_create(THIS_MODULE, FPC1020_CLASS_NAME);

	if (IS_ERR(fpc1020->class)) {
		dev_err(&fpc1020->spi->dev, "failed to create class.\n");
		error = PTR_ERR(fpc1020->class);
	}

	return error;
}


/* -------------------------------------------------------------------- */
static int fpc1020_device_count = 0;
#define FPC1020_DEV_NAME                        "fpc1020"
static int fpc1020_create_device(struct fpc1020_data *fpc1020)
{
	int error = 0;

	dev_info(&fpc1020->spi->dev, "%s\n", __func__);

	if (FPC1020_MAJOR > 0) {
		fpc1020->devno = MKDEV(FPC1020_MAJOR, fpc1020_device_count++);

		error = register_chrdev_region(fpc1020->devno,
						1,
						FPC1020_DEV_NAME);
	} else {
		error = alloc_chrdev_region(&fpc1020->devno,
					fpc1020_device_count++,
					1,
					FPC1020_DEV_NAME);
	}

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
				"%s: FAILED %d.\n", __func__, error);
		goto out;

	} else {
		dev_info(&fpc1020->spi->dev, "%s: major=%d, minor=%d\n",
						__func__,
						MAJOR(fpc1020->devno),
						MINOR(fpc1020->devno));
	}

	fpc1020->device = device_create(fpc1020->class, NULL, fpc1020->devno,
						NULL, "%s", FPC1020_DEV_NAME);

	if (IS_ERR(fpc1020->device)) {
		dev_err(&fpc1020->spi->dev, "device_create failed.\n");
		error = PTR_ERR(fpc1020->device);
	}
out:
	return error;
}

/* -------------------------------------------------------------------- */
static int fpc1020_open(struct inode *inode, struct file *file)

{
    struct fpc1020_data *fpc1020;

	printk(KERN_INFO "%s\n", __func__);

	fpc1020 = container_of(inode->i_cdev, struct fpc1020_data, cdev);

    dev_info(fpc1020->device, "%s\n", __func__);
    dev_info(&fpc1020->spi->dev, "%s\n", __func__);    

	file->private_data = fpc1020;

	return 0;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos)
{
	printk(KERN_INFO "%s\n", __func__);

	return -ENOTTY;
}


/* -------------------------------------------------------------------- */
static ssize_t fpc1020_read(struct file *file, char *buff,
				size_t count, loff_t *ppos)
{
	printk(KERN_INFO "%s\n", __func__);

	return -ENOTTY;
}


/* -------------------------------------------------------------------- */
static int fpc1020_release(struct inode *inode, struct file *file)
{
    struct fpc1020_data *fpc1020 = file->private_data;
	int status = 0;

    dev_info(fpc1020->device, "%s\n", __func__);

	printk(KERN_INFO "%s\n", __func__);

	return status;
}


/* -------------------------------------------------------------------- */
static unsigned int fpc1020_poll(struct file *file, poll_table *wait)
{
	printk(KERN_INFO "%s\n", __func__);

	return -ENOTTY;
}

#define FPC1020_BUF_MAX_SIZE   (16*1024)
u8 fpc_buffer[FPC1020_BUF_MAX_SIZE];
static long fpc1020_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct fpc1020_data *fpc1020 = file->private_data;    
	struct spi_device	*spi = fpc1020->spi;
	int			retval = 0;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;


	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC) {
        dev_err(&spi->dev, "type %02x\n", _IOC_TYPE(cmd));
		return -ENOTTY;
    }

	switch (cmd) {
	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

        if (_IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(1))) {
            #if 0
            dev_info(&spi->dev, "ioc:%p n_ioc:%u\n", ioc, n_ioc);
    		dev_info(&spi->dev,
    			"  xfer len(%u) rx(%p) tx(%p) %uHz\n",
    			ioc->len,
    			(void *)ioc->rx_buf,
    			(void *)ioc->tx_buf,
    			fpc1020->spi_freq_khz * 1000u);
            #endif

            if (ioc->tx_buf && ioc->len <= FPC1020_BUF_MAX_SIZE) {
            	int error = 0;
                u8 *k_tx_buffer = fpc_buffer;
                u8 *k_rx_buffer = fpc_buffer;
            	struct spi_message msg;
            	struct spi_transfer t = {
            		.cs_change = ioc->cs_change,
            		.delay_usecs = ioc->delay_usecs,
            		.speed_hz = (u32)fpc1020->spi_freq_khz * 1000u,
            		.tx_buf = (ioc->tx_buf) ? k_tx_buffer : NULL,
            		.rx_buf = (ioc->rx_buf) ? k_rx_buffer : NULL,
            		.len    = ioc->len,
            		.tx_dma = 0,
            		.rx_dma = 0,
            		.bits_per_word = ioc->bits_per_word,
            	};

                error = copy_from_user(k_tx_buffer, (const u8 __user *)(uintptr_t) ioc->tx_buf, ioc->len);
                #if 0
                dev_info(&spi->dev, "k_tx_buffer:0x%x 0x%x, error:%d\n", k_tx_buffer[0], k_tx_buffer[1], error);                
                #endif
#ifdef MANUAL_CS
            	if (gpio_is_valid(fpc1020->cs_gpio))
            		gpio_set_value(fpc1020->cs_gpio, 0);
#endif


            	spi_message_init(&msg);
            	spi_message_add_tail(&t,  &msg);
            	error = spi_sync(fpc1020->spi, &msg);
            	if (error)
            		dev_err(&fpc1020->spi->dev, "spi_sync failed.\n");

                #if 0
                dev_info(&spi->dev, "k_rx_buffer:0x%x 0x%x, irq:%d\n", k_rx_buffer[0], k_rx_buffer[1], gpio_get_value(fpc1020->irq_gpio));
                #endif

                error = __copy_to_user((u8 __user *)(uintptr_t) ioc->rx_buf, k_rx_buffer, ioc->len);

#ifdef MANUAL_CS
            	if (gpio_is_valid(fpc1020->cs_gpio))
            		gpio_set_value(fpc1020->cs_gpio, 1);
#endif

            	//return error;
            }
            else {
                dev_err(&fpc1020->spi->dev, "Invalid ioc->len:%u !!!\n", ioc->len);
                retval = -ENOMEM;
            }

        }        

		/* translate to spi_message, execute */
		//retval = spidev_message(spidev, ioc, n_ioc);
		
		kfree(ioc);
		break;
	}


	return retval;
}

static long
fpc1020_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return fpc1020_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}


static const struct file_operations fpc1020_fops = {
	.owner          = THIS_MODULE,
	.open           = fpc1020_open,
	.write          = fpc1020_write,
	.read           = fpc1020_read,
	.release        = fpc1020_release,
	.poll           = fpc1020_poll,
	.unlocked_ioctl = fpc1020_ioctl,
	.compat_ioctl   = fpc1020_compat_ioctl,
	
};

static int fpc1020_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int rc = 0;
	size_t i;
	int irqf;
	struct device_node *np = dev->of_node;
	u32 val;

	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);

	dev_err(dev, "%s\n", __func__);

	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);
	fpc1020->spi = spi;

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;
#if 0
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_cs0",
			&fpc1020->cs0_gpio);
	if (rc)
		goto exit;

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_cs1",
			&fpc1020->cs1_gpio);
	if (rc)
		goto exit;
#endif
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_rst",
			&fpc1020->rst_gpio);
	if (rc)
		goto exit;
#if 0 //HTC: REE doesn't need it
	fpc1020->iface_clk = clk_get(dev, "iface_clk");
	if (IS_ERR(fpc1020->iface_clk)) {
		dev_err(dev, "%s: Failed to get iface_clk\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	fpc1020->core_clk = clk_get(dev, "core_clk");
	if (IS_ERR(fpc1020->core_clk)) {
		dev_err(dev, "%s: Failed to get core_clk\n", __func__);
		rc = -EINVAL;
		goto exit;
	}
#endif
	rc = of_property_read_u32(np, "spi-qup-id", &val);
	if (rc < 0) {
		dev_err(dev, "spi-qup-id not found\n");
		goto exit;
	}
	fpc1020->qup_id = val;
	dev_info(dev, "spi-qup-id %d\n", fpc1020->qup_id);

	rc = of_property_read_u32(np, "fpc,use_fpc2050", &val);
	if (rc < 0) {
		dev_err(dev, "fpc,use_fpc2050 not found\n");
		goto exit;
	}
	fpc1020->use_fpc2050 = val;
	dev_info(dev, "use_fpc2050 %d\n", fpc1020->use_fpc2050);

	rc = of_property_read_u32(np, "fpc,spi-freq-khz", &val);
	if (rc < 0) {
		dev_err(dev, "fpc,spi-freq-khz not found\n");
		goto exit;
	}
	fpc1020->spi_freq_khz = val;
	dev_info(dev, "spi_freq_khz %d\n", fpc1020->spi_freq_khz);

#if HTC_LDO_CONTROL
    //Get 1V8 power Start
    fpc1020->regulator_ldo = regulator_get(dev, "fpc,ldo");
    if (IS_ERR(fpc1020->regulator_ldo))
    {
        printk("[fp][FPC]Fail to get ldo regulator source\n");
        fpc1020->use_regulator_ldo = false;
        //rc = PTR_ERR(fpc1020->regulator_ldo);
        //return rc;
    }else
    {
        fpc1020->use_regulator_ldo = true;
        dev_info(dev, "Get ldo Regulator Source\n");
    }
    //Get 1V8 power End
#endif

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			rc = -EPROBE_DEFER;
			goto exit;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto exit;
		}
		dev_info(dev, "found pin control %s\n", n);
		fpc1020->pinctrl_state[i] = state;
	}

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	rc = select_pin_ctl(fpc1020, "fpc1020_cs_low");
#if 0
	if (rc)
		goto exit;
#endif
	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		goto exit;
	rc = select_pin_ctl(fpc1020, "fpc1020_spi_active");
#if 0    
	if (rc)
		goto exit;
#endif
	fpc1020->wakeup_enabled = false;
	fpc1020->clocks_enabled = false;
	fpc1020->clocks_suspended = false;
	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}
	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}
	dev_info(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
		dev_info(dev, "Enabling hardware\n");
		(void)device_prepare(fpc1020, true);
		(void)set_clks(fpc1020, false);
	}

    rc = fpc1020_spi_setup(fpc1020);
    dev_info(dev, "%s: fpc1020_spi_setup rc:%d\n", __func__, rc);

	rc = fpc1020_create_class(fpc1020);
	if (rc) {
		dev_err(&fpc1020->spi->dev, "fpc1020_create_class failed.\n");
		goto exit;
	}

	rc = fpc1020_create_device(fpc1020);
	if (rc) {
		dev_err(&fpc1020->spi->dev, "fpc1020_create_device failed.\n");
		goto exit;
	}

	cdev_init(&fpc1020->cdev, &fpc1020_fops);
	fpc1020->cdev.owner = THIS_MODULE;

	rc = cdev_add(&fpc1020->cdev, fpc1020->devno, 1);
	if (rc) {
		dev_err(&fpc1020->spi->dev, "cdev_add failed.\n");
		goto exit;
	}
    
	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}

static int fpc1020_remove(struct spi_device *spi)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(&spi->dev);

	sysfs_remove_group(&spi->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);
	wake_lock_destroy(&fpc1020->ttw_wl);
	(void)vreg_setup(fpc1020, "vdd_io", false);
	(void)vreg_setup(fpc1020, "vcc_spi", false);
	(void)vreg_setup(fpc1020, "vdd_ana", false);
	dev_info(&spi->dev, "%s\n", __func__);
	return 0;
}

static int fpc1020_suspend(struct device *dev)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	fpc1020->clocks_suspended = fpc1020->clocks_enabled;
	set_clks(fpc1020, false);
	return 0;
}

static int fpc1020_resume(struct device *dev)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (fpc1020->clocks_suspended)
		set_clks(fpc1020, true);

	return 0;
}

static const struct dev_pm_ops fpc1020_pm_ops = {
	.suspend = fpc1020_suspend,
	.resume = fpc1020_resume,
};

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct spi_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1020",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
		.pm = &fpc1020_pm_ops,
	},
	.probe		= fpc1020_probe,
	.remove		= fpc1020_remove,
};

static int __init fpc1020_init(void)
{
	int rc = spi_register_driver(&fpc1020_driver);
	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);
	return rc;
}

static void __exit fpc1020_exit(void)
{
	pr_info("%s\n", __func__);
	spi_unregister_driver(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
