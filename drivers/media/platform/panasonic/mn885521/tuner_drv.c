/******************************************************************************
 *
 *  file name       : tuner_drv.c
 *  brief note      : The Control Layer of Tmm Tuner Driver
 *
 *  creation data   : 2011.07.25
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 1720                       $ Revision of Last commit
 *  $Date:: 2013-05-08 22:02:48 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/07/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/

#include <linux/module.h>       
#include <linux/kernel.h>       
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include "tuner_drv.h"
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
#include <linux/mutex.h>
#endif

#define _XOPEN_SOURCE

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

enum {
        GPIO_CFG_INPUT,
        GPIO_CFG_OUTPUT,
};

enum {
        GPIO_CFG_NO_PULL,
        GPIO_CFG_PULL_DOWN,
        GPIO_CFG_KEEPER,
        GPIO_CFG_PULL_UP,
};

enum {
        GPIO_CFG_2MA,
        GPIO_CFG_4MA,
        GPIO_CFG_6MA,
        GPIO_CFG_8MA,
        GPIO_CFG_10MA,
        GPIO_CFG_12MA,
        GPIO_CFG_14MA,
        GPIO_CFG_16MA,
};

enum {
        GPIO_CFG_ENABLE,
        GPIO_CFG_DISABLE,
};



#define XA 0
#define XB 1
#define XC 2
#define XD 3

#define GPIO_NRST_XA 90
#define GPIO_NPDREG_XA 25
#define GPIO_NPDXTAL_XA 26
#define GPIO_FULLSEG_INT_XA 93
#define GPIO_SRIO_1V8_EN_XA 95
#define GPIO_FULLSEG_1V1_EN_XA 96
#define FM_FULLSEG_ANT_SW_XA 24

#define GPIO_NRST_XB 90
#define GPIO_NPDREG_XB 25
#define GPIO_NPDXTAL_XB 26
#define GPIO_FULLSEG_INT_XB 81
#define GPIO_SRIO_1V8_EN_XB 83
#define GPIO_FULLSEG_1V1_EN_XB 84
#define FM_FULLSEG_ANT_SW_XB 24

#define ANT_SW_PHY_2P85_VOL_MIN     2850000 
#define ANT_SW_PHY_2P85_VOL_MAX     2850000 

#define DEV_NAME "TUNER" 
int TUNER_CONFIG_DRV_MAJOR = 0; 
int TUNER_CONFIG_DRV_MINOR = 0; 

void *mem_p;

wait_queue_head_t g_tuner_poll_wait_queue;       
spinlock_t        g_tuner_lock;                  
unsigned long     g_tuner_wakeup_flag;           

unsigned char g_tuner_intcnd_f;                  
unsigned char g_tuner_intcnd_s;                  
unsigned char g_tuner_intst_f;
unsigned char g_tuner_intst_s;

struct task_struct *g_tuner_kthread_id;          
u32                 g_tuner_kthread_flag;        
wait_queue_head_t   g_tuner_kthread_wait_queue;  

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
struct mutex g_tuner_mutex;                       
#endif

static ssize_t tuner_module_entry_read( struct file* FIle,
                                        char* Buffer,
                                        size_t Count,
                                        loff_t* OffsetPosition );
static ssize_t tuner_module_entry_write( struct file* FIle,
                                         const char* Buffer,
                                         size_t Count,
                                         loff_t* OffsetPosition );
static unsigned int tuner_module_entry_poll( struct file *file,
                                             struct poll_table_struct *poll_tbl );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int tuner_module_entry_ioctl( struct inode* Inode,
                                     struct file* FIle, 
                                     unsigned int uCommand,
                                     unsigned long uArgument );
#else   
static long tuner_module_entry_ioctl( struct file *file,
                                      unsigned int uCommand,
                                      unsigned long uArgument );
#endif 
static int tuner_module_entry_open( struct inode* Inode,
                                    struct file* FIle );
static int tuner_module_entry_close( struct inode* Inode,
                                     struct file* FIle );
static int tuner_probe( struct platform_device *pdev );
static int __exit tuner_remove( struct platform_device *pdev );
static int  __init tuner_drv_start( void );
static void __exit tuner_drv_end( void );

static struct file_operations TunerFileOperations =
{
   .owner   = THIS_MODULE,
   .read    = tuner_module_entry_read,
   .write   = tuner_module_entry_write,
   .poll    = tuner_module_entry_poll,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
   .ioctl   = tuner_module_entry_ioctl,
#else  
   .unlocked_ioctl = tuner_module_entry_ioctl,
   #ifdef CONFIG_COMPAT
   .compat_ioctl = tuner_module_entry_ioctl,
   #endif
#endif 
   .open    = tuner_module_entry_open,
   .release = tuner_module_entry_close
};

static const struct of_device_id mn885521_mt[] = {
        { .compatible = "fullseg_tuner,mn885521"},
        { },
};

static struct platform_driver mmtuner_driver = {
    .probe  = tuner_probe,
    .remove = __exit_p(tuner_remove),
    .driver = { 
		.name = "mmtuner",
            	.owner = THIS_MODULE,
		.of_match_table = mn885521_mt,
    },
};

static struct platform_device *mmtuner_device;
static struct class *device_class;
unsigned long open_cnt;                           
static struct regulator *ant_sw_2p85;

struct fullseg_platform_data fullseg_gpios;

#ifndef TUNER_CONFIG_IRQ_PC_LINUX
irqreturn_t tuner_interrupt( int irq, void *dev_id );
#else  
int tuner_interrupt( void );
#endif 

 
static int fullseg_pinctrl_init(struct platform_device *pdev)
{
    int ret;
    struct pinctrl *pinctrl;
    struct pinctrl_state *set_state;

    pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR_OR_NULL(pinctrl)) {
        pr_err("%s: Failed to get pinctrl!\n", __func__);
        return PTR_ERR(pinctrl);
    }

    set_state = pinctrl_lookup_state(pinctrl, "default");
    if (IS_ERR_OR_NULL(set_state)) {
        pr_err("%s: Can not get active pin state!\n", __func__);
        return PTR_ERR(set_state);
    }
    ret = pinctrl_select_state(pinctrl, set_state);
    return ret;
}

int board_gpio_request(struct fullseg_platform_data *gpios)
{
	int ret =0;
	
	gpio_free(fullseg_gpios._1v8_en);
	ret = gpio_request(gpios->_1v8_en, "fullseg_1v8_en");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}

    ret = gpio_request(gpios->_1v1_en, "fullseg_1v1_en");
    if (ret < 0) {
            pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
            return ret;
    }

	ret = gpio_request(gpios->npdreg, "fullseg_npd_reg");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}
	
	ret = gpio_request(gpios->npdxtal, "fullseg_npd_xtal");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}

	ret = gpio_request(gpios->nrst, "fullseg_nrst");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}
		
	return ret;
}

void board_gpio_free(void)
{
        gpio_free(fullseg_gpios._1v8_en);
        gpio_free(fullseg_gpios.npdreg);
        gpio_free(fullseg_gpios.npdxtal);
        gpio_free(fullseg_gpios.nrst);

        return;
}

static int fm_fullseg_antenna_sw_power_enable(void)
{
    int rc;

    rc = regulator_enable(ant_sw_2p85);
    if (rc < 0) {
        printk(KERN_ERR "[FULLSEG] %s: Enable regulator 2.85v failed\n", __func__);
        return -ENODEV;
    }

    return rc;
}

static int fm_fullseg_antenna_sw_power_disable(void)
{
    int rc;

    rc = regulator_disable(ant_sw_2p85);
    if (rc < 0){
        printk(KERN_ERR "[FULLSEG] %s: disable regulator failed\n", __func__);
	return -ENODEV;
    }
	return rc;
}

int fm_ant_power_fullseg(int on)
{
	int ret=0;

	DEBUG_PRINT("fm_ant_power_fullseg: %d", on);
	if(on)
	{
		fm_fullseg_antenna_sw_power_enable();
		ret = gpio_request(fullseg_gpios.fm_fullseg_ant_sw, "fm_fullseg_ant_sw");
		if (ret < 0) {
			pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
			return ret;
		}
		ret = gpio_direction_output(fullseg_gpios.fm_fullseg_ant_sw, 0);
		if (ret < 0) {
			pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(fullseg_gpios.fm_fullseg_ant_sw);
			return ret;
		}
	}
	else
		fm_fullseg_antenna_sw_power_disable();
	return ret;
}

int poweron_tuner(struct fullseg_platform_data *gpios, int on)
{
	int ret =0;

	if (on)
	{
		printk("[FULLSEG] %s, on\r\n", __func__); 
		ret = gpio_direction_output(gpios->_1v8_en, 0);
		if (ret < 0) {
			pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(gpios->_1v8_en);
			return ret;
		}
		
		
		gpio_set_value(gpios->_1v8_en, 1);
		
		gpio_direction_output(gpios->_1v1_en, 0);
		
        gpio_set_value(gpios->_1v1_en, 1);

		
		printk("[FULLSEG] %s, set gpio state to low\r\n", __func__);
		ret = gpio_direction_output(gpios->nrst, 0);
		msleep(10); 
		ret = gpio_direction_output(gpios->npdxtal, 0);
		ret = gpio_direction_output(gpios->npdreg, 0);
		msleep(10);

        ret = gpio_direction_output(gpios->npdreg, 1);
	    if (ret < 0) {
  			pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
  			gpio_free(gpios->npdreg);
			return ret;
        }
		msleep(10); 

		ret = gpio_direction_output(gpios->npdxtal, 1);
		if (ret < 0) {
			pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(gpios->npdxtal);
			return ret;
		}
		msleep(10); 

        ret = gpio_direction_output(gpios->nrst, 1);
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
            gpio_free(gpios->nrst);
            return ret;
		}

		
        fm_fullseg_antenna_sw_power_enable();

		gpio_free(gpios->fm_fullseg_ant_sw);
        ret = gpio_request(gpios->fm_fullseg_ant_sw, "fm_fullseg_ant_sw");
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
            return ret;
        }
	
        ret = gpio_direction_output(gpios->fm_fullseg_ant_sw, 1);
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
            gpio_free(gpios->fm_fullseg_ant_sw);
            return ret;
		}

	}else{
		printk("[FULLSEG] %s, off\r\n", __func__);
        ret = gpio_direction_output(gpios->fm_fullseg_ant_sw, 0);
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
            gpio_free(gpios->fm_fullseg_ant_sw);
            return ret;
        }
		
	    fm_fullseg_antenna_sw_power_disable();

        ret = gpio_direction_output(gpios->nrst, 0);
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
            gpio_free(gpios->nrst);
            return ret;
        }
		msleep(10); 

        ret = gpio_direction_output(gpios->npdxtal, 0);
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
            gpio_free(gpios->npdxtal);
            return ret;
        }
        ret = gpio_direction_output(gpios->npdreg, 0);
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
            gpio_free(gpios->npdreg);
            return ret;
		}
                

		gpio_direction_output(gpios->_1v1_en, 0);
        gpio_set_value(gpios->_1v1_en, 0);

		
        ret = gpio_direction_output(gpios->_1v8_en, 0);
        if (ret < 0) {
        	pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
            gpio_free(gpios->_1v8_en);
            return ret;
		}
        gpio_set_value(gpios->_1v8_en, 0);
        mdelay(10);

	}

	return ret;

}
static int read_gpio_from_dt(struct device_node *dt, struct fullseg_platform_data *pdata)
{
        struct property *prop = NULL;

	INFO_PRINT("%s \n", __func__);
        prop = of_find_property(dt, "fullseg,npdreg", NULL);
        if (prop) {
                pdata->npdreg = of_get_named_gpio(dt, "fullseg,npdreg", 0);
		printk("%s: pdata->npdreg = %d\n", __func__, pdata->npdreg);
        }

        prop = of_find_property(dt, "fullseg,npdxtal", NULL);
        if (prop) {
                pdata->npdxtal = of_get_named_gpio(dt, "fullseg,npdxtal", 0);
		printk("%s: pdata->npdxtal = %d\n", __func__, pdata->npdxtal);
        }

        prop = of_find_property(dt, "fullseg,nrst", NULL);
        if (prop) {
                pdata->nrst = of_get_named_gpio(dt, "fullseg,nrst", 0);
		printk("%s: pdata->nrst = %d\n", __func__, pdata->nrst);
        }

        prop = of_find_property(dt, "fullseg,interrupt", NULL);
        if (prop) {
                pdata->interrupt = of_get_named_gpio(dt, "fullseg,interrupt", 0);
		printk("%s: pdata->interrupt = %d\n", __func__, pdata->interrupt);
        }

        prop = of_find_property(dt, "fullseg,fm_fullseg_ant_sw", NULL);
        if (prop) {
                pdata->fm_fullseg_ant_sw = of_get_named_gpio(dt, "fullseg,fm_fullseg_ant_sw", 0);
		printk("%s: pdata->fm_fullseg_ant_sw = %d\n", __func__, pdata->fm_fullseg_ant_sw);
        }

        prop = of_find_property(dt, "fullseg,1v8_en", NULL);
        if (prop) {
                pdata->_1v8_en = of_get_named_gpio(dt, "fullseg,1v8_en", 0);
                printk("%s: pdata->_1v8_en = %d\n", __func__, pdata->_1v8_en);
        }

        prop = of_find_property(dt, "fullseg,1v1_en", NULL);
        if (prop) {
                pdata->_1v1_en = of_get_named_gpio(dt, "fullseg,1v1_en", 0);
                printk("%s: pdata->_1v1_en = %d\n", __func__, pdata->_1v1_en);
        }

		
		

        return 0;
}

#if 0
int set_i2c_spi_mode(void)
{
	int ret;

	ret = gpio_request(GPIO_SPISEL, "fullseg_spisel");
        if (ret < 0) {
        	pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
	ret = gpio_direction_output(GPIO_SPISEL, 0);
        if (ret < 0) {
        	pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(GPIO_SPISEL);
                return ret;
	}

        ret = gpio_request(GPIO_SADR2, "fullseg_sadr2");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(GPIO_SADR2, 1);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(GPIO_SADR2);
                return ret;
        }

	return ret;
}
#endif

int init_regulator(struct device *dev)
{
	int rc = 1;

	ant_sw_2p85 = devm_regulator_get(dev, "ANT_SW_2p85");
        if (IS_ERR(ant_sw_2p85)) {
		ERROR_PRINT("%s: unable to get antenna sw 2.85v\n", __func__);
		return -1;
        }
        rc = regulator_set_voltage(ant_sw_2p85, ANT_SW_PHY_2P85_VOL_MIN,
                                ANT_SW_PHY_2P85_VOL_MAX);
        if (rc) {
		ERROR_PRINT("%s: unable to get antenna sw 2.85v\n", __func__);
		return -1;
        }
	return rc;
}

static int tuner_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct fullseg_platform_data *pdata;
    struct device *dev = &pdev->dev;

    

    INFO_PRINT("mmtuner_probe: Called. -6-\n");

    
    if (register_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME, &TunerFileOperations))
    {
        ERROR_PRINT("mmtuner_probe: register_chrdev()\
                     Failed Major:%d.\n", TUNER_CONFIG_DRV_MAJOR);
        return -1;
    }

    
    init_waitqueue_head( &g_tuner_poll_wait_queue );
    spin_lock_init( &g_tuner_lock );
    g_tuner_wakeup_flag = TUNER_OFF;
    g_tuner_intcnd_f = 0x00;
    g_tuner_intcnd_s = 0x00;
	open_cnt         = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    mutex_init(&g_tuner_mutex);
#endif
    if (pdev->dev.of_node) {
	INFO_PRINT("tuner_probe: -1-\n");
	pdata = (struct fullseg_platform_data *)&fullseg_gpios;
        ret = read_gpio_from_dt(pdev->dev.of_node, pdata);
        if (ret < 0) {
		ERROR_PRINT("dt parsing error");
		return -1;
	}
    }else
	INFO_PRINT("tuner_probe: -2-\n");
	
	
    board_gpio_request(pdata);
	ret = fullseg_pinctrl_init(pdev);
    if (ret) {
        pr_err("%s: pinctrl init failed!\n", __func__);
    }else{
		INFO_PRINT("pinctrl OK!\n");
	}
    init_regulator(dev);

    INFO_PRINT("tuner_probe: END.\n");
    return 0;
}

static int __exit tuner_remove(struct platform_device *pdev)
{
    INFO_PRINT("tuner_remove: Called.\n");
    TRACE();

    
    tuner_drv_release_interrupt();

    
    unregister_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME);
    
    INFO_PRINT("tuner_remove: END.\n");

    return 0;
}


int tuner_kernel_thread( void * arg )
{
    int ret = 0;
    unsigned long flags;
    unsigned long ktread_flg;
    
    mm_segment_t    oldfs;
    struct sched_param  param; 

    struct i2c_adapter	*adap;
    struct i2c_msg		msgs[4];
    
    
    unsigned char		buff[3];
    unsigned char		bufs[3];
    int i;
	
    INFO_PRINT("tuner_kernel_thread: START.\n");

    
    ret = 0;
    flags = 0;
    ktread_flg = 0;
    param.sched_priority = TUNER_CONFIG_KTH_PRI;

    
    INFO_PRINT("tuner_kernel_thread: TUNER_CONFIG_I2C_BUSNUM = %x\n", TUNER_CONFIG_I2C_BUSNUM);

    oldfs = get_fs();
    set_fs( KERNEL_DS );
    ret = sched_setscheduler( g_tuner_kthread_id, SCHED_FIFO, &param );
    set_fs( oldfs );

    buff[0] = (unsigned char)TUNER_DRV_ADR_INTCND_F;
    bufs[0] = (unsigned char)TUNER_DRV_ADR_INTCND_S;

    while(1)
    {
        DEBUG_PRINT("tuner_kernel_thread waiting... ");
        wait_event_interruptible( g_tuner_kthread_wait_queue, g_tuner_kthread_flag );

        spin_lock_irqsave( &g_tuner_lock, flags );
        ktread_flg = g_tuner_kthread_flag;
        g_tuner_kthread_flag &= ~TUNER_KTH_IRQHANDLER;	
        spin_unlock_irqrestore( &g_tuner_lock, flags);

        memset( msgs, 0x00, sizeof(struct i2c_msg) * 4 );
	for (i=0; i<10; i++)
	{
		adap = i2c_get_adapter(i);
	        if (adap != NULL) {
			printk(KERN_INFO "%s: get i2c adapter: %d", __func__, i);
			break;
        	}else
			printk(KERN_INFO "%s: get i2c adapter fail: %d", __func__, i);
		if (i==10)
		{
			printk(KERN_INFO "%s: get i2c adapter done, i=%d", __func__, i);
			return 0;
		}
	}
	

        
        if ( ( ktread_flg & TUNER_KTH_IRQHANDLER ) == TUNER_KTH_IRQHANDLER )
        {
            DEBUG_PRINT("tuner_kernel_thread IRQHANDLER start ");

			buff[1] = buff[2] = 0;
			bufs[1] = bufs[2] = 0;

            
            
            msgs[0].addr	= TUNER_SLAVE_ADR_M1;
            msgs[0].flags	= 0;	
            msgs[0].len		= 1;
            msgs[0].buf		= &buff[0];
            msgs[1].addr	= TUNER_SLAVE_ADR_M1;
            msgs[1].flags	= I2C_M_RD;
            msgs[1].len		= 2;
            msgs[1].buf		= buff+1;
            msgs[2].addr	= TUNER_SLAVE_ADR_M2;
            msgs[2].flags	= 0;	
            msgs[2].len		= 1;
            msgs[2].buf		= &bufs[0];
            msgs[3].addr	= TUNER_SLAVE_ADR_M2;
            msgs[3].flags	= I2C_M_RD;
            msgs[3].len		= 2;
            msgs[3].buf		= bufs+1;

            ret = i2c_transfer(adap, msgs, 4);
            if (ret < 0) {
            	TRACE();
              	i2c_put_adapter(adap);
               	break;
            }
            DEBUG_PRINT("read        slv:0x%02x adr:0x%02x len:%-4d 0x%02x ... 0x%02x ",
                		msgs[0].addr, *(msgs[0].buf), msgs[1].len, msgs[1].buf[0], msgs[1].buf[1]);
            DEBUG_PRINT("read        slv:0x%02x adr:0x%02x len:%-4d 0x%02x ... 0x%02x ",
						msgs[2].addr, *(msgs[2].buf), msgs[3].len, msgs[3].buf[0], msgs[3].buf[1]);

            g_tuner_intcnd_f |= buff[1];
            g_tuner_intcnd_s |= bufs[1];
            g_tuner_intst_f	= buff[2];
            g_tuner_intst_s	= bufs[2];


            DEBUG_PRINT( "// IRQ factor update: INTCND_F:0x%02x INTST_F:0x%02x"
            		,g_tuner_intcnd_f, g_tuner_intst_f );
            DEBUG_PRINT( "// IRQ factor update: INTCND_S:0x%02x INTST_S:0x%02x"
            		,g_tuner_intcnd_s, g_tuner_intst_s );
            
            
            memset( msgs, 0x00, sizeof(struct i2c_msg) * 4 );
            msgs[0].addr	= TUNER_SLAVE_ADR_M1;
            msgs[0].flags	= 0;	
            msgs[0].len		= 2;
            msgs[0].buf		= buff;
            msgs[1].addr	= TUNER_SLAVE_ADR_M2;
            msgs[1].flags	= 0;	
            msgs[1].len		= 2;
            msgs[1].buf		= bufs;
            ret = i2c_transfer(adap, msgs, 2);
            if (ret < 0) {
            	TRACE();
                i2c_put_adapter(adap);
                break;
            }
            i2c_put_adapter(adap);


            
            g_tuner_wakeup_flag = TUNER_ON;
            wake_up( &g_tuner_poll_wait_queue );

            DEBUG_PRINT("tuner_interrupt end ");


#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
            
            tuner_drv_enable_interrupt();
#endif 
        }

        
        if ( ( ktread_flg & TUNER_KTH_END ) == TUNER_KTH_END )
        {
            DEBUG_PRINT("tuner_kernel_thread KTH_END start ");
            spin_lock_irqsave( &g_tuner_lock, flags );
            g_tuner_kthread_flag &= ~TUNER_KTH_END;
            spin_unlock_irqrestore( &g_tuner_lock, flags );
            break;
        }
    }

    INFO_PRINT("tuner_kernel_thread: END. ");

    return 0;
}


static int __init tuner_drv_start(void)
{
    int ret =0;
    struct device *dev = NULL;

    
    
    int count = 1;
    dev_t tuner_dev;

    
    ret = alloc_chrdev_region(&tuner_dev, TUNER_CONFIG_DRV_MAJOR, count, DEV_NAME);
    if (ret < 0) 
       printk("Major number allocation is failed\n");

    
    TUNER_CONFIG_DRV_MAJOR = 255; 
    TUNER_CONFIG_DRV_MINOR = 0; 
    

    INFO_PRINT("mmtuner_tuner_drv_start: Called\n");

    

    ret = platform_driver_register(&mmtuner_driver);

    if( ret != 0 )
    {
        ERROR_PRINT("init_module: Error:\
                     failed in platform_driver_register.\n");
        return ret;
    }
   INFO_PRINT("mmtuner: platform_driver_register\n");
#if 0 
    
    mmtuner_device = platform_device_alloc("mmtuner", -1);

    if (!mmtuner_device)
    {
        ERROR_PRINT("init_module: Error: failed in platform_device_alloc.\n");
        platform_driver_unregister(&mmtuner_driver);
        return -ENOMEM;
    }
    
    
    ret = platform_device_add(mmtuner_device);
    if ( ret )
    {
        ERROR_PRINT("init_module: Error: failed in platform_device_add.\n");
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return ret;
    }

    INFO_PRINT("mmtuner: platform_device_add\n");
#endif
    
    device_class = class_create(THIS_MODULE, "mmtuner");
    if (IS_ERR(device_class)) 
    {
        ERROR_PRINT("init_module: Error: failed in class_create.\n");
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return PTR_ERR(device_class);
    }

    
    dev = device_create (device_class, NULL, MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR), NULL, "mmtuner");

    if(IS_ERR(dev))
    {
        ERROR_PRINT("init_module: Error: failed in device_create.\n");
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return PTR_ERR(dev);
    }
   
    
    g_tuner_kthread_flag = TUNER_KTH_NONE;

    init_waitqueue_head( &g_tuner_kthread_wait_queue );

    g_tuner_kthread_id = kthread_create( tuner_kernel_thread,
                                         NULL,
                                         "tuner_kthread" );
    if( IS_ERR( g_tuner_kthread_id ) )
    {
        g_tuner_kthread_id = NULL;
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return -EIO;
    }

    wake_up_process( g_tuner_kthread_id );

    INFO_PRINT("mmtuner_tuner_drv_start: END\n");
    return 0;
}

static void __exit tuner_drv_end(void)
{
    INFO_PRINT("mmtuner_tuner_drv_end: Called\n");

    board_gpio_free();

    
    g_tuner_kthread_flag |= TUNER_KTH_END;
    if( waitqueue_active( &g_tuner_kthread_wait_queue ))
    {
        wake_up( &g_tuner_kthread_wait_queue );
    }

    
    if( g_tuner_kthread_id )
    {
        kthread_stop( g_tuner_kthread_id );
    }

    
    device_destroy(device_class, MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR));
    
    class_destroy(device_class);
    
    platform_device_unregister(mmtuner_device);
    
    platform_driver_unregister(&mmtuner_driver);

    INFO_PRINT("mmtuner_tuner_drv_end: END\n");
}

static int tuner_module_entry_open(struct inode* Inode, struct file* FIle)
{

    INFO_PRINT("tuner_module_entry_open: Called\n");

#ifdef  TUNER_CONFIG_DRV_MULTI      
    open_cnt++;
#else   
	
    if( open_cnt > 0 )
    {
        INFO_PRINT("tuner_module_entry_open: open error\n");
        return -1;
    }
	
    else
    {
        INFO_PRINT("tuner_module_entry_open: open_cnt = 1\n");
        open_cnt++;
    }
#endif 

    poweron_tuner(&fullseg_gpios,1);	
    return 0;
}

static int tuner_module_entry_close(struct inode* Inode, struct file* FIle)
{
    	struct devone_data *dev;

    	INFO_PRINT("tuner_module_entry_close: Called\n");

	
	
	poweron_tuner(&fullseg_gpios, 0);
	

	
	if( open_cnt <= 0 )
	{
        INFO_PRINT("tuner_module_entry_close: close error\n");
        return -1;
	}
	else
	{
		open_cnt--;
	}

	
	if( open_cnt == 0 )
	{
        
	tuner_drv_release_interrupt();

        if( FIle == NULL )
        {
            return -1;
        }

        dev = FIle->private_data;

        if( dev )
        {
            kfree( dev );
        }
	}

    return 0;

}

static ssize_t tuner_module_entry_read(struct file * FIle, char * Buffer,
                                 size_t Count, loff_t * OffsetPosition)
{
    return 0;

}

static ssize_t tuner_module_entry_write(struct file* FIle,
		const char* Buffer, size_t Count, loff_t* OffsetPosition)
{
    int				ret;
    unsigned long	copy_ret;
    
    unsigned char	*buf;				

    struct i2c_adapter	*adap;
    struct i2c_msg		msgs[1];

    
    if (Count < 3) {
    	TRACE();
    	return -EINVAL;
    }

    
    buf = (unsigned char *)vmalloc(Count);
    if (buf == NULL) {
        return -EINVAL;
    }

    copy_ret = copy_from_user(buf, Buffer, Count);
    if (copy_ret != 0) {
        vfree(buf);
        return -EINVAL;
    }

    
    adap = i2c_get_adapter(TUNER_CONFIG_I2C_BUSNUM);
    if (adap == NULL) {
    	TRACE();
    	vfree(buf);
    	return -EINVAL;
    }

    
    memset(msgs, 0x00, sizeof(struct i2c_msg) * 1);

    msgs[0].addr	= buf[0];
    msgs[0].flags	= 0;		
    msgs[0].len		= Count - 1;
    msgs[0].buf		= buf + 1;

    ret = i2c_transfer(adap, msgs, 1);
    if (ret < 0) {
    	TRACE();
    	i2c_put_adapter(adap);
    	vfree(buf);
    	return -EINVAL;
    }
    
    

    vfree(buf);
    return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int tuner_module_entry_ioctl(struct inode* Inode, struct file* FIle,
                              unsigned int uCommand, unsigned long uArgument)
#else  
static long tuner_module_entry_ioctl(struct file *file,
                              unsigned int uCommand, unsigned long uArgument)
#endif 

{
    int                   ret;
    TUNER_DATA_RW         data;
    unsigned long         copy_ret;
    int                   param;
    TUNER_DATA_RW         event_status[ TUNER_EVENT_REGNUM ];

    

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    
    mutex_lock(&g_tuner_mutex);
#endif

    
    if( uArgument == 0 )
    {
        TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
        
        mutex_unlock(&g_tuner_mutex);
#endif  
        return -EINVAL;
    }
    
    switch( uCommand )
    {
        
        case TUNER_IOCTL_VALGET:
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            ret = tuner_drv_hw_access( uCommand, &data, 1 );

            if( ret == 0 )
            {
                
                copy_ret = copy_to_user( &( *(TUNER_DATA_RW *)uArgument ),
                                         &data,
                                         sizeof( TUNER_DATA_RW ));
                if( copy_ret != 0 )
                {
                    TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                    
                    mutex_unlock(&g_tuner_mutex);
#endif  
                    return -EINVAL;
                }
            }

            break;


        
        case TUNER_IOCTL_VALSET:
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            ret = tuner_drv_hw_access( uCommand, &data, 1 );
            break;

        case TUNER_IOCTL_VALGET_EVENT:
            
            copy_ret = copy_to_user( &( *( unsigned char *)uArgument ),
                                     &g_tuner_intcnd_f,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }
            
            copy_ret = copy_to_user( &( *( unsigned char *)( uArgument + 1 )),
                                     &g_tuner_intcnd_s,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;
            }
            
            copy_ret = copy_to_user( &( *( unsigned char *)(uArgument + 2)),
                                     &g_tuner_intst_f,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;
            }
            
            copy_ret = copy_to_user( &( *( unsigned char *)(uArgument + 3)),
                                     &g_tuner_intst_s,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            DEBUG_PRINT( "// IRQ factor send: INTCND_F:0x%02x INTST_F:0x%02x"
            		,g_tuner_intcnd_f, g_tuner_intst_f );
            DEBUG_PRINT( "// IRQ factor send: INTCND_S:0x%02x INTST_S:0x%02x"
            		,g_tuner_intcnd_s, g_tuner_intst_s );

            
            g_tuner_intcnd_f = 0x00;
            g_tuner_intcnd_s = 0x00;
            g_tuner_intst_f = 0x00;
            g_tuner_intst_s = 0x00;

            ret = copy_ret;

            break;
        
        case TUNER_IOCTL_VALSET_EVENT:
        	DEBUG_PRINT("*** VALSET_EVENT ***\n");
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            
            event_status[0].slave_adr = TUNER_SLAVE_ADR_M1;  
            event_status[0].adr       = REG_INTDEF1_F;       
            event_status[0].sbit      = SIG_ENS_INTDEF1_F;   
            event_status[0].ebit      = SIG_ENE_INTDEF1_F;   
            event_status[0].param     = 0x00;                
            event_status[0].enabit    = SIG_ENA_INTDEF1_F;   
            event_status[1].slave_adr = TUNER_SLAVE_ADR_M1;
            event_status[1].adr       = REG_INTDEF2_F;
            event_status[1].sbit      = SIG_ENS_INTDEF2_F;
            event_status[1].ebit      = SIG_ENE_INTDEF2_F;
            event_status[1].param     = 0x00;
            event_status[1].enabit    = SIG_ENA_INTDEF2_F;
            event_status[2].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[2].adr       = REG_INTDEF1_S;
            event_status[2].sbit      = SIG_ENS_INTDEF1_S;
            event_status[2].ebit      = SIG_ENE_INTDEF1_S;
            event_status[2].param     = 0x00;
            event_status[2].enabit    = SIG_ENA_INTDEF1_S;
            event_status[3].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[3].adr       = REG_INTDEF2_S;
            event_status[3].sbit      = SIG_ENS_INTDEF2_S;
            event_status[3].ebit      = SIG_ENE_INTDEF2_S;
            event_status[3].param     = 0x00;
            event_status[3].enabit    = SIG_ENA_INTDEF2_S;

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALGET, event_status, TUNER_EVENT_REGNUM );
            
            if( ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;  
            }

            if (((event_status[0].param & event_status[0].enabit) == 0x00) &&
            	((event_status[1].param & event_status[1].enabit) == 0x00) &&
                ((event_status[2].param & event_status[2].enabit) == 0x00) &&
                ((event_status[3].param & event_status[3].enabit) == 0x00))
            {
            	DEBUG_PRINT("*** REQUEST IRQ ***");
                ret = tuner_drv_set_interrupt();
                
                if( ret != 0 )
                {
                    TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;  
                }
            }

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALSET, &data, 1 );

            break;
        
        case TUNER_IOCTL_VALREL_EVENT:
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALSET, &data, 1 );

            if( ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            
            event_status[0].slave_adr = TUNER_SLAVE_ADR_M1;  
            event_status[0].adr       = REG_INTDEF1_F;       
            event_status[0].sbit      = SIG_ENS_INTDEF1_F;   
            event_status[0].ebit      = SIG_ENE_INTDEF1_F;   
            event_status[0].param     = 0x00;                
            event_status[0].enabit    = SIG_ENA_INTDEF1_F;   
            event_status[1].slave_adr = TUNER_SLAVE_ADR_M1;
            event_status[1].adr       = REG_INTDEF2_F;
            event_status[1].sbit      = SIG_ENS_INTDEF2_F;
            event_status[1].ebit      = SIG_ENE_INTDEF2_F;
            event_status[1].param     = 0x00;
            event_status[1].enabit    = SIG_ENA_INTDEF2_F;
            event_status[2].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[2].adr       = REG_INTDEF1_S;
            event_status[2].sbit      = SIG_ENS_INTDEF1_S;
            event_status[2].ebit      = SIG_ENE_INTDEF1_S;
            event_status[2].param     = 0x00;
            event_status[2].enabit    = SIG_ENA_INTDEF1_S;
            event_status[3].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[3].adr       = REG_INTDEF2_S;
            event_status[3].sbit      = SIG_ENS_INTDEF2_S;
            event_status[3].ebit      = SIG_ENE_INTDEF2_S;
            event_status[3].param     = 0x00;
            event_status[3].enabit    = SIG_ENA_INTDEF2_S;

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALGET, event_status, TUNER_EVENT_REGNUM );

            if (((event_status[0].param & event_status[0].enabit) == 0x00) &&
                ((event_status[1].param & event_status[1].enabit) == 0x00) &&
                ((event_status[2].param & event_status[2].enabit) == 0x00) &&
                ((event_status[3].param & event_status[3].enabit) == 0x00))
            {
            	DEBUG_PRINT("*** release IRQ REQUEST ***");
                tuner_drv_release_interrupt();
            }

            break;
        case TUNER_IOCTL_VALSET_POWER:
            copy_ret = copy_from_user( &param,
                                       &( *( int * )uArgument ),
                                       sizeof( int ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }
            ret = tuner_drv_ctl_power( param );

            break;
        default:
            TRACE();
            ret = -EINVAL;
            break;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    
    mutex_unlock(&g_tuner_mutex);
#endif    
    
    return ret;
}

static unsigned int tuner_module_entry_poll(
                        struct file *file,
                        struct poll_table_struct *poll_tbl )
{
    unsigned long tuner_flags;
    unsigned int  tuner_mask;


    
    tuner_mask = 0;

    
    poll_wait( file, &g_tuner_poll_wait_queue, poll_tbl );

    
    spin_lock_irqsave( &g_tuner_lock, tuner_flags );

    
    if( g_tuner_wakeup_flag == TUNER_ON )
    {
        tuner_mask = ( POLLIN | POLLRDNORM );
    }
    
    g_tuner_wakeup_flag = TUNER_OFF;

    
    spin_unlock_irqrestore( &g_tuner_lock, tuner_flags );

    return tuner_mask;
}

#ifndef TUNER_CONFIG_IRQ_PC_LINUX
irqreturn_t tuner_interrupt( int irq, void *dev_id )
#else  
int tuner_interrupt( void )
#endif 
{
    DEBUG_PRINT("tuner_interrupt start ");

    
    g_tuner_kthread_flag |= TUNER_KTH_IRQHANDLER;
    if( waitqueue_active( &g_tuner_kthread_wait_queue ))
    {
#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
        
        tuner_drv_disable_interrupt();
#endif 
        wake_up( &g_tuner_kthread_wait_queue );
    }
    else
    {
        DEBUG_PRINT("tuner_interrupt waitqueue_active err!!! ");
        
        
    }

    DEBUG_PRINT("tuner_interrupt end ");

    return IRQ_HANDLED;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Panasonic Co., Ltd.");
MODULE_DESCRIPTION("MM Tuner Driver");

module_init(tuner_drv_start);
module_exit(tuner_drv_end);
/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
