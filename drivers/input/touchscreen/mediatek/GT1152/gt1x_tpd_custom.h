/* drivers/input/touchscreen/gt1x_tpd_custom.h
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.4   
 * Release Date:  2015/07/10
 */

#ifndef GT1X_TPD_CUSTOM_H__
#define GT1X_TPD_CUSTOM_H__

#include <asm/uaccess.h>
//#ifdef CONFIG_MTK_BOOT
#include "mt_boot_common.h"
//#endif
#include "tpd.h"
#include "upmu_common.h"
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
//#include <pmic_drv.h>

#define PLATFORM_MTK
#define TPD_I2C_NUMBER		        1
//#define TPD_SUPPORT_I2C_DMA         1	// if gt9l, better enable it if hardware platform supported
#define TPD_HAVE_BUTTON             1	//report key as coordinate,Vibration feedback

#ifdef CONFIG_MTK_I2C_EXTENSION
#define TPD_SUPPORT_I2C_DMA         1	/* if gt9l, better enable it if hardware platform supported*/
#else
#define TPD_SUPPORT_I2C_DMA         0
#endif

#if TPD_HAVE_BUTTON
#define TPD_KEY_COUNT   3
#define key_1           60,2200
#define key_2           180,2200
#define key_3           300,2200
#define TPD_KEY_MAP_ARRAY {{key_1},{key_2},{key_3}}
#define TPD_KEYS        {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM    {{key_1,50,20},{key_2,50,20},{key_3,50,20}}
#endif

#define RGK_SAMPLE_VERSION		0x000107 
/*****************************add for 9375 info start***********************************/
#define MODULE_MAX_LEN 		40
#define YIJINGTONG_SENSOR_ID	2
#define JINLONG_SENSOR_ID 	0

/*****************************add for 9375 info end***********************************/

// Change I/O define & I/O operation mode.
#define GTP_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define GTP_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)

#define IIC_MAX_TRANSFER_SIZE         8
#define IIC_DMA_MAX_TRANSFER_SIZE     250
#define I2C_MASTER_CLOCK              300

#define TPD_MAX_RESET_COUNT           3

#define TPD_HAVE_CALIBRATION
#define TPD_CALIBRATION_MATRIX        {962,0,0,0,1600,0,0,0};

extern void tpd_on(void);
extern void tpd_off(void);

#endif /* GT1X_TPD_CUSTOM_H__ */
