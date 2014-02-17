/* drivers/input/touchscreen/gt813_827_828.h
 * 
 * 2010 - 2012 Goodix Technology.
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
 * Version:1.2
 *        V1.0:2012/05/01,create file.
 *        V1.2:2012/06/08,add some macro define.
 *
 */

#ifndef _ATA5701_TOUCH_H
#define _ATA5701_TOUCH_H

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/earlysuspend.h>
#include "../../../arch/arm/mach-mx5/mx50_io_cfg.h"

#define ATA5701_I2C_NAME          "ata5701"

#define ATA5701_COMMAND_GETGRADE	0x00
#define ATA5701_COMMAND_GETBITMAP	0x11


struct ata5701_data {
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
   // struct hrtimer timer;
    struct work_struct  work;
    struct early_suspend early_suspend;
    s32 irq_is_disable;
    s32 use_irq;
   // u16 abs_x_max;
    //u16 abs_y_max;
   // u8  max_touch_num;
    //u8  int_trigger_type;
   // u8  green_wake_mode;
   // u8  chip_type;
};

#define ATA5701_DEBUG_FUNC_ON 0
#define ATA5701_DEBUG_ON 0


#define ATA5701_GPIO_AS_INPUT(pin)          do{\
                                            gpio_direction_input(pin);\
                                        }while(0)
#define ATA5701_GPIO_AS_INT(pin)            do{\
                                            GTP_GPIO_AS_INPUT(pin);\
                                        }while(0)
#define ATA5701_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define ATA5701_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)
#define ATA5701_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define ATA5701_GPIO_FREE(pin)              gpio_free(pin)
#define ATA5701_INT_PORT    MX50_RDP_ATA5701_IRQ
#define ATA5701_INT_IRQ     gpio_to_irq(ATA5701_INT_PORT)

#define ATA5701_INFO(fmt,arg...)           printk("<<-ATA5701_INFO->>[%d]"fmt"\n", __LINE__, ##arg)
#define ATA5701_ERROR(fmt,arg...)          printk("<<-ATA5701_ERROR->>[%d]"fmt"\n", __LINE__, ##arg)
#define ATA5701_DEBUG(fmt,arg...)          do{\
                                         if(ATA5701_DEBUG_ON)\
                                         printk("<<-ATA5701_DEBUG->>[%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)

#define ATA5701_DEBUG_FUNC()               do{\
                                         if(ATA5701_DEBUG_FUNC_ON)\
                                         printk("<<-ATA5701-FUNC->>[%d]Func:%s\n", __LINE__, __func__);\
                                       }while(0)

#endif /* _LINUX_GOODIX_TOUCH_H */
