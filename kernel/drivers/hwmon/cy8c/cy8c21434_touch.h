/* 
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
 * 
 *
 */

#ifndef _CY8C21434_TOUCH_H
#define _CY8C21434_TOUCH_H

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

#define CY8C_I2C_NAME          "cy8c"

struct cy8c_data {
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;  
    struct work_struct  work;
    struct early_suspend early_suspend;
    s32 irq_is_disable;
    s32 use_irq;
    bool in_suspend;
};

#define CY8C_DEBUG_FUNC_ON 0
#define CY8C_DEBUG_ON 1


#define CY8C_GPIO_AS_INPUT(pin)          do{\
                                            gpio_direction_input(pin);\
                                        }while(0)
#define CY8C_GPIO_AS_INT(pin)            do{\
                                            GTP_GPIO_AS_INPUT(pin);\
                                        }while(0)
#define CY8C_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define CY8C_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)
#define CY8C_GPIO_REQUEST(pin, label)    gpio_request(pin, label)
#define CY8C_GPIO_FREE(pin)              gpio_free(pin)
#define CY8C_INT_PORT    MX50_RDP_CY8C_IRQ
#define CY8C_RESET  MX50_RDP_CY8C_RESET
#define CY8C_INT_IRQ     gpio_to_irq(CY8C_INT_PORT)

#define CY8C_INFO(fmt,arg...)           printk("<<-CY8C_INFO->>[%d]"fmt"\n", __LINE__, ##arg)
#define CY8C_ERROR(fmt,arg...)          printk("<<-CY8C_ERROR->>[%d]"fmt"\n", __LINE__, ##arg)
#define CY8C_DEBUG(fmt,arg...)          do{\
                                         if(CY8C_DEBUG_ON)\
                                         printk("<<-CY8C_DEBUG->>[%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)

#define CY8C_DEBUG_FUNC()               do{\
                                         if(CY8C_DEBUG_FUNC_ON)\
                                         printk("<<CY8C-FUNC->>[%d]Func:%s\n", __LINE__, __func__);\
                                       }while(0)

#endif 
