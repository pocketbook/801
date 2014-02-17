/*
* Freescale touchscreen driver
*
* Copyright (C) 2007-2011 Freescale Semiconductor, Inc. All Rights Reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/*!
* @file mxc_ts.c
*
* @brief Driver for the Freescale Semiconductor MXC touchscreen with calibration support.
*
* The touchscreen driver is designed as a standard input driver which is a
* wrapper over low level PMIC driver. Most of the hardware configuration and
* touchscreen functionality is implemented in the low level PMIC driver. During
* initialization, this driver creates a kernel thread. This thread then calls
* PMIC driver to obtain touchscreen values continously. These values are then
* passed to the input susbsystem.
*
* @ingroup touchscreen
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_adc.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/timer.h>
#include <mach/gpio.h>


#include"../../../arch/arm/mach-mx5/mx50_io_cfg.h"

 struct usb_detect_irq {
	int usb_irq;
};

static struct usb_detect_irq usbDetect;
static struct input_dev *virtual_power_input;

bool bInSuspendMode = false;
bool b5vOrUSBInserted = false;

static irqreturn_t usb_detect_wakeup_handler(int irq, void *dev_id)
{
	//static bool b5vDetect = false;
	static int iProcessNumCount = 0;
	
	iProcessNumCount++;
	
	printk("%s %s %d iProcessNumCount=%d ,bInSuspendMode=%d \n",__FILE__,__func__,__LINE__,iProcessNumCount,bInSuspendMode); 
	/*
	 * First received the irq,mean the 5v had been detected or usb had been detected.
	 * Second received the irq,mean 5v had been moved or usb had been moved.
	 */
	  if(iProcessNumCount == 1){
	  	 b5vOrUSBInserted = true;
	  }else if( iProcessNumCount ==3){
	  	b5vOrUSBInserted = false;
	  	iProcessNumCount =0;
	  }
	  	
	 if(bInSuspendMode){
	 	/*in suspend mode,we need send a power button message*/
	   input_event(virtual_power_input,EV_KEY,KEY_POWER,1);
	   input_event(virtual_power_input,EV_KEY,KEY_POWER,0);	   
	   input_sync(virtual_power_input);
	   bInSuspendMode = false;
   }else{
   	 /**/
	   input_event(virtual_power_input,EV_KEY,KEY_SPACE,1);
	   input_event(virtual_power_input,EV_KEY,KEY_SPACE,0);	   
	   input_sync(virtual_power_input);   	 
  }	

	return 0;
}

static int __init  usb_detect_init(void)
{
	int retval;
	
      virtual_power_input = input_allocate_device();
	if (!virtual_power_input) {
		retval = -ENOMEM;
		printk("%s %s %d \n",__FILE__,__func__,__LINE__);
		return retval;
	}
	
	virtual_power_input->name = "virtualPowerInput";
	retval = input_register_device(virtual_power_input);
	if (retval < 0) 
	{
		input_free_device(virtual_power_input);
		printk(KERN_ERR"virtualPowerInput_dev:failed to register input device\n");
		return -ENODEV;
	}
	__set_bit(EV_KEY,virtual_power_input->evbit);
	__set_bit(KEY_POWER,virtual_power_input->keybit); 
    ///register the irq
	gpio_request(MX50_RDP_DCDC_INSERT_CHARGE_DETECT, "usb-detect");
	gpio_direction_input(MX50_RDP_DCDC_INSERT_CHARGE_DETECT);
	usbDetect.usb_irq = gpio_to_irq(MX50_RDP_DCDC_INSERT_CHARGE_DETECT);
	set_irq_type(usbDetect.usb_irq, IRQF_TRIGGER_RISING);
	retval = request_irq(usbDetect.usb_irq,usb_detect_wakeup_handler, 0, "usb-detect-irq", 0);
	if (retval)
	{
		pr_info("register  usb detect interrupt failed\n");
	}
	else
	{
	    	printk("enable usb detect wakeup the device. \n");
		    enable_irq_wake(usbDetect.usb_irq);
	}

	return 0;
}

static void  __exit usb_detect_exit(void)
{
	input_unregister_device(virtual_power_input);

	if (virtual_power_input) {
		input_free_device(virtual_power_input);
		virtual_power_input = NULL;
	}	
  // disable_irq(usbDetect.usb_irq);
}

late_initcall(usb_detect_init);
module_exit(usb_detect_exit);

MODULE_DESCRIPTION("usb detect ");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
