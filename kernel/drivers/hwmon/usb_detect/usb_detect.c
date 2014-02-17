/*
* imx-3stack-sgtl5000.c  --  i.MX 3Stack Driver for Freescale SGTL5000 Codec
*
* Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
*
*  This program is free software; you can redistribute  it and/or modify it
*  under  the terms of  the GNU General  Public License as published by the
*  Free Software Foundation;  either version 2 of the  License, or (at your
*  option) any later version.
*
*  Revision history
*    21th Oct 2008   Initial version.
*
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/input.h>

#include <mach/dma.h>
#include <mach/clock.h>

#include <linux/earlysuspend.h>


struct mxc_5vOrUSB_priv {
	struct platform_device *pdev;
};

static struct mxc_5vOrUSB_priv usb_priv;

static struct input_dev *virtual_power_input;

static bool bEarlySuspendState = false;

extern int iXrzSuspendMode;

static struct workqueue_struct *usb_wq;
static struct work_struct  usb_work;

int imx_virtual_power_report(void)
{
	input_event(virtual_power_input,EV_KEY,KEY_POWER,1);
	input_event(virtual_power_input,EV_KEY,KEY_POWER,0);
	input_sync(virtual_power_input);  
	
        return 0;
}

static irqreturn_t imx_usb_detect_handler(int irq, void *data)
{
        /*process the usb queue work */
	queue_work(usb_wq, &usb_work);
	return IRQ_HANDLED;
}
static void usb_work_func(struct work_struct *work)
{
	printk("%s %s %d bEarlySuspendState=%d  \n",__FILE__,__func__,__LINE__,bEarlySuspendState);

	/*
	* check the screen state:how to check?
	*  check the 5v or USB state:if the state is moved, input the power button?
	* when play music and the screen enter early suspend mode,then the 5v or usb had
	* been inserted?what should we do?
	* 
	* so we don't need check the 5v or usb status. 
	*/
	if(bEarlySuspendState /* && (plat->usbdetect_status() == 0) */){
		//input_event(virtual_power_input,EV_KEY,KEY_POWER,1);
		//input_event(virtual_power_input,EV_KEY,KEY_POWER,0);
		//input_sync(virtual_power_input);  
		imx_virtual_power_report();
	}

}


static int __devinit imx_usb_detect_check_probe(struct platform_device *pdev)
{
	struct mxc_usb_detect_data *plat = pdev->dev.platform_data;
	struct mxc_5vOrUSB_priv *priv = &usb_priv;

	int ret = 0;
	int retval;

	priv->pdev = pdev;



	virtual_power_input = input_allocate_device();
	if (!virtual_power_input) {
		retval = -ENOMEM;
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
	__set_bit(KEY_SPACE,virtual_power_input->keybit); 

         ///////////////////////////////////////
         ///init the  usb  work
         INIT_WORK(&usb_work, usb_work_func);
	///////////////////////////////////////


	ret = request_irq(plat->usb_irq,
		imx_usb_detect_handler,
		IRQ_TYPE_EDGE_BOTH, pdev->name, priv);
	if(ret){
		printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	}
	else {
		enable_irq_wake(plat->usb_irq);
	}
	//enable_irq(plat->usb_irq);

	return 0;

}

static int imx_usb_detect_check_remove(struct platform_device *pdev)
{
	struct mxc_usb_detect_data *plat = pdev->dev.platform_data;
	struct mxc_5vOrUSB_priv *priv = &usb_priv;

	free_irq(plat->usb_irq, priv);


	return 0;
}

#ifdef CONFIG_EARLYSUSPEND
static void mx_usb_detect_check_early_suspend(struct early_suspend *h)
{
	int icount = 0;
	bEarlySuspendState = true;

#if  defined(CONFIG_XRZ_EBOOKREADING_MODE)	
	if(iXrzSuspendMode == 0){
		do{
			icount++;
			//printk("we need wait to display the suspend logo.\n");	
			msleep(100);
		}while(icount <  18);
		icount = 0;
	}
#else/* pair with CONFIG_XRZ_EBOOKREADING_MODE*/	
	do{
		icount++;
		//printk("we need wait to display the suspend logo.\n");	
		msleep(100);
	}while(icount <  18);
	icount = 0;
#endif/*end pair with CONFIG_XRZ_EBOOKREADING_MODE*/	
	
	return;
}

static void mx_usb_detect_check_late_resume(struct early_suspend *h)
{
	bEarlySuspendState = false;
	return;
}

static struct early_suspend imx_usbdetect_check_earlysuspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = mx_usb_detect_check_early_suspend,
	.resume = mx_usb_detect_check_late_resume,
};
#endif

static int imx_usb_detect_check_suspend(struct platform_device *pdev,pm_message_t state)
{
	return 0;
}
static int imx_usb_detect_check_resume(struct platform_device *pdev)
{
	struct mxc_usb_detect_data *plat = pdev->dev.platform_data;
	
	/*
	* we need check whether the 5v or USB had been inserted
	* if it had been connected,we need input the power
	* button message to wakeup the screen.
	*
	* if earlysuspend resume had sended the message,we don't need send it 
	*again.
	*/
	if((!bEarlySuspendState)&& plat->usbdetect_status()){
		//input_event(virtual_power_input,EV_KEY,KEY_POWER,1);
		//input_event(virtual_power_input,EV_KEY,KEY_POWER,0);
		//input_sync(virtual_power_input); 
		imx_virtual_power_report();
	}

	//disable_irq_nosync(plat->usb_irq);
	return 0;
}

static struct platform_driver imx_usb_detect_check = {
	.probe = imx_usb_detect_check_probe,
	.remove = imx_usb_detect_check_remove,
	//#ifndef CONFIG_EARLYSUSPEND	
	.suspend =imx_usb_detect_check_suspend,
	.resume = imx_usb_detect_check_resume,
	//#endif	
	.driver = {
		.name = "usb-detect-check",
		.owner  = THIS_MODULE,
	},
};



static int __init imx_usb_detect_check_init(void)
{
	int ret;
	/////create the usb workqueue.
        usb_wq = create_singlethread_workqueue("usb_wq");
         if (!usb_wq)
         {
           return -ENOMEM;
         }
	ret = platform_driver_register(&imx_usb_detect_check);
	if (ret)
		return -ENOMEM;

#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&imx_usbdetect_check_earlysuspend);
#endif

	return ret;
}

static void __exit imx_usb_detect_check_exit(void)
{
#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&imx_usbdetect_check_earlysuspend);
#endif
	input_unregister_device(virtual_power_input);

	if (virtual_power_input) {
		input_free_device(virtual_power_input);
		virtual_power_input = NULL;
	}

	platform_driver_unregister(&imx_usb_detect_check);

	if (usb_wq)
	{
	   destroy_workqueue(usb_wq);
	}

}

module_init(imx_usb_detect_check_init);
module_exit(imx_usb_detect_check_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("usb-detect");
MODULE_LICENSE("GPL");
