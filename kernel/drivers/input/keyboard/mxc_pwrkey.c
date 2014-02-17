/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/kd.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/powerkey.h>
#include <linux/earlysuspend.h>

struct mxc_pwrkey_priv {

	struct input_dev *input;
	int value;
	int (*get_status) (int);
};

static struct mxc_pwrkey_priv *mxc_pwrkey;
/*extern volatile*/static  bool bsuspending = false;

static int resumed_time;

static void pwrkey_event_handler(void *param)
{
	int pressed;
	static int laststate;

	//printk("%s %s %d bsuspending=%d\n",__FILE__,__func__,__LINE__,bsuspending); 
        if(bsuspending)
        {
            pr_info("%s no handle return.\n", __func__);
            return ;
        }
	pressed = mxc_pwrkey->get_status((int)param);

	if (pressed) {
		input_report_key(
			mxc_pwrkey->input, mxc_pwrkey->value, 1);
		pr_info("%s_Keydown,param=%d\n", __func__,((int)param));
	} else {
		if ((laststate == 0) && (resumed_time != 0) && ((jiffies - resumed_time) < HZ / 2)) { // missed power key press
			input_report_key(
				mxc_pwrkey->input, mxc_pwrkey->value, 1);
			pr_info("%s_additional_Keyup,param=%d\n", __func__,((int)param));
		}

		input_report_key(
			mxc_pwrkey->input, mxc_pwrkey->value, 0);
		pr_info("%s_Keyup,param=%d\n", __func__,((int)param));
	}
	laststate = pressed;
	resumed_time = 0;
}

static int mxcpwrkey_suspend(struct platform_device *dev, pm_message_t state)
{
    bsuspending = true; 	 
    return 0;
}
static int mxcpwrkey_resume(struct platform_device *dev)
{
    bsuspending = false; 	
    resumed_time = jiffies;
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxc_pwrkey_early_suspend(struct early_suspend *h)
{
     // bsuspending = true;      
}

static void mxc_pwrkey_late_resume(struct early_suspend *h)
{
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	bsuspending = false;
}

static struct early_suspend mxc_pwrkey_earlysuspend = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	.suspend = mxc_pwrkey_early_suspend,
	.resume = mxc_pwrkey_late_resume,
};
#endif

static int mxcpwrkey_probe(struct platform_device *pdev)
{
	int retval;
	struct input_dev *input;
	struct power_key_platform_data *pdata = pdev->dev.platform_data;

	if (mxc_pwrkey) {
		dev_warn(&pdev->dev, "two power key??\n");
		return -EBUSY;
	}

	if (!pdata || !pdata->get_key_status) {
		dev_err(&pdev->dev, "can not get platform data\n");
		return -EINVAL;
	}

	mxc_pwrkey = kmalloc(sizeof(struct mxc_pwrkey_priv), GFP_KERNEL);
	if (!mxc_pwrkey) {
		dev_err(&pdev->dev, "can not allocate private data");
		return -ENOMEM;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "no memory for input device\n");
		retval = -ENOMEM;
		goto err1;
	}

	input->name = "mxc_power_key";
	input->phys = "mxcpwrkey/input0";
	input->id.bustype = BUS_HOST;
	input->evbit[0] = BIT_MASK(EV_KEY);

	mxc_pwrkey->value = pdata->key_value;
	mxc_pwrkey->get_status = pdata->get_key_status;
	mxc_pwrkey->input = input;
	pdata->register_pwrkey(pwrkey_event_handler);

	input_set_capability(input, EV_KEY, mxc_pwrkey->value);

	retval = input_register_device(input);
	if (retval < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err2;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
		register_early_suspend(&mxc_pwrkey_earlysuspend);
#endif
	printk(KERN_INFO "PMIC powerkey probe\n");

	return 0;

err2:
	input_free_device(input);
err1:
	kfree(mxc_pwrkey);
	mxc_pwrkey = NULL;

	return retval;
}

static int mxcpwrkey_remove(struct platform_device *pdev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&mxc_pwrkey_earlysuspend);
#endif
	input_unregister_device(mxc_pwrkey->input);
	input_free_device(mxc_pwrkey->input);
	kfree(mxc_pwrkey);
	mxc_pwrkey = NULL;

	return 0;
}

static struct platform_driver mxcpwrkey_driver = {
	.driver = {
		.name = "mxcpwrkey",
	},
	.probe = mxcpwrkey_probe,
	.remove = mxcpwrkey_remove,
	.suspend = mxcpwrkey_suspend,
	.resume =mxcpwrkey_resume,
};

static int __init mxcpwrkey_init(void)
{
	return platform_driver_register(&mxcpwrkey_driver);
}

static void __exit mxcpwrkey_exit(void)
{
	platform_driver_unregister(&mxcpwrkey_driver);
}

module_init(mxcpwrkey_init);
module_exit(mxcpwrkey_exit);


MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("MXC board power key Driver");
MODULE_LICENSE("GPL");
