/* drivers/misc/timed_output.c
*
* Copyright (C) 2009 Google, Inc.
* Author: Mike Lockwood <lockwood@android.com>
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>

#include "timed_output.h"
#include <asm/mach/time.h>
#include <linux/android_alarm.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/sysdev.h>
#include <linux/wakelock.h>

#include <asm/gpio.h>
#include <mach/gpio.h>

//#if defined(CONFIG_MACH_MX51_BABBAGE)
//#include "./../arch/arm/mach-mx5/xrz_config.h"
//#elif  (defined(CONFIG_ARCH_MX53) && defined(CONFIG_MACH_MX53_LOCO))
//#include "./../arch/arm/mach-mx5/xrz_mx53_loco_config.h"
//#endif
//#include "./../arch/arm/mach-mx5/mx50_io_cfg.h"

static struct class *timed_output_class;
static atomic_t device_count;

struct work_struct work_vibrator_on,work_vibrator_off;
struct hrtimer vibe_timer;


extern int hrtimer_cancel(struct hrtimer *timer);
extern  int  hrtimer_start(struct hrtimer *timer, ktime_t tim, const enum hrtimer_mode mode);


static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
						   char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	int remaining = tdev->get_time(tdev);

	return sprintf(buf, "%d\n", remaining);
}

static ssize_t enable_store(
struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);
	tdev->enable(tdev, value);

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);

static int create_timed_output_class(void)
{
	if (!timed_output_class) {
		timed_output_class = class_create(THIS_MODULE, "timed_output");
		if (IS_ERR(timed_output_class))
			return PTR_ERR(timed_output_class);
		atomic_set(&device_count, 0);
	}

	return 0;
}

int timed_output_dev_register(struct timed_output_dev *tdev)
{
	int ret;

	if (!tdev || !tdev->name || !tdev->enable || !tdev->get_time)
		return -EINVAL;

	ret = create_timed_output_class();
	if (ret < 0)
		return ret;

	tdev->index = atomic_inc_return(&device_count);
	tdev->dev = device_create(timed_output_class, NULL,
		MKDEV(0, tdev->index), NULL, tdev->name);
	if (IS_ERR(tdev->dev))
		return PTR_ERR(tdev->dev);

	ret = device_create_file(tdev->dev, &dev_attr_enable);
	if (ret < 0)
		goto err_create_file;

	dev_set_drvdata(tdev->dev, tdev);
	tdev->state = 0;
	return 0;

err_create_file:
	device_destroy(timed_output_class, MKDEV(0, tdev->index));
	printk(KERN_ERR "timed_output: Failed to register driver %s\n",
		tdev->name);

	return ret;
}
EXPORT_SYMBOL_GPL(timed_output_dev_register);

void timed_output_dev_unregister(struct timed_output_dev *tdev)
{
	device_remove_file(tdev->dev, &dev_attr_enable);
	device_destroy(timed_output_class, MKDEV(0, tdev->index));
	dev_set_drvdata(tdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(timed_output_dev_unregister);

static void motor_set_pmic_vibrator(int enable)
{
	if (enable == 0)
	{
		gpio_request(MOTOR_ENABLE , "motor-power");
		gpio_direction_output(MOTOR_ENABLE, 0);
	}
	else if(enable == 1)
	{
		gpio_request(MOTOR_ENABLE , "motor-power");
		gpio_direction_output(MOTOR_ENABLE, 1);

	}
}
static void pmic_vibrator_on(struct work_struct *work)
{
	motor_set_pmic_vibrator(1);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	motor_set_pmic_vibrator(0);
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else{
		return 0;
	}
}
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);

	if (value == 0){
		timed_vibrator_off(dev);
	}else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}
}


static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	timed_vibrator_off(NULL);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init pxa_init_pmic_vibrator(void)
{
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;
	printk( "pxa_init_pmic_vibrator_new \n");
	timed_output_dev_register(&pmic_vibrator);
}

static int __init timed_output_init(void)
{
	printk( "timed_output_init \n");

	pxa_init_pmic_vibrator();

	return create_timed_output_class();
}

static void __exit timed_output_exit(void)
{
	printk( "timed_output_exit \n");

	timed_output_dev_unregister(&pmic_vibrator);

	class_destroy(timed_output_class);
}

module_init(timed_output_init);
module_exit(timed_output_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("timed output class driver");
MODULE_LICENSE("GPL");
