/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/delay.h>


#define STEP_BRIGHTNESS_CHANGE 4
#define CHANGE_PERIOD ((int)(0.01 * HZ))

extern int iXrzSuspendMode;

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	int			(*notify)(struct device *,
					  int brightness);
	int target_brightness;
	struct delayed_work adjust_work;
	int current_brightness;
	int max_brightness;
	spinlock_t	lock;
};
#if defined(CONFIG_ATA5701_TOUCH)
struct pwm_bl_data	*mypb;
struct backlight_device *mybl;
#endif

static int fl_switch;
static int fl_brightness;

/*
 *  * Parse user specified options (`fl_switch=')
 *   * Example:
 *    * 		fl_switch=0
 *     * Note: 
 *      * 		default brightness
 *       */
static int __init switch_setup(char *opt)
{
	int ret;

	printk("frontlight switch setup is %s\n", opt);
				
	ret = strict_strtoul(opt, 0, &fl_switch);
	if (ret < 0)
		return ret;

	return 1;
}

/*
 *  * Parse user specified options (`fl_val=')
 *   * Example:
 *    * 		fl_val=30
 *     * Note: 
 *      * 		default brightness
 *       */
static int __init brightness_setup(char *opt)
{
	int ret;

	printk("frontlight brightness setup is %s\n", opt);
				
	ret = strict_strtoul(opt, 0, &fl_brightness);
	if (ret < 0)
		return ret;

	return 1;
}
__setup("fl_val=", brightness_setup);
__setup("fl_switch=", switch_setup);

static int get_next_step_brightness(int current_brightness, int target_brightness) {
	if (target_brightness > current_brightness + STEP_BRIGHTNESS_CHANGE)
		return current_brightness + STEP_BRIGHTNESS_CHANGE;

	if (target_brightness < current_brightness - STEP_BRIGHTNESS_CHANGE)
		return current_brightness - STEP_BRIGHTNESS_CHANGE;
	
	return target_brightness;
}

extern void  mx50_rdp_backlight_control(bool bOpen);


static void set_pwm_brightness(struct pwm_bl_data *pb, int brightness) 
{
	//printk("<3> smooth bl brightness %d period %d\n", brightness, pb->period);

	if (brightness == 0) {
		if (pwm_clock_enabled(pb->pwm)) {
			mx50_rdp_backlight_control(0);
			pwm_config(pb->pwm, 0, pb->period);
			pwm_disable(pb->pwm);
		}
	} else {
		pwm_config(pb->pwm, brightness * pb->period / pb->max_brightness, pb->period);
		if (!pwm_clock_enabled(pb->pwm)) {
			pwm_enable(pb->pwm);
			mdelay(50);
			mx50_rdp_backlight_control(1);
		}
	}

	pb->current_brightness = brightness;
}

static void adjust_proc(struct work_struct *work) {
	
	struct pwm_bl_data *pb;
	int brightness;
	unsigned long flags;

	pb = container_of(work, struct pwm_bl_data, adjust_work.work);
	
	spin_lock_irqsave(&pb->lock, flags);
	// check if need adjust
	if (pb->current_brightness != pb->target_brightness) {
		brightness = get_next_step_brightness(pb->current_brightness, pb->target_brightness);
		set_pwm_brightness(pb, brightness);
	}
	spin_unlock_irqrestore(&pb->lock, flags);
	
	schedule_delayed_work(&pb->adjust_work, CHANGE_PERIOD);	
};


static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
	int change;
	unsigned long flags;

	//printk("%s %s %d bl->props.power=%d,  bl->props.fb_blank =%d \n",
	//	__FILE__,__func__,__LINE__,bl->props.power,bl->props.fb_blank);


	if (bl->props.power == 0)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (brightness > max)
		brightness = max;

	if (brightness < 0)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	//printk("<2> bl brightness %d period %d\n", brightness, pb->period);

	spin_lock_irqsave(&pb->lock, flags);
	change = brightness - pb->current_brightness;
		
	if (change < 0)
		change = -change;
		
	if( (change > max / 5) || (brightness == 0)) {
		// step adjust
		//printk("<3>Step adjust :brightness=%d\n",brightness);
		set_pwm_brightness(pb, brightness);
	}

	pb->target_brightness = brightness;
	spin_unlock_irqrestore(&pb->lock, flags);
	
	//brightness = get_next_step_brightness(pb->current_brightness, brightness);
	
	//set_pwm_brightness(pb, brightness);
	
	return 0;
}

#if defined(CONFIG_ATA5701_TOUCH)
int mx50_rdp_set_brightness(int newBrightness)
{
	int maxValue = mybl->props.max_brightness;
	int  aveValue = 0;
	int  brigness=0;
	 aveValue = maxValue/40;
     /* printk("%s,newBrightness=%d,maxValue=%d,aveValue=%d,mypb->period=%d\n",__func__,
			newBrightness,
			maxValue,
			aveValue,
			mypb->period);
			*/
	brigness = newBrightness*aveValue;
	//printk("brigness=%d\n",brigness);
        pwm_config(mypb->pwm, brigness * mypb->period / maxValue, mypb->period);
		if (!pwm_clock_enabled(mypb->pwm))
	        pwm_enable(mypb->pwm);
}
#endif
static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl, struct fb_info *info)
{
	return 0;
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb = pwm_backlight_check_fb,
};

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = data->max_brightness;
	props.brightness = fl_brightness;
	props.power = fl_switch ? 1 : 0;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	//bl->props.brightness = data->dft_brightness;
	
	pb->max_brightness = bl->props.max_brightness;
	
	printk("<0>PB address %p\n", pb);

	spin_lock_init(&pb->lock);
	INIT_DELAYED_WORK(&pb->adjust_work, adjust_proc);
	
	//pwm_backlight_update_status(bl);
         
	platform_set_drvdata(pdev, bl);

	backlight_update_status(bl);
	
	schedule_delayed_work(&pb->adjust_work, CHANGE_PERIOD);

	//pwm_config(pb->pwm, 0, pb->period);
	//pwm_disable(pb->pwm);
#if defined(CONFIG_ATA5701_TOUCH)
        if(mypb == NULL)
	     mypb = pb;
	if(mybl == NULL)
		mybl = bl;
#endif	
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	cancel_delayed_work_sync(&pb->adjust_work);
	
	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
#if defined(CONFIG_ATA5701_TOUCH)
        if(mypb != NULL)
	     mypb = NULL;
	if(mybl != NULL)
		mybl = NULL;
#endif	
	
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	
	//printk("%s %s %d state.event=0x%x, clockenable=%d iXrzSuspendMode=%d\n",__FILE__,__func__,__LINE__,
	//  state.event,pwm_clock_enabled(pb->pwm),iXrzSuspendMode); 

	if(iXrzSuspendMode == 0){

		if (state.event == PM_EVENT_SUSPEND)
			return 0;

		//printk(KERN_INFO"Probably need cancel and restart delayed work\n");

		if (pb->notify)
			pb->notify(pb->dev, 0);
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	}
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	if(iXrzSuspendMode == 0){
           if (bl->props.power == 0){
		     bl->props.power = 1;
		  }
		
	     backlight_update_status(bl);
	}
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	        = pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int __init pwm_backlight_init(void)
{
	return platform_driver_register(&pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

