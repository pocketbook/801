//#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>

#if    (defined(CONFIG_ARCH_MX50) )
#include"../../../arch/arm/mach-mx5/mx50_io_cfg.h"
#endif
#include <mach/gpio.h>

#define	DEVICE_NAME 		"xrz_io"
#define	DEVICE_MINJOR		190

extern int imx_virtual_power_report(void);
extern int imx_virtual_space_report(void);
extern int mx50_rdp_get_5vOrUsb_detect_state(void);
//////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///3g  control: open and close.
#define IOCTL_3G_ENABLE     125
#define IOCTL_3G_DISABLE     126

///wifi  control: open and close.
#define IOCTL_WIFI_ENABLE     131
#define IOCTL_WIFI_DISABLE     132
////////////////////////////////

#define IOCTL_PHONE_SGTL5000_ENABLED_3G   127
#define IOCTL_PHONE_SGTL5000_DISABLED_3G    128

//////
//#define IOCTL_WIFI_VALUE    133
#define IOCTL_GSENSOR_SUSPEND_MODE    134
#define IOCTL_GSENSOR_RESUME_MODE      135

////
#define IOCTL_OWIRE_CLK_MODE         136
#define IOCTL_OWIRE_GPIO_MODE       137

#define IOCTL_HARDWARE_SWITCH_STATE   140

#define IOCTL_XRZEBOOKREADING_MODE   141

#define IOCTL_VCOM_VOLTAGE_STATE  150
////////////////////////////////////////////////////////
//bool bwifiState = false;/*default wifi is close */
////////////////////////////////////////////////////////


#if defined(CONFIG_USB_WIFI_RTL8192CU)
int xrz_wifi_open(void)
{
	////wifi power.
	gpio_request(USBWIFI_PWER_EN, "usb-wifi-enable");
	gpio_direction_output(USBWIFI_PWER_EN, 1);


	msleep(100);


	return 0;
}
int xrz_wifi_close(void)
{
	///first ,we need close the wifi power
	gpio_request(USBWIFI_PWER_EN, "usb-wifi-enable");
	gpio_direction_output(USBWIFI_PWER_EN, 0);


	///then we set the wifi power and wifi-3g switch in high
	///impedance state. ---
	gpio_direction_input(USBWIFI_PWER_EN);

	return 0;
}
#endif

static int xrz_app_open(struct inode *inode,struct file *filp)
{
	return 0;
}

static int xrz_app_release(struct inode *inode,struct file *filp)
{
	return 0;
}
static int  xrz_app_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	switch(cmd)
	{
	case IOCTL_WIFI_ENABLE:
#if defined(CONFIG_USB_WIFI_RTL8192CU)
		xrz_wifi_open();
#else
		printk("xrz_wifi_open() not supported.\n");
#endif
		printk("IOCTL_WIFI_ENABLE.\n");
		break;
	case IOCTL_WIFI_DISABLE:
#if defined(CONFIG_USB_WIFI_RTL8192CU)
		xrz_wifi_close();
#else
		printk("xrz_wifi_close() not supported.\n");
#endif
		printk("IOCTL_WIFI_DISABLE.\n");
		break;
	default:
		break;
	}
	return retval;
}


static struct file_operations xrz_app_driver= {
	.owner  	=   THIS_MODULE,
	.open   	=   	xrz_app_open,
	.ioctl	        =  	xrz_app_ioctl,
	.release   =   	xrz_app_release,

};

static struct miscdevice xrz_app_device = {
	.minor		= DEVICE_MINJOR,
	.name		= DEVICE_NAME,
	.fops		= &xrz_app_driver,
};

static int __init xrz_app_init(void)
{
	int ret;
	ret = misc_register(&xrz_app_device);
	if (ret < 0) {
		printk("pvi_io: can't get major number\n");
		return ret;
	}
	return 0;
}

static void __exit xrz_app_exit(void) {
	misc_deregister(&xrz_app_device);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jim_dream");
MODULE_VERSION("2011-3-20");
MODULE_DESCRIPTION ("xrz_io driver");

module_init(xrz_app_init);
module_exit(xrz_app_exit);
