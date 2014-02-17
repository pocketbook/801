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

//////////////////////////////////////////////////////
///HDMI control:open and close.
//#define IOCTL_HDMI_ENABLE        121
//#define IOCTL_HDMI_DISABLE	122
extern  int my_usbh1_wake_up_enable(bool enable);
extern int mma7660_enter_suspend(void);
extern int  mma7660_enter_resume(void);
extern int  __init mx50_rdp_owire_ssi_ext1_init(void);
extern int __init mx50_rdp_gpio_init();
 extern int mxc_sgtl5000_init(void);
extern int my_i2c2_clk_disabled(void);
extern int my_i2c2_clk_enabled(void);
///////////////////////////////////////////////////////
///Camera control: open and close.
//#define IOCTL_CAMERA_ENABLE     123
//#define IOCTL_CAMERA_DISABLE     124

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

////////////////////////////////////////////////////////
//extern int sgtl5000_register24_write3G(void);
//extern int sgtl5000_register24_writeMp3(void);
bool bwifiState = false;/*default wifi is close */
bool b3GState = false ;/*default 3G is close */
//bool bneedcloseprocess = false;
////////////////////////////////////////////////////////

#if defined(CONFIG_USB_WIFI_RTL8192CU)
int xrz_wifi_open(void)
{
	printk("%s %s %d   \n",__FILE__,__func__,__LINE__);

#if defined(CONFIG_IDIG_C026)
	/*wifi internel power control,now only use for c026 */
	gpio_request(USBWIFI_ECSPI1_SCLK_GPIO4_12_INTERNEL_POWER, "usb-wifi-in-power");
	gpio_direction_output(USBWIFI_ECSPI1_SCLK_GPIO4_12_INTERNEL_POWER, 1);
#else
	///first, open the total power.
	gpio_request(USBWIFI_3G_POWER_CONTROL, "usbwifi-3g-power");
	gpio_direction_output(USBWIFI_3G_POWER_CONTROL, 1);
	///then open the switch..
	gpio_request(USBWIFI_3G_SWAP_ENABLE , "3G-Wifi-power");
	gpio_direction_output(USBWIFI_3G_SWAP_ENABLE, 1);

#endif

	////wifi power.
	gpio_request(USBWIFI_PWER_EN, "usb-wifi-enable");
	gpio_direction_output(USBWIFI_PWER_EN, 1);


	msleep(100);

	//iprocessend =1;

	return 0;
}
int xrz_wifi_close(void)
{
	printk("%s %s %d   \n",__FILE__,__func__,__LINE__);
	///first ,we need close the wifi power
	gpio_request(USBWIFI_PWER_EN, "usb-wifi-enable");
	gpio_direction_output(USBWIFI_PWER_EN, 0);
	// gpio_direction_input(USBWIFI_PWER_EN);
	//msleep(10);
#if defined(CONFIG_IDIG_C026)
	/*wifi internel power control,now only use for c026 */
	gpio_request(USBWIFI_ECSPI1_SCLK_GPIO4_12_INTERNEL_POWER, "usb-wifi-in-power");
	gpio_direction_output(USBWIFI_ECSPI1_SCLK_GPIO4_12_INTERNEL_POWER, 0);

#else
	////then close the wifi 3g switch
	gpio_request(USBWIFI_3G_SWAP_ENABLE , "3G-Wifi-power");
	gpio_direction_output(USBWIFI_3G_SWAP_ENABLE, 0);


	/////////////close the total power
	gpio_request(USBWIFI_3G_POWER_CONTROL, "usbwifi-3g-power");
	gpio_direction_output(USBWIFI_3G_POWER_CONTROL, 0);

	///then we set the wifi power and wifi-3g switch in high
	///impedance state. ---
	gpio_direction_input(USBWIFI_3G_POWER_CONTROL);
	gpio_direction_input(USBWIFI_3G_SWAP_ENABLE);
#endif

	gpio_direction_input(USBWIFI_PWER_EN);

#if defined(CONFIG_IDIG_C026)
	gpio_direction_input(USBWIFI_ECSPI1_SCLK_GPIO4_12_INTERNEL_POWER);
#endif
	return 0;
}
#endif
#if defined(CONFIG_3G_ENABLED)
int xrz_3g_open(void)
{

#if defined(CONFIG_RTL8192CU_MODULE)
	///first, open the total power.
	gpio_request(USBWIFI_3G_POWER_CONTROL, "usbwifi-3g-power");
	gpio_direction_output(USBWIFI_3G_POWER_CONTROL, 1);

	////we need close the wifi power
	gpio_request(USBWIFI_PWER_EN, "usb-wifi-enable");
	gpio_direction_output(USBWIFI_PWER_EN, 0);

	//then open the switch
	gpio_request(USBWIFI_3G_SWAP_ENABLE , "3G-Wifi-power");
	gpio_direction_output(USBWIFI_3G_SWAP_ENABLE, 0);
#endif

	///last ,open 3g  power enable
	gpio_request(PIN_3G_PWER_ENABLE , "3g-power");
	gpio_direction_output(PIN_3G_PWER_ENABLE, 1);
	mdelay(20);

	return 0;
}
int xrz_3g_close(void)
{
	printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	///close the  3g power supply,
	gpio_request(PIN_3G_PWER_ENABLE , "3g-power");
	gpio_direction_output(PIN_3G_PWER_ENABLE, 0);

#if defined(CONFIG_RTL8192CU_MODULE)
	///close the 3g wifi switch ,sent high or low as the same
	gpio_direction_output(USBWIFI_3G_SWAP_ENABLE, 1);
#endif

	//close the total power(3G and wifi power)
	gpio_request(USBWIFI_3G_POWER_CONTROL, "usbwifi-3g-power");
	gpio_direction_output(USBWIFI_3G_POWER_CONTROL, 0);

	///then we set the 3g power and wifi-3g switch in high
	///impedance state. ---
	gpio_direction_input(USBWIFI_3G_POWER_CONTROL);
	gpio_direction_input(PIN_3G_PWER_ENABLE);
	gpio_direction_input(USBWIFI_3G_SWAP_ENABLE);

	return 0;
}
#endif
static int xrz_app_open(struct inode *inode,struct file *filp)
{
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	return 0;
}

static int xrz_app_release(struct inode *inode,struct file *filp)
{
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	return 0;
}
//int power_state =0;
static int  xrz_app_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	//printk("%s %s %d bneedcloseprocess =%d\n",__FILE__,__func__,__LINE__,bneedcloseprocess);
	//static int wificount =0;
	//static int iprocessend =0;
	//static bool bneedcloseprocess = false;
	//static bool bOwireGpioState = false;
	switch(cmd)
	{
		//#if 0
		//case IOCTL_HDMI_ENABLE:
		//    sii902x_open();//sii9022_open();//sii902x_open();
		//		sii902x_close();//sii9022_open();//sii902x_close();
		//	    sii902x_open();//sii9022_open();//sii902x_open();
		//	printk("%s %s %s \n",__FILE__,__func__,"IOCTL_HDMI_ENABLE");
		//	break;
		//case IOCTL_HDMI_DISABLE:
		//	sii902x_close();//sii9022_open();//sii902x_close();
		//	printk("%s %s %s \n",__FILE__,__func__,"IOCTL_HDMI_DISABLE");
		//	break;
		//#endif
#if defined(CONFIG_3G_ENABLED)
	case IOCTL_3G_ENABLE:
		printk("IOCTL_3G_ENABLE.\n");
		b3GState = true;
		xrz_3g_open();
		//mx50_rdp_owire_ssi_ext1_init();
		//mxc_sgtl5000_init();
		break;
	case IOCTL_3G_DISABLE:
		printk("IOCTL_3G_DISABLE.\n");
		xrz_3g_close();
		//mx50_rdp_gpio_init();
		b3GState = false;
		break;
#endif
	case IOCTL_PHONE_SGTL5000_ENABLED_3G:
		///////////////////////////////////////////////////////////////////
		//////Microphone control H:3g-call ;L: Internal Microphone,default:L
		///////////////////////////////////////////////////////////////////
		//#if (defined(CONFIG_ARCH_MX53) && defined(CONFIG_MACH_MX53_LOCO))
		//gpio_request(MX53_LOCO_MICROPHONE_EIM_D31_GPIO3_31_CONTROL, "microphone");
		//gpio_direction_output(MX53_LOCO_MICROPHONE_EIM_D31_GPIO3_31_CONTROL,1);
		//sgtl5000_register24_write3G();
		//#endif
		break;
	case IOCTL_PHONE_SGTL5000_DISABLED_3G:
		//////////////////////////////////////////////////
		//close the microphone.
		//////////////////////////////////////////////////
		//#if (defined(CONFIG_ARCH_MX53) && defined(CONFIG_MACH_MX53_LOCO))
		//gpio_direction_output(MX53_LOCO_MICROPHONE_EIM_D31_GPIO3_31_CONTROL,0);
		//sgtl5000_register24_writeMp3();
		//#endif
		break;
	case IOCTL_WIFI_ENABLE:
		bwifiState = true;
		//bneedcloseprocess = true;
		//msleep(5);
		//if(!bneedcloseprocess){
		//my_usbh1_wake_up_enable(true);
		//msleep(5);
		xrz_wifi_open();
		printk("IOCTL_WIFI_ENABLE.\n");
		//}
		break;
	case IOCTL_WIFI_DISABLE:
		bwifiState = false;
		//if(!bneedcloseprocess){
		xrz_wifi_close();
		//msleep(5);
		//my_usbh1_wake_up_enable(false);
		//bneedcloseprocess = false;
		printk("IOCTL_WIFI_DISABLE.\n");
		//}
		break;
#if defined(CONFIG_SENSORS_MMA7660)
	case IOCTL_GSENSOR_SUSPEND_MODE:
		printk("IOCTL_GSENSOR_SUSPEND_MODE.\n");
		//mma7660_enter_suspend();
		//msleep(10);
		//my_i2c2_clk_disabled();
		break;
	case IOCTL_GSENSOR_RESUME_MODE:
		printk("IOCTL_GSENSOR_RESUME_MODE.\n");
		//my_i2c2_clk_enabled();
		//msleep(10);
		//mma7660_enter_resume();
		break;
#endif
#if 0
#if defined(CONFIG_SND_SOC_SGTL5000)
         ///only for sgtl5000
         case IOCTL_OWIRE_CLK_MODE:
		if(bOwireGpioState){
		    printk("IOCTL_OWIRE_CLK_MODE.\n");
		  // mx50_rdp_owire_ssi_ext1_init();
		   bOwireGpioState = false;
		}
		break;
	 case IOCTL_OWIRE_GPIO_MODE:
	 	if(!bOwireGpioState){
			printk("IOCTL_OWIRE_GPIO_MODE.\n");
		 	//mx50_rdp_gpio_init();
			bOwireGpioState = true;
	 	}
	 	break;
#endif
#endif
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
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	ret = misc_register(&xrz_app_device);
	if (ret < 0) {
		printk("pvi_io: can't get major number\n");
		return ret;
	}
	return 0;
}

static void __exit xrz_app_exit(void) {
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	misc_deregister(&xrz_app_device);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jim_dream");
MODULE_VERSION("2011-3-20");
MODULE_DESCRIPTION ("xrz_io driver");

module_init(xrz_app_init);
module_exit(xrz_app_exit);
