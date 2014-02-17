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

#define	DEVICE_NAME 		"io_enc"
#define	DEVICE_MINJOR		191


#define IOCTL_SYS_ENCYPT	(150)
#define IOCTL_SYS_ENCYPT1	(1500)
#define IOCTL_SYS_ENCYPT2	(25000)
#define IOCTL_SYS_ENCYPT3	(450000)
#define IOCTL_SYS_ENCYPT4	(6000000)
#define IOCTL_SYS_ENCYPT5	(15000000)
#define IOCTL_SYS_ENCYPT6	(1500000000)
#define IOCTL_SYS_ENCYPT7	(3500000000)


#define IOCTL_SYS_ENCYPT_RET    (888)
#define IOCTL_SYS_ENCYPT_RET1 	(10100)
#define IOCTL_SYS_ENCYPT_RET2 	(11001)
#define IOCTL_SYS_ENCYPT_RET3 	(11301)
#define IOCTL_SYS_ENCYPT_RET4 	(12331)
#define IOCTL_SYS_ENCYPT_RET5 	(32435)
#define IOCTL_SYS_ENCYPT_RET6 	(42430)
#define IOCTL_SYS_ENCYPT_RET7 	(41130)


extern int EncrypICCheck(int x);
extern int EncrypICCheck1(int x);
extern int EncrypICCheck2(int x);
extern int EncrypICCheck3(int x);

static int enc_app_open(struct inode *inode,struct file *filp)
{
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	return 0;
}

static int enc_app_release(struct inode *inode,struct file *filp)
{
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	return 0;
}
//int power_state =0; 
static int  enc_app_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	switch(cmd)
	{
	case IOCTL_SYS_ENCYPT:
		if(EncrypICCheck(0)==65531)
		       	retval = IOCTL_SYS_ENCYPT_RET;
		break;
	case IOCTL_SYS_ENCYPT1:
		if(EncrypICCheck1(1)==24531)	       
	  	       	retval = IOCTL_SYS_ENCYPT_RET1;
		break;
	default:
		break;
	}
	return retval;
}	


static struct file_operations enc_app_driver= {
	.owner  	=   THIS_MODULE,
	.open   	=   	enc_app_open,
	.ioctl	        =  	enc_app_ioctl,
	.release   =   	enc_app_release,

};

static struct miscdevice enc_app_device = {
	.minor		= DEVICE_MINJOR,
	.name		= DEVICE_NAME,
	.fops		= &enc_app_driver,
};

static int __init enc_app_init(void)
{
	int ret;

	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	ret = misc_register(&enc_app_device);
	if (ret < 0) {
		printk("pvi_io: can't get major number\n");
		return ret;
	}
	return 0;
}

static void __exit enc_app_exit(void) {
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
	misc_deregister(&enc_app_device);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lam");
MODULE_VERSION("2012-2-5");
MODULE_DESCRIPTION ("ENC_IO driver");

module_init(enc_app_init);
module_exit(enc_app_exit);
