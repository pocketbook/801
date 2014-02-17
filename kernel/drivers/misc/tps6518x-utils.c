/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
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
 *
 */
/*
 * tps6518x-utils.c
 *
 * Based on the MAX1619 driver.
 * Copyright (C) 2003-2004 Alexey Fisher <fishor@mail.ru>
 *                         Jean Delvare <khali@linux-fr.org>
 *
 * The TPS6518x is a chip made by Texass Instruments.
 * It reports up to two temperatures (its own plus up to
 * one external one).
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/tps6518x.h>
#include <linux/gpio.h>

#define EPDC_PWRCTRL0	(2*32 + 29)	/*GPIO_3_29 */
#define EPDC_PWRCTRL1	(2*32 + 30)	/*GPIO_3_30 */
#define EPDC_PWRCTRL2	(2*32 + 31)	/*GPIO_3_31 */
#define EPDC_VCOM	(3*32 + 21)	/*GPIO_4_21 */

static int gpio_pwrctl2;
static int gpio_pwrctl2_state=-1;

/*
 * Conversions
 */
/*
 * Functions declaration
 */
static int tps6518x_utils_probe(struct platform_device *pdev);
static int tps6518x_utils_remove(struct platform_device *pdev);

/*
 * Driver data (common to all clients)
 */
static struct platform_driver tps6518x_utils_driver = {
	.probe = tps6518x_utils_probe,
	.remove = tps6518x_utils_remove,
	.driver = {
		.name = "tps6518x_utils",
	},
};


/*
 * Client data (each client gets its own)
 */
struct tps6518x_data {
	struct device *misc_dev;
};

/*
 * Sysfs stuff
 */
static ssize_t show_pwr_regs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	/*
	 * report the status register value
	 */
	tps6518x_reg_read(REG_TPS6518x_PG,&reg_val);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

static ssize_t show_intr_regs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned int intr_reg_val;
	/*
	 * get the interrupt status register value
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    tps6518x_reg_read(REG_TPS65180_INT1, &intr_reg_val);
		    tps6518x_reg_read(REG_TPS65180_INT2, &reg_val);
		    intr_reg_val |= reg_val<<8;
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_read(REG_TPS65185_INT1, &intr_reg_val);
		    tps6518x_reg_read(REG_TPS65185_INT2, &reg_val);
		    intr_reg_val |= reg_val<<8;
	        break;
	   default:
		break;	

	}

	return snprintf(buf, PAGE_SIZE, "%d\n", intr_reg_val);
}

static ssize_t show_vcom(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned int cur_reg_val; /* current register value */
	unsigned int cur_reg2_val; /* current register value */
	unsigned int cur_fld_val; /* current bitfield value*/
	int vcomValue=0;
	/*
	 * get the vcom registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		   tps6518x_reg_read(REG_TPS65180_VCOM_ADJUST, &cur_reg_val);
		   cur_fld_val = BITFEXT(cur_reg_val, VCOM_SET);
		   vcomValue = vcom_rs_to_uV(cur_fld_val);
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		   tps6518x_reg_read(REG_TPS65185_VCOM1,&cur_reg_val);
		   tps6518x_reg_read(REG_TPS65185_VCOM2,&cur_reg2_val);
		   cur_reg_val |= 256 * (1 & cur_reg2_val);
		   vcomValue = vcom2_rs_to_uV(cur_reg_val);
	        break;
	   default:
		break;	

	}

	return snprintf(buf, PAGE_SIZE, "%d\n", vcomValue);
}

static ssize_t set_vcom(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int new_reg_val; /* new register value to write */
	int retval;

	long vcom_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the vcom register value
	 */
	 if ((vcom_reg_val < 200000)&&(vcom_reg_val >( -200000)))
	{
		printk(" %s too small return uV=%ld\n",__func__,vcom_reg_val); 
		return 0;
	}
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		   tps6518x_reg_read(REG_TPS65180_VCOM_ADJUST,&cur_reg_val);
		   new_reg_val = to_reg_val(cur_reg_val,
			   BITFMASK(VCOM_SET),
			   BITFVAL(VCOM_SET, vcom_uV_to_rs(vcom_reg_val)));
		   
		   retval = tps6518x_reg_write(REG_TPS65180_VCOM_ADJUST,
			   new_reg_val);
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		   retval = tps6518x_reg_write(REG_TPS65185_VCOM1,
				   vcom2_uV_to_rs(vcom_reg_val) & 255);
			   tps6518x_reg_read( REG_TPS65185_VCOM2,&cur_reg_val);
			   new_reg_val = to_reg_val(cur_reg_val,
				   BITFMASK(VCOM2_SET),
				   BITFVAL(VCOM2_SET, vcom2_uV_to_rs(vcom_reg_val)/256));
		   
	        retval = tps6518x_reg_write(REG_TPS65185_VCOM2,
				   new_reg_val);
	        break;
	   default:
		break;	

	}

	return count;
}

static ssize_t show_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	/*
	 * get the enable registers
	 */
	 //printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	tps6518x_reg_read(REG_TPS65185_ENABLE,&reg_val);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

static ssize_t set_enable(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long enable_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the ENABLE register
	 */
        //printk("%s %s %d  enable_reg_val=%ld \n",__FILE__,__func__,__LINE__,enable_reg_val); 
	tps6518x_reg_write(REG_TPS65185_ENABLE, enable_reg_val);

	return count;
}

static ssize_t show_upseq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned long upseq_reg_val=0l;
	/*
	 * get the upseq registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ0, &reg_val);
		    upseq_reg_val = reg_val << 16;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ1, &reg_val);
		    upseq_reg_val |= reg_val << 8;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ2, &reg_val);
		    upseq_reg_val |= reg_val;
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_read(REG_TPS65185_UPSEQ0, &reg_val);
		    upseq_reg_val = reg_val << 8;
		    tps6518x_reg_read(REG_TPS65185_UPSEQ1, &reg_val);
		    upseq_reg_val |= reg_val;
	        break;
	   default:
		break;	

	}

	return snprintf(buf, PAGE_SIZE, "0x%06lx\n", upseq_reg_val);
}

static ssize_t set_upseq(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	long upseq_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the power up sequence registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    tps6518x_reg_write(REG_TPS65180_PWRSEQ0, (upseq_reg_val >>16) & 0xff);
		    tps6518x_reg_write(REG_TPS65180_PWRSEQ1, (upseq_reg_val >>8) & 0xff);
		    tps6518x_reg_write(REG_TPS65180_PWRSEQ2, (upseq_reg_val) & 0xff);
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_write(REG_TPS65185_UPSEQ0, (upseq_reg_val >>8) & 0xff);
		    tps6518x_reg_write(REG_TPS65185_UPSEQ1, (upseq_reg_val) & 0xff);
	        break;
	   default:
		break;	

	}

	return count;
}

static ssize_t show_dwnseq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned long dwnseq_reg_val=0l;
	/*
	 * get the power down registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    printk(KERN_ERR "Warning: This is the same as pmic_upseq register values.\n");
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ0, &reg_val);
		    dwnseq_reg_val = reg_val << 16;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ1, &reg_val);
		    dwnseq_reg_val |= reg_val << 8;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ2, &reg_val);
		    dwnseq_reg_val |= reg_val;
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_read(REG_TPS65185_DWNSEQ0, &reg_val);
		    dwnseq_reg_val = reg_val << 8;
		    tps6518x_reg_read(REG_TPS65185_DWNSEQ1, &reg_val);
		    dwnseq_reg_val |= reg_val;
	        break;
	   default:
		break;	

	}

	return snprintf(buf, PAGE_SIZE, "0x%06lx\n", dwnseq_reg_val);
}

static ssize_t set_dwnseq(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	long dwnseq_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the power down sequencing registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    printk(KERN_ERR "Please use pmic_upseq attr to change power sequencing.\n");
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_write(REG_TPS65185_DWNSEQ0, (dwnseq_reg_val >>8) & 0xff);
		    tps6518x_reg_write(REG_TPS65185_DWNSEQ1, (dwnseq_reg_val) & 0xff);
	        break;
	   default:
		break;	

	}

	return count;
}

static ssize_t set_vcom_ctrl(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long vcomCtrl = simple_strtol(buf,NULL,0);

	gpio_set_value(EPDC_VCOM, vcomCtrl);

	return count;
}

static ssize_t show_pwrctrl2(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", gpio_pwrctl2_state);

}

static ssize_t set_pwrctrl2(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long pwrctrl2 = simple_strtol(buf,NULL,0);

	if (gpio_pwrctl2_state == -1)
	   printk(KERN_ERR "pwrctrl2 pin is not initialized!\n");
	else
	   gpio_set_value(gpio_pwrctl2, (gpio_pwrctl2_state = pwrctrl2));

	return count;
}

static ssize_t init_pwrctrl2(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{

	gpio_pwrctl2 = EPDC_PWRCTRL2;
	gpio_request(gpio_pwrctl2, "epdc-pwrctrl2");
	gpio_direction_output(gpio_pwrctl2, (gpio_pwrctl2_state = 1));

	return count;
}

static DEVICE_ATTR(pwr_status, S_IRUGO, show_pwr_regs, NULL);
static DEVICE_ATTR(intr_input, S_IRUGO, show_intr_regs, NULL);
static DEVICE_ATTR(vcom_value, S_IWUSR | S_IRUGO, show_vcom, set_vcom);
static DEVICE_ATTR(pmic_enable, S_IWUSR | S_IRUGO, show_enable, set_enable);
static DEVICE_ATTR(vcom_ctrl, S_IWUSR, NULL, set_vcom_ctrl);
static DEVICE_ATTR(pmic_upseq, S_IWUSR | S_IRUGO, show_upseq, set_upseq);
static DEVICE_ATTR(pmic_dwnseq, S_IWUSR | S_IRUGO, show_dwnseq, set_dwnseq);
static DEVICE_ATTR(gpio_pwrctrl2, S_IWUSR | S_IRUGO, show_pwrctrl2, set_pwrctrl2);
static DEVICE_ATTR(init_pwrctrl2, S_IWUSR,  NULL, init_pwrctrl2);

static struct attribute *tps6518x_attributes[] = {
	&dev_attr_pwr_status.attr,
	&dev_attr_intr_input.attr,
	&dev_attr_vcom_value.attr,
	&dev_attr_pmic_enable.attr,
	&dev_attr_vcom_ctrl.attr,
	&dev_attr_pmic_upseq.attr,
	&dev_attr_pmic_dwnseq.attr,
	&dev_attr_gpio_pwrctrl2.attr,
	&dev_attr_init_pwrctrl2.attr,
	NULL
};

static const struct attribute_group tps6518x_group = {
	.attrs = tps6518x_attributes,
};

/*
 * Real code
 */
static int tps6518x_utils_probe(struct platform_device *pdev)
{
	struct tps6518x_data *data;
	int err;
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	data = kzalloc(sizeof(struct tps6518x_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &tps6518x_group);
	if (err)
		goto exit_free;

	platform_set_drvdata(pdev, data);
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int tps6518x_utils_remove(struct platform_device *pdev)
{
	struct tps6518x_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &tps6518x_group);

	kfree(data);
	return 0;
}

static int __init utilss_tps6518x_init(void)
{
 	return platform_driver_register(&tps6518x_utils_driver);
}
module_init(utilss_tps6518x_init);

static void __exit utilss_tps6518x_exit(void)
{
	platform_driver_unregister(&tps6518x_utils_driver);
}
module_exit(utilss_tps6518x_exit);

MODULE_DESCRIPTION("TPS6518x utils driver");
MODULE_LICENSE("GPL");

