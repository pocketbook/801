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
 * max17135-utils.c
 *
 * Based on the MAX1619 driver.
 * Copyright (C) 2003-2004 Alexey Fisher <fishor@mail.ru>
 *                         Jean Delvare <khali@linux-fr.org>
 *
 * The max17135 is a chip made by Texass Instruments.
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
#include <linux/mfd/max17135.h>
#include <linux/gpio.h>

#define EPDC_PWRCTRL0	(2*32 + 29)	/*GPIO_3_29 */
#define EPDC_PWRCTRL1	(2*32 + 30)	/*GPIO_3_30 */
#define EPDC_PWRCTRL2	(2*32 + 31)	/*GPIO_3_31 */

static int gpio_pwrctl2;
static int gpio_pwrctl2_state=-1;

/*
 * Conversions
 */
/*
 * Functions declaration
 */
static int max17135_utils_probe(struct platform_device *pdev);
static int max17135_utils_remove(struct platform_device *pdev);

/*
 * Driver data (common to all clients)
 */
static struct platform_driver max17135_utils_driver = {
	.probe = max17135_utils_probe,
	.remove = max17135_utils_remove,
	.driver = {
		.name = "max17135_utils",
	},
};


/*
 * Client data (each client gets its own)
 */
struct max17135_data {
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
	max17135_reg_read(REG_MAX17135_STATUS,&reg_val);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

static ssize_t show_fault_regs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	/*
	 * report the fault register value
	 */
	max17135_reg_read(REG_MAX17135_FAULT,&reg_val);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

static ssize_t show_vcom(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	/*
	 * report the value of DVR register
	 */
	max17135_reg_read(REG_MAX17135_DVR,&reg_val);

	return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

#define PROG_ENB 0x4000
static ssize_t set_vcom(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long vcom_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the DVR value
	 */
	max17135_reg_write(REG_MAX17135_DVR, vcom_reg_val & 0xff);
	/*
	 * check if the program bit is set
	 */
	if (vcom_reg_val & PROG_ENB)
	{
	    /* power up the pmic */
	    max17135_reg_write(REG_MAX17135_ENABLE, 0x1);
	    msleep(200);
	    /* write DVR to memory */
	    if (max17135_reg_write(REG_MAX17135_PRGM_CTRL, 0x1))
		printk(KERN_ERR "--Cannot commit VCOM to non-volatile memory!\n");
	    /* power down the pmic */
	    max17135_reg_write(REG_MAX17135_ENABLE, 0x0);
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
	max17135_reg_read(REG_MAX17135_ENABLE,&reg_val);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

/*
 * power up the pmic rails
 */
static ssize_t set_enable(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long enable_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the ENABLE register
	 */
	max17135_reg_write(REG_MAX17135_ENABLE, enable_reg_val & 0x3);

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
	max17135_reg_read(REG_MAX17135_TIMING1,&reg_val);
	upseq_reg_val = reg_val << 24;
	max17135_reg_read(REG_MAX17135_TIMING2, &reg_val);
	upseq_reg_val = reg_val << 16;
	max17135_reg_read(REG_MAX17135_TIMING3, &reg_val);
	upseq_reg_val |= reg_val << 8;
	max17135_reg_read(REG_MAX17135_TIMING4, &reg_val);
	upseq_reg_val |= reg_val;

	return snprintf(buf, PAGE_SIZE, "0x%08lx\n", upseq_reg_val);
}

static ssize_t set_upseq(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	long upseq_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the power up sequence registers
	 */
	max17135_reg_write(REG_MAX17135_TIMING1, (upseq_reg_val >>24) & 0xff);
	max17135_reg_write(REG_MAX17135_TIMING2, (upseq_reg_val >>16) & 0xff);
	max17135_reg_write(REG_MAX17135_TIMING3, (upseq_reg_val >>8) & 0xff);
	max17135_reg_write(REG_MAX17135_TIMING4, (upseq_reg_val) & 0xff);

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
	max17135_reg_read(REG_MAX17135_TIMING5,&reg_val);
	dwnseq_reg_val = reg_val << 24;
	max17135_reg_read(REG_MAX17135_TIMING6, &reg_val);
	dwnseq_reg_val = reg_val << 16;
	max17135_reg_read(REG_MAX17135_TIMING7, &reg_val);
	dwnseq_reg_val |= reg_val << 8;
	max17135_reg_read(REG_MAX17135_TIMING8, &reg_val);
	dwnseq_reg_val |= reg_val;

	return snprintf(buf, PAGE_SIZE, "0x%08lx\n", dwnseq_reg_val);
}

static ssize_t set_dwnseq(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	long dwnseq_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the power down sequencing registers
	 */
	max17135_reg_write(REG_MAX17135_TIMING5, (dwnseq_reg_val >>24) & 0xff);
	max17135_reg_write(REG_MAX17135_TIMING6, (dwnseq_reg_val >>16) & 0xff);
	max17135_reg_write(REG_MAX17135_TIMING7, (dwnseq_reg_val >>8) & 0xff);
	max17135_reg_write(REG_MAX17135_TIMING8, (dwnseq_reg_val) & 0xff);

	return count;
}

static ssize_t commit_timings(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long timing_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * check if the program bit is set
	 */
	if ((timing_reg_val & PROG_ENB) == (PROG_ENB || 0x2))
	{
	    /* power up the pmic */
	    max17135_reg_write(REG_MAX17135_ENABLE, 0x1);
	    msleep(200);
	    /* write timings to memory */
	    if (max17135_reg_write(REG_MAX17135_PRGM_CTRL, 0x2))
		printk(KERN_ERR "--Cannot commit timings to non-volatile memory!\n");
	    /* power down the pmic */
	    max17135_reg_write(REG_MAX17135_ENABLE, 0x0);
	}

	return count;
}

static ssize_t set_pwrctrl0(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long pwrctrl0 = simple_strtol(buf,NULL,0);

	gpio_set_value(EPDC_PWRCTRL0, (pwrctrl0 & 1));

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
static DEVICE_ATTR(fault_input, S_IRUGO, show_fault_regs, NULL);
static DEVICE_ATTR(vcom_value, S_IWUSR | S_IRUGO, show_vcom, set_vcom);
static DEVICE_ATTR(pmic_enable, S_IWUSR | S_IRUGO, show_enable, set_enable);
static DEVICE_ATTR(pmic_upseq, S_IWUSR | S_IRUGO, show_upseq, set_upseq);
static DEVICE_ATTR(pmic_dwnseq, S_IWUSR | S_IRUGO, show_dwnseq, set_dwnseq);
static DEVICE_ATTR(pmic_commit_timings, S_IWUSR,  NULL, commit_timings);
static DEVICE_ATTR(pmic_v3p3, S_IWUSR,  NULL, set_pwrctrl0);
static DEVICE_ATTR(gpio_pwrctrl2, S_IWUSR | S_IRUGO, show_pwrctrl2, set_pwrctrl2);
static DEVICE_ATTR(init_pwrctrl2, S_IWUSR,  NULL, init_pwrctrl2);

static struct attribute *max17135_attributes[] = {
	&dev_attr_pwr_status.attr,
	&dev_attr_fault_input.attr,
	&dev_attr_vcom_value.attr,
	&dev_attr_pmic_enable.attr,
	&dev_attr_pmic_upseq.attr,
	&dev_attr_pmic_dwnseq.attr,
	&dev_attr_pmic_commit_timings.attr,
	&dev_attr_pmic_v3p3.attr,
	&dev_attr_gpio_pwrctrl2.attr,
	&dev_attr_init_pwrctrl2.attr,
	NULL
};

static const struct attribute_group max17135_group = {
	.attrs = max17135_attributes,
};

/*
 * Real code
 */
static int max17135_utils_probe(struct platform_device *pdev)
{
	struct max17135_data *data;
	int err;

	data = kzalloc(sizeof(struct max17135_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &max17135_group);
	if (err)
		goto exit_free;

	platform_set_drvdata(pdev, data);

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int max17135_utils_remove(struct platform_device *pdev)
{
	struct max17135_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &max17135_group);

	kfree(data);
	return 0;
}

static int __init utilss_max17135_init(void)
{
	return platform_driver_register(&max17135_utils_driver);
}
module_init(utilss_max17135_init);

static void __exit utilss_max17135_exit(void)
{
	platform_driver_unregister(&max17135_utils_driver);
}
module_exit(utilss_max17135_exit);

MODULE_DESCRIPTION("max17135 utils driver");
MODULE_LICENSE("GPL");

