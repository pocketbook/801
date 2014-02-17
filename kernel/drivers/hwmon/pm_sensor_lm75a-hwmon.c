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
 * tps6518x-hwmon.c
 *
 * Based on the MAX1619 driver.
 * Copyright (C) 2003-2004 Alexey Fisher <fishor@mail.ru>
 *                         Jean Delvare <khali@linux-fr.org>
 *
 * The TPS6518x is a sensor chip made by Texass Instruments.
 * It reports up to two temperatures (its own plus up to
 * one external one).
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/pm_sensor_lm75a.h>

/*
 * Conversions
 */
static int temp_from_reg(int val)
{
	return val;
}

/*
* Functions declaration
*/
static int pm_sensor_lm75a_probe(struct platform_device *pdev);
static int pm_sensor_lm75a_remove(struct platform_device *pdev);

/*
* Driver data (common to all clients)
*/
static struct platform_driver pm_sensor_lm75a_driver = {
	.probe = pm_sensor_lm75a_probe,
	.remove = pm_sensor_lm75a_remove,
	.driver = {
		.name = "pm_sensor_lm75a",
	},
};


/*
* Client data (each client gets its own)
 */
struct pm_sensor_lm75a_data {
	struct device *hwmon_dev;
};

/*
 * Sysfs stuff
 */
static ssize_t show_temp_input(struct device *dev,
	  struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	uint8_t buf_recv[2] = { 0 };
	uint16_t tempbuf=0;
	/*
	* begin Temperature conversion
	*/
	pm_sensor_reg_write(REG_PM_SENSOR_TMST_VAL);
	// wait for completion completed
	msleep(5);
	pm_sensor_reg_read_data(buf_recv,2);

	pr_debug("%s  buf[0]=0x%x  buf[1]=0x%x \n",__func__,buf_recv[0],buf_recv[1]); 
	tempbuf=((buf_recv[0]<<3)|(buf_recv[1]>>5))&0x7ff;
	if(tempbuf&0x400)
	{
		reg_val=-(0x800-tempbuf)*125/1000;
	}
	else
	{
		reg_val=tempbuf*125/1000;
	}
	pr_debug("reg_val=0x%x  tempbuf=0x%x \n",reg_val,tempbuf); 
	
	return snprintf(buf, PAGE_SIZE, "%d\n", temp_from_reg(reg_val));
}

static DEVICE_ATTR(temp_input, S_IRUGO, show_temp_input, NULL);

static struct attribute *pm_sensor_lm75a_attributes[] = {
	&dev_attr_temp_input.attr,
	NULL
};

static const struct attribute_group pm_sensor_lm75a_group = {
	.attrs = pm_sensor_lm75a_attributes,
};

/*
 * Real code
 */
static int pm_sensor_lm75a_probe(struct platform_device *pdev)
{
	struct pm_sensor_lm75a_data *data;
	int err;

	data = kzalloc(sizeof(struct pm_sensor_lm75a_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &pm_sensor_lm75a_group);
	if (err)
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_files;
	}

	platform_set_drvdata(pdev, data);

	return 0;

exit_remove_files:
	sysfs_remove_group(&pdev->dev.kobj, &pm_sensor_lm75a_group);
exit_free:
	kfree(data);
exit:
	return err;
}

static int pm_sensor_lm75a_remove(struct platform_device *pdev)
{
	struct pm_sensor_lm75a_data  *data = platform_get_drvdata(pdev);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &pm_sensor_lm75a_group);

	kfree(data);
	return 0;
}

static int __init sensor_lm75a_init(void)
{
	return platform_driver_register(&pm_sensor_lm75a_driver);
}
module_init(sensor_lm75a_init);

static void __exit sensor_lm75a__exit(void)
{
	platform_driver_unregister(&pm_sensor_lm75a_driver);
}
module_exit(sensor_lm75a__exit);

MODULE_DESCRIPTION("pm  sensor lm75a temp for epdc show driver");
MODULE_LICENSE("GPL");