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
 * Includes
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/uaccess.h>

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/pmic_status.h>
#include <asm/mach-types.h>
#include <linux/mfd/pm_sensor_lm75a.h>
#include <linux/regulator/driver.h>
#include <mach/iomux-mx50.h> 
#include <mach/gpio.h>
#include <asm/mach-types.h>

//extern  int mx50_rdp_usb_power_control(bool bOpen);

#if 1
#include"../../arch/arm/mach-mx5/mx50_io_cfg.h"
#else 
#define EPDC_PWRCTRL0	(2*32 + 29)	/*GPIO_3_29 */
#define EPDC_PWRCTRL1	(2*32 + 30)	/*GPIO_3_30 */
#define EPDC_PWRCTRL2	(2*32 + 31)	/*GPIO_3_31 */
#define EPDC_VCOM	(3*32 + 21)	/*GPIO_4_21 */
#define PM_MX50_RDP_UART1_RTS_GPIO6_9_EDPC_POWER     (5*32+9)
#endif 

#define  PM_SENSOR_LM75A_DRIVER_NAME   "pm_sensor_lm75a"

struct i2c_client *pm_sensor_client;


int pm_sensor_reg_read(int reg_num, uint8_t * reg_val)
{
	int result;

	if (pm_sensor_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_read_byte_data(pm_sensor_client, reg_num);
	if (result < 0) {
		dev_err(&pm_sensor_client->dev,
			"Unable to read pm_sensor register via I2C\n");
		return PMIC_ERROR;
	}

	*reg_val = result;

	return PMIC_SUCCESS;
}

int pm_sensor_reg_read_data(uint8_t * buf_recv,int count)
{
	int rc;	

	if (pm_sensor_client == NULL)
		return PMIC_ERROR;
	rc = i2c_master_recv(pm_sensor_client, buf_recv, count);
	if (rc != count) {
		dev_err(&pm_sensor_client->dev,
			"Unable to read pm_sensor register via I2C :%d\n",rc);
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}

int pm_sensor_reg_write(int reg_num)
{
	int result;

	if (pm_sensor_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_write_byte(pm_sensor_client, reg_num);
	if (result < 0) {
		dev_err(&pm_sensor_client->dev,
			"Unable to write pm_sensor register via I2C\n");
		return PMIC_ERROR;
	}
	return PMIC_SUCCESS;
}

int pm_sensor_reg_write_data(int reg_num, const unsigned int reg_val)
{
	int result;

	if (pm_sensor_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_write_byte_data(pm_sensor_client, reg_num, reg_val);
	if (result < 0) {
		dev_err(&pm_sensor_client->dev,
			"Unable to write data pm_sensor register via I2C\n");
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}

static int pm_sensor_vcom_enable(struct regulator_dev *reg)
{
	//struct pm_sensor *pm_sensor = rdev_get_drvdata(reg);
	//gpio_direction_output(EPDC_VCOM, 1);
	gpio_direction_output(LM75A_EPDC_VCOM, 1);
	msleep(3);
	return 0;
}

static int pm_sensor_vcom_disable(struct regulator_dev *reg)
{
	//struct pm_sensor *pm_sensor = rdev_get_drvdata(reg);
	//gpio_direction_output(EPDC_VCOM, 0);
	msleep(3);
	gpio_direction_output(LM75A_EPDC_VCOM, 0);
	return 0;
}

static int pm_sensor_vcom_is_enabled(struct regulator_dev *reg)
{
	//struct pm_sensor *pm_sensor = rdev_get_drvdata(reg);
	int gpio = gpio_get_value(LM75A_EPDC_VCOM);
	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int pm_sensor_display_enable(struct regulator_dev *reg)
{
	gpio_direction_output(LM75A_EPDC_PWRCTRL0, 1);
	msleep(2);
	gpio_direction_output(LM75A_EPDC_PWRCTRL1, 1);
	msleep(2);
	gpio_direction_output(LM75A_EPDC_PWRCTRL2, 1);
	msleep(2);
	
	return 0;
}

static int pm_sensor_display_disable(struct regulator_dev *reg)
{
	msleep(10);
	gpio_direction_output(LM75A_EPDC_PWRCTRL0, 0);
	msleep(2);
	gpio_direction_output(LM75A_EPDC_PWRCTRL1, 0);
	msleep(2);
	gpio_direction_output(LM75A_EPDC_PWRCTRL2, 0);
	msleep(5);

	return 0;
}

static int pm_sensor_display_is_enabled(struct regulator_dev *reg)
{
	int gpio = gpio_get_value(LM75A_EPDC_PMIC_WAKE);
	
	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int pm_sensor_v3p3reg_enable(struct regulator_dev *reg)
{
	//struct pm_sensor *pm_sensor = rdev_get_drvdata(reg);
	//gpio_set_value(MX50_RDP_UART1_RTS_GPIO6_9_POWER, 0);

	//msleep(1);
	return 0;
}

static int pm_sensor_v3p3reg_disable(struct regulator_dev *reg)
{
	//struct pm_sensor *pm_sensor = rdev_get_drvdata(reg);
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	//gpio_set_value(MX50_RDP_UART1_RTS_GPIO6_9_POWER, 1);
	return 0;
}

static int pm_sensor_v3p3reg_is_enabled(struct regulator_dev *reg)
{
	//struct pm_sensor *pm_sensor = rdev_get_drvdata(reg);
	//int gpio = gpio_get_value(MX50_RDP_UART1_RTS_GPIO6_9_POWER);
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	//if (gpio == 0)
	//	return 0;
	//else
	//	return 1;
	return 0;
}

static int pm_sensor_tmst_enable(struct regulator_dev *reg)
{
	return 0;
}

static int pm_sensor_tmst_disable(struct regulator_dev *reg)
{
	return 0;
}

static int pm_sensor_tmst_get_temperature(struct regulator_dev *reg)
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

	tempbuf=((buf_recv[0]<<3)|(buf_recv[1]>>5))&0x7ff;

	pr_debug("%s	buf[0]=0x%x  buf[1]=0x%x \n",__func__,buf_recv[0],buf_recv[1]); 
	if(tempbuf&0x400)
	{
		reg_val= 0;//-(0x800-tempbuf)*125/1000; 
	}
	else
	{
		reg_val=tempbuf*125/1000;
	}
	pr_debug("reg_val=0x%x  tempbuf=0x%x \n",reg_val,tempbuf); 
	 reg_val = reg_val -5;
	return reg_val;
}


/*
 * Regulator operations
 */

static struct regulator_ops pm_sensor_display_ops = {
	.enable = pm_sensor_display_enable,
	.disable = pm_sensor_display_disable,
	.is_enabled = pm_sensor_display_is_enabled,
};

static struct regulator_ops pm_sensor_vcom_ops = {
	.enable = pm_sensor_vcom_enable,
	.disable = pm_sensor_vcom_disable,
	.is_enabled = pm_sensor_vcom_is_enabled,
};

static struct regulator_ops pm_sensor_v3p3_ops = {
	.enable = pm_sensor_v3p3reg_enable,
	.disable = pm_sensor_v3p3reg_disable,
	.is_enabled = pm_sensor_v3p3reg_is_enabled,
};

static struct regulator_ops pm_sensor_tmst_ops = {
	.enable = pm_sensor_tmst_enable,
	.disable = pm_sensor_tmst_disable,
	.get_voltage = pm_sensor_tmst_get_temperature,
};

/*
 * Regulator descriptors
 */
static struct regulator_desc pm_sensor_reg[PM_SENSOR_NUM_REGULATORS] = {
{
	.name = "DISPLAY",
	.id = PM_SENSOR_DISPLAY,
	.ops = &pm_sensor_display_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VCOM",
	.id = PM_SENSOR_VCOM,
	.ops = &pm_sensor_vcom_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "V3P3",
	.id = PM_SENSOR_V3P3,
	.ops = &pm_sensor_v3p3_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
        .name = "TMST",
	.id =  PM_SENSOR_TMST,
	.ops = &pm_sensor_tmst_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};

/*
 * Regulator init/probing/exit functions
 */
static int pm_sensor_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	rdev = regulator_register(&pm_sensor_reg[pdev->id], &pdev->dev,
		pdev->dev.platform_data,
		dev_get_drvdata(&pdev->dev));

	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			pm_sensor_reg[pdev->id].name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static int pm_sensor_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver pm_sensor_regulator_driver = {
	.probe =  pm_sensor_regulator_probe,
	.remove = pm_sensor_regulator_remove,
	.driver = {
		.name = "pm_sensor-reg",
	},
};

int pm_sensor_register_regulator(struct pm_sensor *pm_sensor, int reg,
									struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	if (pm_sensor->pdev[reg])
	{
		printk("******* error !!!%s %s %d   \n",__FILE__,__func__,__LINE__); 
		return -EBUSY;
	}

	pdev = platform_device_alloc("pm_sensor-reg", reg);
	if (!pdev)
	{
		printk("******* error !!!%s %s %d   \n",__FILE__,__func__,__LINE__); 
		return -ENOMEM;
	}

	pm_sensor->pdev[reg] = pdev;

	initdata->driver_data = pm_sensor;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = pm_sensor->dev;
	platform_set_drvdata(pdev, pm_sensor);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(pm_sensor->dev,
			"Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		pm_sensor->pdev[reg] = NULL;
	}

	if (!pm_sensor->init_done) {
		/*
		* Set up PMIC timing values.
		* Should only be done one time!  Timing values may only be
		* changed a limited number of times according to spec.
		*/

		pm_sensor->init_done = true;
	}

	return ret;
}

static int pm_sensor_probe(struct i2c_client *client,
						   const struct i2c_device_id *id)
{
	struct pm_sensor *pm_sensor;

	struct pm_sensor_platform_data *pdata = client->dev.platform_data;
	int ret = 0;


	if (!pdata || !pdata->init)
	{
		printk("*****error ! %s %s %d   \n",__FILE__,__func__,__LINE__); 
		return -ENODEV;
	}
	/* Create the PMIC data structure */
	pm_sensor = kzalloc(sizeof(struct pm_sensor), GFP_KERNEL);
	if (pm_sensor == NULL) {
		kfree(client);
		return -ENOMEM;
	}

	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, pm_sensor);
	pm_sensor->dev = &client->dev;
	pm_sensor->i2c_client = client;

	pm_sensor_client = client;

	if (pdata && pdata->init) {
		ret = pdata->init(pm_sensor);
		if (ret != 0)
		{
			printk("*****error ! %s %s %d   \n",__FILE__,__func__,__LINE__); 
			goto err;
		}
	}

	//platform_driver_register(&pm_sensor_regulator_driver);
	dev_info(&client->dev, "PMIC pm_sensor for eInk display \n");

	return ret;
err:
	kfree(pm_sensor);

	return ret;
}


static int pm_sensor_remove(struct i2c_client *i2c)
{
	struct pm_sensor *pm_sensor = i2c_get_clientdata(i2c);
	int i;

	for (i = 0; i < ARRAY_SIZE(pm_sensor->pdev); i++)
		platform_device_unregister(pm_sensor->pdev[i]);

	kfree(pm_sensor);

	return 0;
}

static int pm_sensor_suspend(struct i2c_client *client, pm_message_t state)
{
	//mx50_rdp_usb_power_switch(false);

	return 0;
}

static int pm_sensor_resume(struct i2c_client *client)
{
	//mx50_rdp_usb_power_switch(true);
	return 0;
}

static const struct i2c_device_id pm_sensor_id[] = {
	{ PM_SENSOR_LM75A_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pm_sensor_id);


static struct i2c_driver pm_sensor_driver = {
	.probe = pm_sensor_probe,
	.remove = pm_sensor_remove,
	.suspend = pm_sensor_suspend,
	.resume = pm_sensor_resume,
	.id_table = pm_sensor_id,
	.driver = {
		   .name = PM_SENSOR_LM75A_DRIVER_NAME,
		   .owner = THIS_MODULE,
	},
};

static int __init pm_sensor_regulator_init(void)
{
	return platform_driver_register(&pm_sensor_regulator_driver);
}
subsys_initcall(pm_sensor_regulator_init);

static void __exit pm_sensor_regulator_exit(void)
{
	platform_driver_unregister(&pm_sensor_regulator_driver);
}
module_exit(pm_sensor_regulator_exit);


static int __init pm_sensor_init(void)
{
	return  i2c_add_driver(&pm_sensor_driver);
}

static void __exit pm_sensor_exit(void)
{
	//platform_driver_unregister(&pm_sensor_regulator_driver);
	i2c_del_driver(&pm_sensor_driver);
}

/*
 * Module entry points
 */
subsys_initcall_sync(pm_sensor_init);
module_exit(pm_sensor_exit);
