/*
* Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
*
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/tps6518x.h>
#include <linux/gpio.h>
#include <linux/pmic_status.h>

//kernel_imx_imx508/drivers/regulator
//kernel_imx_imx508/arch/arm/mach-mx5
#include "../../arch/arm/mach-mx5/mx50_io_cfg.h"


static int tps6518x_pass_num = { 1 };
static int tps6518x_vcom = { -2070000 };//1250000 //2370000
static int tps65180_current_Enable_Register = 0;

static int tps6518x_is_power_good(struct tps6518x *tps6518x);

/*
* to_reg_val(): Creates a register value with new data
*
* Creates a new register value for a particular field.  The data
* outside of the new field is not modified.
*
* @cur_reg: current value in register
* @reg_mask: mask of field bits to be modified
* @fld_val: new value for register field.
*/
/*static*/ unsigned int to_reg_val(unsigned int cur_reg, unsigned int fld_mask,
							   unsigned int fld_val)
{
	return (cur_reg & (~fld_mask)) | fld_val;
}

/*
* Regulator operations
*/
/* Convert uV to the VCOM register bitfield setting */

/*static*/  int vcom_rs_to_uV(unsigned int reg_setting)
{
	if (reg_setting <= TPS65180_VCOM_MIN_SET)
		return TPS65180_VCOM_MIN_uV;
	if (reg_setting >= TPS65180_VCOM_MAX_SET)
		return TPS65180_VCOM_MAX_uV;
	return -(reg_setting * TPS65180_VCOM_STEP_uV);
}
/*static*/ int vcom2_rs_to_uV(unsigned int reg_setting)
{
	if (reg_setting <= TPS65185_VCOM_MIN_SET)
		return TPS65185_VCOM_MIN_uV;
	if (reg_setting >= TPS65185_VCOM_MAX_SET)
		return TPS65185_VCOM_MAX_uV;
	return -(reg_setting * TPS65185_VCOM_STEP_uV);
}


/*static*/ int vcom_uV_to_rs(int uV)
{
	if (uV <= TPS65180_VCOM_MIN_uV)
		return TPS65180_VCOM_MIN_SET;
	if (uV >= TPS65180_VCOM_MAX_uV)
		return TPS65180_VCOM_MAX_SET;
	return (-uV) / TPS65180_VCOM_STEP_uV;
}

/*static*/ int vcom2_uV_to_rs(int uV)
{
	if (uV <= TPS65185_VCOM_MIN_uV)
		return TPS65185_VCOM_MIN_SET;
	if (uV >= TPS65185_VCOM_MAX_uV)
		return TPS65185_VCOM_MAX_SET;
	return (-uV) / TPS65185_VCOM_STEP_uV;
}

static int epdc_pwr0_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);


	gpio_set_value(tps6518x->gpio_pmic_powerup, 1);

	return 0;

}

static int epdc_pwr0_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);


	gpio_set_value(tps6518x->gpio_pmic_powerup, 0);

	return 0;

}
static int tps6518x_v3p3_enable(struct regulator_dev *reg)
{
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int new_reg_val; /* new register value to write */


	cur_reg_val = tps65180_current_Enable_Register & 0x3f;
	new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val,
		BITFMASK(V3P3_SW_EN),
		BITFVAL(V3P3_SW_EN, true));

	return tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);
}

static int tps6518x_v3p3_disable(struct regulator_dev *reg)
{
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int new_reg_val; /* new register value to write */


	cur_reg_val = tps65180_current_Enable_Register & 0x20;
	new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val,
		BITFMASK(V3P3_SW_EN),
		BITFVAL(V3P3_SW_EN, false));

	return tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);
}

static int tps6518x_vcom_set_voltage(struct regulator_dev *reg,
									 int minuV, int uV)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int new_reg_val; /* new register value to write */
	int retval;

	/*
	* this will not work on tps65182
	*/
	if (tps6518x->revID == 65182)
	{
		//printk("%s %s %d RETURN ERROR 65182! \n",__FILE__,__func__,__LINE__); 
		return 0;
	}
	///del,compare for patch:TPS6518x-unable-to-set-VCOM.patch
	if ((-uV) < 200000)
	{
		//printk("%s %s %d  too small return uV=%ld\n",__FILE__,__func__,__LINE__,uV); 
		return 0;
	}
	//printk(" tps vcom uV=%ld \n",uV); 

	switch (tps6518x->revID & 15)
	{
	case 0 : /* TPS65180 */
	case 1 : /* TPS65181 */
	case 4 : /* TPS65180-rev1 */
		tps6518x_reg_read(REG_TPS65180_VCOM_ADJUST,&cur_reg_val);
		new_reg_val = to_reg_val(cur_reg_val,
			BITFMASK(VCOM_SET),
			BITFVAL(VCOM_SET, vcom_uV_to_rs(uV)));

		retval = tps6518x_reg_write(REG_TPS65180_VCOM_ADJUST,
			new_reg_val);
		break;
	case 5 : /* TPS65185 */
	case 6 : /* TPS65186 */
		retval = tps6518x_reg_write(REG_TPS65185_VCOM1,
			vcom2_uV_to_rs(uV) & 255);
		tps6518x_reg_read( REG_TPS65185_VCOM2,&cur_reg_val);
		new_reg_val = to_reg_val(cur_reg_val,
			BITFMASK(VCOM2_SET),
			BITFVAL(VCOM2_SET, vcom2_uV_to_rs(uV)/256));

		retval = tps6518x_reg_write(REG_TPS65185_VCOM2,
			new_reg_val);
		break;
	default :
		retval = -1;
	}
	return retval;
}

static int tps6518x_vcom_get_voltage(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value */
	unsigned int cur_reg2_val; /* current register value */
	unsigned int cur_fld_val; /* current bitfield value*/
	int vcomValue;


	/*
	* this will not work on tps65182
	*/
	if (tps6518x->revID == 65182)
		return 0;

	switch (tps6518x->revID & 15)
	{
	case 0 : /* TPS65180 */
	case 1 : /* TPS65181 */
	case 4 : /* TPS65180-rev1 */
		tps6518x_reg_read(REG_TPS65180_VCOM_ADJUST, &cur_reg_val);
		cur_fld_val = BITFEXT(cur_reg_val, VCOM_SET);
		vcomValue = vcom_rs_to_uV(cur_fld_val);
		break;
	case 5 : /* TPS65185 */
	case 6 : /* TPS65186 */
		tps6518x_reg_read(REG_TPS65185_VCOM1,&cur_reg_val);
		tps6518x_reg_read(REG_TPS65185_VCOM2,&cur_reg2_val);
		cur_reg_val |= 256 * (1 & cur_reg2_val);
		vcomValue = vcom2_rs_to_uV(cur_reg_val);
		break;
	default:
		vcomValue = 0;
	}

	return vcomValue;

}

static int tps6518x_vcom_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value */
	int vcomEnable = 0;


	/*
	* check for the TPS65182 device
	*/
	if (tps6518x->revID == 65182)
	{
		gpio_set_value(tps6518x->gpio_pmic_vcom_ctrl,vcomEnable);
		return 0;
	}

	/*
	* Check to see if we need to set the VCOM voltage.
	* Should only be done one time. And, we can
	* only change vcom voltage if we have been enabled.
	*/
	if (!tps6518x->vcom_setup && tps6518x_is_power_good(tps6518x)) {
		//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
		tps6518x_vcom_set_voltage(reg,
			tps6518x->vcom_uV,
			tps6518x->vcom_uV);
		tps6518x->vcom_setup = true;
	}

	switch (tps6518x->revID & 15)
	{
	case 0 : /* TPS65180 */
	case 1 : /* TPS65181 */
	case 4 : /* TPS65180-rev1 */
		vcomEnable = 1;
		break;
	case 5 : /* TPS65185 */
	case 6 : /* TPS65186 */
		tps6518x_reg_read(REG_TPS65185_VCOM2,&cur_reg_val);
		// do not enable vcom if HiZ bit is set
		if (cur_reg_val & 0x20)
			vcomEnable = 0;
		else
			vcomEnable = 1;
		break;
	default:
		vcomEnable = 0;
	}
	gpio_set_value(tps6518x->gpio_pmic_vcom_ctrl,vcomEnable);

	return 0;
}

static int tps6518x_vcom_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);


	gpio_set_value(tps6518x->gpio_pmic_vcom_ctrl,0);
	return 0;
}

static int tps6518x_vcom_is_enabled(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);


	int gpio = gpio_get_value(tps6518x->gpio_pmic_vcom_ctrl);
	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int tps6518x_is_power_good(struct tps6518x *tps6518x)
{
	/*
	* XOR of polarity (starting value) and current
	* value yields whether power is good.
	*/
	//return gpio_get_value(tps6518x->gpio_pmic_pwrgood) ^

	//tps6518x->pwrgood_polarity;
	return 1;
}

static int tps6518x_wait_power_good(struct tps6518x *tps6518x)
{
	int i;
	

	for (i = 0; i < tps6518x->max_wait * 3; i++) {
		if (tps6518x_is_power_good(tps6518x))
			return 0;

		msleep(1);
	}
	//printk(" fail error!!!%s %s %d   gpio_pmic_pwrgood=%d \n",__FILE__,__func__,__LINE__,gpio_get_value(tps6518x->gpio_pmic_pwrgood) ); 
	return -ETIMEDOUT;
}

static int tps6518x_display_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int fld_mask;	  /* register mask for bitfield to modify */
	unsigned int fld_val;	  /* new bitfield value to write */
	unsigned int new_reg_val; /* new register value to write */


	if (tps6518x->revID == 65182)
	{
		epdc_pwr0_enable(reg);
	}
	else
	{
		//printk("%s %s %d   tps6518x->revID=%d \n",__FILE__,__func__,__LINE__,tps6518x->revID); 

		/* enable display regulators */
		cur_reg_val = tps65180_current_Enable_Register & 0x3f;
		fld_mask = BITFMASK(VDDH_EN) | BITFMASK(VPOS_EN) |
			BITFMASK(VEE_EN) | BITFMASK(VNEG_EN);
		fld_val = BITFVAL(VDDH_EN, true) | BITFVAL(VPOS_EN, true) |
			BITFVAL(VEE_EN, true) | BITFVAL(VNEG_EN, true) | BITFVAL(VCOM_EN, false);
		new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val, fld_mask, fld_val);
		tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);

		/* turn on display regulators */
		cur_reg_val = tps65180_current_Enable_Register & 0x3f;
		fld_mask = BITFMASK(ACTIVE);
		fld_val = BITFVAL(ACTIVE, true);
		new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val, fld_mask, fld_val);
		tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);
	}
	return tps6518x_wait_power_good(tps6518x);
}

static int tps6518x_display_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int fld_mask;	  /* register mask for bitfield to modify */
	unsigned int fld_val;	  /* new bitfield value to write */
	unsigned int new_reg_val; /* new register value to write */



	if (tps6518x->revID == 65182)
	{
		epdc_pwr0_disable(reg);
	}
	else
	{
		/* turn off display regulators */
		cur_reg_val = tps65180_current_Enable_Register & 0x3f;
		fld_mask = BITFMASK(VCOM_EN) | BITFMASK(STANDBY);
		fld_val = BITFVAL(VCOM_EN, false) | BITFVAL(STANDBY, true);
		new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val, fld_mask, fld_val);
		tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);

	}

	msleep(tps6518x->max_wait);

	return 0;
}

static int tps6518x_display_is_enabled(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);


	if (tps6518x->revID == 65182){
		return gpio_get_value(tps6518x->gpio_pmic_wakeup) ? 1:0;
	}else{
		return tps65180_current_Enable_Register & BITFMASK(ACTIVE);
	}

	
}

static int tps6518x_v3p3reg_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);


	if (tps6518x->revID == 65182)
	{
		//epdc_pwr0_enable(reg);
		// printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	}
	else
	{
		/* turn on 3p3 rail */
		tps6518x_v3p3_enable(reg);
		//printk("%s %s %d  tps6518x->revID=%d \n",__FILE__,__func__,__LINE__,tps6518x->revID); 
	}
	return 0;
}

static int tps6518x_v3p3reg_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);



	if (tps6518x->revID == 65182)
	{
		//epdc_pwr0_disable(reg);

		//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	}
	else
	{
		/* turn off 3p3 rail */
		tps6518x_v3p3_disable(reg);
		//printk("%s %s %d  tps6518x->revID=%d \n",__FILE__,__func__,__LINE__,tps6518x->revID); 
	}

	return 0;
}

static int tps6518x_v3p3reg_is_enabled(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	if (tps6518x->revID == 65182)
		return 1;
	else
		return tps65180_current_Enable_Register & BITFMASK(V3P3_SW_EN)? 1:0;
}

static int tps6518x_tmst_enable(struct regulator_dev *reg)
{
	return 0;
}

static int tps6518x_tmst_disable(struct regulator_dev *reg)
{
	return 0;
}

static int tps6518x_tmst_get_temperature(struct regulator_dev *reg)
{
	unsigned int reg_val;
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	//
	// begin Temperature conversion
	//
	switch (tps6518x->revID)
	{
	case TPS65180_PASS1 :
	case TPS65180_PASS2 :
	case TPS65181_PASS1 :
	case TPS65181_PASS2 :
		reg_val = 0x80;
		tps6518x_reg_write(REG_TPS65180_TMST_CONFIG, reg_val);
		// wait for completion completed
		while ((0x20 & reg_val) == 0)
		{
			msleep(1);
			tps6518x_reg_read(REG_TPS65180_TMST_CONFIG, &reg_val);
		}
		break;
	case TPS65185_PASS0 :
	case TPS65186_PASS0 :
	case TPS65185_PASS1 :
	case TPS65186_PASS1 :
	case TPS65185_PASS2 :
	case TPS65186_PASS2 :
		reg_val = 0x80;
		tps6518x_reg_write(REG_TPS65185_TMST1, reg_val);
		// wait for completion completed
		while ((0x20 & reg_val) == 0)
		{
			msleep(1);
			tps6518x_reg_read(REG_TPS65185_TMST1, &reg_val);
		}
		break;
	default:
		return 25;
	}

	tps6518x_reg_read(REG_TPS6518x_TMST_VAL, &reg_val);
	return reg_val;
}


/*
* Regulator operations
*/

static struct regulator_ops tps6518x_display_ops = {
	.enable = tps6518x_display_enable,
	.disable = tps6518x_display_disable,
	.is_enabled = tps6518x_display_is_enabled,
};

static struct regulator_ops tps6518x_vcom_ops = {
	.enable = tps6518x_vcom_enable,
	.disable = tps6518x_vcom_disable,
	.get_voltage = tps6518x_vcom_get_voltage,
	.set_voltage = tps6518x_vcom_set_voltage,
	.is_enabled = tps6518x_vcom_is_enabled,
};

static struct regulator_ops tps6518x_v3p3_ops = {
	.enable = tps6518x_v3p3reg_enable,
	.disable = tps6518x_v3p3reg_disable,
	.is_enabled = tps6518x_v3p3reg_is_enabled,
};

static struct regulator_ops tps6518x_tmst_ops = {
	.enable = tps6518x_tmst_enable,
	.disable = tps6518x_tmst_disable,
	.get_voltage = tps6518x_tmst_get_temperature,
};

/*
 * Regulator descriptors
 */
static struct regulator_desc tps6518x_reg[TPS6518x_NUM_REGULATORS] = {
{
	.name = "DISPLAY",
	.id = TPS6518x_DISPLAY,
	.ops = &tps6518x_display_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VCOM",
	.id = TPS6518x_VCOM,
	.ops = &tps6518x_vcom_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "V3P3",
	.id = TPS6518x_V3P3,
	.ops = &tps6518x_v3p3_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
    .name = "TMST",
	.id =  TPS6518x_TMST,
	.ops = &tps6518x_tmst_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};

static void tps6518x_setup_timings(struct tps6518x *tps6518x)
{
	int temp0, temp1, temp2, temp3;


	/* read the current setting in the PMIC */
	if ((tps6518x->revID == TPS65180_PASS1) || (tps6518x->revID == TPS65181_PASS1) ||
	   (tps6518x->revID == TPS65180_PASS2) || (tps6518x->revID == TPS65181_PASS2)) {
	   tps6518x_reg_read(REG_TPS65180_PWRSEQ0, &temp0);
	   tps6518x_reg_read(REG_TPS65180_PWRSEQ1, &temp1);
	   tps6518x_reg_read(REG_TPS65180_PWRSEQ2, &temp2);

	   if ((temp0 != tps6518x->pwr_seq0) ||
		(temp1 != tps6518x->pwr_seq1) ||
		(temp2 != tps6518x->pwr_seq2)) {
		tps6518x_reg_write(REG_TPS65180_PWRSEQ0, tps6518x->pwr_seq0);
		tps6518x_reg_write(REG_TPS65180_PWRSEQ1, tps6518x->pwr_seq1);
		tps6518x_reg_write(REG_TPS65180_PWRSEQ2, tps6518x->pwr_seq2);
	    }
	}

	if ((tps6518x->revID == TPS65185_PASS0) ||
		 (tps6518x->revID == TPS65186_PASS0) ||
		 (tps6518x->revID == TPS65185_PASS1) ||
		 (tps6518x->revID == TPS65186_PASS1) ||
		 (tps6518x->revID == TPS65185_PASS2) ||
		 (tps6518x->revID == TPS65186_PASS2)) {
	   tps6518x_reg_read(REG_TPS65185_UPSEQ0, &temp0);
	   tps6518x_reg_read(REG_TPS65185_UPSEQ1, &temp1);
	   tps6518x_reg_read(REG_TPS65185_DWNSEQ0, &temp2);
	   tps6518x_reg_read(REG_TPS65185_DWNSEQ1, &temp3);

	   if ((temp0 != tps6518x->upseq0) ||
		(temp1 != tps6518x->upseq1) ||
		(temp2 != tps6518x->dwnseq0) ||
		(temp3 != tps6518x->dwnseq1)) {
		tps6518x_reg_write(REG_TPS65185_UPSEQ0, tps6518x->upseq0);
		tps6518x_reg_write(REG_TPS65185_UPSEQ1, tps6518x->upseq1);
		tps6518x_reg_write(REG_TPS65185_DWNSEQ0, tps6518x->dwnseq0);
		tps6518x_reg_write(REG_TPS65185_DWNSEQ1, tps6518x->dwnseq1);
	    }
	}

}


/*
 * Regulator init/probing/exit functions
 */
static int tps6518x_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	rdev = regulator_register(&tps6518x_reg[pdev->id], &pdev->dev,
		pdev->dev.platform_data,
		dev_get_drvdata(&pdev->dev));

	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			tps6518x_reg[pdev->id].name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static int tps6518x_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	regulator_unregister(rdev);
	return 0;
}
//extern int mx50_rdp_set_3v3_voltage(int ivalue);
///wen add .according to freescale test result,I add these funtion to set 
///gpio in output low
///test result:ok.
//wen del,now we control these gpio in mx50_rdp.c 2012-03-28
/*
static int tps6518x_regulator_suspend(struct platform_device *dev, pm_message_t state)
{
	gpio_set_value(EPDC_PMIC_WAKE,0);

	gpio_set_value(EPDC_PWRCTRL0,0);
	
	//msleep(10);
	
	//mx50_rdp_set_3v3_voltage(1);

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
        return 0;
}
static int tps6518x_regulator_resume(struct platform_device *dev)
{

	 //mx50_rdp_set_3v3_voltage(0);
	 
	// msleep(10);
	 
	gpio_set_value(EPDC_PMIC_WAKE,1);

	gpio_set_value(EPDC_PWRCTRL0,1);

	

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

       return 0;
}
*/
static struct platform_driver tps6518x_regulator_driver = {
	.probe = tps6518x_regulator_probe,
	.remove = tps6518x_regulator_remove,
	//.suspend =tps6518x_regulator_suspend,
	//.resume =tps6518x_regulator_resume,
	.driver = {
		.name = "tps6518x-reg",
	},
};

int tps6518x_register_regulator(struct tps6518x *tps6518x, int reg,
									struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;


	/* If we can't find PMIC via I2C, we should not register regulators */
	if (PMIC_ERROR == tps6518x_reg_read(REG_TPS6518x_REVID,&tps6518x->revID))
	{
		/*
		* it is possible that we have wrong polarity of the Wakeup pin
		*/
		//printk("!!!!!!!!!!!!can 't find %s %s %d  regulators  \n",__FILE__,__func__,__LINE__); 
		gpio_set_value(tps6518x->gpio_pmic_wakeup,1);
		msleep(10);
		if (PMIC_ERROR == tps6518x_reg_read(REG_TPS6518x_REVID,&tps6518x->revID))
		{
			dev_err(tps6518x->dev,
				"TPS6518x PMIC not found!\n");
			return -ENXIO;
		}
	};
	if (!(tps6518x->revID == TPS65180_PASS1 || 
		tps6518x->revID == TPS65180_PASS2 || 
		tps6518x->revID == TPS65181_PASS1 || 
		tps6518x->revID == TPS65181_PASS2 || 
		tps6518x->revID == TPS65185_PASS0 || 
		tps6518x->revID == TPS65186_PASS0 || 
		tps6518x->revID == TPS65185_PASS1 || 
		tps6518x->revID == TPS65186_PASS1 || 
		tps6518x->revID == TPS65185_PASS2 || 
		tps6518x->revID == TPS65186_PASS2)) {
			//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
			tps6518x->revID = 65182; /* assume it is TPS65182 - it has not been tested */
	}

	if (tps6518x->pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc("tps6518x-reg", reg);
	if (!pdev)
		return -ENOMEM;

	tps6518x->pdev[reg] = pdev;

	initdata->driver_data = tps6518x;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = tps6518x->dev;
	platform_set_drvdata(pdev, tps6518x);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(tps6518x->dev,
			"Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		tps6518x->pdev[reg] = NULL;
	}

	if (!tps6518x->init_done) {
		tps6518x->pass_num = tps6518x_pass_num;
		tps6518x->vcom_uV = tps6518x_vcom;

		/*
		* Set up PMIC timing values.
		* Should only be done one time!  Timing values may only be
		* changed a limited number of times according to spec.
		*/
		tps6518x_setup_timings(tps6518x);

		tps6518x->pwrgood_polarity = gpio_get_value(tps6518x->gpio_pmic_pwrgood);

		tps6518x->init_done = true;
	}

	return ret;
}

static int __init tps6518x_regulator_init(void)
{
	return platform_driver_register(&tps6518x_regulator_driver);
}
subsys_initcall(tps6518x_regulator_init);

static void __exit tps6518x_regulator_exit(void)
{
	platform_driver_unregister(&tps6518x_regulator_driver);
}
module_exit(tps6518x_regulator_exit);


/*
* Parse user specified options (`tps6518x:')
* example:
*   tps6518x:pass=2,vcom=-1250000
*/
static int __init tps6518x_setup(char *options)
{
	int ret;
	char *opt;
	unsigned long ulResult;
	//printk("%s %s %d  options=%s  \n",__FILE__,__func__,__LINE__,options); 
	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;
		if (!strncmp(opt, "pass=", 5)) {
			ret = strict_strtoul((const char *)(opt + 5), 0, &ulResult);
			tps6518x_pass_num = ulResult;
			if (ret < 0)
				return ret;
		}
		if (!strncmp(opt, "vcom=", 5)) {
			int offs = 5;
			if (opt[5] == '-')
				offs = 6;
			ret = strict_strtoul((const char *)(opt + offs), 0, &ulResult);
			tps6518x_vcom = (int) ulResult;
			if (ret < 0)
				return ret;
			tps6518x_vcom = -tps6518x_vcom;
		}
	}

	return 1;
}

__setup("tps6518x:", tps6518x_setup);

static int __init tps65182_setup(char *options)
{
	int ret;
	char *opt;
	unsigned long ulResult;
	//printk("%s %s %d  options=%s  \n",__FILE__,__func__,__LINE__,options); 
	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;
		if (!strncmp(opt, "pass=", 5)) {
			ret = strict_strtoul((const char *)(opt + 5), 0, &ulResult);
			tps6518x_pass_num = ulResult;
			if (ret < 0)
				return ret;
		}
		if (!strncmp(opt, "vcom=", 5)) {
			int offs = 5;
			if (opt[5] == '-')
				offs = 6;
			ret = strict_strtoul((const char *)(opt + offs), 0, &ulResult);
			tps6518x_vcom = (int) ulResult;
			if (ret < 0)
				return ret;
			tps6518x_vcom = -tps6518x_vcom;
		}
	}

	return 1;
}

__setup("tps65182:", tps65182_setup);


/* Module information */
MODULE_DESCRIPTION("TPS6518x regulator driver");
MODULE_LICENSE("GPL");
