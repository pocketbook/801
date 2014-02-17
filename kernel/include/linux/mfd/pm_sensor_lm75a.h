/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#ifndef __LINUX_REGULATOR_PM_SENSOR_LM75A_H_
#define __LINUX_REGULATOR_PM_SENSOR_LM75A_H_

/*
 * EPDC TEMP SENSOR  I2C address 0x78
 */
#define EPDC_PM_SENSOR_LM75A_I2C_ADDR  0x48

/*
 * PMIC Register Addresses
 */
enum {
    REG_PM_SENSOR_TMST_VAL = 0x0,
    REG_PM_SENSOR_CONF,
    REG_PM_SENSOR_THYST,
    REG_PM_SENSOR_TOS,
    PM_SENSOR_REG_NUM,
};

enum {
    /* In alphabetical order */
     PM_SENSOR_LM75A_DISPLAY, /* virtual master enable */
     PM_SENSOR_LM75A_VCOM,
     PM_SENSOR_LM75A_V3P3,
     PM_SENSOR_LM75A_TMST,
     PM_SENSOR_LM75A_NUM_REGULATORS,
};

#define PM_SENSOR_MAX_REGISTER   0xFF


struct regulator_init_data;

struct pm_sensor {
	/* chip revision */
	int revID;

	struct device *dev;

	/* Platform connection */
	struct i2c_client *i2c_client;

	/* Client devices */
	struct platform_device *pdev[PM_SENSOR_REG_NUM];

	/* Timings */
	unsigned int pwr_seq0;
	unsigned int pwr_seq1;
	unsigned int pwr_seq2;
	unsigned int upseq0;
	unsigned int upseq1;
	unsigned int dwnseq0;
	unsigned int dwnseq1;

	/* GPIOs */
	int gpio_pmic_pwrgood;
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_wakeup;
	int gpio_pmic_intr;
	int gpio_pmic_powerup;

	/* TPS6518x part variables */
	int pass_num;
	int vcom_uV;

	/* One-time VCOM setup marker */
	bool vcom_setup;
	bool init_done;

	/* powerup/powerdown wait time */
	int max_wait;

	/* Dynamically determined polarity for PWRGOOD */
	int pwrgood_polarity;
};

enum {
    /* In alphabetical order */
    PM_SENSOR_DISPLAY, /* virtual master enable */
    PM_SENSOR_VCOM,
    PM_SENSOR_V3P3,
    PM_SENSOR_TMST,
    PM_SENSOR_NUM_REGULATORS,
};

/*
 * Declarations
 */
struct regulator_init_data;

struct pm_sensor_platform_data {
	unsigned int pwr_seq0;
	unsigned int pwr_seq1;
	unsigned int pwr_seq2;
	unsigned int upseq0;
	unsigned int upseq1;
	unsigned int dwnseq0;
	unsigned int dwnseq1;
	int gpio_pmic_pwrgood;
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_wakeup;
	int gpio_pmic_intr;
	int gpio_pmic_powerup;
	int pass_num;
	int vcom_uV;
	struct regulator_init_data *regulator_init;
	int (*init)(struct pm_sensor *);
};

int pm_sensor_register_regulator(struct pm_sensor *pm_sensor, int reg,
				     struct regulator_init_data *initdata);

int pm_sensor_reg_read(int reg_num, uint8_t * reg_val);
int pm_sensor_reg_write_data(int reg_num, const unsigned int reg_val);
int pm_sensor_reg_write(int reg_num);
int pm_sensor_reg_read_data(uint8_t * buf_recv,int count);
#endif

