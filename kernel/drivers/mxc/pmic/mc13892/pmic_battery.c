/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 */

/*
 * Includes
 */
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/reboot.h>
#include <asm/mach-types.h>
#include <linux/pmic_battery.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>
#include <linux/pmic_external.h>


#define PMIC_VOLTAGE_MAX_DESIGN   4190000/*4200000 4130000*/
#define PMIC_VOLTAGE_MIN_DESIGN    3610000

#define PMIC_VOLTAGE_MAX_WORK    4170000/*4180000 4160000 4200000 */
#define PMIC_VOLTAGE_MIN_WORK    3600000 /*3500000 3620000 */


#define BIT_CHG_VOL_LSH		0
#define BIT_CHG_VOL_WID		3

#define BIT_CHG_CURR_LSH		3
#define BIT_CHG_CURR_WID		4

#define BIT_CHG_PLIM_LSH		15
#define BIT_CHG_PLIM_WID		2

#define BIT_CHG_DETS_LSH 6
#define BIT_CHG_DETS_WID 1
#define BIT_CHG_CURRS_LSH 11
#define BIT_CHG_CURRS_WID 1

#define TRICKLE_CHG_EN_LSH	7
#define LOW_POWER_BOOT_ACK_LSH	8
#define BAT_TH_CHECK_DIS_LSH	9
#define BATTFET_CTL_EN_LSH	10
#define BATTFET_CTL_LSH		11
#define REV_MOD_EN_LSH		13
#define PLIM_DIS_LSH		17
#define CHG_LED_EN_LSH		18
#define CHGTMRRST_LSH		19
#define RESTART_CHG_STAT_LSH	20
#define AUTO_CHG_DIS_LSH	21
#define CYCLING_DIS_LSH		22
#define VI_PROGRAM_EN_LSH	23

#define TRICKLE_CHG_EN_WID	1
#define LOW_POWER_BOOT_ACK_WID	1
#define BAT_TH_CHECK_DIS_WID	1
#define BATTFET_CTL_EN_WID	1
#define BATTFET_CTL_WID		1
#define REV_MOD_EN_WID		1
#define PLIM_DIS_WID		1
#define CHG_LED_EN_WID		1
#define CHGTMRRST_WID		1
#define RESTART_CHG_STAT_WID	1
#define AUTO_CHG_DIS_WID	1
#define CYCLING_DIS_WID		1
#define VI_PROGRAM_EN_WID	1

#define ACC_STARTCC_LSH		0
#define ACC_STARTCC_WID		1
#define ACC_RSTCC_LSH		1
#define ACC_RSTCC_WID		1
#define ACC_CCFAULT_LSH		7
#define ACC_CCFAULT_WID		7
#define ACC_CCOUT_LSH		8
#define ACC_CCOUT_WID		16
#define ACC1_ONEC_LSH		0
#define ACC1_ONEC_WID		15

#define ACC_CALIBRATION 0x17
#define ACC_START_COUNTER 0x07
#define ACC_STOP_COUNTER 0x2
#define ACC_CONTROL_BIT_MASK 0x1f
#define ACC_ONEC_VALUE 2621
#define ACC_COULOMB_PER_LSB 1
#define ACC_CALIBRATION_DURATION_MSECS 20

#define PMIC_BAT_VOLTAGE_UNIT_UV 4692
#define PMIC_BAT_CURRENT_UNIT_UA 5870
#define PMIC_CHG_VOLTAGE_UINT_UV 23474
#define PMIC_CHG_MIN_CURRENT_UA 3500
#define PMIC_CHG_VOLTAGE_CORT    1023

#define MC13892_ISENSE0_VBUSVALIDS	(1<<3)	
#define MC13892_ISENSE0_CHGDETS		(1<<6)	
#define MC13892_ISENSE0_CHGCURRS	(1<<11)
#define IsChargerPresence(a)	( ((a) & (MC13892_ISENSE0_VBUSVALIDS|MC13892_ISENSE0_CHGDETS)) ==  \
								  (MC13892_ISENSE0_VBUSVALIDS|MC13892_ISENSE0_CHGDETS) )


#define COULOMB_TO_UAH(c) (10000 * c / 36)

//battery capacity in mAh
#define BAT_CAP_MAH 3000UL

#define CHG_CUR_MA   400UL
#define BAT_CAP_UL  3600UL
////Battary voltage need poweroff
#define BATTARY_VOLTAGE_POWEROFF  2950 /*2800 3500*/

int get_battery_mV(void);
static int get_charger_state(void);
extern void kernel_power_off(void);
extern void mx50_rdp_wdog_control(void);

#define PMIC_DEV_NAME "mc13892_battery"/*"pmic_battery"*/
/*
ICHRG[3:0]     Charge Regulator
                       Current Limit (mA)          Specific Use Case
0000                       0                                  Off
0001                     80                             Standalone Charging Default for precharging,
                                                             USB charging, and LPB
0010                    240
0011                    320
0100                   400                              Advised setting for USB charging with PHY active
0101                   480
0110                   560                              Standalone Charging Default
0111                   640
1000                   720
1001                   800
1010                   880
1011                   960
1100                  1040
1101                  1200                            High Current Charger
1110                  1600                            High Current Charger
1111                  Fully On ¨C M3 Open   Externally Powered
*/
#define  PMIC_SET_REG_CHARGE  0x6/*0x8*/
#define  PMIC_SET_DCDC_REG_CHARGE  0xc

enum chg_setting {
       TRICKLE_CHG_EN,/*0*/
       LOW_POWER_BOOT_ACK,
       BAT_TH_CHECK_DIS,
       BATTFET_CTL_EN,
       BATTFET_CTL,
       REV_MOD_EN,/*5*/
       PLIM_DIS,
       CHG_LED_EN,
       CHGTMRRST,
       RESTART_CHG_STAT,
       AUTO_CHG_DIS,/*10*/
       CYCLING_DIS,
       VI_PROGRAM_EN/*12*/
};

enum chg_state {
	CHG_POWER_OFF,/*0*/
	CHG_RESTART,/*1*/
	CHG_CHARGING,/*2*/
	CHG_DISCHARGING_WITH_CHARGER,/*3*/
	CHG_DISCHARGING,/*4*/
};

/* Flag used to indicate if Charger workaround is active. */
int chg_wa_is_active;
/* Flag used to indicate if Charger workaround timer is on. */
int chg_wa_timer;
int disable_chg_timer;
static unsigned long expire;
static int state=CHG_RESTART;
struct workqueue_struct *chg_wq;
struct delayed_work chg_work;

/*
* whether need reset the CHGAUTOVIB indicator.
* false: set CHGAUTOVIB as 0
* true:  set CHGAUTOVIB as 1
*/
static bool bNeedReset = false;
/* 
  * process  close the charger LED indicator
  * true: mean the LED had been closed
  * fale: mead the LED don't close when battery in charging
  */
static bool bProcessSign = false;
/*
  * in order to distinguish charger type:DC-DC or USB(connect to PC),
  * we need check the connect state,then distinguish the charger type
  * in file (arcotg_udc.c)
  */
bool bConnectState = false;
/*
  *  indicator  the charger state charger or not.
  *
  */
static bool bUsbstateChanged= false;
/*
  * bootup type : 
   *  1).connect to charger (DC-DC or USB)
    * 2).press power button to bootup
   */
static bool bBootupWithCharger  =  true;

/* capacity offset 
  */
static int capacityOffset =0;

extern  int get_bootup_type(void);
#define PMIC_MC13892_BATTERY_WORK

#if defined(PMIC_MC13892_BATTERY_WORK)
/*
* whether need reset the CHGAUTOVIB indicator.
* false: set CHGAUTOVIB as 0
* true:  set CHGAUTOVIB as 1
*/
struct workqueue_struct *chgautovib_wq;
struct delayed_work chgautovib_work;
static int work_destroy(void);
int get_battery_mA(void);
static void pmic_chgautovib_funtion(struct work_struct *work)
{
	/*
	* indicator  the device whether connect to charger or USB 
	* 0: disconnect
	* 1: connect
	*/
	int CheckChargerState  =0;
	/*
	   * read the charger current,if the charger current  > 240,  set the  bNeedReset as true;
	   */
	int ChargerMA=0;


     ///////////////////////////////////////////////
	////check the device whether connect to charger or USB
	//// if the device had been connected,we need reset the 
	////charger current. so we need changes the state,
	CheckChargerState= get_charger_state();
	ChargerMA = get_battery_mA();
	//printk("%s %s %d  CheckChargerState=%d, state=%d ,ChargerMA=%d\n",
	//	__FILE__,__func__,__LINE__,CheckChargerState,state,ChargerMA);
	 
	if((CheckChargerState == 1) 
		&& (state != CHG_RESTART)
		&& (ChargerMA > 240)){
		bNeedReset  =true;
		
		bProcessSign = false;
		state = CHG_RESTART;
	}

       ///check the result : if the bNeedReset still false;we need mod the timer,
	//printk("%s %s %d   bNeedReset=%d\n",__FILE__,__func__,__LINE__,bNeedReset);
       if(bNeedReset){
	   work_destroy();
       	}else{
	  queue_delayed_work(chgautovib_wq, &chgautovib_work, 10*HZ);
	}
}
static int work_init(void)
{
	int retval = 0;

	INIT_DELAYED_WORK(&chgautovib_work, pmic_chgautovib_funtion);
	chgautovib_wq = create_singlethread_workqueue("vib_wp");
	if (!chgautovib_wq) {
		retval = -ESRCH;
	}

	queue_delayed_work(chgautovib_wq, &chgautovib_work, 1*HZ);

	return 0;

}
static int work_destroy(void)
{
	cancel_rearming_delayed_workqueue(chgautovib_wq, &chgautovib_work);
	destroy_workqueue(chgautovib_wq);
 
	return 0;
}

#endif
static int pmic_set_chg_current(unsigned short curr)
{
	unsigned int mask;
	unsigned int value;

	value = BITFVAL(BIT_CHG_CURR, curr);
	mask = BITFMASK(BIT_CHG_CURR);
	CHECK_ERROR(pmic_write_reg(REG_CHARGE, value, mask));

	return 0;
}
////////////////////////
///charger is DCDC ,set charger current value is 1000mA: (0xc)
///charger is USB connect to PC,set charger current value is  560mA(0x6)
///true: charger is DCDC;
///false:charger is USB connect to PC
//////then we need reset the charger current,so we need set  the state
////// as CHG_RESTART
static bool bChargerIsDCDC = false;
int set_DCDC_charger_state(bool bState)
{
     bChargerIsDCDC = bState;

	bProcessSign = false;
     state = CHG_RESTART;

     return 0;
}
static int pmic_set_charger_current_value(void)
{
	//printk("%s %s %d  bChargerIsDCDC=%d\n",__FILE__,__func__,__LINE__,bChargerIsDCDC);
	if(bChargerIsDCDC){
		pmic_set_chg_current(PMIC_SET_DCDC_REG_CHARGE);
	}else{
		pmic_set_chg_current(PMIC_SET_REG_CHARGE);
	}
	return 0;		
}

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag)
{

	unsigned int reg_value = 0;
	unsigned int mask = 0;


	switch (type) {
	case TRICKLE_CHG_EN:
		reg_value = BITFVAL(TRICKLE_CHG_EN, flag);
		mask = BITFMASK(TRICKLE_CHG_EN);
		break;
	case LOW_POWER_BOOT_ACK:
		reg_value = BITFVAL(LOW_POWER_BOOT_ACK, flag);
		mask = BITFMASK(LOW_POWER_BOOT_ACK);
		break;
	case BAT_TH_CHECK_DIS:
		reg_value = BITFVAL(BAT_TH_CHECK_DIS, flag);
		mask = BITFMASK(BAT_TH_CHECK_DIS);
		break;
	case BATTFET_CTL_EN:
		reg_value = BITFVAL(BATTFET_CTL_EN, flag);
		mask = BITFMASK(BATTFET_CTL_EN);
		break;
	case BATTFET_CTL:
		reg_value = BITFVAL(BATTFET_CTL, flag);
		mask = BITFMASK(BATTFET_CTL);
		break;
	case REV_MOD_EN:
		reg_value = BITFVAL(REV_MOD_EN, flag);
		mask = BITFMASK(REV_MOD_EN);
		break;
	case PLIM_DIS:
		reg_value = BITFVAL(PLIM_DIS, flag);
		mask = BITFMASK(PLIM_DIS);
		break;
	case CHG_LED_EN:
		reg_value = BITFVAL(CHG_LED_EN, flag);
		mask = BITFMASK(CHG_LED_EN);
		break;
	case CHGTMRRST:
		reg_value = BITFVAL(CHGTMRRST, flag);
	        mask = BITFMASK(CHGTMRRST);
		break;		
	case RESTART_CHG_STAT:
		reg_value = BITFVAL(RESTART_CHG_STAT, flag);
		mask = BITFMASK(RESTART_CHG_STAT);
		break;
	case AUTO_CHG_DIS:
		reg_value = BITFVAL(AUTO_CHG_DIS, flag);
		mask = BITFMASK(AUTO_CHG_DIS);
		break;
	case CYCLING_DIS:
		reg_value = BITFVAL(CYCLING_DIS, flag);
		mask = BITFMASK(CYCLING_DIS);
		break;
	case VI_PROGRAM_EN:
		reg_value = BITFVAL(VI_PROGRAM_EN, flag);
		mask = BITFMASK(VI_PROGRAM_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	

	CHECK_ERROR(pmic_write_reg(REG_CHARGE, reg_value, mask));

	return 0;
}

static int pmic_get_batt_voltage(unsigned short *voltage)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_VOLTAGE;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*voltage = result[0];

	return 0;
}

static int pmic_get_batt_current(unsigned short *curr)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*curr = result[0];

	return 0;
}

static int coulomb_counter_calibration;
static unsigned int coulomb_counter_start_time_msecs;

static int pmic_start_coulomb_counter(void)
{
	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		ACC_COULOMB_PER_LSB * ACC_ONEC_VALUE, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_START_COUNTER, ACC_CONTROL_BIT_MASK));
	coulomb_counter_start_time_msecs = jiffies_to_msecs(jiffies);
	pr_debug("coulomb counter start time %u\n",
		coulomb_counter_start_time_msecs);
	
	return 0;
}

static int pmic_stop_coulomb_counter(void)
{
	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_STOP_COUNTER, ACC_CONTROL_BIT_MASK));
	return 0;
}

static int pmic_calibrate_coulomb_counter(void)
{
	int ret;
	unsigned int value;

	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		0x1, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_CALIBRATION, ACC_CONTROL_BIT_MASK));
	msleep(ACC_CALIBRATION_DURATION_MSECS);

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("calibrate value = %x\n", value);
	coulomb_counter_calibration = (int)((s16)((u16) value));
	pr_debug("coulomb_counter_calibration = %d\n",
		coulomb_counter_calibration);

	return 0;

}

static int pmic_get_charger_coulomb(int *coulomb)
{
	int ret;
	unsigned int value;
	int calibration;
	unsigned int time_diff_msec;

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	//printk("counter value = %x\n", value);
	*coulomb = ((s16)((u16)value)) * ACC_COULOMB_PER_LSB;

	if (abs(*coulomb) >= ACC_COULOMB_PER_LSB) {
			/* calibrate */
		time_diff_msec = jiffies_to_msecs(jiffies);
		time_diff_msec =
			(time_diff_msec > coulomb_counter_start_time_msecs) ?
			(time_diff_msec - coulomb_counter_start_time_msecs) :
			(0xffffffff - coulomb_counter_start_time_msecs
			+ time_diff_msec);
		calibration = coulomb_counter_calibration * (int)time_diff_msec
			/ (ACC_ONEC_VALUE * ACC_CALIBRATION_DURATION_MSECS);
		*coulomb -= calibration;
	}

	return 0;
}

static int pmic_get_charger_coulomb_raw(int *coulomb)
{
	int ret;
	unsigned int value;

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	//printk("counter value = %x\n", value);
	*coulomb = ((s16)((u16)value)) * ACC_COULOMB_PER_LSB;
	return 0;
}

static void init_charger_timer(void)
{
	pmic_set_chg_misc(CHGTMRRST, 1);
	expire = jiffies + ((BAT_CAP_MAH*BAT_CAP_UL*HZ)/CHG_CUR_MA);
	pr_notice("%s,%s\n", __FILE__,__func__);
}

static bool charger_timeout(void)
{
	return time_after(jiffies, expire);
}

static void reset_charger_timer(void)
{
	if(!charger_timeout())
		pmic_set_chg_misc(CHGTMRRST, 1);
}
//////
///close the charger led when battery voltage > 4.18v
/////
static int pmic_close_charger_led(void)
{
	if(bNeedReset){
	     ///first,changes to the Software controlled charging
	     pmic_set_chg_misc(AUTO_CHG_DIS, 1);

	       ////close the led
	       pmic_set_chg_misc(CHG_LED_EN, 0);
	  }
   
   return 0;

}
static int pmic_restart_charging(void)
{
	pmic_set_chg_misc(BAT_TH_CHECK_DIS, 1);

	pmic_set_chg_misc(PLIM_DIS, 3);

	//0 = Standalone charging
	///1 = Software controlled charging
	///default is 0
	////charger state from CHG_DISCHARGING_WITH_CHARGER to
	////  CHG_RESTART,we need make sure the LED is close.
	if(bProcessSign){
	   pmic_close_charger_led();
	}else{
	   pmic_set_chg_misc(AUTO_CHG_DIS, 0);
	}

	//pmic_set_chg_misc(RESTART_CHG_STAT, 1);

	///To override the charging current, we execute these 2 statements
	//////////////////////////////////////////////////////
	///in order to solve the question: when the device don't solder the 
	///battery and connect to USB or charger,the device can't bootup,
	///we need set the CHGAUTOVIB as 0. after power up,we set 1 again
	////////////////////////////////////////////////////////
#if defined(PMIC_MC13892_BATTERY_WORK)	
	if(bNeedReset){
		pmic_set_chg_misc(VI_PROGRAM_EN, 1);
		//bNeedReset = false;
	}else{
		pmic_set_chg_misc(VI_PROGRAM_EN, 0);
	}
#else
	pmic_set_chg_misc(VI_PROGRAM_EN, 1);
#endif		
	//pmic_set_chg_current(PMIC_SET_REG_CHARGE);

	pmic_set_chg_misc(RESTART_CHG_STAT, 1);

	//pmic_set_chg_misc(PLIM_DIS, 3); //Bug?

	return 0;
}


struct mc13892_dev_info {
	struct device *dev;

	unsigned short voltage_raw;
	int voltage_uV;
	unsigned short current_raw;
	int current_uA;
	int battery_status;
	int full_counter;
	int charger_online;
	int charger_voltage_uV;
	int accum_current_uAh;

	int init_charge;
	int capacity;
	struct delayed_work calc_capacity;

	struct power_supply bat;
	struct power_supply charger;

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

#define mc13892_SENSER	25
#define to_mc13892_dev_info(x) container_of((x), struct mc13892_dev_info, \
					      bat);

static enum power_supply_property mc13892_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	//wen del:POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
	//add for report battary valtoge
 	POWER_SUPPLY_PROP_VOLTAGE_MAX, 
 	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,  
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,   
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY
	//add end	
};

static enum power_supply_property mc13892_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

//static int pmic_get_chg_value(unsigned int *value)
//{
//	t_channel channel;
//	unsigned short result[8], max1 = 0, min1 = 0, max2 = 0, min2 = 0, i;
//	unsigned int average = 0, average1 = 0, average2 = 0;
//
//	channel = CHARGE_CURRENT;
//	CHECK_ERROR(pmic_adc_convert(channel, result));
//
//
//	for (i = 0; i < 8; i++) {
//		if ((result[i] & 0x200) != 0) {
//			result[i] = 0x400 - result[i];
//			average2 += result[i];
//			if ((max2 == 0) || (max2 < result[i]))
//				max2 = result[i];
//			if ((min2 == 0) || (min2 > result[i]))
//				min2 = result[i];
//		} else {
//			average1 += result[i];
//			if ((max1 == 0) || (max1 < result[i]))
//				max1 = result[i];
//			if ((min1 == 0) || (min1 > result[i]))
//				min1 = result[i];
//		}
//	}
//
//	if (max1 != 0) {
//		average1 -= max1;
//		if (max2 != 0)
//			average2 -= max2;
//		else
//			average1 -= min1;
//	} else
//		average2 -= max2 + min2;
//
//	if (average1 >= average2) {
//		average = (average1 - average2) / 6;
//		*value = average;
//	} else {
//		average = (average2 - average1) / 6;
//		*value = ((~average) + 1) & 0x3FF;
//	}
//
//	return 0;
//}
//
static int get_charger_state(void)
{
       unsigned int value = 0;
       int charger =0;
       int retval=-1;

	retval = pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
	if(retval == 0){
	     charger = BITFEXT(value, BIT_CHG_DETS);
	}
	bConnectState  = charger;
      return charger;
}

int get_battery_mA(void) /* get charging current float into battery */
{
	unsigned short value=0;
	int bat_curr=0;
	int bat_cur=0;
	int chargerState=0;
	bool bPoweroffProcessSign = false;

	pmic_get_batt_current(&value);
	bat_curr = ((value&0x200) ? (value|0xfffffc00) : value);
	bat_curr = (bat_curr*3000)/512;



	 ////wen add ,only for test,
	 if((-bat_curr) >= 0){
		 bat_cur = (-bat_curr);
	 }else{
	 	chargerState =get_charger_state();
		 if(chargerState == 1){
			 bat_cur = 0;
		 }else{
			 ///power off,only executed once
			 ////whether check the battaty voltage ?
			 if((!bPoweroffProcessSign) && (get_battery_mV()< 3400)){
				 pr_notice("Charger current < 0 and Battery voltage < 3.4v, Power off \n");
				// orderly_poweroff(1);
				 kernel_power_off();
				 bPoweroffProcessSign = true;
			 }
		 }
	 }


	return bat_cur;
}

int get_battery_mV(void)
{
	unsigned short value=0;
	
	pmic_get_batt_voltage(&value);

	return  (value*PMIC_BAT_VOLTAGE_UNIT_UV/PMIC_CHG_VOLTAGE_CORT);
}
int poweroff_chg_thread(void)
{
	//int chargerMa =0;
	//int ii=0;
	unsigned int value=0;

	//printk("%s %s %d   state=%d\n",__FILE__,__func__,__LINE__,state);

	///when device connect to DC-DC or USB.
	if(state == CHG_CHARGING )
	{
		///when the voltage > 4000,the current 
	    /// charger is less than 700
		if((get_battery_mV() < 4000)
			&&( get_battery_mA() < 700 )){
			bNeedReset =true;
			bChargerIsDCDC =true;
			state = CHG_RESTART;
		 }
		 msleep(1000);
		/*device in charging*/
		while(1)
		{

			pmic_read_reg(REG_INT_SENSE0, &value, 0xffffff);
			if(!IsChargerPresence(value))	
				break;

			if( get_battery_mA() <60)
				break;				
			
			pmic_read_reg(REG_INT_SENSE1, &value, 0xffffff);		
			if ((value&(1<<2))== 0 )
			{
				// PCCOUNT=0
				pmic_write_reg(REG_POWER_CTL1, 0, 0x000f00);
				// WDIRESET=1
				pmic_write_reg(REG_POWER_CTL2, 0x1000, 0x1000);
				// assert WDI pin
				mx50_rdp_wdog_control();
				break;	
			}
			msleep(500);
		}	
	}
	return 0;
}
static void chg_thread(struct work_struct *work)
{
	//unsigned int value = 0;
	int charger=0;
	int ChargerMA=0;

	charger = get_charger_state();
	switch(state)
	{
	case CHG_RESTART:
		pmic_restart_charging();
		pmic_set_chg_current(0);
		if(charger){
			if(get_battery_mV()>BATTARY_VOLTAGE_POWEROFF){
				init_charger_timer();
				pmic_set_charger_current_value();
				state = CHG_CHARGING;
			}else{
				pmic_set_charger_current_value();
				msleep(50);
				if(get_battery_mA()>240){ /* if PMIC can provide 400mA */
					init_charger_timer();
					state = CHG_CHARGING;
				}else{
					state = CHG_POWER_OFF;
				}
			}
		}else{
			state = CHG_DISCHARGING;
		}
		queue_delayed_work(chg_wq, &chg_work, HZ*1);
		break;

	case CHG_POWER_OFF:
		pr_notice("Battery level < 3.0V!\n");
		pr_notice("After power off, PMIC will charge up battery.\n");
		//pmic_set_chg_current(PMIC_SET_REG_CHARGE); /* charge battery during power off */
		pmic_set_charger_current_value();/* charge battery during power off */
		orderly_poweroff(1);
		break;
 
	case CHG_CHARGING:
		
		reset_charger_timer();
		ChargerMA = get_battery_mA();
		if(charger_timeout() || (ChargerMA<50)){
			pmic_set_chg_current(0);
			state = CHG_DISCHARGING_WITH_CHARGER;
		}
		if(!charger){
			pmic_set_chg_current(0);
			state = CHG_DISCHARGING;
			bProcessSign = false;
		}
		if((charger) && (ChargerMA > 0) && (ChargerMA < 60)
			&& (!bProcessSign) && bNeedReset){
                pmic_close_charger_led();
                bProcessSign = true;
          }				
		queue_delayed_work(chg_wq, &chg_work, HZ*5);
		break;

	case CHG_DISCHARGING:
		bProcessSign = false;
		if(charger)
			state = CHG_RESTART;
		queue_delayed_work(chg_wq, &chg_work, HZ*10);
		break;

	case CHG_DISCHARGING_WITH_CHARGER:
		if(get_battery_mV() < PMIC_VOLTAGE_MAX_WORK)/*4000*/
			state = CHG_RESTART;
		if(!charger)
			state = CHG_DISCHARGING;
		queue_delayed_work(chg_wq, &chg_work, HZ*2);
		break;
	}
 }

static int mc13892_charger_update_status(struct mc13892_dev_info *di)
{
	int ret;
	unsigned int value;
	int online;

	ret = pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));

	if (ret == 0) {
		online = BITFEXT(value, BIT_CHG_DETS);
		if (online != di->charger_online) {
			di->charger_online = online;
			dev_info(di->charger.dev, "charger status: %s\n",
				online ? "online" : "offline");
			power_supply_changed(&di->charger);

			if (online) {
				cancel_delayed_work_sync(&di->calc_capacity);
				queue_delayed_work(di->monitor_wqueue, &di->calc_capacity,0);
			} else {
				cancel_delayed_work_sync(&di->calc_capacity);
				queue_delayed_work(di->monitor_wqueue, &di->calc_capacity, HZ * 10);
			}

			cancel_delayed_work(&di->monitor_work);
			queue_delayed_work(di->monitor_wqueue,
				&di->monitor_work, HZ / 10);
			if (online) {
				//pmic_restart_charging();
				//queue_delayed_work(chg_wq, &chg_work, 100);
				chg_wa_timer = 1;
			} else {
				//cancel_delayed_work(&chg_work);
				chg_wa_timer = 0;
		        }
	        }
	}
	
        ////////////////////////////////////////////////
         bChargerIsDCDC = false;
	
	return ret;
}

static int mc13892_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di =
		container_of((psy), struct mc13892_dev_info, charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->charger_online;
		return 0;
	default:
		break;
	}
	return -EINVAL;
}

static int mc13892_battery_read_status(struct mc13892_dev_info *di)
{
	int retval;
	int coulomb;
	retval = pmic_get_batt_voltage(&(di->voltage_raw));
	if (retval == 0)
		di->voltage_uV = di->voltage_raw * PMIC_BAT_VOLTAGE_UNIT_UV;

	retval = pmic_get_batt_current(&(di->current_raw));
	if (retval == 0) {
		if (di->current_raw & 0x200)
			di->current_uA =
				(0x1FF - (di->current_raw & 0x1FF)) *
				PMIC_BAT_CURRENT_UNIT_UA * (-1);
		else
			di->current_uA =
				(di->current_raw & 0x1FF) * PMIC_BAT_CURRENT_UNIT_UA;
	}
	retval = pmic_get_charger_coulomb(&coulomb);
	if (retval == 0)
		di->accum_current_uAh = COULOMB_TO_UAH(coulomb);
	else
		di->accum_current_uAh = -1;

	return retval;
}

//#define MAX_Charger_V_LiPo	 4160/*4100*/
#define MAX_V_LiPo	4140/*4100*/
#define MIN_V_LiPo	3600
#define MAX_Charger_V_LiPo	 4200   /* 4160 MAX_V_LiPo 4100*/
#define MIN_Charger_V_LiPo	 3650/*3620*/

static const struct {
	int v; /*in mV*/
	int c; /*in percent */
} discharge[] ={ 
	{MAX_V_LiPo, 100},  
	{4138, 98 }, 
	{4130, 96 },  
	{4120, 94 },  
	{4106, 92 }, 
	{4096, 90 },  
	{4054, 86 },  
	{4030, 82 },  
	{4021, 80 },  
	{3992, 76 },  
	{3979, 74 },  
	{3951, 70 },  
	{3941, 68 },  
	{3937, 64 },  
	{3904, 60 },  
	{3890, 58 },  
	{3880, 56 },  
	{3876, 54 },  
	{3857, 52 },  
	{3847, 50 },  
	{3838, 45 },  
	{3815, 40 },  
	{3805, 35 },  
	{3781, 34 },  
	{3791, 32 },  
	{3786, 30 },  
	{3781, 28 },  
	{3768, 25 },  
	{3749, 22 },  
	{3730, 20 },  
	{3729, 18 },  
	{3728, 16 },  
	{3725, 14 },  
	{3707, 12 },  
	{3702, 10 },  
	{3688, 8  }, 
	{3679, 6  },  
	{3660, 5  },  
	{3648, 3  },  
	{3618, 1  },  
	{3600, 0  },  
	{0,0}
}; 

static const struct {
	int v; /*in mV*/
	int c; /*in percent */
} charge[] ={
	{MAX_Charger_V_LiPo, 100 },
	{4198, 98  },
	{4194, 96  },
	{4180, 94  },
	{4185, 92  },
	{4180, 90  },
	{4166, 86  },
	{4134, 82  },
	{4115, 80  },
	{4091, 76  },
	{4082, 74  },
	{4063, 70  },
	{4058, 68  },
	{4044, 64  },
	{4030, 60  },
	{4002, 58  },
	{3992, 56  },
	{3984, 54  },
	{3979, 52  },
	{3974, 50  },
	{3969, 45  },
	{3956, 40  },
	{3946, 35  },
	{3932, 34  },
	{3927, 32  },
	{3923, 30  },
	{3918, 28  },
	{3904, 25  },
	{3890, 22  },
	{3884, 20  },
	{3876, 18  },
	{3868, 16  },
	{3860, 14  },
	{3852, 12  },
	{3847, 10  },
	{3824, 8   },
	{3800, 6   },
	{3768, 5   },
	{3744, 3   },
	{3725, 1   },
	{3700, 0   },
	{0,  0}
};

//return a charge of battery in mAh. It get mV.
static int mc13892_compute_battery_charge_from_V(int volts) 
{

	long c;
	int  chargerState =0;
	int i = 0;
	int top_range_v = 0;
	int top_range_c = 0;


	chargerState = get_charger_state();

	if(chargerState ==1){
		if (volts > MAX_Charger_V_LiPo) return BAT_CAP_MAH;
		if (volts <= MIN_Charger_V_LiPo) return 0;
		top_range_v=MAX_Charger_V_LiPo;
		while (volts < charge[i].v) {
			top_range_v = charge[i].v;
			top_range_c = charge[i].c;
			i++;
		}
		c = ((volts - charge[i].v) * 100 / (top_range_v - charge[i].v) * (top_range_c - charge[i].c) / 100 + charge[i].c) *
			BAT_CAP_MAH / 100;
	}else if(chargerState ==0){
		if (volts > MAX_V_LiPo) return BAT_CAP_MAH;
		if (volts <= MIN_V_LiPo) return 0;
		top_range_v=MAX_V_LiPo;
		while (volts < discharge[i].v) {
			top_range_v = discharge[i].v;
			top_range_c = discharge[i].c;
			i++;
		}
		c = ((volts - discharge[i].v) * 100 / (top_range_v - discharge[i].v) * (top_range_c - discharge[i].c) / 100 + discharge[i].c) *
			BAT_CAP_MAH / 100;
	}

	return (int)c;
}
static int mc13892_compute_battery_capacity_from_CC(struct work_struct *work) 
{
	int v=0;
	static int old_state = -1;
	//static int old_init_charge = 0;
	int CurVolts =0;
	int iCount=0;
	int  CurRangeNextC =0;
	int  CurRangeC =0;
	int CurCountNext = 0;
	int CurCountPre = 0;
	static int old_CurRangeC = 0;
	long cc = 0;

	struct mc13892_dev_info *di = container_of(work, struct mc13892_dev_info, calc_capacity.work);

	if (di->charger_online != old_state) {
		//pmic_calibrate_coulomb_counter();
		pmic_stop_coulomb_counter();
		di->init_charge = -1;

	}

	if (di->init_charge == -1) 
	{ /*first entry - init */
		//pmic_set_chg_current(0);
		mc13892_battery_read_status(di);
		//printk("[%s] di->voltage_uV = %i; %i\n",__func__,di->voltage_uV,di->voltage_uV/1000);
		di->init_charge = mc13892_compute_battery_charge_from_V(di->voltage_uV / 1000);
		pmic_start_coulomb_counter();
		//mc13892_battery_read_status(di);
	}

	old_state = di->charger_online;


	pmic_get_charger_coulomb_raw(&cc);

	if (di->charger_online) {
		v = (di->init_charge + ((long)abs(cc) * 10 / 36)) * 100 /  BAT_CAP_MAH;
	} else {
		v = (di->init_charge - ((long)abs(0xffff - cc ? cc : 0xffff) * 10 / 36)) * 100 /  BAT_CAP_MAH;
	}


	if(bUsbstateChanged)
	{
		bUsbstateChanged = false;
		capacityOffset = v - di->capacity;
	}

	v=v -capacityOffset;

	///Bootup with DC-DC,sometimes the capacity read error,so we
	///need reread again.
	if((di->charger_online) && bBootupWithCharger){
		CurVolts =(di->voltage_uV / 1000) ;
		while (CurVolts < charge[iCount].v) {
			CurCountNext = iCount+1;
			CurRangeNextC = charge[CurCountNext].c;
			CurRangeC  = charge[iCount].c;
			iCount++;
		}

		if(( v > CurRangeC ) 
			||  (v< CurRangeNextC) 
			)
		{
			di->init_charge = -1;
			if (di->monitor_wqueue)
				queue_delayed_work(di->monitor_wqueue, &di->calc_capacity, msecs_to_jiffies(50));
			else
				schedule_delayed_work(&di->calc_capacity, msecs_to_jiffies(50));
		}
		if(v< old_CurRangeC){
			v= old_CurRangeC;
		}else{
			old_CurRangeC =v;
		}
	}

	if (v > 100) v = 100;
	if (v < 0) v = 0;
	di->capacity = v;

	////sometimes when the voltage is low 3600,
	///the capacity will be read error. it as 100,so we use the
	///follow funtions to avoid it.
	if(!(di->charger_online)){
		if(((di->voltage_uV / 1000) < 3550)
			&& ((di->voltage_uV / 1000) > 3525))
		{
			if(di->capacity > 5)
				di->capacity = 4;

		}else if (((di->voltage_uV / 1000) < 3525)
			&& ((di->voltage_uV / 1000) > 3500))
		{
			if(di->capacity > 2)
			   di->capacity = 2;
		}else if ((di->voltage_uV / 1000) < 3500)
		{
			di->capacity = 0;
		}
	}

	if (di->monitor_wqueue)
		queue_delayed_work(di->monitor_wqueue, &di->calc_capacity, msecs_to_jiffies(5000));
	else
		schedule_delayed_work(&di->calc_capacity, msecs_to_jiffies(5000));
	return 0;
}

static void mc13892_battery_update_status(struct mc13892_dev_info *di)
{
	unsigned int value;
	int retval;
	int old_battery_status = di->battery_status;


	if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN)
		di->full_counter = 0;

	if (di->charger_online) {

		value = get_charger_state();

		if (value){
			di->battery_status =
				POWER_SUPPLY_STATUS_CHARGING;
		}else{
			di->battery_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

		if (di->battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			di->full_counter++;
		else
			di->full_counter = 0;
	} else {
		di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->full_counter = 0;
	}

	dev_dbg(di->bat.dev, "bat status: %d\n",di->battery_status);

	if (old_battery_status != POWER_SUPPLY_STATUS_UNKNOWN 
	 /*&& di->battery_status != old_battery_status */){
	//if(di->battery_status !=POWER_SUPPLY_STATUS_UNKNOWN){
		power_supply_changed(&di->bat);
	}
}

static void mc13892_battery_work(struct work_struct *work)
{
	struct mc13892_dev_info *di = container_of(work,
						     struct mc13892_dev_info,
						     monitor_work.work);
	const int interval =HZ * 30; /*HZ * 60  msecs_to_jiffies(5000);*/


	mc13892_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static void charger_online_event_callback(void *para)
{
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	pr_info("\n\n DETECTED charger plug/unplug event, %s \n",__func__);
	mc13892_charger_update_status(di);
	/*usb connect state had been changed*/
        bUsbstateChanged = true;
	/*make sure the bootup is over. */
	bBootupWithCharger =false;

}


static int mc13892_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di = to_mc13892_dev_info(psy);
	unsigned int  online=0;
	static int lowBattaryCount =0;
	//bool bInLowBattaryState = false;

	online = get_charger_state();
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN) {
			mc13892_charger_update_status(di);
			mc13892_battery_update_status(di);
		}
		val->intval = di->battery_status;
		return 0;
	default:
		break;
	}

	mc13892_battery_read_status(di);
	

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->accum_current_uAh;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = PMIC_VOLTAGE_MAX_WORK;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = PMIC_VOLTAGE_MIN_WORK;		
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->capacity;
		break;
				
				
 	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
 		val->intval = PMIC_VOLTAGE_MAX_WORK;
 		break;
 	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
 		val->intval = PMIC_VOLTAGE_MIN_WORK; 
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD; 
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t chg_wa_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (chg_wa_is_active & chg_wa_timer)
		return sprintf(buf, "Charger LED workaround timer is on\n");
	else
		return sprintf(buf, "Charger LED workaround timer is off\n");
}

static ssize_t chg_wa_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer)
				printk(KERN_INFO "Charger timer is already on\n");
			else {
				//queue_delayed_work(chg_wq, &chg_work, 100);
				chg_wa_timer = 1;
				printk(KERN_INFO "Turned on the timer\n");
			}
		}
	} else if (strstr(buf, "0") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer) {
				//cancel_delayed_work(&chg_work);
				chg_wa_timer = 0;
				printk(KERN_INFO "Turned off charger timer\n");
			 } else {
				printk(KERN_INFO "The Charger workaround timer is off\n");
			}
		}
	}

	return size;
}

static DEVICE_ATTR(enable, 0644, chg_wa_enable_show, chg_wa_enable_store);

static int pmic_battery_remove(struct platform_device *pdev)
{
	pmic_event_callback_t bat_event_callback;
	struct mc13892_dev_info *di = platform_get_drvdata(pdev);

	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_CHGDETI, bat_event_callback);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	cancel_rearming_delayed_workqueue(di->monitor_wqueue, &di->calc_capacity);
	cancel_rearming_delayed_workqueue(chg_wq,
					  &chg_work);
	destroy_workqueue(di->monitor_wqueue);
	destroy_workqueue(chg_wq);
	chg_wa_timer = 0;
	chg_wa_is_active = 0;
	disable_chg_timer = 0;
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->charger);

	kfree(di);

	return 0;
}

static int pmic_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct mc13892_dev_info *di;
	pmic_event_callback_t bat_event_callback;
	pmic_version_t pmic_version;

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	/* Only apply battery driver for MC13892 V2.0 due to ENGR108085 */
	pmic_version = pmic_get_version();
	if (pmic_version.revision < 20) {
		pr_debug("Battery driver is only applied for MC13892 V2.0\n");
		return -1;
	}
	if (machine_is_mx50_arm2()) {
		pr_debug("mc13892 charger is not used for this platform\n");
		return -1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	di->init_charge = -1;

	platform_set_drvdata(pdev, di);
   
	di->charger.name	= "mc13892_charger";
	di->charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->charger.properties = mc13892_charger_props;
	di->charger.num_properties = ARRAY_SIZE(mc13892_charger_props);
	di->charger.get_property = mc13892_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, mc13892_battery_work);
	
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 10);
	//queue_delayed_work(di->monitor_wqueue, &di->monitor_work, msecs_to_jiffies(10000));

	pmic_stop_coulomb_counter();
	pmic_calibrate_coulomb_counter();

	//for get correct voltage on the battery when booting with external power.
	//chg_thread will change it, when next work (chg_work) is start.
	pmic_set_chg_current(0);
	INIT_DELAYED_WORK(&di->calc_capacity,mc13892_compute_battery_capacity_from_CC);
	queue_delayed_work(di->monitor_wqueue, &di->calc_capacity, 0);

	INIT_DELAYED_WORK(&chg_work, chg_thread);
	chg_wq = create_singlethread_workqueue("mxc_chg");
	if (!chg_wq) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(chg_wq, &chg_work, HZ);

	di->dev	= &pdev->dev;
	di->bat.name	= "mc13892_bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = mc13892_battery_props;
	di->bat.num_properties = ARRAY_SIZE(mc13892_battery_props);
	di->bat.get_property = mc13892_battery_get_property;
	di->bat.use_for_apm = 1;

	di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
	//di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING; 


	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}

   


	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_subscribe(EVENT_CHGDETI, bat_event_callback);

	retval = sysfs_create_file(&pdev->dev.kobj, &dev_attr_enable.attr);
	if (retval) {
		printk(KERN_ERR
		       "Battery: Unable to register sysdev entry for Battery");
		goto workqueue_failed;
	}
	
	chg_wa_is_active = 1;
	chg_wa_timer = 0;
	disable_chg_timer = 0;

#if defined(PMIC_MC13892_BATTERY_WORK)
      work_init();
#endif
	
	goto success;

workqueue_failed:
	power_supply_unregister(&di->charger);
charger_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	dev_dbg(di->dev, "%s battery probed!\n", __func__);
	return retval;


	return 0;
}

static struct platform_driver pmic_battery_driver_ldm = {
	.driver = {
		   .name = PMIC_DEV_NAME,
		   .bus = &platform_bus_type,
		   },
	.probe = pmic_battery_probe,
	.remove = pmic_battery_remove,
};

static int __init pmic_battery_init(void)
{
	pr_debug("PMIC Battery driver loading...\n");
	return platform_driver_register(&pmic_battery_driver_ldm);
}

static void __exit pmic_battery_exit(void)
{
	platform_driver_unregister(&pmic_battery_driver_ldm);
	pr_debug("PMIC Battery driver successfully unloaded\n");
}

module_init(pmic_battery_init);
module_exit(pmic_battery_exit);

MODULE_DESCRIPTION("pmic_battery driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
