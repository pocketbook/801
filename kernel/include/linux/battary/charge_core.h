#ifndef __CHARGE_CORE_H__
#define __CHARGE_CORE_H__


#define ADAPTER_THIRDPARTY_INSERT 0x3

#define USB_OFFLINE 4


enum bat_state {
	BAT_BLANK,
        BAT_CRITICALLOW,
	//BAT_DISCHARGE_WITH_CHARGER,
	BAT_DISCHARGE_WITHOUT_CHARGER,
	BAT_CHARGE_WITH_CHARGER,
	BAT_CHARGEFULL,
	BAT_STATUS_NUM,
};
int get_bat_charge_status(void);

struct charge_info{
	int battery_level_curr_status;			
	int battery_level_prev_status;
};


#endif
