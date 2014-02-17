/*
 * An I2C driver for the Philips PCF8563 RTC
 * Copyright 2005-06 Tower Technologies
 *
 * Author: Alessandro Zummo <a.zummo@towertech.it>
 * Maintainers: http://www.nslu2-linux.org/
 *
 * based on the other drivers in this same directory.
 *
 * http://www.semiconductors.philips.com/acrobat/datasheets/PCF8563-04.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/mach/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#define DRV_VERSION "0.4.3"
#define RTC_PCF8563_NAME	"rtc-pcf8563"

#define PCF8563_REG_ST1		0x00 /* status */
#define PCF8563_REG_ST2		0x01

#define PCF8563_REG_SC		0x02 /* datetime */
#define PCF8563_REG_MN		0x03
#define PCF8563_REG_HR		0x04
#define PCF8563_REG_DM		0x05
#define PCF8563_REG_DW		0x06
#define PCF8563_REG_MO		0x07
#define PCF8563_REG_YR		0x08

#define PCF8563_REG_AMN		0x09 /* alarm */
#define PCF8563_REG_AHR		0x0A
#define PCF8563_REG_ADM		0x0B
#define PCF8563_REG_ADW		0x0C

#define PCF8563_REG_CLKO	0x0D /* clock out */
#define PCF8563_REG_TMRC	0x0E /* timer control */
#define PCF8563_REG_TMR		0x0F /* timer */

#define PCF8563_SC_LV		0x80 /* low voltage */
#define PCF8563_MO_C		0x80 /* century */


//#define PCF8563_RTC_ALARM


static struct i2c_driver pcf8563_driver;

struct pcf8563 {
	struct rtc_device *rtc;
	/*
	 * The meaning of MO_C bit varies by the chip type.
	 * From PCF8563 datasheet: this bit is toggled when the years
	 * register overflows from 99 to 00
	 *   0 indicates the century is 20xx
	 *   1 indicates the century is 19xx
	 * From RTC8564 datasheet: this bit indicates change of
	 * century. When the year digit data overflows from 99 to 00,
	 * this bit is set. By presetting it to 0 while still in the
	 * 20th century, it will be set in year 2000, ...
	 * There seems no reliable way to know how the system use this
	 * bit.  So let's do it heuristically, assuming we are live in
	 * 1970...2069.
	 */
	int c_polarity;	/* 0: MO_C=1 means 19xx, otherwise MO_C=1 means 20xx */
};

/*
 * In the routines that deal directly with the pcf8563 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int pcf8563_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);
	unsigned char buf[13] = { PCF8563_REG_ST1 };

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, buf },	/* setup read ptr */
		{ client->addr, I2C_M_RD, 13, buf },	/* read status + date */
	};

	/* read registers */
	if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	if (buf[PCF8563_REG_SC] & PCF8563_SC_LV)
		dev_info(&client->dev,
		"low voltage detected, date/time is not reliable.\n");

	dev_dbg(&client->dev,
		"%s: raw data is st1=%02x, st2=%02x, sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x\n",
		__func__,
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8]);


	tm->tm_sec = bcd2bin(buf[PCF8563_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(buf[PCF8563_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF8563_REG_HR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF8563_REG_DM] & 0x3F);
	tm->tm_wday = buf[PCF8563_REG_DW] & 0x07;
	tm->tm_mon = bcd2bin(buf[PCF8563_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[PCF8563_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */
	/* detect the polarity heuristically. see note above. */
	pcf8563->c_polarity = (buf[PCF8563_REG_MO] & PCF8563_MO_C) ?
		(tm->tm_year >= 100) : (tm->tm_year < 100);

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* the clock can give out invalid datetime, but we cannot return
	* -EINVAL otherwise hwclock will refuse to set the time on bootup.
	*/
	if (rtc_valid_tm(tm) < 0)
		dev_err(&client->dev, "retrieved date/time is not valid.\n");

	return 0;
}

static int pcf8563_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);
	int i, err;
	unsigned char buf[9];

	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* hours, minutes and seconds */
	buf[PCF8563_REG_SC] = bin2bcd(tm->tm_sec);
	buf[PCF8563_REG_MN] = bin2bcd(tm->tm_min);
	buf[PCF8563_REG_HR] = bin2bcd(tm->tm_hour);

	buf[PCF8563_REG_DM] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[PCF8563_REG_MO] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	buf[PCF8563_REG_YR] = bin2bcd(tm->tm_year % 100);
	if (pcf8563->c_polarity ? (tm->tm_year >= 100) : (tm->tm_year < 100))
		buf[PCF8563_REG_MO] |= PCF8563_MO_C;

	buf[PCF8563_REG_DW] = tm->tm_wday & 0x07;

	/* write register's data */
	for (i = 0; i < 7; i++) {
		unsigned char data[2] = { PCF8563_REG_SC + i,
			buf[PCF8563_REG_SC + i] };

		err = i2c_master_send(client, data, sizeof(data));
		if (err != sizeof(data)) {
			dev_err(&client->dev,
				"%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, data[0], data[1]);
			return -EIO;
		}
	};

	return 0;
}
#if defined(PCF8563_RTC_ALARM)
/*
* Read alarm time and date in RTC
*/
static int pcf8563_rtc_readalarm(struct i2c_client *client, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;

	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);
	unsigned char buf[13] = { PCF8563_REG_ST1 };

	struct i2c_msg msgs[] = {
		{ client->addr, 0, 1, buf },	/* setup read ptr */
		{ client->addr, I2C_M_RD, 13, buf },	/* read status + date */
	};

	/* read registers */
	if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	if (buf[PCF8563_REG_SC] & PCF8563_SC_LV)
		dev_info(&client->dev,"low voltage detected, date/time is not reliable.\n");

	dev_info(&client->dev,
		"%s: raw data is st1=%02x, st2=%02x, sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x\n",
		__func__,
		buf[0], buf[1], buf[2], buf[0x09],
		buf[0x0a], buf[0x0b], buf[0x0c]);
	//AIE bit1 TIE bit0 
	alrm->enabled =(buf[PCF8563_REG_SC] & 0x03)?1:0;
	alrm->pending =0; //? what means?
	if(alrm->enabled)
	{
		//second is always :00
		alrm->time.tm_sec = 00;//bcd2bin(buf[PCF8563_REG_SC] & 0x7F);
		//min
		alrm->time.tm_min = bcd2bin(buf[PCF8563_REG_AMN] & 0x7F);
		alrm->time.tm_hour = bcd2bin(buf[PCF8563_REG_AHR] & 0x3F); /* rtc hr 0-23 */
		alrm->time.tm_mday = bcd2bin(buf[PCF8563_REG_ADM] & 0x3F);
		alrm->time.tm_wday = buf[PCF8563_REG_ADW] & 0x07;

		alrm->time.tm_mon = bcd2bin(buf[PCF8563_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
		alrm->time.tm_year = bcd2bin(buf[PCF8563_REG_YR]);
		if (alrm->time.tm_year < 70)
			alrm->time.tm_year += 100;	/* assume we are in 1970...2069 */
		/* detect the polarity heuristically. see note above. */
		pcf8563->c_polarity = (buf[PCF8563_REG_MO] & PCF8563_MO_C) ?
			(alrm->time.tm_year >= 100) : (alrm->time.tm_year < 100);
	}
	else // no alarm
	{
		alrm->time.tm_sec = 00;
		alrm->time.tm_min = 00;
		alrm->time.tm_hour =  00;
		alrm->time.tm_mday = 00;
		alrm->time.tm_wday = 00;
		alrm->time.tm_mon = 00;
		alrm->time.tm_year = 00;
	}
	dev_info(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		alrm->time.tm_sec, alrm->time.tm_min, alrm->time.tm_hour,
		alrm->time.tm_mday, alrm->time.tm_mon, alrm->time.tm_year, alrm->time.tm_wday);

	/* the clock can give out invalid datetime, but we cannot return
	* -EINVAL otherwise hwclock will refuse to set the time on bootup.
	*/
	if (rtc_valid_tm(&alrm->time) < 0)
	{
		dev_err(&client->dev, "retrieved date/time is not valid.\n");
		return -10;
	}
	return 0;
}

/*
* Set alarm time and date in RTC
*/
static int pcf8563_rtc_setalarm(struct i2c_client *client, struct rtc_wkalrm *alrm)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);
	int i, err;
	unsigned char buf[13];

	dev_info(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		alrm->time.tm_sec, alrm->time.tm_min, alrm->time.tm_hour,
		alrm->time.tm_mday, alrm->time.tm_mon, alrm->time.tm_year, alrm->time.tm_wday);

	/* hours, minutes and seconds */
	//buf[PCF8563_REG_SC] = bin2bcd(tm->tm_sec);
	buf[PCF8563_REG_AMN] = bin2bcd(alrm->time.tm_min)&0x7F;
	buf[PCF8563_REG_AHR] = bin2bcd(alrm->time.tm_hour)&0x7F;
	buf[PCF8563_REG_ADM] = bin2bcd(alrm->time.tm_mday)&0x7F;
	buf[PCF8563_REG_ADW] = (alrm->time.tm_wday & 0x07)&0x7F;
	//enable alarm
	buf[PCF8563_REG_ST2] = 0x03;
	{
		unsigned char data[2] = { PCF8563_REG_ST2 ,buf[PCF8563_REG_ST2] };

		err = i2c_master_send(client, data, sizeof(data));
		if (err != sizeof(data)) {
			dev_err(&client->dev,
				"%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, data[0], data[1]);
			return -EIO;
		}
	}
	/* write register's data */
	for (i = 0; i < 4; i++) {
		unsigned char data[2] = { PCF8563_REG_AMN + i,
			buf[PCF8563_REG_AMN + i] };

		err = i2c_master_send(client, data, sizeof(data));
		if (err != sizeof(data)) {
			dev_err(&client->dev,
				"%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, data[0], data[1]);
			return -EIO;
		}
	};

	return 0;
}


/*
* Handle commands from user-space
*/
static int pcf8563_rtc_ioctl(struct i2c_client *client,  unsigned int cmd,
							 unsigned long arg)
{
	int ret = 0;
	int err = 0;
	pr_debug("%s(): cmd=%08x, arg=%08lx.\n", __func__, cmd, arg);
	/* important:  scrub old status before enabling IRQs */
	switch (cmd)
	{
	case RTC_AIE_OFF:	/* alarm off */
		{
			unsigned char data[2] = {0x01,0x00};

			err = i2c_master_send(client, data, sizeof(data));
			if (err != sizeof(data)) {
				dev_err(&client->dev,
					"%s: err=%d addr=%02x, data=%02x\n",
					__func__, err, data[0], data[1]);
				return -EIO;
			}
		}
		break;
	case RTC_AIE_ON:	/* alarm on */
		{
			unsigned char data[2] = {0x01,0x13};

			err = i2c_master_send(client, data, sizeof(data));
			if (err != sizeof(data)) {
				dev_err(&client->dev,
					"%s: err=%d addr=%02x, data=%02x\n",
					__func__, err, data[0], data[1]);
				return -EIO;
			}
		}
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
/*
* IRQ handler for the RTC
*/
static irqreturn_t pcf8563_rtc_interrupt(int irq, void *dev_id)
{
	struct i2c_client *client = to_i2c_client(dev_id);
	//int err = 0;

	struct pcf8563 *pcf8563=i2c_get_clientdata(client);
	//unsigned int rtsr;
	unsigned long events = 0;
	//unsigned char buf[1] = { PCF8563_REG_ST2};

	//struct i2c_msg msgs[] = {
	//{ client->addr, 0, 1, buf },	/* setup read ptr */
	//{ client->addr, I2C_M_RD, 1, buf },	/* read status + date */
	//};

	/* read registers */
	//if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
	//dev_err(&client->dev, "%s: read error\n", __func__);
	//return -EIO;
	//}

	//if (buf[0] & 0X0C)
	//{	/* this interrupt is shared!  Is it ours? */
	///{
	//unsigned char data[2] = {0x02,0x13};
	//data[1]=buf[0]&(~0X0C);
	//err = i2c_master_send(client, data, sizeof(data));
	//if (err != sizeof(data)) {
	//dev_err(&client->dev,
	//"%s: err=%d addr=%02x, data=%02x\n",
	//__func__, err, data[0], data[1]);
	//}
	//}
	disable_irq(client->irq);
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	events |= (RTC_AF | RTC_IRQF);
	rtc_update_irq(pcf8563->rtc, 1, events);
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	enable_irq(client->irq);
	return IRQ_HANDLED;
	//}
	//else
	//dev_info(&client->dev,"BUF=0x%x no alarm is active!\n",buf[0]);
}
#endif
static int pcf8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return pcf8563_get_datetime(to_i2c_client(dev), tm);
}

static int pcf8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return pcf8563_set_datetime(to_i2c_client(dev), tm);
}
#if defined(PCF8563_RTC_ALARM)
static int mxc_rtc_ioctl(struct device *dev, unsigned int cmd,unsigned long arg)
{

	return pcf8563_rtc_ioctl(to_i2c_client(dev), cmd,arg);
}

static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	return pcf8563_rtc_readalarm(to_i2c_client(dev), tm);
}

static int mxc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	return pcf8563_rtc_setalarm(to_i2c_client(dev), tm);
}
#endif
static const struct rtc_class_ops pcf8563_rtc_ops = {
	.read_time	= pcf8563_rtc_read_time,
	.set_time	= pcf8563_rtc_set_time,
#if defined(PCF8563_RTC_ALARM)	
	.ioctl		= mxc_rtc_ioctl,
	.read_alarm   = mxc_rtc_read_alarm,
	.set_alarm     = mxc_rtc_set_alarm,
#endif	
};

static int pcf8563_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
{
	struct pcf8563 *pcf8563;

	int err = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, " check_functionality error!\n");
		return -ENODEV;
	}

	pcf8563 = kzalloc(sizeof(struct pcf8563), GFP_KERNEL);
	if (!pcf8563)
	{
		dev_err(&client->dev, " alloc error!\n");
		return -ENOMEM;
	}

	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");

	i2c_set_clientdata(client, pcf8563);

	pcf8563->rtc = rtc_device_register(pcf8563_driver.driver.name,
		&client->dev, &pcf8563_rtc_ops, THIS_MODULE);

	if (IS_ERR(pcf8563->rtc)) {
		dev_err(&client->dev, " pcf8563 rtc error!\n");
		err = PTR_ERR(pcf8563->rtc);
		goto exit_kfree;
	}
#if defined(PCF8563_RTC_ALARM)
	if(!client->irq)
		printk("!!!ERROR rtc no irq !!\n");
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	set_irq_type(client->irq,IRQF_TRIGGER_RISING);
	err = request_irq(client->irq, pcf8563_rtc_interrupt,IRQF_DISABLED, "rtc-at8563",&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "at8563: request irq failed\n");
		goto exit_kfree;
	}	
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	set_irq_type(client->irq,IRQF_TRIGGER_RISING);
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	//enable_irq(client->irq);
	//disable_irq(client->irq);
	device_init_wakeup(&client->dev, 1);//add bylin
#endif
	return 0;

exit_kfree:
	kfree(pcf8563);

	return err;
}

static int pcf8563_remove(struct i2c_client *client)
{
	struct pcf8563 *pcf8563 = i2c_get_clientdata(client);

	if (pcf8563->rtc)
		rtc_device_unregister(pcf8563->rtc);

	kfree(pcf8563);

	return 0;
}
//static u32 at91_rtc_imr;
//
//static int at91_rtc_suspend(struct device *dev)
//{
//if (device_may_wakeup(dev))
//enable_irq_wake(AT91_ID_SYS);
//return 0;
//}
//
//static int at91_rtc_resume(struct device *dev)
//{
//if (device_may_wakeup(dev))
//disable_irq_wake(AT91_ID_SYS);
//return 0;
//} 

//static const struct i2c_device_id pcf8563_id[] = {
//	{ "pcf8563", 0 },
//	{ "rtc8564", 0 },
//	/
//	{ }
//};
static const struct i2c_device_id pcf8563_id[] = {
	{ RTC_PCF8563_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, pcf8563_id);

static struct i2c_driver pcf8563_driver = {
	.driver		= {
		.name	=RTC_PCF8563_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= pcf8563_probe,
	.remove		= pcf8563_remove,
	.id_table	= pcf8563_id,
	//.suspend =	at91_rtc_suspend,
	//.resume =	at91_rtc_resume,
};

static int __init pcf8563_init(void)
{
	//printk(" rtc pcf8563_init \n");
	return i2c_add_driver(&pcf8563_driver);
}

static void __exit pcf8563_exit(void)
{
	i2c_del_driver(&pcf8563_driver);
}

MODULE_AUTHOR("Alessandro Zummo <a.zummo@towertech.it>");
MODULE_DESCRIPTION("Philips PCF8563/Epson RTC8564 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(pcf8563_init);
module_exit(pcf8563_exit);
