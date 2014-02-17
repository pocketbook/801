#include <common.h>
#if defined(CONFIG_IMX_CSPI)

#include "mc13892_check_battery.h"

extern int mx50_rdp_charger_gpio_control(int iopen);
//extern struct spi_slave *myslave;
//#if defined CONFIG_SPLASH_SCREEN/* && defined CONFIG_VIDEO_MX5*/
//extern int setup_splash_img();
//#endif

#if defined(CONFIG_MX50_RD3/*CONFIG_MX50_RDP */)
void check_battery(void){}
#else
static unsigned int bat_volt=0;
static int get_charging_vol(struct spi_slave *slave)
{
	int rtn=0;
	int adcValue=0, i=0, adcValue2=0;
	unsigned int val;
	int adcChargingCurValue=0;
	int adcChargingTotalValue =0;

	val = pmic_reg(slave, 43, 0, 0);
	val |= 0x4;
	pmic_reg(slave, 43, val, 1);	

	for(i=0;i<10;i++)
	{
		val = 0x1 | (0x43<<8)| (0x71<<16);
		pmic_reg(slave, 44, val, 1);	
		udelay(1000); 

		val = pmic_reg(slave, 45, 0, 0);/*Register 45, ADC 2 */
		adcValue2 = (val>>14)&0x1ff; /*read charge voltage value */

		adcChargingCurValue = (adcValue2*235)/20;
		//printf(" i=%d, adcChargingCurValue =%d mV \n",i,adcChargingCurValue);

		adcChargingTotalValue = adcChargingTotalValue + adcChargingCurValue;

		adcChargingCurValue = 0;
	}
	printf(" (adcChargingTotalValue/10) =%d mV \n",(adcChargingTotalValue/10));

	return (adcChargingTotalValue/10);

}
int  get_charger_connect_state(struct spi_slave *slave)
{
	int adcChargingCurValue=0;
	int  bChargerInsertState = 0;/*0:no insert Charger; 1: insert Charger */

	adcChargingCurValue = get_charging_vol(slave);

	printf("%s %s %d adcChargingCurValue =%d \n",__FILE__,__func__,__LINE__ ,adcChargingCurValue);

	if(adcChargingCurValue > 4000)
		bChargerInsertState = 1;
	else 
		bChargerInsertState =0;	


	return bChargerInsertState;
}
int get_battery_voltage(void)
{
	 if(bat_volt >0)
	 	return bat_volt*10;
}
void check_battery(void)
{
	struct spi_slave *slave;
	unsigned int val,volt_bat;
	unsigned int adc1,adc1_cur;
	int  chg_curr, ii, iDelay;
	int timeout;
	int bChargerInsertState = -1;
	//static int iDisplaylogo = 0;
	int retry = 0;

	printf("Check battery\n");

	/* Enable VGEN1 to enable ethernet */
	slave = spi_pmic_probe();

	// reset ADC
	val = pmic_reg(slave, MC13892_REG_ADC0, 0, 0);
	pmic_reg(slave, MC13892_REG_ADC0, (val|MC13892_ADC0_ADRESET), 1);
	udelay(100);
	val &= (~MC13892_ADC0_ADRESET);
	pmic_reg(slave, MC13892_REG_ADC0, val, 1);
	udelay(1000);

	// enable ADC calibration once
	adc1 = MC13892_ADC1_ADEN | MC13892_ADC1_ADCCAL 
	            | MC13892_ADC1_ADTRIGIGN | MC13892_ADC1_ASC
	            | MC13892_ADC1_DEFAULT_ATO;
	
	for(ii=0; ii<3; ii++)
	{
		//arbitrarily set ADA2 to channel 3 
		adc1 |= (3<< MC13892_ADC1_ADA2_SHITT);
		pmic_reg(slave, MC13892_REG_ADC1, adc1, 1);

		/* Wait for conversation complete */
		for (iDelay=0; iDelay<10000; iDelay++) {
			adc1_cur = pmic_reg(slave, MC13892_REG_ADC1, 0, 0);
			if(ii==0)	/* calibration loop */
			{
				if ( !(adc1_cur & (MC13892_ADC1_ADCCAL|MC13892_ADC1_ASC)) )
					break;
			}
			else /* ADC conversion loop */
			{
				if (!(adc1_cur & MC13892_ADC1_ASC))
					break;
			}
			udelay(100);
		}

		// no ADCCAL here
		adc1 = MC13892_ADC1_ADEN | MC13892_ADC1_ADTRIGIGN 
		            | MC13892_ADC1_ASC | MC13892_ADC1_DEFAULT_ATO;

		//read ADC2
		val = pmic_reg(slave, MC13892_REG_ADC2, 0, 0);
	}
	// always take the 2nd ADC read and ignore the 1 ADC read after calibration

	//get Battery voltage from ADD1
	volt_bat = val >> MC13892_ADC2_ADD1_SHIFT;
	volt_bat &= MC13892_ADC2_ADD1_MASK;

	volt_bat = (volt_bat+1)*480/1024;
	bat_volt = volt_bat;
	printf("MC13892 volt: %d\n",volt_bat);


	if(volt_bat < VBATT_SAFE_BOOT_VOLT)
	{
		val = pmic_reg(slave, MC13892_REG_ISENSE0, 0, 0);
		//if( IsChargerPresence(val) )
		bChargerInsertState = get_charger_connect_state(slave);
		printf("bChargerInsertState : %d \n", bChargerInsertState);
		if(1==bChargerInsertState)
		{
			// set power limit = 1200mW
			val = pmic_reg(slave, MC13892_REG_CHARGE0, 0, 0);
			val |= (3<< MC13892_CHARGE0_PLIM_SHIFT);
			pmic_reg(slave,  MC13892_REG_CHARGE0, val, 1);

			//set charge current 640ma
			puts("change charge current to 560ma ...\n");

			// set CHARGE0_CHGAUTOVIB = 1
			val |= MC13892_CHARGE0_CHGAUTOVIB;

			// set CHARGE0_ICHRG = 640mA
			val &= ~MC13892_CHARGE0_ICHRG_MASK;

			///400ma  0100  4<<3;
			///560ma   0110  6<<3
			///640ma  0111  7<<3;
		        ///720ma  1000  8<<3;
			val |= (6<<MC13892_CHARGE0_ICHRG_SHIFT);

			pmic_reg(slave,  MC13892_REG_CHARGE0, val, 1);

			udelay(5000000);
read_current:
			// READ CHARGE CURRENT

			// enable charge current reading
			val = pmic_reg(slave, MC13892_REG_ADC0, 0, 0);
			val |= MC13892_ADC0_CHRGICON_MASK;
			pmic_reg(slave, MC13892_REG_ADC0, val, 1);

			//set ADA1 to channel 1 && 
			//ADA2 to channel 4 for charge current
			adc1 = MC13892_ADC1_ADEN | MC13892_ADC1_ADTRIGIGN 
			          | MC13892_ADC1_ASC | MC13892_ADC1_DEFAULT_ATO;
			adc1 |= (4<< MC13892_ADC1_ADA2_SHITT);
			pmic_reg(slave, MC13892_REG_ADC1, adc1, 1);

			/* Wait for conversation complete */
			for (iDelay=0; iDelay<10000;iDelay++) {
				adc1_cur = pmic_reg(slave, MC13892_REG_ADC1, 0, 0);
				if (!(adc1_cur & MC13892_ADC1_ASC))
					break;

				udelay(100);
			}

			//read ADC2
			val = pmic_reg(slave, MC13892_REG_ADC2, 0, 0);

			// get charging current from ADD2
			chg_curr = (val >> MC13892_ADC2_ADD2_SHIFT) & MC13892_ADC2_ADD2_MASK;
			if((chg_curr & 0x200))
			{
				// negative value
				chg_curr = 0 - ((~chg_curr + 1) & 0x1FF);
			}
		       chg_curr = (chg_curr * 5865)/1000;

			printf("charger current:%d mA\n", chg_curr);

			// disable charge current measurement
			val = pmic_reg(slave, MC13892_REG_ADC0, 0, 0);
			val &= (~MC13892_ADC0_CHRGICON_MASK);
			pmic_reg(slave, MC13892_REG_ADC0, val, 1);

			
			if(/*(volt_bat < VBATT_TRICKLE_POWER_VOLT) && (*/ chg_curr < VBATT_CURRENT_MA_OK )
			{
				if(retry < ADC_RETRIES) {
					printf("retry=%d\n",retry);
					retry++;
					udelay(1000);
					goto read_current;
				}
				//charger current too low, shut down
				puts("Low charge current, MC13982 power off for tickle charge\n");
				udelay(1000);
				val = pmic_reg(slave, MC13892_REG_POWER_CTL0, 0, 0);
				val |= 0x8;
				pmic_reg(slave,  MC13892_REG_POWER_CTL0, val, 1);		
			}
			else  if(volt_bat < VBATT_SAFE_BOOT_VOLT)
			{
				timeout = 0;
				//init the gpio control and set to high
				mx50_rdp_charger_gpio_control(1);
				do{
				// 10 sec delay loop
				for(ii=0; ii<1000; ii++) {
					val = pmic_reg(slave, MC13892_REG_ISENSE0, 0, 0);
					if( !IsChargerPresence(val) ) {
							printf("Charger is removed during battery charging\n");
						val = pmic_reg(slave, MC13892_REG_POWER_CTL0, 0, 0);
						val |= 0x8;
						pmic_reg(slave,  MC13892_REG_POWER_CTL0, val, 1);
					}
					udelay(10000);
				}

				// read battery voltage & charging current

				// enable charge current reading
				val = pmic_reg(slave, MC13892_REG_ADC0, 0, 0);
				val |= MC13892_ADC0_CHRGICON_MASK;
				pmic_reg(slave, MC13892_REG_ADC0, val, 1);

				adc1 = MC13892_ADC1_ADEN | MC13892_ADC1_ADTRIGIGN 
				            | MC13892_ADC1_ASC | MC13892_ADC1_DEFAULT_ATO;

				//set ADA2 to channel 4
				adc1 |= (4<< MC13892_ADC1_ADA2_SHITT);
				pmic_reg(slave, MC13892_REG_ADC1, adc1, 1);

				/* Wait for conversation complete */
				for (iDelay=0; iDelay<10000; iDelay++) {
					adc1_cur = pmic_reg(slave, MC13892_REG_ADC1, 0, 0);
					if (!(adc1_cur & MC13892_ADC1_ASC))	
						break;
					udelay(100);
				}

				//read ADC2
				val = pmic_reg(slave, MC13892_REG_ADC2, 0, 0);

				//get Battery voltage from ADD1
				volt_bat = val >> MC13892_ADC2_ADD1_SHIFT;
				volt_bat &= MC13892_ADC2_ADD1_MASK;

				volt_bat = (volt_bat+1)*480/1024;

				// get charging current from ADD2
				chg_curr = (val >> MC13892_ADC2_ADD2_SHIFT) & MC13892_ADC2_ADD2_MASK;
				if((chg_curr & 0x200))
				{
					// negative value
					chg_curr = 0 - ((~chg_curr + 1) & 0x1FF);
				}
				chg_curr = (chg_curr * 5865)/1000;

					printf("battery voltage:%d mV, charger current:%d mA timeout=%d\n", volt_bat*10, chg_curr,timeout);

				// disable charge current measurement
				val = pmic_reg(slave, MC13892_REG_ADC0, 0, 0);
				val &= (~MC13892_ADC0_CHRGICON_MASK);
				pmic_reg(slave, MC13892_REG_ADC0, val, 1);

				timeout++;
				
				 /////we need to display charging  logo 
				 //// only executed once
				// if(0 == iDisplaylogo){
				//     setup_splash_img();
				//      udelay(10000);
				//      iDisplaylogo =1;
				// }
				 
				 
				 
				//printf("timeout=%d,\n",timeout);
				}while(volt_bat < VBATT_SAFE_BOOT_VOLT && timeout< TIMEOUT_NUM);
				///after do{}while(),we need close the charger gpio
				mx50_rdp_charger_gpio_control(0);
			}			
		}
		else
		{
			//no charger, shut down
			puts("battery is lower than 3.5V and no charger, MC13982 power off\n");
			udelay(1000);
			val = pmic_reg(slave, MC13892_REG_POWER_CTL0, 0, 0);
			val |= 0x8;
			pmic_reg(slave,  MC13892_REG_POWER_CTL0, val, 1);

			//spi_pmic_free(slave);

		}
	}

        ////wen add. low battary power off
        ///
	/*val = pmic_reg(slave, MC13892_REG_ISENSE0, 0, 0);
	if( (!( IsChargerPresence(val) ) )&&  (volt_bat < 360))
	{
		puts("battery is lower than 3.5V, MC13982 power off\n");
		udelay(1000);
		val = pmic_reg(slave, MC13892_REG_POWER_CTL0, 0, 0);
		val |= 0x8;
		pmic_reg(slave,  MC13892_REG_POWER_CTL0, val, 1);
  	}*/

      //iDisplaylogo =0;
	spi_pmic_free(slave);

}
#endif
#endif
