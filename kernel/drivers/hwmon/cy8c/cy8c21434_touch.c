/* 
 * 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 *  
 */

#include <linux/irq.h>
#include "cy8c21434_touch.h"
//#include <linux/pwm.h>

//struct i2c_client * i2c_cy8c_client = NULL; 

static struct workqueue_struct *cy8c_wq;

extern int iXrzSuspendMode;
//extern int mx50_rdp_get_brightness();
//extern struct pwm_bl_data *mypb;
//extern struct backlight_device *mybl;

extern int mx50_rdp_set_brightness(int newBrightness);


static void cy8c_work_func(struct work_struct *work)
{
   struct cy8c_data *ataData;
   int uData=0;
   static int uOldDate =0;
  // bool bInSlide = false;

   ataData = container_of(work, struct cy8c_data, work);

   uData = i2c_smbus_read_byte_data(ataData->client, 0x12);
   CY8C_DEBUG("%s,uData=%d, ",__func__, uData);
  //printk("uOldDate=%d\n",uOldDate);
   //uData=uData << 4;
   //CY8C_DEBUG("%s,uData2=%d, ",__func__, uData);

  /* if( (uOldDate = uData+1)
	 ||(uOldDate=uData-1)){
	 bInSlide  =true;
    }else{
  	bInSlide = false;
    }*/
  	
	 
     if((uData >= 0 && uData < 40)  
	 &&(uOldDate != uData ))
     {
                //printk("report the Data\n");
		uOldDate = uData ;
		// printk("%d,uOldDate=%d\n",__LINE__,uOldDate);
		//mx50_rdp_set_brightness(uData);
	    input_report_abs(ataData->input_dev, ABS_X, uData);
	    input_sync(ataData->input_dev);
     }
}

int cy8c_ic_reset(void)
{
      //static int icount =0;
      // icount++;
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);
      // if(icount == 1){
	CY8C_GPIO_OUTPUT(CY8C_RESET,1);
	msleep(100);
	CY8C_GPIO_OUTPUT(CY8C_RESET,0);
      // 	}else if(icount == 2)
      // {
      // 	  icount =0;
      // }

	  return 0;
}

static irqreturn_t cy8c_touch_irq_handler(int irq, void *dev_id)
{
    struct cy8c_data *ataData = dev_id;

   CY8C_DEBUG_FUNC();	
    //printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
    queue_work(cy8c_wq, &ataData->work);

    return IRQ_HANDLED;
}
void cy8c_irq_disable(struct cy8c_data *ataData)
{	
    unsigned long irqflags;
	
   CY8C_DEBUG_FUNC();
	
    spin_lock_irqsave(&ataData->irq_lock, irqflags);
    if (!ataData->irq_is_disable)
    {
        ataData->irq_is_disable = 1; 
        disable_irq_nosync(ataData->client->irq);
    }
    spin_unlock_irqrestore(&ataData->irq_lock, irqflags);
}

void cy8c_irq_enable(struct cy8c_data *ataData)
{	
    unsigned long irqflags;
	
   CY8C_DEBUG_FUNC();
		
    spin_lock_irqsave(&ataData->irq_lock, irqflags);
    if (ataData->irq_is_disable) 
    {
        enable_irq(ataData->client->irq);
        ataData->irq_is_disable = 0;	
    }
    spin_unlock_irqrestore(&ataData->irq_lock, irqflags);
}

static s8 cy8c_request_irq(struct cy8c_data *ataData)
{
    s32 ret = -1;

  CY8C_DEBUG_FUNC();	

	

    ret  = request_irq(ataData->client->irq, 
                       cy8c_touch_irq_handler,
                       IRQ_TYPE_EDGE_FALLING,
                       //IRQ_TYPE_LEVEL_LOW,
                       ataData->client->name,
                       ataData);
    
       CY8C_DEBUG("%s,ret=:%d, ",__func__, ret);
	if(ret ==0)
	{
             cy8c_irq_disable(ataData);
             ataData->use_irq = 1;	
	}else
	{
            CY8C_ERROR("Request IRQ failed!ERRNO:%d.", ret);
            CY8C_GPIO_AS_INPUT(CY8C_INT_PORT);
            CY8C_GPIO_FREE(CY8C_INT_PORT);

		return -1;
	}
	
        return 0;
}
static s8 cy8c_request_io_port(struct cy8c_data *ataData)
{
    s32 ret = 0;


    ret =CY8C_GPIO_REQUEST(CY8C_INT_PORT, "CY8C_INT_IRQ");
    if (ret < 0) 
    {
       CY8C_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32)CY8C_INT_PORT, ret);
        ret = -ENODEV;
    }
    else
    {
       CY8C_GPIO_AS_INPUT(CY8C_INT_PORT);	
        ataData->client->irq =CY8C_INT_IRQ;
    }
	
   ret = CY8C_GPIO_REQUEST(CY8C_RESET, "CY8C_RST");
    if (ret < 0) 
    {
        CY8C_ERROR("Failed to request GPIO:%d, ERRNO:%d",(s32)CY8C_RESET,ret);
        ret = -ENODEV;
    }else{
       ////set as low,
       CY8C_GPIO_OUTPUT(CY8C_RESET,0);
    }

//    wakeup_slider();
//    ataData->in_suspend = false;

    return ret;
}
static s8 cy8c_request_input_dev(struct cy8c_data *ataData)
{
    s8 ret = -1;
   s8 phys[32];
   int icount=0;

  CY8C_DEBUG_FUNC();	

      ataData->input_dev = input_allocate_device();
    if (ataData->input_dev == NULL)
    {
       CY8C_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }
    ataData->input_dev->evbit[0] = BIT_MASK(EV_SYN) | \
		BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    ataData->input_dev->absbit[0] = BIT(ABS_X) |\
		BIT(ABS_Y) | BIT(ABS_PRESSURE);

    ataData->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    
    input_set_abs_params(ataData->input_dev, ABS_X, 0, 14, 0, 0);
   // input_set_abs_params(ataData->input_dev, ABS_Y, 0, 14, 0, 0);
   //input_set_abs_params(ataData->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
   // input_set_abs_params(ataData->input_dev, ABS_MT_POSITION_X, 0, 14, 0, 0);
   // input_set_abs_params(ataData->input_dev, ABS_MT_POSITION_Y, 0, 14, 0, 0);
   // input_set_abs_params(ataData->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
   // input_set_abs_params(ataData->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
   // input_set_abs_params(ataData->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);	

   memset(phys, 0, sizeof(phys));

    sprintf(phys, "input/atatouch");
    ataData->input_dev->name = "cy8c touch";
    ataData->input_dev->phys = phys;
    ataData->input_dev->id.bustype = BUS_I2C;
    ataData->input_dev->id.vendor = 0xDDAD;
    ataData->input_dev->id.product = 0xBFEF;
    ataData->input_dev->id.version = 20327;
    ret = input_register_device(ataData->input_dev);
    if (ret)
    {
       CY8C_ERROR("Register %s input device failed", ataData->input_dev->name);
        return -ENODEV;
    }
    
    return 0;
}
static int cy8c_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct cy8c_data *ataData;
    u16 version_info;

    client->addr = 0x24;
   CY8C_DEBUG("%s,I2C addr:%x",__func__, client->addr);
    
	//i2c_cy8c_client = client;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
       CY8C_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }
    ataData = kzalloc(sizeof(*ataData), GFP_KERNEL);
    if (ataData == NULL)
    {
       CY8C_ERROR("Alloc GFP_KERNEL memory failed");
        return -ENOMEM;
    }
    
    memset(ataData, 0, sizeof(*ataData));
    INIT_WORK(&ataData->work, cy8c_work_func);
    ataData->client = client;
    //ataData->in_suspend = true;
    i2c_set_clientdata(client, ataData);
    spin_lock_init(&ataData->irq_lock);

     ret = cy8c_request_io_port(ataData);
    if (ret < 0)
    {
       CY8C_ERROR("GTP request IO port failed.");
        kfree(ataData);
        return ret;
    }
	
   ret = cy8c_request_input_dev(ataData);
    if (ret < 0)
    {
       CY8C_ERROR("GTP request input dev failed");
    }


    ret = cy8c_request_irq(ataData); 
    if (ret < 0)
    {
       CY8C_INFO("ata works in polling mode.");
    }
    else
    {
       CY8C_INFO("ata works in interrupt mode.");
    }
   cy8c_irq_enable(ataData);

   //i2c_smbus_write_byte_data(client,0x00,0x10);

    return 0;
}


static int cy8c_ts_remove(struct i2c_client *client)
{
    struct cy8c_data *ataData = i2c_get_clientdata(client);
	

    i2c_set_clientdata(client, NULL);
    input_unregister_device(ataData->input_dev);
    kfree(ataData);

    return 0;
}

#if defined(CONFIG_PM)
static int cy8c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cy8c_data *data = i2c_get_clientdata(client);
	
	//printk("%s %s %d pm = %x\n",__FILE__,__func__,__LINE__, mesg);


        //// normal suspend mode. we close the power,so
        /// we don't need do anything.
        if(iXrzSuspendMode == 1){
		if (data->use_irq)
			enable_irq_wake(data->client->irq);
        }///else{
	//i2c_smbus_write_byte_data(client,0x00,0x10);
      //  }
	return 0;

}
static int cy8c_ts_resume(struct i2c_client *client)
{
	struct cy8c_data *data = i2c_get_clientdata(client);
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	////i2c_smbus_write_byte_data(client,0x00,0x20);
	//if (data->in_suspend)
	//	wakeup_slider();
	//return 0;

	if(iXrzSuspendMode ==0){
	    cy8c_ic_reset();
        } else if(iXrzSuspendMode == 1){
		if (data->use_irq)
		    disable_irq_wake(data->client->irq);
         }
	
	return 0;
	
}
#endif

static const struct i2c_device_id cy8c_ts_id[] = {
    {CY8C_I2C_NAME, 0 },
    { }
};

static struct i2c_driver cy8c_ts_driver = {
    .probe      = cy8c_ts_probe,
    .remove     = cy8c_ts_remove,
#ifdef CONFIG_PM
    .suspend    = cy8c_ts_suspend,
    .resume     = cy8c_ts_resume,
#endif
    .id_table   = cy8c_ts_id,
    .driver = {
        .name     =CY8C_I2C_NAME,
        .owner    = THIS_MODULE,
    },
};

/*******************************************************	
Function:
	Driver Install function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit cy8c_ts_init(void)
{
    s32 ret;
   CY8C_DEBUG_FUNC();	

    cy8c_wq = create_singlethread_workqueue("cy8c_wq");
    if (!cy8c_wq)
    {
       CY8C_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
	
    ret = i2c_add_driver(&cy8c_ts_driver);
    return ret; 

}

/*******************************************************	
Function:
	Driver uninstall function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit cy8c_ts_exit(void)
{
   CY8C_DEBUG_FUNC();	

    i2c_del_driver(&cy8c_ts_driver);
    if (cy8c_wq)
    {
        destroy_workqueue(cy8c_wq);
    }
	
}

late_initcall(cy8c_ts_init);
module_exit(cy8c_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
