/* drivers/input/touchscreen/gt813_827_828.c
 * 
 * 2010 - 2012 Goodix Technology.
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
 * Version:1.2
 * Author:scott@goodix.com
 * Release Date:2012/06/08
 * Revision record:
 *      V1.0:2012/05/01,create file.
 *      V1.0:2012/06/08,add slot report mode.
 */

#include <linux/irq.h>
#include "ata5701_touch.h"

struct i2c_client * i2c_ata5701_client = NULL; 

static struct workqueue_struct *ata5701_wq;





static void ata5701_work_func(struct work_struct *work)
{
   struct ata5701_data *ataData;
   int uData=0;
   uData = i2c_smbus_read_byte_data(i2c_ata5701_client, 0x00);
    printk("uData=%d\n",uData);
    // if(uData > 0 && uData <15)
    // {
      //  if(uData == 10){
      		//input_event(ataData->input_dev,EV_KEY,KEY_HOME,1);
		//input_event(ataData->input_dev,EV_KEY,KEY_HOME,0);
		//input_sync(ataData->input_dev);  
		//input_report_abs(ataData->input_dev, ABS_X, uData);
              // input_report_abs(ataData->input_dev, ABS_MT_POSITION_X, uData);
//	        input_report_abs(ataData->input_dev, ABS_X, uData);
		
        //	}
      //  }else{
     // 		input_event(ataData->input_dev,EV_KEY,KEY_MENU,1);
	//	input_event(ataData->input_dev,EV_KEY,KEY_MENU,0);
	//	input_sync(ataData->input_dev);  
      //  }

  //   }
}



static irqreturn_t ata5701_touch_irq_handler(int irq, void *dev_id)
{
    struct ata5701_data *ataData = dev_id;

    //ATA5701_DEBUG_FUNC();	
    queue_work(ata5701_wq, &ataData->work);

    return IRQ_HANDLED;
}
void ata5701_irq_disable(struct ata5701_data *ataData)
{	
    unsigned long irqflags;
	
    ATA5701_DEBUG_FUNC();
	
    spin_lock_irqsave(&ataData->irq_lock, irqflags);
    if (!ataData->irq_is_disable)
    {
        ataData->irq_is_disable = 1; 
        disable_irq_nosync(ataData->client->irq);
    }
    spin_unlock_irqrestore(&ataData->irq_lock, irqflags);
}

void ata5701_irq_enable(struct ata5701_data *ataData)
{	
    unsigned long irqflags;
	
    ATA5701_DEBUG_FUNC();
		
    spin_lock_irqsave(&ataData->irq_lock, irqflags);
    if (ataData->irq_is_disable) 
    {
        enable_irq(ataData->client->irq);
        ataData->irq_is_disable = 0;	
    }
    spin_unlock_irqrestore(&ataData->irq_lock, irqflags);
}

static s8 ata5701_request_irq(struct ata5701_data *ataData)
{
    s32 ret = -1;

   ATA5701_DEBUG_FUNC();	

    ret  = request_irq(ataData->client->irq, 
                       ata5701_touch_irq_handler,
                       IRQ_TYPE_EDGE_FALLING,
                       ataData->client->name,
                       ataData);
	printk("ret=%d\n",ret);

	if(ret ==0)
	{
        ata5701_irq_disable(ataData);
        ataData->use_irq = 1;	
	}else
	{
             ATA5701_ERROR("Request IRQ failed!ERRNO:%d.", ret);
            ATA5701_GPIO_AS_INPUT( ATA5701_INT_PORT);
            ATA5701_GPIO_FREE( ATA5701_INT_PORT);

		return -1;
	}
	
        return 0;
}
static s8 ata5701_request_io_port(struct ata5701_data *ataData)
{
    s32 ret = 0;


    ret = ATA5701_GPIO_REQUEST(ATA5701_INT_PORT, "GTP_INT_IRQ");
    if (ret < 0) 
    {
        ATA5701_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32)ATA5701_INT_PORT, ret);
        ret = -ENODEV;
    }
    else
    {
        ATA5701_GPIO_AS_INPUT(ATA5701_INT_PORT);	
        ataData->client->irq = ATA5701_INT_IRQ;
    }


    return ret;
}
static s8 ata5701_request_input_dev(struct ata5701_data *ataData)
{
    s8 ret = -1;
   s8 phys[32];

   ATA5701_DEBUG_FUNC();	

      ataData->input_dev = input_allocate_device();
    if (ataData->input_dev == NULL)
    {
        ATA5701_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

    ataData->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    ataData->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);

    ataData->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    
    input_set_abs_params(ataData->input_dev, ABS_X, 0, 14, 0, 0);
    input_set_abs_params(ataData->input_dev, ABS_Y, 0, 0, 0, 0);
    input_set_abs_params(ataData->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ataData->input_dev, ABS_MT_POSITION_X, 0, 0, 0, 0);
    input_set_abs_params(ataData->input_dev, ABS_MT_POSITION_Y, 0, 14, 0, 0);
    input_set_abs_params(ataData->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ataData->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ataData->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);	

   memset(phys, 0, sizeof(phys));

    sprintf(phys, "input/atatouch");
    ataData->input_dev->name = "ata5701 touch";
    ataData->input_dev->phys = phys;
    ataData->input_dev->id.bustype = BUS_I2C;
    ataData->input_dev->id.vendor = 0xDDAD;
    ataData->input_dev->id.product = 0xBFEF;
    ataData->input_dev->id.version = 20327;
	
    ret = input_register_device(ataData->input_dev);
    if (ret)
    {
        ATA5701_ERROR("Register %s input device failed", ataData->input_dev->name);
        return -ENODEV;
    }
    
    return 0;
}
static int ata5701_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct ata5701_data *ataData;
    u16 version_info;

    client->addr = 0x33;
    ATA5701_INFO("%s,I2C addr:%x\n",__func__, client->addr);
    
	i2c_ata5701_client = client;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        ATA5701_ERROR("I2C check functionality failed.\n");
        return -ENODEV;
    }
    ataData = kzalloc(sizeof(*ataData), GFP_KERNEL);
    if (ataData == NULL)
    {
        ATA5701_ERROR("Alloc GFP_KERNEL memory failed.\n");
        return -ENOMEM;
    }
    
    memset(ataData, 0, sizeof(*ataData));
    INIT_WORK(&ataData->work, ata5701_work_func);
    ataData->client = client;
    i2c_set_clientdata(client, ataData);
    spin_lock_init(&ataData->irq_lock);

     ret = ata5701_request_io_port(ataData);
    if (ret < 0)
    {
        ATA5701_ERROR("GTP request IO port failed.");
        kfree(ataData);
        return ret;
    }
	
    ret = ata5701_request_input_dev(ataData);
    if (ret < 0)
    {
        ATA5701_ERROR("GTP request input dev failed");
    }else
    {
        ATA5701_INFO("ata request input dev success.\n");
    } 


    ret = ata5701_request_irq(ataData); 
    if (ret < 0)
    {
        ATA5701_INFO("ata works in polling mode.\n");
    }
    else
    {
        ATA5701_INFO("ata works in interrupt mode.\n");
    }
	ata5701_irq_enable(ataData);

    return 0;
}


static int ata5701_ts_remove(struct i2c_client *client)
{
    struct ata5701_data *ataData = i2c_get_clientdata(client);
	

    i2c_set_clientdata(client, NULL);
    input_unregister_device(ataData->input_dev);
    kfree(ataData);

    return 0;
}


static const struct i2c_device_id ata5701_ts_id[] = {
    { ATA5701_I2C_NAME, 0 },
    { }
};

static struct i2c_driver ata5701_ts_driver = {
    .probe      = ata5701_ts_probe,
    .remove     = ata5701_ts_remove,
//#ifndef CONFIG_HAS_EARLYSUSPEND
//    .suspend    = ata5701_ts_suspend,
//    .resume     = ata5701_ts_resume,
//#endif
    .id_table   = ata5701_ts_id,
    .driver = {
        .name     = ATA5701_I2C_NAME,
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
static int __devinit ata5701_ts_init(void)
{
    s32 ret;
    ATA5701_DEBUG_FUNC();	

    ata5701_wq = create_singlethread_workqueue("ata5701_wq");
    if (!ata5701_wq)
    {
        ATA5701_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
	
    ret = i2c_add_driver(&ata5701_ts_driver);
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
static void __exit ata5701_ts_exit(void)
{
    ATA5701_DEBUG_FUNC();	

    i2c_del_driver(&ata5701_ts_driver);
    if (ata5701_wq)
    {
        destroy_workqueue(ata5701_wq);
    }
	
}

late_initcall(ata5701_ts_init);
module_exit(ata5701_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
