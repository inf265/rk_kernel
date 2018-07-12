/*
 * leds-sn3106b.c -  LED Driver
 *
 * Copyright (C) 2011 Rockchips
 * deng dalong <ddl@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet: http://www.rohm.com/products/databook/driver/pdf/sn3106bgu-e.pdf
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/leds-sn3106b.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>

#include "tp_suspend.h"

static int debug;
module_param(debug, int, S_IRUGO|S_IWUSR);

#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	printk(KERN_WARNING"leds-sn3106b: " fmt , ## arg); } while (0)

#define LEDS_sn3106b_TR(format, ...) printk(KERN_ERR format, ## __VA_ARGS__)
#define LEDS_sn3106b_DG(format, ...) dprintk(0, format, ## __VA_ARGS__)


static unsigned char g_LedCTRL=0;
#define	SN3106B_REG_CHIP_SSD  0x00
#define	SN3106B_REG_CTRL_D1 		0x07
#define	SN3106B_REG_CTRL_D2 		0x08
#define	SN3106B_REG_CTRL_D3 		0x09
#define	SN3106B_REG_CTRL_D4 		0x0a
#define	SN3106B_REG_CTRL_D5		0x0b
#define	SN3106B_REG_CTRL_D6		0x0c
#define	SN3106B_REG_CTRL  		0x14
#define	SN3106B_REG_DATA_UPDATE		0x16
#define	SN3106B_REG_SW_RST 		0x17


int nowoffBreath;
struct class *leds_sn3106b;



/* Delay limits for hardware accelerated blinking (in ms). */
#define LM3533_LED_DELAY_ON_MAX \
	((LM3533_LED_DELAY2_TMAX + LM3533_LED_DELAY2_TSTEP / 2) / 1000)
#define LM3533_LED_DELAY_OFF_MAX \
	((LM3533_LED_DELAY3_TMAX + LM3533_LED_DELAY3_TSTEP / 2) / 1000)


const unsigned char bData_All_on[18]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
const unsigned char Gamma_T32[32]={0,1,2,4,6,10,13,18,22,28,33,39,46,53,61,69,78,86,96,106,116,126,138,149,161,173,186,199,212,226,240,255};
const unsigned char Gamma_T64[64]=
{
0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
0x08,0x09,0x0b,0x0d,0x0f,0x11,0x13,0x16,
0x1a,0x1c,0x1d,0x1f,0x22,0x25,0x28,0x2e,
0x34,0x38,0x3c,0x40,0x44,0x48,0x4b,0x4f,
0x55,0x5a,0x5f,0x64,0x69,0x6d,0x72,0x77,
0x7d,0x80,0x88,0x8d,0x94,0x9a,0xa0,0xa7,
0xac,0xb0,0xb9,0xbf,0xc6,0xcb,0xcf,0xd6,
0xe1,0xe9,0xed,0xf1,0xf6,0xfa,0xfe,0xff
};


static struct regmap *regmap_3106b = NULL;

#define ldev_to_led(c)       container_of(c, struct sn3106b_led, cdev_led)

struct led_classdev	cdev_led1;


struct sn3106b_led {
	struct sn3106b_led_platform_data	*pdata;
	struct i2c_client		*client;
	struct rw_semaphore		rwsem;
	struct delayed_work		work;
	struct mutex lock;
	 int delay_off;
	 int delay_on;
	 struct  tp_device  tp;
		
	/*
	 * Making led_classdev as array is not recommended, because array
	 * members prevent using 'container_of' macro. So repetitive works
	 * are needed.
	 */
	struct led_classdev		cdev_led;

};


int sn3106b_i2c_read_reg(int reg, int *val)
{
    int ret;
    
    ret = regmap_read(regmap_3106b, reg, val); 
    return ret;
}

int sn3106b_i2c_write_reg(int reg, int val)
{
    int ret;
    
    ret = regmap_write(regmap_3106b, reg, val);  
    return ret;
}








void Led_SetBrightness(unsigned char Ch, unsigned char brightness)
{

	if  ( (Ch < 1) || (Ch>6) ) return;

	sn3106b_i2c_write_reg(SN3106B_REG_CTRL_D1+Ch-1,brightness);
}

void Led_DataUpdate()
{															
 	sn3106b_i2c_write_reg(SN3106B_REG_DATA_UPDATE,0x00);
}


/*********************************************************/
//Chip on, open  6 channels
/********************************************************/
void Led_ChipOn()
{
	g_LedCTRL=0x3f;
	
	gpio_direction_output(116,1);//SDB	 
	msleep(1);		
	sn3106b_i2c_write_reg(SN3106B_REG_CHIP_SSD,1); //开总开关
	sn3106b_i2c_write_reg(SN3106B_REG_CTRL,g_LedCTRL);//开1-6channel

}
/*********************************************************/
//Chip off by Software and hardware 
/********************************************************/
void Led_ChipOff()
{
	g_LedCTRL=0x00;

	sn3106b_i2c_write_reg(SN3106B_REG_CHIP_SSD,0);

	sn3106b_i2c_write_reg(SN3106B_REG_CTRL,g_LedCTRL);//关1-6channel

	gpio_direction_output(116,0);//SDB		 
	
}


/*************************************************/
//选择要呼吸的Channel (1-3), 
//Interval 呼吸亮，灭过程的步长间隔，单位是ms
//Hold 最亮时候的保持时间,单位是ms
/*************************************************/
void Led_Breath(struct led_classdev *cdev,unsigned char Ch,unsigned int Interval,unsigned int Hold)
{
	signed char step =0;
	struct sn3106b_led *led = ldev_to_led(cdev);

	for (step = 0; step < 32;step++)
	{
	if(nowoffBreath==0)
		{
		printk("nowoffBreath1\n");
		Led_SetBrightness(6,255);
		Led_DataUpdate();
		 break;
		}
//	Led_SetBrightness(Ch,Gamma_T32[step]);
	Led_SetBrightness(6,Gamma_T32[step]);
	Led_DataUpdate();
	msleep(30);
	}
	
	msleep(led->delay_on);
	
	for (step = 31; step >=0;step--)
	{
	if(nowoffBreath==0)
		{
		printk("nowoffBreath2\n");
		Led_SetBrightness(6,255);
		Led_DataUpdate();
		 break;
		}	
//	Led_SetBrightness(Ch,Gamma_T32[step]);
	Led_SetBrightness(6,Gamma_T32[step]);
	Led_DataUpdate();
	msleep(30);
	}	
		}



static void sn3106b_led_work(struct work_struct *work)
{
	struct sn3106b_led *led = container_of(work, struct sn3106b_led, work);
	
	Led_Breath(led,6,led->delay_on,led->delay_off);
	

	
		if(led->cdev_led.activated)
		{
		schedule_delayed_work(&led->work,msecs_to_jiffies(led->delay_off));
		}
	
}



static int sn3106b_led_blink_set(struct led_classdev *cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	struct sn3106b_led *led = ldev_to_led(cdev);
	int ret;

	printk("%s - on = %lu, off = %lu\n", __func__,
							*delay_on, *delay_off);


	if (*delay_on == 0 && *delay_off == 0) {
		*delay_on = 500;
		*delay_off = 500;
	}
	led->delay_off = *delay_off;
	led->delay_on = *delay_on;
	schedule_delayed_work(&led->work,msecs_to_jiffies(5));
	
	return 0;
}




static const struct regmap_config sn3106b_regmap_config = {
	.val_bits = 8,
	.reg_bits = 8,
	.max_register = 0xFF,
};

static void sn3106b_strobe_brightness_set(struct led_classdev *cdev,
					 enum led_brightness brightness)
{
	struct sn3106b_led *led = ldev_to_led(cdev);
	Led_SetBrightness(6,brightness);
	
	if(brightness==0)
	nowoffBreath = 0;
	else
	nowoffBreath =1;
	
	Led_DataUpdate();
}

static int hyn_ts_early_suspend(struct tp_device *tp_d)
{
	printk("sn3106b Enter %s\n", __func__);
	Led_SetBrightness(6,0);
	Led_SetBrightness(4,0);
	Led_SetBrightness(5,0);
	Led_DataUpdate();
	return 0;
}

static int hyn_ts_late_resume(struct tp_device *tp_d)
{
	printk("sn3106b Enter %s\n", __func__);
	Led_SetBrightness(6,255);
	Led_SetBrightness(4,255);
	Led_SetBrightness(5,255);
	Led_DataUpdate();
	return 0;
}
static void sn3106_control(int l,int r)
{
	printk("sn3106_control----l===%d,r===%d\n",l,r);

		if(l==00)
		{
		Led_SetBrightness(4,0);
		Led_SetBrightness(5,0);
		Led_DataUpdate();

		}
		if(l==11)
		{
		Led_SetBrightness(4,255);
		Led_SetBrightness(5,255);
		Led_DataUpdate();

		}
		if(l==01)
		{
		Led_SetBrightness(4,0);
		Led_SetBrightness(5,255);
		Led_DataUpdate();

		}
		if(l==10)
		{
		Led_SetBrightness(4,255);
		Led_SetBrightness(5,0);
		Led_DataUpdate();

		}

}


static ssize_t sn3106b_status_read(struct class *cls, struct class_attribute *attr, char *_buf)
	{
	
		return sprintf(_buf, "%d\n", _buf);
		
	}

static ssize_t sn3106b_status_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
	{
     unsigned int    val;

    if(!(sscanf(_buf, "%u\n", &val)))   
		return -EINVAL;

		printk("sn3106b :%d\n",val);

		int l;
		int r;	
		int n = val;
		int one = n / 1 % 10;
		int two = n / 10 % 10;
		int three = n / 100 % 10;
		int four = n / 1000 % 10;	
		l=two*10+one;
		r=four*10+three;

		sn3106_control(l,r);

		return _count; 
	}

static CLASS_ATTR(modem_status, 0777, sn3106b_status_read, sn3106b_status_write);


static int sn3106b_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sn3106b_led *led;
	struct sn3106b_led_platform_data *pdata;
	int ret, i;
	int err;
	g_LedCTRL=0x3f;
	nowoffBreath =1;
	printk("sn3106b_probe\n");
	LEDS_sn3106b_TR("%s enter: client->addr:0x%x\n",__FUNCTION__,client->addr);

	led = kzalloc(sizeof(struct sn3106b_led), GFP_KERNEL);
	if (!led) {
		LEDS_sn3106b_TR("failed to allocate driver data\n");
		return -ENOMEM;
	}

	led->client = client;
	pdata = led->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, led);


	regmap_3106b = devm_regmap_init_i2c(client, &sn3106b_regmap_config);
	if (IS_ERR(regmap_3106b)) {
		err = PTR_ERR(regmap_3106b);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", err);
	//	failed_free;
	}


	printk("sn3106b i2c write.....\n");
	Led_ChipOn();
	Led_SetBrightness(4,255);
	Led_SetBrightness(5,255);
	Led_SetBrightness(6,255);
	Led_DataUpdate();
	
	init_rwsem(&led->rwsem);


	/* flash */
	//INIT_WORK(&led->work, sn3106b_deferred_strobe_brightness_set);
	led->cdev_led.name = "sn3106b";
	led->cdev_led.max_brightness = 255;
	led->cdev_led.brightness_set =sn3106b_strobe_brightness_set;
	led->cdev_led.default_trigger = "leds";
	led->cdev_led.blink_set = sn3106b_led_blink_set;
	err = led_classdev_register((struct device *)
				    &client->dev, &led->cdev_led);
	
	led->tp.tp_suspend = hyn_ts_early_suspend;
	led->tp.tp_resume = hyn_ts_late_resume;
    tp_register_fb(&led->tp);
	
	//ret = sn3106b_register_led_classdev(led);

	INIT_DELAYED_WORK(&led->work, sn3106b_led_work);

	return 0;


failed_free:
	i2c_set_clientdata(client, NULL);
	kfree(led);

	return ret;
}

static int __exit sn3106b_remove(struct i2c_client *client)
{
	struct sn3106b_led *led = i2c_get_clientdata(client);
	led_classdev_unregister(&led->cdev_led);
	return 0;
}

static int sn3106b_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct sn3106b_led *led = i2c_get_clientdata(client);
	Led_ChipOff();
	printk("sn3106b_suspend\n");
	return 0;
}

static int sn3106b_resume(struct i2c_client *client)
{
	struct sn3106b_led *led = i2c_get_clientdata(client);
	Led_ChipOn();
	flush_work(&led->work);
	printk("sn3106b_resume\n");
	return 0;
}

static const struct i2c_device_id sn3106b_id[] = {
	{ "sn3106b", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn3106b_id);


static struct of_device_id sn3106b_of_match[] = 
{	
{ 
	.compatible = "sn3106b" },
		{ },
		};
MODULE_DEVICE_TABLE(of, sn3106b_of_match);








static struct i2c_driver sn3106b_i2c_driver = {
	.driver	= {
		.name	= "sn3106b",
		.owner   = THIS_MODULE,		
		.of_match_table = of_match_ptr(sn3106b_of_match),	
	},
	.probe		= sn3106b_probe,
	.remove		= __exit_p(sn3106b_remove),
	.suspend	= sn3106b_suspend,
	.resume		= sn3106b_resume,
	.id_table	= sn3106b_id,
};

static int __init sn3106b_init(void)
{
	printk("sn3106b_init\n");
		leds_sn3106b = class_create(THIS_MODULE, "sn3016b_l_r");
	   class_create_file(leds_sn3106b, &class_attr_modem_status);
	
	return i2c_add_driver(&sn3106b_i2c_driver);
}
module_init(sn3106b_init);

static void __exit sn3106b_exit(void)
{
	i2c_del_driver(&sn3106b_i2c_driver);
}
module_exit(sn3106b_exit);

MODULE_AUTHOR("deng dalong <ddl@rock-chips.com>");
MODULE_DESCRIPTION("sn3106b LED driver");
MODULE_LICENSE("GPL v2");

