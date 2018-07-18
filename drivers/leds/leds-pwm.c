/*
 * linux/drivers/leds-pwm.c
 *
 * simple PWM based LED control
 *
 * Copyright 2009 Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * based on leds-gpio.c by Raphael Assenat <raph@8d.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/fb.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/leds-sn3106b.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

 #include "tp_suspend.h"
 
 // 89 red 27 blue 28 green
int red = 89;
int blue = 27;
int green = 28;
int laohutou =122;
struct class *leds_lbc_pwm;


struct lbc_pwm_led {
	struct delayed_work		work;
	struct mutex lock;
	 int delay_off;
	 int delay_on;
	 struct  tp_device  tp;

};
int one = 0;
int two =0;
int three = 0;
int four =0;
int Fives = 0;
int six =0;

int suspend1 =0;
int suspend2 =0;
int suspend3 =0;


static void lbc_pwm_control(struct device *dev, int val)
{
	struct platform_device *pdev = to_platform_device(dev); 
	struct lbc_pwm_led *led = platform_get_drvdata(pdev);
	

	int g;
	int b;
	int n = val;
	 one = n / 1 % 10;//flash
	 two = n / 10 % 10;//Only bright
	 three = n / 100 % 10;//flash
	 four = n / 1000 % 10;//Only bright	
	 Fives = n / 10000 % 10; //flash
	 six = n / 100000 % 10;//Only bright	
	//r=two*10+one;
	//g=four*10+three;
	//b=six*10+Fives;

	printk("lbc_pwm_control----1===%d,2===%d,3===%d,4===%d,5===%d,6===%d\n",one,two,three,four,Fives,six);


		#if 1
			if(two==1)
			gpio_set_value(red,1);
			else
			gpio_set_value(red,0);

			if(four==1)
			gpio_set_value(blue,1);
			else
			gpio_set_value(blue,0);

			if(six==1)
			gpio_set_value(green,1);
			else
			gpio_set_value(green,0);
			#endif

			if(one==1||three==1||Fives==1)
				schedule_delayed_work(&led->work,msecs_to_jiffies(5));
				

}
static void led_pwm_led_work(struct work_struct *work)
{
	struct lbc_pwm_led *led = container_of(work, struct lbc_pwm_led, work);

	
	printk("led_pwm_led_work----1===%d,2===%d,3===%d,4===%d,5===%d,6===%d\n",one,two,three,four,Fives,six);

			 if(one==1&&two==1)
			gpio_set_value(red,1);
			 if(three ==1&&four==1)
			gpio_set_value(blue,1);
			 if(Fives ==1&&six ==1)
			gpio_set_value(green,1);

			if((one==1|three==1|Fives==1))
			msleep(led->delay_on);

			 if(one==1&&two==1)
			gpio_set_value(red,0);
			if(three ==1&&four==1)
			gpio_set_value(blue,0);
			 if(Fives ==1&&six ==1)
			gpio_set_value(green,0);
			
			if((one==1|three==1|Fives==1))
		schedule_delayed_work(&led->work,msecs_to_jiffies(led->delay_off));

}



static int lbc_pwm_early_suspend(struct tp_device *tp_d)
{
	printk("lbc_pwm Enter %s\n", __func__);
	suspend1 = two;
	suspend2 = four;
	suspend3 = six;
	two = four = six =0;
	gpio_set_value(red,0);
	gpio_set_value(blue,0);
	gpio_set_value(green,0);
	return 0;
}

static int lbc_pwm_late_resume(struct tp_device *tp_d)
{
	printk("lbc_pwm Enter %s\n", __func__);
	two = suspend1;
	four = suspend2;
	six = suspend3;
	return 0;
}


	static ssize_t lbc_pwm_status_read(struct device *dev,struct device_attribute *attr, char *buf)

	{
	
		return sprintf(buf, "%d\n", buf);
		
	}
static ssize_t lbc_pwm_status_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)

	{
     unsigned int    val;


    if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

		printk("lbc_pwm :%d\n",val);

		lbc_pwm_control(dev,val);

		return count; 
	}

	static ssize_t time_on_status_read(struct device *dev,struct device_attribute *attr, char *buf)

	{
	
		return sprintf(buf, "%d\n", buf);
		
	}
static ssize_t time_on_status_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)

	{
     unsigned int    val;

	struct platform_device *pdev = to_platform_device(dev); 
	struct lbc_pwm_led *led = platform_get_drvdata(pdev);

    if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

		printk("delay_on :%d\n",val);

		led->delay_on = val;

		return count; 
	}


	static ssize_t time_off_status_read(struct device *dev,struct device_attribute *attr, char *buf)

	{
	
		return sprintf(buf, "%d\n", buf);
		
	}
static ssize_t time_off_status_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)

	{
     unsigned int    val;

	struct platform_device *pdev = to_platform_device(dev); 
	struct lbc_pwm_led *led = platform_get_drvdata(pdev);

    if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

		printk("delay_off :%d\n",val);

		led->delay_off = val;

		return count; 
	}


	static ssize_t laohutou_status_read(struct device *dev,struct device_attribute *attr, char *buf)

	{
	
		return sprintf(buf, "%d\n", buf);
		
	}
static ssize_t laohutou_status_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)

	{
     unsigned int    val;


    if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

		

		gpio_direction_output(laohutou,val);
		
		printk("lbc_pwm :%d,  gpio_set_value(laohutou,val)===%d\n",val,gpio_get_value(laohutou));
		return count; 
	}
	
static struct kobj_attribute laohutou_attribute =
		__ATTR(laohutou_status, 0666, laohutou_status_read, laohutou_status_write);

static struct kobj_attribute scr_sysfs_attribute =
		__ATTR(modem_status, 0666, lbc_pwm_status_read, lbc_pwm_status_write);

static struct kobj_attribute time_on_sysfs_attribute =
		__ATTR(time_on, 0666, time_on_status_read, time_on_status_write);

static struct kobj_attribute time_off_sysfs_attribute =
		__ATTR(time_off, 0666, time_off_status_read, time_off_status_write);

struct attribute *rockchip_led_attributes[] = {
	&laohutou_attribute.attr,
	&scr_sysfs_attribute.attr,
	&time_on_sysfs_attribute.attr,
	&time_off_sysfs_attribute.attr,
	NULL
};

static const struct attribute_group lbc_led_group = {
	.attrs = rockchip_led_attributes,
};

static int led_pwm_probe(struct platform_device *pdev)
{
	int ret;
	struct lbc_pwm_led *led;

led = devm_kzalloc(&pdev->dev, sizeof(*led), GFP_KERNEL);
if (!led) {
	printk("failed to allocate driver led_pwm_probe data\n");
	return -ENOMEM;
}
ret = sysfs_create_group(&pdev->dev.kobj, &lbc_led_group);
if (ret < 0)
	dev_err(&pdev->dev, "Create sysfs group failed (%d)\n", ret);


led->tp.tp_suspend = lbc_pwm_early_suspend;
led->tp.tp_resume = lbc_pwm_late_resume;
tp_register_fb(&led->tp);

//ret = sn3106b_register_led_classdev(led);

INIT_DELAYED_WORK(&led->work, led_pwm_led_work);

led->delay_off = 3000;
led->delay_on = 3000;

platform_set_drvdata(pdev,led);


#if 1

	ret = gpio_request(red,NULL);

	if (ret) 
		{
		printk("failed to request red gpio\n");
		}			
	gpio_direction_output(red,0);

		
	ret = gpio_request(blue, NULL);
	if (ret) 
		{
		printk("failed to request blue gpio\n");
		}			
	gpio_direction_output(blue,0);
	
	ret = gpio_request(green, NULL);
	if (ret) 
		{
		printk("failed to request green gpio\n");
		}			
	gpio_direction_output(green,0);

#endif
// leds_lbc_pwm = class_create(THIS_MODULE, "leds_lbc_pwm");
//class_create_file(leds_lbc_pwm, &class_attr_modem_status);


return 0; 
}

static int led_pwm_remove(struct platform_device *pdev)
{ 
	return 0;
}

static const struct of_device_id of_pwm_leds_match[] = {
	{ .compatible = "pwm-leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_leds_match);

static struct platform_driver led_pwm_driver = {
	.probe		= led_pwm_probe,
	.remove		= led_pwm_remove,
	.driver		= {
		.name	= "leds_pwm",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_pwm_leds_match),
	},
};

module_platform_driver(led_pwm_driver);

MODULE_AUTHOR("Luotao Fu <l.fu@pengutronix.de>");
MODULE_DESCRIPTION("PWM LED driver for PXA");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-pwm");
