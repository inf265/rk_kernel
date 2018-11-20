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
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/decompress/mm.h>

 #include "tp_suspend.h"

 // 89 red 27 blue 28 green
int red = 27;//27 pwm1
int green = 28;//pwm2
int blue = 89;//
int laohutou =122;
struct class *leds_lbc_pwm;

typedef enum {
    BUZZER_DISABLE = 0,
    BUZZER_ENABLE,
}BUZZER_STATUS_t;

typedef enum{
	M_LED_BLUE = 0,
	M_LED_LAOHUTOU = 1
}LED_TYPE_t;

typedef enum{
	M_PWM_RED = 0,
	M_PWM_GREEN = 1
}PWM_TYPE_t;


#define BUZZER_DELAY_TIME_HIGHT (190000) //2.7KHZ
#define BUZZER_DELAY_TIME_LOW (190000) //2.7KHZ

//ml=================== 
static struct delayed_work	gpio_work;
static unsigned int flash_on = 1000;
static unsigned int flash_off = 1000;

	/*-------- PWM ----------*/
bool G_IsOpenFlash = false;
struct pwm_device *pwm1 = NULL;
struct pwm_device *pwm2 = NULL;
bool G_openPwmLed = false;
bool G_openPwm2Led = false;
unsigned int pwm_sleeptime = 20;
unsigned int pwm2_sleeptime = 20;
#define G_arrycount  72
const unsigned long duty_ns[G_arrycount]={
	0,100,300,500,800,1100,1400,1700,2000,2300,2600,
	3000,4000,5500,7000,9000,13000,16000,20000,25000,30000,
	35000,40000,45000,50000,60000,70000,80000,90000,100000,110000,
	120000,135000,150000,165000,180000,210000,240000,270000,300000,330000,
	360000,390000,420000,450000,500000,600000,700000,800000,900000,1000000,
	1100000,1200000,1300000,1400000,1600000,1800000,2000000,2200000,2400000,2600000,
	2800000,3000000,3200000,3400000,3600000,3800000,4000000,4200000,4400000,4600000,
	4900000};

	/*-------- GPIO ----------*/
bool m_ledIsEnd = true;
bool G_openbreathLed = false;
bool m_LedIsUp = false;
bool m_ledIsEnd2 = true;	//2 ----> laohutou
bool G_openbreathLed2 = false;
bool m_LedIsUp2 = false;
#define Gamma_T32_len	39
static int LOOP_ONCECOUNT = 600;
static int LOOP_ONCECOUNT2 = 600;
const unsigned long int Gamma_T32[Gamma_T32_len]={
0,300,600,1000,1500,2000,2500,3000,3500,4100,
4800,5500,6500,7500,8500,10000,
12000,13000,15000,19000,24000,30000,35000,40000,47000,55000,
65000,75000,85000,96000,103000,110000,120000,130000,140000,150000,
160000,170000,184000};

static bool G_openlaohutou_greade = false;
long int laohutou_greade = 0;
const unsigned long int Laohutou_T32[5]={
55000,96000,120000,150000,184000};
static struct work_struct wq_hrtimer; 



void Start_Pwm_led(const unsigned int msleeptime);
void enable_Pwm_led(void);
void disable_Pwm_led(void);
void free_pwm_led(void);
void Start_Pwm2_led(const unsigned int msleeptime);
void enable_Pwm2_led(void);
void disable_Pwm2_led(void);
void free_pwm2_led(void);


void enable_Gpio_led(LED_TYPE_t t_led);
void disable_Gpio_led(LED_TYPE_t t_led);
void Start_Gpio_led(const unsigned int t_timerLoopOnceCount);
void Start_Gpio_led2(const unsigned int t_timerLoopOnceCount);
static void laohutou_greade_start(void);
//========================ml 

struct lbc_pwm_led {
	struct delayed_work	work;	
	struct mutex lock;
	 int delay_off;
	 int delay_on;
	 unsigned long int bre_on_pwm;
	 int bre;
	 struct  tp_device  tp;
	 #if 1
    struct hrtimer mytimer;
    ktime_t kt;     //设置定时时间	 
    unsigned long count; //从应用空间读取的数据
    wait_queue_head_t wait_queue;
    BUZZER_STATUS_t status;
	struct hrtimer mytimer2;
    ktime_t kt2;     //设置定时时间
    unsigned long int bre_on_pwm2;
	int bre2;
	unsigned long count2;
	 struct semaphore sem;

	struct hrtimer greade_timer;
	ktime_t greade_kt;
	 #endif
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

struct lbc_pwm_led *led;

static void lbc_pwm_control(struct device *dev, int val)
{
	struct platform_device *pdev = to_platform_device(dev); 
	struct lbc_pwm_led *led = platform_get_drvdata(pdev);

	

	//int g;
	//int b;
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

	//printk("lbc_pwm_control----1===%d,2===%d,3===%d,4===%d,5===%d,6===%d\n",one,two,three,four,Fives,six);


		#if 1
			if(two==1)
			pwm_config(pwm1, duty_ns[G_arrycount-1], 5000000);
			else
			pwm_config(pwm1, duty_ns[0], 5000000);

			if(four==1)
			gpio_set_value(blue,1);
			else
			gpio_set_value(blue,0);

			if(six==1)
			pwm_config(pwm2, duty_ns[G_arrycount-1], 5000000);
			else
			pwm_config(pwm2, duty_ns[0], 5000000);
		#endif

			if(one==1||three==1||Fives==1)
				schedule_delayed_work(&gpio_work,msecs_to_jiffies(5));
				

}
static void led_pwm_led_work(struct work_struct *work)
{
	struct lbc_pwm_led *led = NULL;
	led = container_of(work, struct lbc_pwm_led, work);
	if(IS_ERR(led))
		printk("=========== lbc_pwm_led *led is NULL\n");
	
	//printk("led_pwm_led_work----1=%d,2=%d,3=%d,4=%d,5=%d,6=%d,G_IsOpenFlash=%d,led->delay_off = %d\n",one,two,three,four,Fives,six,G_IsOpenFlash,led->delay_off);

			if(one==1&&two==1)
				pwm_config(pwm1, duty_ns[G_arrycount-1], 5000000);
			if(three ==1&&four==1)
				gpio_set_value(blue,1);
			if(Fives ==1&&six ==1)
				pwm_config(pwm2, duty_ns[G_arrycount-1], 5000000);

			if((one==1|three==1|Fives==1))
				msleep(flash_on);

			if(one==1&&two==1)
				pwm_config(pwm1, duty_ns[0], 5000000);
			if(three ==1&&four==1)
				gpio_set_value(blue,0);
			if(Fives ==1&&six ==1)
				pwm_config(pwm2, duty_ns[0], 5000000);
			
			//if((one==1|three==1|Fives==1))
		if(G_IsOpenFlash && (one==1||three==1||Fives==1))
		{
			//printk("================led_pwm_led_work\n");
			schedule_delayed_work(&gpio_work,msecs_to_jiffies(flash_off));			
		}

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
     unsigned int val,ret;

    if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

		printk("lbc_pwm :%d\n",val);



	if(!G_IsOpenFlash)
	{
		disable_Pwm_led();
		disable_Pwm2_led();
		disable_Gpio_led(M_LED_BLUE);
		disable_Gpio_led(M_LED_LAOHUTOU);
		G_IsOpenFlash = true;
	}

	lbc_pwm_control(dev,val);

		if(val == 0)
		{
			msleep(500);
			disable_Pwm_led();
			disable_Pwm2_led();
			disable_Gpio_led(M_LED_BLUE);
			disable_Gpio_led(M_LED_LAOHUTOU);
			return count;
		}
		

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

		printk("flash_on :%d\n",val);

		flash_on = val;

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

		printk("flash_off :%d\n",val);

		flash_off = val;

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

	if(0 <= val && val <= 5)
	{
		if(val == 0)
		{
			G_openlaohutou_greade = false;
			//msleep(500);
			//gpio_set_value(laohutou,1);
		}
		else{
			laohutou_greade = Laohutou_T32[val - 1];
			if(!G_openlaohutou_greade)
			{
				G_openlaohutou_greade = true;
				laohutou_greade_start();	
			}
		}		
	}

		//gpio_direction_output(laohutou,!val);
		
		printk("val :%d,  gpio_set_value(laohutou,val)===%d\n",val,gpio_get_value(laohutou));
		return count; 
	}


//ml=========================
	static ssize_t breathled_on_status_read(struct device *dev,struct device_attribute *attr, char *buf)

	{
	
		return sprintf(buf, "%d\n", buf);
		
	}
static ssize_t breathled_on_status_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
     unsigned int    val,ret;
	
	struct platform_device *pdev = to_platform_device(dev); 
	struct lbc_pwm_led *led = platform_get_drvdata(pdev);
	
	if(G_IsOpenFlash)
	{
		//printk("============= Close Flash =============\n");
		G_IsOpenFlash = false;
		gpio_set_value(blue,0);
		pwm_config(pwm1, duty_ns[0], 5000000);
		pwm_config(pwm2, duty_ns[0], 5000000);
	}
    if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

		if(val == 1)
			enable_Gpio_led(M_LED_BLUE);
		else if(val == 2)
			disable_Gpio_led(M_LED_BLUE);
		else if(val == 3)
			enable_Gpio_led(M_LED_LAOHUTOU);
		else if(val == 4)
			disable_Gpio_led(M_LED_LAOHUTOU);
		else if(val == 5)
			enable_Pwm_led();		
		else if(val == 6)
			disable_Pwm_led();
		else if(val == 7)
			enable_Pwm2_led();	
		else if(val == 8)
			disable_Pwm2_led();

		schedule_delayed_work(&led->work,msecs_to_jiffies(led->delay_off));

		return count; 
}

static ssize_t pwmled_speed_status_read(struct device *dev,struct device_attribute *attr, char *buf)
{
	
		return sprintf(buf, "%d\n", pwm_sleeptime);
		
}

static ssize_t pwmled_speed_status_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev); 
	struct lbc_pwm_led *led = platform_get_drvdata(pdev);
	unsigned int val;
	if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

	pwm_sleeptime = val;
	//printk("------------%s,val = %d,pwm_sleeptime = %d\n",__FUNCTION__,val,pwm_sleeptime);
}

static ssize_t laohutou_greade_read(struct device *dev,struct device_attribute *attr, char *buf)
{
	
		return sprintf(buf, "%d\n", laohutou_greade);
		
}

static ssize_t laohutou_greade_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int val;
	if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

	printk("laohutou_greade_write: val = %d\n",val);

	if(0 <= val && val <= 5)
	{
		if(val == 0)
		{
			G_openlaohutou_greade = false;
			//msleep(500);
			//gpio_set_value(laohutou,1);
		}
		else{
			laohutou_greade = Laohutou_T32[val - 1];
			if(!G_openlaohutou_greade)
			{
				laohutou_greade_start();
				G_openlaohutou_greade = true;
			}
		}		
	}
}

//=========================ml

//echo 1 > /sys/devices/pwmleds.20/laohutou_status
//echo 111111 > sys/devices/pwmleds.20/modem_status
//echo 500 > sys/devices/pwmleds.20/time_on

// echo 1 > /sys/devices/pwmleds.20/breathled_switch
// echo 1 > /sys/devices/pwmleds.20/pwmled_speed
static struct kobj_attribute laohutou_attribute =
		__ATTR(laohutou_status, 0666, laohutou_status_read, laohutou_status_write);

static struct kobj_attribute scr_sysfs_attribute =
		__ATTR(modem_status, 0666, lbc_pwm_status_read, lbc_pwm_status_write);

static struct kobj_attribute time_on_sysfs_attribute =
		__ATTR(time_on, 0666, time_on_status_read, time_on_status_write);

static struct kobj_attribute time_off_sysfs_attribute =
		__ATTR(time_off, 0666, time_off_status_read, time_off_status_write);

static struct kobj_attribute breathled_switch_attribute =
		__ATTR(breathled_switch, 0666, breathled_on_status_read, breathled_on_status_write);

static struct kobj_attribute pwmled_speed_attribute =
		__ATTR(pwmled_speed, 0666, pwmled_speed_status_read, pwmled_speed_status_write);

static struct kobj_attribute laohutou_greade_attribute =
		__ATTR(laohutou_greate, 0666, laohutou_greade_read, laohutou_greade_write);


struct attribute *rockchip_led_attributes[] = {
	&laohutou_attribute.attr,
	&scr_sysfs_attribute.attr,
	&time_on_sysfs_attribute.attr,
	&time_off_sysfs_attribute.attr,
	&breathled_switch_attribute.attr,
	&pwmled_speed_attribute.attr,
	&laohutou_greade_attribute.attr,
	NULL
};

static const struct attribute_group lbc_led_group = {
	.attrs = rockchip_led_attributes,
};

//ml=========================
#if 1
static enum hrtimer_restart    hrtimer_handler(struct hrtimer *timer)
{
	if(!G_openbreathLed)
	{
		gpio_set_value(blue, 0);
		return HRTIMER_NORESTART;
	}

	int t_skip = 2;
	if(led->bre >=0 && led->bre < Gamma_T32_len && !m_LedIsUp)
	{
		led->bre_on_pwm = Gamma_T32[led->bre];
	}

	if(led->bre >=0 && led->bre < Gamma_T32_len && m_LedIsUp)
	{
		led->bre_on_pwm = Gamma_T32[Gamma_T32_len - 1 - led->bre];
		//if(led->bre_on_pwm <= Gamma_T32[20])
			//led->bre_on_pwm = 0;
	}
	
	if(led->count%LOOP_ONCECOUNT==0 && !m_LedIsUp)
	{
		//printk("------------- led->count = %d, led->bre = %d, m_LedIsUp = %d\n",led->count,led->bre,m_LedIsUp);
		led->bre--;
	}
	if(led->count%LOOP_ONCECOUNT==0 && m_LedIsUp)
	//if(led->count%400==0)
	{
		//printk("+++++++++++++ led->count = %d, led->bre = %d, m_LedIsUp = %d\n",led->count,led->bre,m_LedIsUp);
		led->bre++;
	}

		#if 1
		if(led->count != 0 && !m_LedIsUp){
        	if (gpio_get_value(blue) == 1) {
            	gpio_set_value(blue, 0);

            	led->kt = ktime_set(0, led->bre_on_pwm);	//纳秒级别 * LOOP_ONCECOUNT(400)  (led->bre_on_pwm--)
            	hrtimer_forward_now(&led->mytimer, led->kt);

        	} else {
            	gpio_set_value(blue, 1);

            	led->kt = ktime_set(0, 184000 -led->bre_on_pwm);
            	hrtimer_forward_now(&led->mytimer, led->kt);
        	}
				led->count --;
				if(led->count == 0)
				{
					led->bre = t_skip;
					m_LedIsUp = true;
				}
				return HRTIMER_RESTART;
		}
		#endif
		

			#if 1
			//scl 1 sda 1 0 
		if(led->count <= ((Gamma_T32_len - t_skip) * LOOP_ONCECOUNT) && m_LedIsUp)
		//if(led->count <= Gamma_T32_len * 400)
		{
			//printk("=============================== led down =======================\n");

			if (gpio_get_value(blue) == 1) {
            gpio_set_value(blue, 0);

            led->kt = ktime_set(0,  184000 - led->bre_on_pwm);
            hrtimer_forward_now(&led->mytimer, led->kt);

        } else {
            gpio_set_value(blue, 1);

            led->kt = ktime_set(0,  led->bre_on_pwm);
            hrtimer_forward_now(&led->mytimer, led->kt);
        }
			led->count ++;
			//if(led->count ==(Gamma_T32_len - 23) * LOOP_ONCECOUNT)
			if(led->bre_on_pwm <= Gamma_T32[20])
			{
				//printk("=========== 	m_ledIsEnd = true  =============\n");
				m_LedIsUp = false;
				m_ledIsEnd = true;
				led->status = BUZZER_DISABLE;
				 gpio_set_value(blue, 0);
				//wake_up(&led->wait_queue);
				return HRTIMER_NORESTART;
			}
			return HRTIMER_RESTART;
		} 
			#endif   
}
#endif

static enum hrtimer_restart    hrtimer_handler2(struct hrtimer *timer)
{
	if(!G_openlaohutou_greade)
	{
		gpio_set_value(laohutou, 1);
		return HRTIMER_NORESTART;
	}

	int t_skip = 2;
	if(led->bre2 >=0 && led->bre2 < Gamma_T32_len && !m_LedIsUp2)
	{
		led->bre_on_pwm2 = Gamma_T32[led->bre2];
	}

	if(led->bre2 >=0 && led->bre2 < Gamma_T32_len && m_LedIsUp2)
	{
		led->bre_on_pwm2 = Gamma_T32[Gamma_T32_len - 1 - led->bre2];
		//if(led->bre_on_pwm2 <= Gamma_T32[20])
			//led->bre_on_pwm2 = 0;
	}
	
	if(led->count2%LOOP_ONCECOUNT2==0 && !m_LedIsUp2)
	{
		//printk("------------- led->count2 = %d, led->bre2 = %d, m_LedIsUp2 = %d\n",led->count2,led->bre2,m_LedIsUp2);
		led->bre2--;
	}
	if(led->count2%LOOP_ONCECOUNT2==0 && m_LedIsUp2)
	{
		//printk("+++++++++++++ led->count2 = %d, led->bre2 = %d, m_LedIsUp2 = %d\n",led->count2,led->bre2,m_LedIsUp2);
		led->bre2++;
	}

		#if 1
		if(led->count2 != 0 && !m_LedIsUp2){
        	if (gpio_get_value(laohutou) == 1) {
            	gpio_set_value(laohutou, 0);

            	led->kt2 = ktime_set(0, 184000 - led->bre_on_pwm2);	//纳秒级别 * LOOP_ONCECOUNT(400)  (led->bre_on_pwm--)
            	hrtimer_forward_now(&led->mytimer2, led->kt2);

        	} else {
            	gpio_set_value(laohutou, 1);

            	led->kt2 = ktime_set(0, led->bre_on_pwm2);
            	hrtimer_forward_now(&led->mytimer2, led->kt2);
        	}
				led->count2 --;
				if(led->count2 == 0)
				{
					led->bre2 = t_skip;
					m_LedIsUp2 = true;
				}
				return HRTIMER_RESTART;
		}
		#endif
		
		#if 1
			//scl 1 sda 1 0 
		if(led->count2 <= ((Gamma_T32_len - t_skip) * LOOP_ONCECOUNT2) && m_LedIsUp2)
		{
			//printk("=============================== led down =======================\n");

			if (gpio_get_value(laohutou) == 1) {
            gpio_set_value(laohutou, 0);

            led->kt2 = ktime_set(0,  led->bre_on_pwm2);
            hrtimer_forward_now(&led->mytimer2, led->kt2);

        } else {
            gpio_set_value(laohutou, 1);

            led->kt2 = ktime_set(0,  184000 - led->bre_on_pwm2);
            hrtimer_forward_now(&led->mytimer2, led->kt2);
        }
			led->count2 ++;
			//if(led->count ==(Gamma_T32_len - 23) * LOOP_ONCECOUNT)
			if(led->bre_on_pwm2 <= Gamma_T32[20])
			{
				//printk("=========== 	m_ledIsEnd2 = true  =============\n");
				m_LedIsUp2 = false;
				m_ledIsEnd2 = true;
				//led->status = BUZZER_DISABLE;
				return HRTIMER_NORESTART;
			}
			return HRTIMER_RESTART;
		} 
			#endif   
}


static void buzzer_gpio_start(void)
{
   // printk("-----------buzzer_gpio_start------------ %d\n",m_LedIsUp);
    
    //高精度定时器
	hrtimer_init(&led->mytimer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
    led->mytimer.function = hrtimer_handler;
    led->kt = ktime_set(0, 0);

	m_LedIsUp = false;
	led->count = Gamma_T32_len * LOOP_ONCECOUNT;
	led->bre = Gamma_T32_len - 1;
	
    hrtimer_start(&led->mytimer,led->kt,HRTIMER_MODE_REL); 
}

static void led2_gpio_start(void)
{
	hrtimer_init(&led->mytimer2,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
    led->mytimer2.function = hrtimer_handler2;
    led->kt2 = ktime_set(0, 0);

	m_LedIsUp2= false;
	led->count2 = Gamma_T32_len * LOOP_ONCECOUNT2;
	led->bre2 = Gamma_T32_len - 1;
	
    hrtimer_start(&led->mytimer2,led->kt2,HRTIMER_MODE_REL); 
}


//laohutou greade setting
static enum hrtimer_restart    laohutou_hrtimer_handler(struct hrtimer *timer)
{
	led->greade_timer.function = laohutou_hrtimer_handler;
	if(G_openlaohutou_greade)
	{
		if (gpio_get_value(laohutou) == 1) {
			gpio_set_value(laohutou, 0);
			led->greade_kt = ktime_set(0, laohutou_greade); 
			hrtimer_forward_now(&led->greade_timer, led->greade_kt);
		} else {
			gpio_set_value(laohutou, 1);
			led->greade_kt = ktime_set(0, 184000 - laohutou_greade);
			hrtimer_forward_now(&led->greade_timer,led->greade_kt);
		}
		return HRTIMER_RESTART;
	}
	gpio_set_value(laohutou, 1);
	return HRTIMER_NORESTART;
}

static void laohutou_greade_start(void)
{
	led->greade_timer.function = laohutou_hrtimer_handler;
	led->greade_kt = ktime_set(0, 0);	
	hrtimer_start(&led->greade_timer,led->greade_kt,HRTIMER_MODE_REL); 
}

//GPIO
void enable_Gpio_led(LED_TYPE_t t_led)
{
	if(t_led == M_LED_BLUE)
		G_openbreathLed =  true;
	else if(t_led == M_LED_LAOHUTOU)
		G_openbreathLed2 =  true;
	return;
}

void disable_Gpio_led(LED_TYPE_t t_led)
{
	if(t_led == M_LED_BLUE)
		G_openbreathLed =  false;
	else if(t_led == M_LED_LAOHUTOU)
		G_openbreathLed2 =  false;
	return;
}

void SetGpioSpeed(LED_TYPE_t t_led,const unsigned int t_timerLoopOnceCount)
{
	if(t_timerLoopOnceCount >= 100)
	{
		if(t_led == M_LED_BLUE)
			LOOP_ONCECOUNT = t_timerLoopOnceCount;
		else if(t_led == M_LED_LAOHUTOU)
			LOOP_ONCECOUNT2 = t_timerLoopOnceCount;		
	}
	return;
}

void Start_Gpio_led(const unsigned int t_timerLoopOnceCount)
{
	if(m_ledIsEnd && G_openbreathLed)
	{
		m_ledIsEnd = false;
		msleep(500);
		SetGpioSpeed(M_LED_BLUE,t_timerLoopOnceCount);
		buzzer_gpio_start();
		wait_event(led->wait_queue, led->status == BUZZER_DISABLE);
	}
	return;
}

void Start_Gpio_led2(const unsigned int t_timerLoopOnceCount)
{
	if(m_ledIsEnd2 && G_openbreathLed2)
	{
		m_ledIsEnd2 = false;
		msleep(500);
		SetGpioSpeed(M_LED_LAOHUTOU,t_timerLoopOnceCount);
		led2_gpio_start();
		//wait_event(led->wait_queue, led->status == BUZZER_DISABLE);
	}
	return;
}

#if 1
//pwm1
void Start_Pwm_led(const unsigned int msleeptime)
{	
	//printk("void Start_Pwm_led--------msleeptime = %d\n",msleeptime);
	
	unsigned long period_ns = 5000000;
	unsigned int i = 0;
	pwm_config(pwm1, duty_ns[i], period_ns);//1200000ns即表示1.2ms,说明一个波形中高电平值时持续时间为1.2ms,5000000ns即表示5ms,说明一个波形的周期是5ms
	pwm_enable(pwm1); //将pwm使能

	while(i < G_arrycount-1 && G_openPwmLed)
	{
		pwm_config(pwm1, duty_ns[i], period_ns);
		//printk("duty_ns  = %ld,i = %d\n",duty_ns[i],i);
		msleep(msleeptime);	//ms
		i++;
	}
	
	while(i > 0 && G_openPwmLed)
	{
		pwm_config(pwm1, duty_ns[i], period_ns);
		//printk("duty_ns  = %ld, i = %d\n",duty_ns[i],i);
		msleep(msleeptime);
		i--;
	}
	pwm_config(pwm1, 0, period_ns);
	msleep(400);
}
void enable_Pwm_led()
{
	G_openPwmLed = true;
	return;
}

void disable_Pwm_led()
{
	G_openPwmLed =  false;
	return;
}

void free_pwm_led()
{
	int ret;
	if(pwm1 != NULL){
		pwm_free(pwm1);
		pwm1 = NULL;
	}
}

//PWM2
void Start_Pwm2_led(const unsigned int msleeptime)
{

	//printk("void Start_Pwm1_led--------msleeptime = %d\n",msleeptime);
	
	unsigned long period_ns = 5000000;
	unsigned int i = 0;
	pwm_config(pwm2, duty_ns[i], period_ns);//1200000ns即表示1.2ms,说明一个波形中?叩缙街凳背中奔湮?.2ms,5000000ns即表示5ms,说明一个波形的周期是5ms
	pwm_enable(pwm2); //将pwm使能

	while(i < G_arrycount-1 && G_openPwm2Led)
	{
		pwm_config(pwm2, duty_ns[i], period_ns);
		//printk("duty_ns  = %ld,i = %d\n",duty_ns[i],i);
		msleep(msleeptime);	//ms
		i++;
	}
	
	while(i > 0 && G_openPwm2Led)
	{
		pwm_config(pwm2, duty_ns[i], period_ns);
		//printk("duty_ns  = %ld, i = %d\n",duty_ns[i],i);
		msleep(msleeptime);
		i--;
	}
	pwm_config(pwm1, 0, period_ns);
	msleep(400);
}

void enable_Pwm2_led()
{
	G_openPwm2Led = true;
	return;
}

void disable_Pwm2_led()
{
	G_openPwm2Led =  false;
	return;
}

void free_pwm2_led()
{
	int ret;	
	if(pwm2 != NULL){
		pwm_free(pwm2);
		pwm2 = NULL;
	}
}

static void Pwm_led_work(struct work_struct *work)
{

	//printk("================ %s==================\n",__FUNCTION__);
	
	struct lbc_pwm_led *led = container_of(work, struct lbc_pwm_led, work);
	
	Start_Pwm_led(pwm_sleeptime);	//20

	Start_Pwm2_led(pwm2_sleeptime);

	Start_Gpio_led(500);//600

	Start_Gpio_led2(500);

	if(G_openPwmLed || G_openPwm2Led || G_openbreathLed || G_openbreathLed2)
		schedule_delayed_work(&led->work,msecs_to_jiffies(led->delay_off));
	
}
//==============================ml


#endif
//1、探测设备是否正常
//2、cdev_add(struct file_operations)注册操作接口
//3、device_create()创建设备文件
static int led_pwm_probe(struct platform_device *pdev)
{
	int ret;
	
	gpio_set_value(blue, 0);
	gpio_set_value(green,0);
	gpio_set_value(red,0);

#if 1
//ml==============================
	printk("====>>>> this is my test PWM \n");

//	struct pwm_device *pwm1 = NULL; //初始化一个pwm设备的结构体变量
	pwm1 = pwm_request(1, "mypwm1");//申请pwm设备函数,前面是pwm几,后面是给他的简称
	if(IS_ERR(pwm1))//申请是否成功 
	{
		printk("pwm1 err %ld\n", PTR_ERR(pwm1));
		ret = PTR_ERR(pwm1);
			return ret;
	}
	else 
		printk("mypwm1 success\n");

	
	pwm2 = pwm_request(2, "mypwm2");//申请pwm设备函数,前面是pwm几,后面是给他的简称
	if(IS_ERR(pwm2))//申请是否成功 
	{
		printk("pwm2 err %ld\n", PTR_ERR(pwm2));
		ret = PTR_ERR(pwm2);
			return ret;
	}
	else 
		printk("mypwm1 success\n");
		
#endif

led = devm_kzalloc(&pdev->dev, sizeof(*led), GFP_KERNEL);//分配内存
if (!led) {
	printk("failed to allocate driver led_pwm_probe data\n");
	return -ENOMEM;
}

//创建属性文件的sysfs接口
//sysfs_create_file(struct kobject * kobj,const struct attribute * attr)创建属性文件
//sysfs_create_dir(struct kobject * kobj)创建目录
//int sysfs_open_file(struct inode *inode,struct file *file)打开sysfs文件系统格式的文件
//sysfs_read_file(struct file *file, char__user *buf, size_t count, loff_t *ppos) 读操作
//sysfs_write_file(struct file *file, constchar __user *buf, size_t count, loff_t *ppos) 写操作
ret = sysfs_create_group(&pdev->dev.kobj, &lbc_led_group);	//pdev->dev.kobj
if (ret < 0)
	dev_err(&pdev->dev, "Create sysfs group failed (%d)\n", ret);


led->tp.tp_suspend = lbc_pwm_early_suspend;
led->tp.tp_resume = lbc_pwm_late_resume;
tp_register_fb(&led->tp);

led->delay_off = 100;
led->delay_on = 3000;

INIT_DELAYED_WORK(&led->work, Pwm_led_work);
//schedule_delayed_work(&led->work,msecs_to_jiffies(led->delay_off));//led->delay_off ms

INIT_DELAYED_WORK(&gpio_work, led_pwm_led_work);


hrtimer_init(&led->greade_timer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);
//INIT_WORK(&wq_hrtimer, laohutou_greade_start);


//buzzer_gpio_start();
//wait_event(led->wait_queue, led->status == BUZZER_DISABLE);

platform_set_drvdata(pdev,led);







	ret = gpio_request(laohutou,NULL);

	if (ret) 
		{
		printk("failed to request red gpio\n");
		}			
	gpio_direction_output(laohutou,1);

#if 0

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


//信号量
//sema_init(&led->sem,1);

//if(down_interruptible(&led->sem))
	//return -ERESTARTSYS;


//init_waitqueue_head(&led->wait_queue);
//led->status = BUZZER_DISABLE;


//if(led->status == BUZZER_DISABLE){
	//启动定时器  
//	led->status = BUZZER_ENABLE;

//buzzer_gpio_start();
//wait_event(led->wait_queue, led->status == BUZZER_DISABLE);

//}

return 0; 
}

static int led_pwm_remove(struct platform_device *pdev)
{ 
	return 0;
}

// echo 1 > /sys/devices/pwmleds.20/breathled_on
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
MODULE_ALIAS("platform:leds-pwm");//alias_module是在当前域名下设置虚拟目录
