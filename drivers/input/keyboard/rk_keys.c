/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright (C) 2015, Fuzhou Rockchip Electronics Co., Ltd
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/adc.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#define EMPTY_ADVALUE			950
#define DRIFT_ADVALUE			70
#define INVALID_ADVALUE			-1
#define EV_ENCALL                       KEY_F4
#define EV_MENU                         KEY_F1

#if 1
#define key_dbg(bdata, format, arg...)		\
	dev_info(&bdata->input->dev, format, ##arg)
#else
#define key_dbg(bdata, format, arg...)
#endif

#define DEBOUNCE_JIFFIES	(10 / (MSEC_PER_SEC / HZ))	/* 10ms */
#define ADC_SAMPLE_JIFFIES	(100 / (MSEC_PER_SEC / HZ))	/* 100ms */
#define WAKE_LOCK_JIFFIES	(1 * HZ)			/* 1s */

enum rk_key_type {
	TYPE_GPIO = 1,
	TYPE_ADC
};

struct rk_keys_button {
	struct device *dev;
	u32 type;		/* TYPE_GPIO, TYPE_ADC */
	u32 code;		/* key code */
	const char *desc;	/* key label */
	u32 state;		/* key up & down state */
	u32 long_press_count; /* add a custom long press count */
	int gpio;		/* gpio only */
	int adc_value;		/* adc only */
	int adc_state;		/* adc only */
	int active_low;		/* gpio only */
	int wakeup;		/* gpio only */
	struct timer_list timer;
};

struct rk_keys_drvdata {
	int nbuttons;
	/* flag to indicate if we're suspending/resuming */
	bool in_suspend;
	int result;
	int rep;
	struct wake_lock wake_lock;
	struct input_dev *input;
	struct delayed_work adc_poll_work;
	struct iio_channel *chan;
	struct rk_keys_button button[0];
};

static struct input_dev *sinput_dev;

void rk_send_power_key(int state)
{
	if (!sinput_dev)
		return;
	if (state) {
		input_report_key(sinput_dev, KEY_POWER, 1);
		input_sync(sinput_dev);
	} else {
		input_report_key(sinput_dev, KEY_POWER, 0);
		input_sync(sinput_dev);
	}
}
EXPORT_SYMBOL(rk_send_power_key);

void rk_send_wakeup_key(void)
{
	if (!sinput_dev)
		return;

	input_report_key(sinput_dev, KEY_WAKEUP, 1);
	input_sync(sinput_dev);
	input_report_key(sinput_dev, KEY_WAKEUP, 0);
	input_sync(sinput_dev);
}
EXPORT_SYMBOL(rk_send_wakeup_key);

int aaa =0;
int bbb =0;
int ccc =0;


static void keys_timer(unsigned long _data)
{
	struct rk_keys_button *button = (struct rk_keys_button *)_data;
	struct rk_keys_drvdata *pdata = dev_get_drvdata(button->dev);
	struct input_dev *input = pdata->input;
	int state;

	if (button->type == TYPE_GPIO)
		state = !!((gpio_get_value(button->gpio) ? 1 : 0) ^
			   button->active_low);
	else
		state = !!button->adc_state;

	//printk("button->code===========%d\n",button->code);
	if (button->state != state) {
		button->state = state;
		
		
		if (button->code == 140) 
		{
					input_report_key(input, KEY_PAGEDOWN, 1);
					input_sync(input);
					input_report_key(input, KEY_PAGEDOWN, 0);
					input_sync(input);
					printk("phm  KEY_PAGEDOWN 140\n");
					goto READY_END;
		}

		if (button->code == 147) {
				
			if(button->state == 0) {
				aaa =0;
				if (button->long_press_count > 88) {
					// KEY KEY_KP7 71
					
					input_report_key(input, KEY_KP7, 0);
					input_sync(input);
				} else {
					// KEY KEY_KP8 72
					rk_send_wakeup_key();
					input_report_key(input, KEY_KP8, 1);
					input_sync(input);
					input_report_key(input, KEY_KP8, 0);
					input_sync(input);
					
					printk("phm  KEY_PAGEDOWN 109\n");
				}
				button->long_press_count = 0;
			} else if (button->state == 1) {
				button->long_press_count = 0;
				aaa=1;
			}
			
			button->long_press_count++;
			goto READY_END;
		}
		

		// xujie@yf-space.com add 20170810
		if (button->code == 158/*back key adc*/) {
		//	key_dbg(pdata, "KEY_BACK state[%d] long_press_count[%d]", 
			//	button->state, button->long_press_count);
			if(button->state == 0) {
				bbb=0;
				if (button->long_press_count > 88) {
					// KEY LAUNCHER UI - KEY_PAGEUP
				//	input_report_key(input, KEY_PAGEUP, 1);
				////	input_sync(input);
				//	input_report_key(input, KEY_PAGEUP, 0);
				//	input_sync(input);

				input_report_key(input, KEY_HOME, 0);
				input_sync(input);
					key_dbg(pdata, "KEY_HOME  up[%d].", KEY_HOME);
				} else {
					// KEY BACK
					input_report_key(input, KEY_BACK, 1);
					input_sync(input);
					input_report_key(input, KEY_BACK, 0);
					input_sync(input);
					key_dbg(pdata, "KEY_BACK[%d].", KEY_BACK);
				}

				button->long_press_count = 0;
			} else if (button->state == 1) {
				bbb=1;
				button->long_press_count = 0;
			}
			button->long_press_count++;
			goto READY_END;
		}
		//KEY_KP2 80 left button


		if (button->code == 165) {
		//	key_dbg(pdata, "KEY_KP2 state[%d] long_press_count[%d]", 
			//	button->state, button->long_press_count);

			if(button->state == 0) {
				ccc =0;
				if (button->long_press_count > 88) {
					// KEY KEY_KP1 79
					
					input_report_key(input, KEY_KP1, 0);
					input_sync(input);
					printk("phm  KEY_KP1==%d\n",KEY_KP1);
				} else {
					// KEY KEY_KP2 80
					input_report_key(input, KEY_KP2, 1);
					input_sync(input);
					input_report_key(input, KEY_KP2, 0);
					input_sync(input);
					
					printk("phm  KEY_KP2==%d\n",KEY_KP2);
				}
				button->long_press_count = 0;
			} else if (button->state == 1) {
				button->long_press_count = 0;
				ccc=1;
			}
			
			button->long_press_count++;
			goto READY_END;
		}


		

		input_event(input, EV_KEY, button->code, button->state);
		key_dbg(pdata, "%skey[%s]: report event[%d] state[%d], count[%d]\n",
			button->type == TYPE_ADC ? "adc" : "gpio",
			button->desc, button->code, button->state, button->long_press_count);
		//input_event(input, EV_KEY, button->code, button->state);
		input_sync(input);
	} else {
	if(button->code == 147||button->code == 158||button->code == 165)
		button->long_press_count ++;
			
	}

if(aaa==1 && button->long_press_count > 88)
	{
	aaa =0;
	input_report_key(input, KEY_KP7, 1);
	input_sync(input);
key_dbg(pdata, "KEY_KP7 11111[%d].", KEY_KP7);
	}
if(bbb==1 && button->long_press_count > 88)
	{
	bbb =0;
	input_report_key(input, KEY_HOME, 1);
	input_sync(input);

key_dbg(pdata, "KEY_HOME  down  [%d].", KEY_HOME);
	}

if(ccc==1 && button->long_press_count > 88)
	{
	ccc =0;
	input_report_key(input, KEY_KP1, 1);
	input_sync(input);

key_dbg(pdata, "KEY_KP1  down  [%d].", KEY_KP1);
	}


READY_END:



	if (state)
		mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);
}

static irqreturn_t keys_isr(int irq, void *dev_id)
{
	struct rk_keys_button *button = (struct rk_keys_button *)dev_id;
	struct rk_keys_drvdata *pdata = dev_get_drvdata(button->dev);
	struct input_dev *input = pdata->input;

	BUG_ON(irq != gpio_to_irq(button->gpio));
	if (button->wakeup && pdata->in_suspend) {
		button->state = 1;
		key_dbg(pdata,
			"wakeup: %skey[%s]: report event[%d] state[%d]\n",
			(button->type == TYPE_ADC) ? "adc" : "gpio",
			button->desc, button->code, button->state);
		input_event(input, EV_KEY, button->code, button->state);
		input_sync(input);
	}
	if (button->wakeup)
		wake_lock_timeout(&pdata->wake_lock, WAKE_LOCK_JIFFIES);
	mod_timer(&button->timer, jiffies + DEBOUNCE_JIFFIES);

	return IRQ_HANDLED;
}

/*
static ssize_t adc_value_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);

	return sprintf(buf, "adc_value: %d\n", ddata->result);
}
static DEVICE_ATTR(get_adc_value, S_IRUGO | S_IWUSR, adc_value_show, NULL);
*/

static const struct of_device_id rk_key_match[] = {
	{ .compatible = "rockchip,key", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(of, rk_key_match);

static int rk_key_adc_iio_read(struct rk_keys_drvdata *data)
{
	struct iio_channel *channel = data->chan;
	int val, ret;

	if (!channel)
		return INVALID_ADVALUE;
	ret = iio_read_channel_raw(channel, &val);
	if (ret < 0) {
		pr_err("read channel() error: %d\n", ret);
		return ret;
	}
	return val;
}

static void adc_key_poll(struct work_struct *work)
{
	struct rk_keys_drvdata *ddata;
	int i, result = -1;

	ddata = container_of(work, struct rk_keys_drvdata, adc_poll_work.work);
	if (!ddata->in_suspend) {
		result = rk_key_adc_iio_read(ddata);
		
		//printk("phm adc_key_poll result=====%d\n",result);
		if (result > INVALID_ADVALUE && result < EMPTY_ADVALUE)
			ddata->result = result;
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = &ddata->button[i];

			if (!button->adc_value)
				continue;
			if (result < button->adc_value + DRIFT_ADVALUE &&
			    result > button->adc_value - DRIFT_ADVALUE)
				button->adc_state = 1;
			else
				button->adc_state = 0;
			if (button->state != button->adc_state)
				mod_timer(&button->timer,
					  jiffies + DEBOUNCE_JIFFIES);
		}
	}

	schedule_delayed_work(&ddata->adc_poll_work, ADC_SAMPLE_JIFFIES);
}

static int rk_key_type_get(struct device_node *node,
			   struct rk_keys_button *button)
{
	u32 adc_value;

	if (!of_property_read_u32(node, "rockchip,adc_value", &adc_value))
		return TYPE_ADC;
	else if (of_get_gpio(node, 0) >= 0)
		return TYPE_GPIO;
	else
		return -1;
}

static int rk_keys_parse_dt(struct rk_keys_drvdata *pdata,
			    struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *child_node;
	struct iio_channel *chan;
	int ret, gpio, i = 0;
	u32 code, adc_value, flags;

	chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(chan)) {
		dev_info(&pdev->dev, "no io-channels defined\n");
		chan = NULL;
	}
	pdata->chan = chan;

	for_each_child_of_node(node, child_node) {
		if (of_property_read_u32(child_node, "linux,code", &code)) {
			dev_err(&pdev->dev,
				"Missing linux,code property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		pdata->button[i].code = code;
		pdata->button[i].desc =
		    of_get_property(child_node, "label", NULL);
		pdata->button[i].type =
		    rk_key_type_get(child_node, &pdata->button[i]);
		switch (pdata->button[i].type) {
		case TYPE_GPIO:
			gpio = of_get_gpio_flags(child_node, 0, &flags);
			if (gpio < 0) {
				ret = gpio;
				if (ret != -EPROBE_DEFER)
					dev_err(&pdev->dev,
						"Failed to get gpio flags, error: %d\n",
						ret);
				goto error_ret;
			}

			pdata->button[i].gpio = gpio;
			pdata->button[i].active_low =
			    flags & OF_GPIO_ACTIVE_LOW;
			pdata->button[i].wakeup =
			    !!of_get_property(child_node, "gpio-key,wakeup",
					      NULL);
			break;

		case TYPE_ADC:
			if (of_property_read_u32
			    (child_node, "rockchip,adc_value", &adc_value)) {
				dev_err(&pdev->dev,
					"Missing rockchip,adc_value property in the DT.\n");
				ret = -EINVAL;
				goto error_ret;
			}
			pdata->button[i].adc_value = adc_value;
			break;

		default:
			dev_err(&pdev->dev,
				"Error rockchip,type property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		i++;
	}

	return 0;

error_ret:
	return ret;
}

static int keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct rk_keys_drvdata *ddata = NULL;
	struct input_dev *input = NULL;
	int i, error = 0;
	int wakeup, key_num = 0;

	key_num = of_get_child_count(np);
	if (key_num == 0)
		dev_info(&pdev->dev, "no key defined\n");

	ddata = devm_kzalloc(dev, sizeof(struct rk_keys_drvdata) +
			     key_num * sizeof(struct rk_keys_button),
			     GFP_KERNEL);

	input = devm_input_allocate_device(dev);
	if (!ddata || !input) {
		error = -ENOMEM;
		return error;
	}
	platform_set_drvdata(pdev, ddata);
	dev_set_drvdata(&pdev->dev, ddata);

	input->name = "rk29-keypad";	/* pdev->name; */
	input->phys = "gpio-keys/input0";
	input->dev.parent = dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	ddata->input = input;

	/* parse info from dt */
	ddata->nbuttons = key_num;
	error = rk_keys_parse_dt(ddata, pdev);
	if (error)
		goto fail0;

	/* Enable auto repeat feature of Linux input subsystem */
	if (ddata->rep)
		__set_bit(EV_REP, input->evbit);

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, error: %d\n",
		       error);
		goto fail0;
	}
	sinput_dev = input;

	for (i = 0; i < ddata->nbuttons; i++) {
		struct rk_keys_button *button = &ddata->button[i];

		if (button->code) {
			setup_timer(&button->timer,
				    keys_timer, (unsigned long)button);
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, EV_KEY, button->code);
	}
	input_set_capability(input, EV_KEY, KEY_PAGEUP);
	input_set_capability(input, EV_KEY, KEY_PAGEDOWN);
	input_set_capability(input, EV_KEY, KEY_KP7);
	input_set_capability(input, EV_KEY, KEY_KP8);
	input_set_capability(input, EV_KEY, KEY_KP1);
	input_set_capability(input, EV_KEY, KEY_KP2);
	input_set_capability(input, EV_KEY, KEY_HOME);	

	wake_lock_init(&ddata->wake_lock, WAKE_LOCK_SUSPEND, input->name);
	device_init_wakeup(dev, wakeup);

	for (i = 0; i < ddata->nbuttons; i++) {
		struct rk_keys_button *button = &ddata->button[i];

		button->dev = &pdev->dev;
		if (button->type == TYPE_GPIO) {
			int irq;

			error =
			    devm_gpio_request(dev, button->gpio,
					      button->desc ? : "keys");
			if (error < 0) {
				pr_err("gpio-keys: failed to request GPIO %d, error %d\n",
				       button->gpio, error);
				goto fail1;
			}

			error = gpio_direction_input(button->gpio);
			if (error < 0) {
				pr_err("gpio-keys: failed to configure input direction for GPIO %d, error %d\n",
				       button->gpio, error);
				gpio_free(button->gpio);
				goto fail1;
			}

			irq = gpio_to_irq(button->gpio);
			if (irq < 0) {
				error = irq;
				pr_err("gpio-keys: Unable to get irq number for GPIO %d, error %d\n",
				       button->gpio, error);
				gpio_free(button->gpio);
				goto fail1;
			}

			error = devm_request_irq(dev, irq, keys_isr,
						 button->active_low ?
						 IRQF_TRIGGER_FALLING :
						 IRQF_TRIGGER_RISING,
						 button->desc ?
						 button->desc : "keys",
						 button);
			if (error) {
				pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
				       irq, error);
				gpio_free(button->gpio);
				goto fail1;
			}
		}
	}

	input_set_capability(input, EV_KEY, KEY_WAKEUP);
	/* adc polling work */
	if (ddata->chan) {
		INIT_DELAYED_WORK(&ddata->adc_poll_work, adc_key_poll);
		schedule_delayed_work(&ddata->adc_poll_work,
				      ADC_SAMPLE_JIFFIES);
	}

	return error;

fail1:
	while (--i >= 0)
		del_timer_sync(&ddata->button[i].timer);
	device_init_wakeup(dev, 0);
	wake_lock_destroy(&ddata->wake_lock);
fail0:
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int keys_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(dev, 0);
	for (i = 0; i < ddata->nbuttons; i++)
		del_timer_sync(&ddata->button[i].timer);
	if (ddata->chan)
		cancel_delayed_work_sync(&ddata->adc_poll_work);
	input_unregister_device(input);
	wake_lock_destroy(&ddata->wake_lock);

	sinput_dev = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int keys_suspend(struct device *dev)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	ddata->in_suspend = true;
	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = ddata->button + i;

			if (button->wakeup)
				enable_irq_wake(gpio_to_irq(button->gpio));
		}
	}

	return 0;
}

static int keys_resume(struct device *dev)
{
	struct rk_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->nbuttons; i++) {
			struct rk_keys_button *button = ddata->button + i;

			if (button->wakeup)
				disable_irq_wake(gpio_to_irq(button->gpio));
		}
		preempt_disable();
		/* for call resend_irqs, which may call keys_isr */
		if (local_softirq_pending())
			do_softirq();
		preempt_enable_no_resched();
	}

	ddata->in_suspend = false;

	return 0;
}

static const struct dev_pm_ops keys_pm_ops = {
	.suspend	= keys_suspend,
	.resume		= keys_resume,
};
#endif

static struct platform_driver keys_device_driver = {
	.probe		= keys_probe,
	.remove		= keys_remove,
	.driver		= {
		.name	= "rk-keypad",
		.owner	= THIS_MODULE,
		.of_match_table = rk_key_match,
#ifdef CONFIG_PM
		.pm	= &keys_pm_ops,
#endif
	}
};

module_platform_driver(keys_device_driver);
