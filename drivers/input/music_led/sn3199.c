/*
 * Copyright (C) 2010 Trusted Logic S.A.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/regmap.h>

#include <linux/gpio.h>  //mach/gpio.h
#include <linux/irq.h> //mach/irq.h
#include <linux/of_gpio.h>


#define sn3199_DEBUG
static struct regmap *regmap_sn3199 = NULL;

struct sn3199_dev	{
	unsigned int sdb;
	struct regmap *regmap;
	struct i2c_client	*client;
};
#define REG_MAX		0x1F

static const struct regmap_config sn3199_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

//phm add
static struct of_device_id msm_match_table[] = {
	{.compatible = "sn3199"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

int sn3199_i2c_read_reg(int reg, int *val)
{
    int ret;
    
    ret = regmap_read(regmap_sn3199, reg, val); 
	
	printk(" reg==0x%x, reg_val====0x%x\n",reg,val);	
    return ret;
}

int sn3199_i2c_write_reg(int reg, int val)
{
    int ret;
    
    ret = regmap_write(regmap_sn3199, reg, val);  
    return ret;
}



#if defined(sn3199_DEBUG)
/* sysfs interface */
static ssize_t led_show(struct device *dev,struct device_attribute *attr, char *buf)
{

	return sprintf(buf,"%s\n","OK");
}

/* debug fs, "echo @1 > /sys/bus/i2c/devices/xxx/led_debug" @1:debug_flag  */
static ssize_t led_store(struct device *dev,
        struct device_attribute *attr, const char *buf,size_t count)
{
	
	return count;
}

static struct device_attribute sn3199_dev_attr =
	__ATTR(led_debug, S_IRUGO | S_IWUGO, led_show, led_store);
#endif


static int nfc_parse_dt(struct device *dev, struct sn3199_dev *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;


	pdata->sdb = of_get_named_gpio(np, "sdb-gpio", 0);
	if ((!gpio_is_valid(pdata->sdb)))
		return -EINVAL;
		printk("nfc_parse_dt  sdb==%d ,val===\n",pdata->sdb,gpio_get_value(pdata->sdb));
	return r;
}



static int sn3199_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{	
	int r = 0;
	int ret;
	struct sn3199_dev *sn3199_dev;
	unsigned int reg_val;


        printk("sn3199 enter probe\n");
	sn3199_dev = devm_kzalloc(&client->dev, sizeof(struct sn3199_dev), GFP_KERNEL);
	if (sn3199_dev == NULL)
		return -ENOMEM;
 
	sn3199_dev->sdb = of_get_named_gpio(client->dev.of_node, "sdb-gpio", 0);

	regmap_sn3199 = devm_regmap_init_i2c(client,&sn3199_regmap);
	if (IS_ERR(regmap_sn3199)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(regmap_sn3199);
	}

    if(gpio_request(sn3199_dev->sdb,"led_int") != 0)
	{
	  printk("sn3199: gpio_IRQ_request error\n");
	}
	gpio_direction_output(sn3199_dev->sdb, 1);
         printk("led probe GPIO is ok\n");

	i2c_set_clientdata(client, sn3199_dev);

	printk("nfc_parse_dt 22222 sdb==%d ,val===%d\n",sn3199_dev->sdb,gpio_get_value(sn3199_dev->sdb));

	//ret = sn3199_i2c_write_reg(0x07,0xff);
	ret = sn3199_i2c_write_reg(0x00,0x01);
	ret = sn3199_i2c_write_reg(0x03,0x14);//16 off agc
	
	ret = sn3199_i2c_write_reg(0x04,0x77);

	ret = sn3199_i2c_write_reg(0x01,0x07);
	
	//ret = sn3199_i2c_write_reg(0x05,0x70);
	//ret = sn3199_i2c_write_reg(0x1a,0x40);
	//ret = sn3199_i2c_write_reg(0x06,0x10);
	ret = sn3199_i2c_write_reg(0x07,0xff);
	sn3199_i2c_write_reg(0x08,0xff);
	sn3199_i2c_write_reg(0x08,0xff);
	ret = sn3199_i2c_write_reg(0x26,0x00);
	ret = sn3199_i2c_write_reg(0x10,0x00);
	//msleep(100);
	//sn3199_i2c_write_reg(0x01,0x01);
	if (ret < 0)
		printk("led probe i2c failed to access register\n"); 

	printk("led probe successful\n");


#if defined(sn3199_DEBUG)
		ret = device_create_file(&client->dev, &sn3199_dev_attr);

#endif
	return ret;
}

static int sn3199_remove(struct i2c_client *client)
{
	struct sn3199_dev *sn3199_dev;

	//sn3199_dev = i2c_get_clientdata(client);

#if defined(sn3199_DEBUG)
	device_remove_file(&client->dev, &sn3199_dev_attr);
#endif


	return 0;
}

static const struct i2c_device_id sn3199_id[] = {
	{ "sn3199", 0 },
	{ }
};

static struct i2c_driver sn3199_driver = {
	.id_table	= sn3199_id,
	.probe		= sn3199_probe,
	.remove		= sn3199_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sn3199",
		.of_match_table = msm_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init sn3199_dev_init(void)
{
	pr_info("Loading sn3199 driver\n");
	return i2c_add_driver(&sn3199_driver);
}
late_initcall(sn3199_dev_init);

static void __exit sn3199_dev_exit(void)
{
	pr_info("Unloading sn3199 driver\n");
	i2c_del_driver(&sn3199_driver);
}
module_exit(sn3199_dev_exit);

MODULE_AUTHOR("Ernest Li");
MODULE_DESCRIPTION("led sn3199 driver");
MODULE_LICENSE("GPL");
