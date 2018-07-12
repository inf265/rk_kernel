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
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
//#include <linux/pn544.h>
#include "pn544.h"
//#include <plat/gpio.h>
//#include <mach/gpio.h>
//#include <plat/gpio-core.h>
//#include <plat/gpio-cfg.h>
//#include <plat/gpio-cfg-helpers.h>
#include <linux/of_gpio.h>

#define pr_err printk
#define pr_debug printk
#define pr_warning printk

#define GPIO_HIGH 1
#define GPIO_LOW 0

#define MAX_BUFFER_SIZE	512
 bool iswakeup ;

static struct i2c_client *pn544_client = NULL;


struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
};

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	//if (!gpio_get_value(pn544_dev->irq_gpio)) {
	//	return IRQ_HANDLED;
	//}
	 iswakeup = true;                 //  Modify
	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	printk("%s : reading  %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {

		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}
		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		printk("dddddddddddddddd\n");
		//ret = wait_event_interruptible(pn544_dev->read_wq,
				//gpio_get_value(pn544_dev->irq_gpio));
		ret = wait_event_interruptible(pn544_dev->read_wq,
              iswakeup == true);           //  Modify

			  printk("jjjjjjjjjjjjjjj\n");
    iswakeup = false;   //  Modify 
		pn544_disable_irq(pn544_dev);

		if (ret)
			{
			goto fail;
			}
	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	
	printk("IFD->PC:");
	for(i = 0; i < ret; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");
	
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	printk("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	printk("PC->IFD:");
	for(i = 0; i < count; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");
	
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;

	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static int pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			printk("%s power on with firmware\n", __func__);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			printk("%s power on\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			printk("%s power off\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(10);
		} else {
			printk("%s bad arg %u\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		printk("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl  = pn544_dev_ioctl,
};


static int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	
	struct device_node *np = client->dev.of_node;
	struct pn544_dev *pn544_dev;

	pn544_client = client;

	printk("nfc probe step01 is ok\n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	
	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
		printk("nfc probe step02 is ok\n");

	pn544_dev->ven_gpio = of_get_named_gpio_flags(np, "nfc_ven", 0, NULL);
	
	pn544_dev->irq_gpio = of_get_named_gpio_flags(np, "nfc_int", 0, NULL);

	pn544_dev->firm_gpio = of_get_named_gpio_flags(np, "nfc_firm", 0, NULL);
	
	printk("pn544_ts_init********* ven_gpio pin =%d \n",pn544_dev->ven_gpio);
	printk("pn544_ts_init********* irq_gpio  pin =%d \n",pn544_dev->irq_gpio);
	printk("pn544_ts_init********* firm_gpio  pin =%d \n",pn544_dev->firm_gpio);



	client->irq = pn544_dev->irq_gpio;
	printk("nfc probe step03 is ok client->addr===0x%x\n",client->addr);


	pn544_dev->client   = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}


	printk("nfc probe step04 is ok\n");
	




	printk("nfc probe step05 is ok\n");

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */

	gpio_free(pn544_dev->ven_gpio);
	gpio_request(pn544_dev->ven_gpio, "ven_gpio");
	gpio_direction_output(pn544_dev->ven_gpio, GPIO_HIGH);


	gpio_free(pn544_dev->firm_gpio);
	gpio_request(pn544_dev->firm_gpio, "firm_gpio");

	printk("firm---------===%d\n",gpio_get_value(pn544_dev->firm_gpio));
	gpio_direction_output(pn544_dev->firm_gpio, GPIO_HIGH);	
	printk("firm---222------===%d\n",gpio_get_value(pn544_dev->firm_gpio));

	
//	gpio_pull_updown(platform_data->firm_gpio, GPIOPullDown);

	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;
#if 0
	ret = request_irq(client->irq, pn544_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn544_dev);
#else
	gpio_to_irq(client->irq);
	//printk("+++run irq:0x%x set GPIOPullDown:%d \n", client->irq, GPIOPullDown);
	//gpio_pull_updown(client->irq,GPIOPullDown);
	ret = request_irq(client->irq, pn544_dev_irq_handler,
			  IRQ_TYPE_EDGE_RISING, client->name, pn544_dev);//IRQ_TYPE_EDGE_FALLING IRQF_TRIGGER_HIGH IRQ_TYPE_EDGE_RISING
#endif
	if (ret) {
		printk(&client->dev, "request_irq failed\n");
		//goto err_request_irq_failed;
	}
	printk("nfc probe step06 is ok\n");
	
	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);
	
	printk("nfc probe step07 is ok\n");

	return 0;

err_request_irq_failed:
	//misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_exit:
//	gpio_free(pn544_dev->firm_gpio);
err_firm:
	gpio_free(pn544_dev->ven_gpio);
err_ven:
	gpio_free(pn544_dev->irq_gpio);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}

static struct of_device_id nfc_ids[] = {

	{.compatible = "nfc548"},
	{ }
};


static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pn544_id);

static struct i2c_driver pn544_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn544",
		.of_match_table = of_match_ptr(nfc_ids),
	},
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,	
};
 
/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
