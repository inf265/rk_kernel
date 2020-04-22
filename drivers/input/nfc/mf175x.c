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
#include <linux/spinlock.h>
#include <linux/gpio.h>  //mach/gpio.h
#include <linux/irq.h> //mach/irq.h
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/input.h>
#define MAX_BUFFER_SIZE	112

#define mf175x_DEBUG

struct mf175x_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	mf175x_device;
	struct regmap *m_regmap;
	unsigned int 		ven_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	unsigned int		irq_gpio;
	/* CLK_REQ IRQ to signal the state has changed */
	unsigned int		irq_gpio_clk_req;	
	unsigned int		clkreq_gpio;
	/* CLK control */
	unsigned int		clk_src_gpio;
	const	char		*clk_src_name;
	struct	clk		*s_clk;
	bool			clk_run;
	struct fasync_struct *async_queue;
	struct input_dev *input;
	char data2[112];
};

#define REG_MAX		0x1F



static struct regmap *regmap_mf175x = NULL;


//Éè±¸¼Ä´æÆ÷ÅäÖÃÐÅÏ¢
static const struct regmap_config mf175x_regmap = {
	.reg_bits = 8,	//¼Ä´æÆ÷µØÖ·Î»Êý£¬±ØÐëÅäÖÃ
	.val_bits = 8,	//¼Ä´æÆ÷ÖµµÄÎ»Êý£¬±ØÐëÅäÖÃ
	.max_register = REG_MAX,
};


int mf175x_i2c_read_reg(int reg)
{
    int ret;
	unsigned int val;

	if(regmap_mf175x == NULL)
	{
		printk(" t_regmap = NULL \n");
		return -1;
	}
	
    ret = regmap_read(regmap_mf175x, reg, &val); 
	
	printk(" reg==0x%x, reg_val====0x%x\n",reg,&val);	
    return ret;
}

int mf175x_i2c_write_reg(int reg, int val)
{
    int ret;

	if(regmap_mf175x == NULL)
	{
		printk(" t_regmap = NULL \n");
		return -1;
	}

    ret = regmap_write(regmap_mf175x, reg, val);  
    return ret;
}


//kingsun, zhudm add
static struct of_device_id msm_match_table[] = {
	{.compatible = "mf175x"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);



#if defined(mf175x_DEBUG)
/* sysfs interface */
static ssize_t nfc_show(struct device *dev,struct device_attribute *attr, char *buf)
{

	//	if (copy_to_user(buf, data2, sizeof(data2))) {
		//printk("%s : failed to copy to user space\n", __func__);
	//	return -EFAULT;
	//}

	return sprintf(buf,"%s\n","OK");
}

/* debug fs, "echo @1 > /sys/bus/i2c/devices/xxx/nfc_debug" @1:debug_flag  */
static ssize_t nfc_store(struct device *dev,
        struct device_attribute *attr, const char *buf,size_t count)
{
		unsigned int    val;
			struct i2c_client *client = to_i2c_client(dev);
	struct mf175x_dev *mf175x_dev = i2c_get_clientdata(client);
	
	
	
	    if(!(sscanf(buf, "%u\n", &val)))
			return -EINVAL;
		printk("nfc_store :%d\n",val);
		
		
		gpio_set_value(mf175x_dev->ven_gpio, val);
   
		printk("ven_gpio===%d\n",gpio_get_value(mf175x_dev->ven_gpio));
	return count;
}
/////---------------sys/devices/2005a000.i2c/i2c-2/2-0016/nfc_debug  -------------//////
static struct device_attribute mf175x_dev_attr =
	__ATTR(nfc_debug, S_IRUGO | S_IWUGO, nfc_show, nfc_store);
#endif

static int nfc_parse_dt(struct device *dev, struct mf175x_dev *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;



	pdata->ven_gpio = of_get_named_gpio(np, "ven-gpio", 0);
	if ((!gpio_is_valid(pdata->ven_gpio)))
		return -EINVAL;

	pdata->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;
  printk("pdata->irq_gpio 22 ==%d\n",pdata->irq_gpio);
	return r;
}

static void mf175x_disable_irq(struct mf175x_dev *mf175x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&mf175x_dev->irq_enabled_lock, flags);
	if (mf175x_dev->irq_enabled) {
		disable_irq_nosync(mf175x_dev->client->irq);
		mf175x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&mf175x_dev->irq_enabled_lock, flags);
}

extern void bts84_send_KEY_KP7(void);
extern void rk_send_wakeup_key(void);

int nfcon = 0;

static int i2c_read_bytes(struct i2c_client *client, uint8_t *data, uint16_t len,struct mf175x_dev *dev)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int i;
	

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = len;
	msgs[0].buf = &data[0];
	msgs[0].scl_rate = 200 * 1000;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = MAX_BUFFER_SIZE;
	msgs[1].buf = dev->data2;
	msgs[1].scl_rate = 200 * 1000;

	ret = i2c_transfer(client->adapter,msgs, 2);
	//printk("%s Data:0x%X 0x%X 0x%X, Return:0x%X 0x%X 0x%X\n",__func__,
	//		data[0], data2[0], data2[1], data2[2], data2[3], data2[4]);


	for(i = 0; i < MAX_BUFFER_SIZE; i++){
		 // msleep(1);
	 	  printk("%X ",dev->data2[i]);
	 	}
	 	printk("\n");
	 	
	 	if(dev->data2[0]!=0)
	 		{
	 			nfcon = 1;
	 			printk("bts84_send_KEY_KP7 \n");
	 		}
	 		else
	 			{
	 				nfcon = 0;
	 				
	 		memset(dev->data2, 0, MAX_BUFFER_SIZE);
	 			printk("bts84_send_KEY_KP8 \n");
	 			//rk_send_wakeup_key();
	 		}
	 			
	 			

	return ret;
}

int mf175x_ReadCmd (unsigned char *CmdList, struct i2c_client *i2c_client,struct mf175x_dev *dev) 
{
	int ret;
	mutex_lock(&dev->read_mutex);
	ret = i2c_read_bytes(i2c_client, CmdList, 1,dev);
	if (ret < 0)
		pr_info("%s i2c_read_bytes fail\n", __func__);
		
		mutex_unlock(&dev->read_mutex);
		
		
		//enable_irq_wake(i2c_client->irq);

	return ret;
}


static irqreturn_t mf175x_dev_irq_handler(int irq, void *dev_id)
{
	struct mf175x_dev *mf175x_dev = dev_id;
     int ret;

	unsigned char Cmd_read [8] =
		{
		0x2d,
		};
  
  
  rk_send_wakeup_key();
  	printk(" mf175x_dev_irq_handler  client->addr ===0x%x\n ",mf175x_dev->client->addr);
  	
  	
	if (!gpio_get_value(mf175x_dev->irq_gpio)) {
		printk(" mf175x_dev_irq_handler  irq is low!!!!!!!!!!!\n ");
		return IRQ_HANDLED;
	}
	
//	disable_irq_nosync(mf175x_dev->client->irq);
	


	mf175x_ReadCmd(Cmd_read,mf175x_dev->client,mf175x_dev);

		
      if(nfcon==1)
      	{
	 		input_report_key(mf175x_dev->input, KEY_KP1, 1);
	    input_sync(mf175x_dev->input);
	    input_report_key(mf175x_dev->input, KEY_KP1, 0);
	    input_sync(mf175x_dev->input);
	  }
	  else
	  	{
	 		input_report_key(mf175x_dev->input, KEY_KP2, 1);
	    input_sync(mf175x_dev->input);
	    input_report_key(mf175x_dev->input, KEY_KP2, 0);
	    input_sync(mf175x_dev->input);
	  	}
	    
	    
//  if (mf175x_dev->async_queue)
 //       kill_fasync(&mf175x_dev->async_queue, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

static ssize_t mf175x_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct mf175x_dev *mf175x_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(tmp, 0, MAX_BUFFER_SIZE);

  mutex_lock(&mf175x_dev->read_mutex);

	printk("%s : reading %zu bytes.\n", __func__, count);
   

	if (copy_to_user(buf, mf175x_dev->data2, count)) {
		printk("%s : failed to copy to user space\n", __func__);
		mutex_unlock(&mf175x_dev->read_mutex);
		return -EFAULT;
	}


mutex_unlock(&mf175x_dev->read_mutex);
	printk("IFD->PC:");
	for(i = 0; i < MAX_BUFFER_SIZE; i++){
		printk(" %02X", buf[i]);
	}
	printk("\n");

	return count;

}

static ssize_t mf175x_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct mf175x_dev  *mf175x_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	mf175x_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		printk("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	printk("%s : writing %zu bytes.\n", __func__, count);

	printk("PC->IFD:");
	for(i = 0; i < count; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");
	/* Write data */
	ret = i2c_master_send(mf175x_dev->client, tmp, count);
	if (ret != count) {
		//printk("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	return ret;
}

static int mf175x_dev_open(struct inode *inode, struct file *filp)
{
	struct mf175x_dev *mf175x_dev = container_of(filp->private_data,
						struct mf175x_dev,
						mf175x_device);

	filp->private_data = mf175x_dev;

	printk("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long mf175x_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mf175x_dev *mf175x_dev = filp->private_data;

	switch (cmd) {
	case 1:
		 if (arg == 0) {
			/* power on */
			printk("%s power on\n", __func__);
			gpio_set_value(mf175x_dev->ven_gpio, 1);
		//	irq_set_irq_wake(mf175x_dev->client->irq, 1);
			msleep(10);
		} else  if (arg == 1) {
			/* power off */
			printk("%s power off\n", __func__);
			gpio_set_value(mf175x_dev->ven_gpio, 0);
			//irq_set_irq_wake(mf175x_dev->client->irq, 0);
			msleep(10);
		} else {
			printk("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		printk("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

int mf175x_fasync(int fd, struct file *filp, int mode)
{
	struct mf175x_dev *mf175x_dev = filp->private_data;
	return fasync_helper(fd, filp, mode, &mf175x_dev->async_queue);
}

int mf175x_close(struct inode *node, struct file *filp)
{
	mf175x_fasync(-1, filp, 0);
	return 0;
}

static const struct file_operations mf175x_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= mf175x_dev_read,
	.write	= mf175x_dev_write,
	.open	= mf175x_dev_open,
	.fasync  = mf175x_fasync,
	.release = mf175x_close,
	.unlocked_ioctl  = mf175x_dev_ioctl,
};

extern int tige_box_v2(void);

static int mf175x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{	
	int r = 0;
	int ret;
	struct mf175x_dev *mf175x_dev;
	struct input_dev *input_device;

        printk("mf175x enter probe\n");
	
		if(!tige_box_v2())
		return -ENODEV;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	printk("nfc probe step02 is ok\n");


	printk("nfc probe step03 is ok\n");

	mf175x_dev = kzalloc(sizeof(*mf175x_dev), GFP_KERNEL);
	if (mf175x_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
	}

	printk("nfc probe step04 is ok\n");

	mf175x_dev->client   = client;
	ret=nfc_parse_dt(&client->dev,mf175x_dev);

	regmap_mf175x = devm_regmap_init_i2c(client,&mf175x_regmap);
	if (IS_ERR(regmap_mf175x)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(regmap_mf175x);
	}
	mf175x_dev->m_regmap = regmap_mf175x;	

	/* init mutex and queues */
	init_waitqueue_head(&mf175x_dev->read_wq);
	mutex_init(&mf175x_dev->read_mutex);
	spin_lock_init(&mf175x_dev->irq_enabled_lock);



	input_device = devm_input_allocate_device(&mf175x_dev->client->dev);
	if (!input_device) {
		printk("input_device errrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr\n");
	}

	mf175x_dev->input = input_device;
	input_device->name = "mf175x";
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	
	ret = input_register_device(input_device);
	if (ret)
		pr_err("mf175x: register input device err, ret: %d\n", ret);
			
	
	input_set_drvdata(input_device, mf175x_dev);
	input_set_capability(input_device, EV_KEY, KEY_KP1);
	input_set_capability(input_device, EV_KEY, KEY_KP2);
	
	mf175x_dev->mf175x_device.minor = MISC_DYNAMIC_MINOR;
	mf175x_dev->mf175x_device.name = "mf175x";
	mf175x_dev->mf175x_device.fops = &mf175x_dev_fops;

	ret = misc_register(&mf175x_dev->mf175x_device);
	if (ret) {
		printk("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}
	printk("nfc probe step05 is ok\n");


    if(gpio_request(mf175x_dev->irq_gpio,"nfc_int") != 0)
	{
	  printk("mf175x: gpio_IRQ_request error\n");
	  goto err_irq;
	}

    if(gpio_request(mf175x_dev->ven_gpio,"nfc_ven") != 0)
	{
	  printk("mf175x: gpio_VEN_request error\n");
	  goto err_ven;
	}


   printk("mf175x_dev->ven_gpio====%d\n",gpio_get_value(mf175x_dev->ven_gpio));
   
	gpio_direction_output(mf175x_dev->ven_gpio, 1);
         printk("nfc probe GPIO is ok\n");
	
	gpio_direction_input(mf175x_dev->irq_gpio);
	printk("mf175x mf175x_dev->irq_gpio = %d \n", mf175x_dev->irq_gpio); //kingsun, zhudm
	client->irq = gpio_to_irq(mf175x_dev->irq_gpio);
	printk("%s : requesting IRQ %d\n", __func__, client->irq);
	mf175x_dev->irq_enabled = true;

	ret = request_irq(client->irq, mf175x_dev_irq_handler,IRQF_TRIGGER_RISING, client->name, mf175x_dev);
#if 1

	if (ret) {
		printk("request_irq failed\n");
		goto err_request_irq_failed;
	}
#endif

	enable_irq_wake(client->irq);


	printk("nfc probe step06 is ok\n");

	//mf175x_disable_irq(mf175x_dev);

	//return 0;
	i2c_set_clientdata(client, mf175x_dev);

	printk("nfc probe successful\n");


#if defined(mf175x_DEBUG)
		ret = device_create_file(&client->dev, &mf175x_dev_attr);
		if (ret) {
			//NFC_ERR("sysfs registration failed, error %d \n", ret);
			goto err_request_irq_failed;
		}
#endif

	return 0;

err_request_irq_failed:
	misc_deregister(&mf175x_dev->mf175x_device);
err_misc_register:
	mutex_destroy(&mf175x_dev->read_mutex);
err_ven:
	gpio_free(mf175x_dev->ven_gpio);
err_irq:
	gpio_free(mf175x_dev->irq_gpio);	
	return ret;
}

static int mf175x_remove(struct i2c_client *client)
{
	struct mf175x_dev *mf175x_dev;

	mf175x_dev = i2c_get_clientdata(client);
	free_irq(client->irq, mf175x_dev);
	misc_deregister(&mf175x_dev->mf175x_device);
	//kingsun, jerome/zhudm add	
#if defined(mf175x_DEBUG)
	device_remove_file(&client->dev, &mf175x_dev_attr);
#endif
	//mf175x_clock_deselect(mf175x_dev);
	mutex_destroy(&mf175x_dev->read_mutex);
	gpio_free(mf175x_dev->irq_gpio);
	gpio_free(mf175x_dev->ven_gpio);
	kfree(mf175x_dev);

	return 0;
}

static const struct i2c_device_id mf175x_id[] = {
	{ "mf175x", 0 },
	{ }
};

static struct i2c_driver mf175x_driver = {
	.id_table	= mf175x_id,
	.probe		= mf175x_probe,
	.remove		= mf175x_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "mf175x",
		.of_match_table = msm_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init mf175x_dev_init(void)
{
	printk("Loading mf175x driver\n");
	return i2c_add_driver(&mf175x_driver);
}
late_initcall(mf175x_dev_init);

static void __exit mf175x_dev_exit(void)
{
	printk("Unloading mf175x driver\n");
	i2c_del_driver(&mf175x_driver);
}
module_exit(mf175x_dev_exit);

MODULE_AUTHOR("phm");
MODULE_DESCRIPTION("NFC mf175x driver");
MODULE_LICENSE("GPL");
