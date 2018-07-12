#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
// vibrator 对应的GPIO
#define  VIBRATOR_POWER_PORT1 41//1b1
#define  VIBRATOR_POWER_PORT2 42// 1b2
#define  VIBRATOR_POWER_PORT3 44// 1b4
#define  VIBRATOR_POWER_PORT4 2 // 0a2
typedef struct combo_module__t    {
    unsigned char            status_vibrator;
}    combo_module_t    ;

static combo_module_t combo_module;

#define GPIO_LOW 0
#define GPIO_HIGH 1
/*
 * vibrator初始化函数：申请GPIO，并初始化vibrator状态。
 */
static void combo_module_init(void)
{

	    if(gpio_request(14, "vibrator power2"))    {
        printk("misc_sysfs.c request vibrator gpio 14 failse.\n");
    }
      gpio_direction_output(14, 1);  

	  
    if(gpio_request(VIBRATOR_POWER_PORT1, "vibrator power1"))    {
        printk("misc_sysfs.c request vibrator gpio failse.\n");
    }
      gpio_direction_output(VIBRATOR_POWER_PORT1, GPIO_LOW);  
    if(gpio_request(VIBRATOR_POWER_PORT2, "vibrator power2"))    {
        printk("misc_sysfs.c request vibrator gpio failse.\n");
    }
      gpio_direction_output(VIBRATOR_POWER_PORT2, GPIO_LOW);  

    if(gpio_request(VIBRATOR_POWER_PORT3, "vibrator power3"))    {
        printk("misc_sysfs.c request vibrator gpio failse.\n");
    }
      gpio_direction_output(VIBRATOR_POWER_PORT3, GPIO_LOW);  

    if(gpio_request(VIBRATOR_POWER_PORT4, "vibrator power4"))    {
        printk("misc_sysfs.c request vibrator gpio failse.\n");
    }
      gpio_direction_output(VIBRATOR_POWER_PORT4, GPIO_LOW);  

	  
	  

    combo_module.status_vibrator  = 0;
}

/*
 * vibrator控制函数
 */
static void combo_module_control(int l,int r)
{
	printk("combo_module_control----l===%d,r===%d\n",l,r);
    if(l==00)
    {
        gpio_direction_output(VIBRATOR_POWER_PORT1, GPIO_LOW);
		gpio_direction_output(VIBRATOR_POWER_PORT2, GPIO_LOW);
    }
    if(l==10)
    {
        gpio_direction_output(VIBRATOR_POWER_PORT1, GPIO_HIGH);
		gpio_direction_output(VIBRATOR_POWER_PORT2, GPIO_LOW);
    }	
    if(l==11)
    {
        gpio_direction_output(VIBRATOR_POWER_PORT1, GPIO_HIGH);
		gpio_direction_output(VIBRATOR_POWER_PORT2, GPIO_HIGH);
    }
    if(l==01)
    {
        gpio_direction_output(VIBRATOR_POWER_PORT1, GPIO_LOW);
		gpio_direction_output(VIBRATOR_POWER_PORT2, GPIO_HIGH);
    }
    if(r==00)
    {
		gpio_direction_output(VIBRATOR_POWER_PORT3, GPIO_LOW);
		gpio_direction_output(VIBRATOR_POWER_PORT4, GPIO_LOW);
    }
    if(r==10)
    {
		gpio_direction_output(VIBRATOR_POWER_PORT3, GPIO_HIGH);
		gpio_direction_output(VIBRATOR_POWER_PORT4, GPIO_LOW);
    }
    if(r==11)
    {
		gpio_direction_output(VIBRATOR_POWER_PORT3, GPIO_HIGH);
		gpio_direction_output(VIBRATOR_POWER_PORT4, GPIO_HIGH);
    }
    if(r==01)
    {
		gpio_direction_output(VIBRATOR_POWER_PORT3, GPIO_LOW);
		gpio_direction_output(VIBRATOR_POWER_PORT4, GPIO_HIGH);
    }
}
 

///////////////////////////////////////////////////////////////////////////////////////////////////////////

static ssize_t show_vibrator_onoff (struct device *dev, struct device_attribute *attr, char *buf)
{
    return    sprintf(buf, "%d\n", combo_module.status_vibrator);
}

static ssize_t set_vibrator_onoff (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
     unsigned int    val;

    if(!(sscanf(buf, "%u\n", &val)))   
		return -EINVAL;

	int l;
	int r;	
    int n = val;
    int one = n / 1 % 10;
    int two = n / 10 % 10;
    int three = n / 100 % 10;
    int four = n / 1000 % 10;	
	
    printk("set_vibrator_onoff:%d\n",val);
	printk("ggg%d\n sss%d\n bbb:%d\n qqq:%d\n", one, two, three, four);

	l=two*10+one;
	r=three*10+four;


      combo_module_control(l,r);

    
    return count;
}

static    ssize_t show_vibrator_onoff    (struct device *dev, struct device_attribute *attr, char *buf);
static     ssize_t set_vibrator_onoff    (struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
// 将vibrator注册到sysfs文件系统。
// 参数说明：
//       vibrator_onoff      : vibrator对应在sysfs下的文件节点名称
//       S_IRWXUGO           : 文件节点的属性
//       show_vibrator_onoff : 对应的读函数
//       set_vibrator_onoff  : 对应的写函数
static DEVICE_ATTR(wheel_ctrl, S_IRWXUGO, show_vibrator_onoff, set_vibrator_onoff);


static struct attribute *control_sysfs_entries[] = {
    &dev_attr_wheel_ctrl.attr,
    NULL
};

static struct attribute_group control_sysfs_attr_group = {
    .name   = NULL,
    .attrs  = control_sysfs_entries,
};

static int control_sysfs_probe(struct platform_device *pdev)    
{
    printk("vibrator probe\n");
    combo_module_init();
   combo_module_control(00,00);
    return    sysfs_create_group(&pdev->dev.kobj, &control_sysfs_attr_group);
}

static int control_sysfs_remove(struct platform_device *pdev)    
{
    sysfs_remove_group(&pdev->dev.kobj, &control_sysfs_attr_group);
    
    return    0;
}

#ifdef CONFIG_PM
static int control_sysfs_resume(struct platform_device *dev)
{

   
   combo_module_control(00,00);
   

    return  0;
}

static int control_sysfs_suspend(struct platform_device *dev, pm_message_t state)
{
    
  
  combo_module_control(00,00);
  
    
    return  0;
}
#else
#define control_sysfs_suspend NULL
#define control_sysfs_resume NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id rockchip_motor_dt_match[] = {
	{ .compatible = "hy_motor",},
	{ },
};
MODULE_DEVICE_TABLE(of, rockchip_motor_dt_match);
#endif /* CONFIG_OF */


static struct platform_driver control_sysfs_driver = {
    .driver = {
        .name = "misc_ctl",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(rockchip_motor_dt_match),
    },
    .probe         = control_sysfs_probe,
    .remove     = control_sysfs_remove,
    .suspend    = control_sysfs_suspend,
    .resume        = control_sysfs_resume,
};

static int __init control_sysfs_init(void)
{    
    // 将vibrator注册到platform总线
    printk("vibrator init");
    return platform_driver_register(&control_sysfs_driver);
}

static void __exit control_sysfs_exit(void)
{
   platform_driver_unregister(&control_sysfs_driver);
}


module_init(control_sysfs_init);
module_exit(control_sysfs_exit);


MODULE_DESCRIPTION("misc control driver");
MODULE_AUTHOR("other");
MODULE_LICENSE("GPL");
