	 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <linux/rockchip/common.h>
#include <linux/gpio.h>
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

static int pwr_on_thrsd = 1;          //power on capcity threshold

static int __init pwr_on_thrsd_setup(char *str)
{

	pwr_on_thrsd = simple_strtol(str,NULL,10);
	DBG("power on threshold:%d",pwr_on_thrsd);
	return 0;
}

__setup("pwr_on_thrsd=", pwr_on_thrsd_setup);

static int usb_status;
static int ac_status;
static int __rk_get_system_battery_status(struct device *dev, void *data)
{
	union power_supply_propval val_status = {POWER_SUPPLY_STATUS_DISCHARGING};
	struct power_supply *psy = dev_get_drvdata(dev);

	psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val_status);

	if (val_status.intval != 0) {
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			usb_status = POWER_SUPPLY_TYPE_USB;
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			ac_status = POWER_SUPPLY_TYPE_MAINS;
	}

	return 0;
}

// POWER_SUPPLY_TYPE_BATTERY --- discharge
// POWER_SUPPLY_TYPE_USB     --- usb_charging
// POWER_SUPPLY_TYPE_MAINS   --- AC_charging
int rk_get_system_battery_status(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __rk_get_system_battery_status);

	if (ac_status == POWER_SUPPLY_TYPE_MAINS) {
		return POWER_SUPPLY_TYPE_MAINS;
	} else if (usb_status == POWER_SUPPLY_TYPE_USB) {
		return POWER_SUPPLY_TYPE_USB;
	}

	return POWER_SUPPLY_TYPE_BATTERY;
}
EXPORT_SYMBOL(rk_get_system_battery_status);

static union power_supply_propval battery_capacity = { 100 };
static int __rk_get_system_battery_capacity(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &battery_capacity);

	return 0;
}

int rk_get_system_battery_capacity(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __rk_get_system_battery_capacity);

	return battery_capacity.intval;
}
EXPORT_SYMBOL(rk_get_system_battery_capacity);

#ifdef CONFIG_CHARGER_DISPLAY
static void add_bootmode_charger_to_cmdline(bool charger)
{
	char *pmode=" androidboot.start=charger";
	char *new_command_line = kzalloc(strlen(saved_command_line) + strlen(pmode) + 1, GFP_KERNEL);

	if(charger)
		sprintf(new_command_line, "%s%s", saved_command_line, pmode);
	else
		sprintf(new_command_line, "%s%s", saved_command_line, " androidboot.start=no");
	saved_command_line = new_command_line;

	DBG("Kernel command line: %s\n", saved_command_line);
}

static int  __init start_charge_logo_display(void)
{
	union power_supply_propval val_status = {POWER_SUPPLY_STATUS_DISCHARGING};
	union power_supply_propval val_capacity ={ 100} ;

	DBG("start_charge_logo_display\n");

	if(rockchip_boot_mode() == BOOT_MODE_RECOVERY)  //recovery mode
	{
		DBG("recovery mode \n");
		return 0;
	}
	if (rk_get_system_battery_status() != POWER_SUPPLY_TYPE_BATTERY)
		val_status.intval = POWER_SUPPLY_STATUS_CHARGING;

	val_capacity.intval = rk_get_system_battery_capacity();
	
	// low power   and  discharging
#if 1
	if((val_capacity.intval < pwr_on_thrsd )&&(val_status.intval != POWER_SUPPLY_STATUS_CHARGING))
	{
		printk("low power 2\n");
		add_bootmode_charger_to_cmdline(1);
		//kernel_power_off();
		//while(1);
		return 0;
	}
#endif

	if(val_status.intval == POWER_SUPPLY_STATUS_CHARGING){
		if (((rockchip_boot_mode() == BOOT_MODE_NORMAL) ||(rockchip_boot_mode() == BOOT_MODE_CHARGE))) {		
					gpio_direction_output(30,0);
			  	gpio_direction_output(39,0);	
			add_bootmode_charger_to_cmdline(1);
			DBG("power in charge mode %d %d  %d\n\n",rockchip_boot_mode(),val_capacity.intval,pwr_on_thrsd);
			return 0;
	   }
	}

	add_bootmode_charger_to_cmdline(0);
	return 0;
} 

late_initcall(start_charge_logo_display);
#endif
