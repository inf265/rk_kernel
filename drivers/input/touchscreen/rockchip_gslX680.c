/*
 * drivers/input/touchscreen/gslX680.c
 *
 * Copyright (c) 2012 Shanghai Basewin
 *	Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/ioc4.h>
#include <linux/io.h>

#include <linux/fb.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <linux/input/mt.h>
#include "gslX680.h"
#define RK_GPIO_NR(port, index)		(port*32)+(index&31)

#define REPORT_DATA_ANDROID_4_0
#define SLEEP_CLEAR_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	9
#endif
static int suspend_flag = 0;
#define CONFIG_GSL_GESTURE //TPshoushi
#undef CONFIG_GSL_GESTURE //TPshoushi
#ifdef CONFIG_GSL_GESTURE 
static struct wake_lock touch_wakelock;
extern void rk_send_power_key(int state);
extern void rk_send_wakeup_key(void);
extern int gsl_obtain_gesture(void);
extern void gsl_FunIICRead(unsigned int (*ReadIICInt));
extern void gsl_GestureExternInt(unsigned int *model,int len);

static int gsl_gesture_flag = 1; 
static char gesture_key = 0;
static int gsl_lcd_flag = 0;

static int suspend_flag = 0;

#define KEY_CTL_C KEY_C//250
#define KEY_CTL_E KEY_E//251
#define KEY_CTL_W KEY_W//252
#define KEY_CTL_O KEY_O//253 
#define KEY_CTL_M KEY_M//254
#define KEY_CTL_Z KEY_Z//255
#define KEY_CTL_V KEY_V//256
#define KEY_CTL_DOUBLE_CLICK KEY_F11//252
#define KEY_CTL_RIGHT KEY_RIGHT//253 
#define KEY_CTL_DOWN KEY_DOWN//254
#define KEY_CTL_UP KEY_UP//255
#define KEY_CTL_LEFT KEY_LEFT//255
#define KEY_CTL_LESS KEY_F2
#define KEY_CTL_GREATER KEY_F3
#define KEY_CTL_RAISE KEY_F4

const u16 key_array_gesture[]={
	KEY_CTL_C, KEY_CTL_E, KEY_CTL_W, KEY_CTL_O, KEY_CTL_M, KEY_CTL_Z, KEY_CTL_V,
	KEY_CTL_DOUBLE_CLICK, KEY_CTL_RIGHT, KEY_CTL_DOWN, KEY_CTL_UP, KEY_CTL_LEFT,
};
#define MAX_GESTURE_KEY_NUM     (sizeof(key_array_gesture)/sizeof(key_array_gesture[0]))

#endif 

#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif
static struct i2c_client *gsl_client = NULL;


#define GSLX680_I2C_NAME 	"gsl2682"

#define GSL_DATA_REG		0x80
#define GSL_STATUS_REG		0xe0
#define GSL_PAGE_REG		0xf0

#define PRESS_MAX    			255
#define MAX_FINGERS 		10
#define MAX_CONTACTS 		10
#define DMA_TRANS_LEN		0x20
#ifdef GSL_MONITOR
static struct delayed_work gsl_monitor_work;
static struct workqueue_struct *gsl_monitor_workqueue = NULL;
static u8 int_1st[4] = {0};
static u8 int_2nd[4] = {0};
static char dac_counter = 0;
static char b0_counter = 0;
static char bc_counter = 0;
static char i2c_lock_flag = 0;
#endif 

static struct i2c_client *this_client = NULL;

struct ts_plat_data_t {
	unsigned int irq;
	unsigned int rst;
}*pin_data;

static unsigned int gpio_irq;
static unsigned int gpio_rst;

#define GSL_IRQ 		gpio_to_irq(gpio_irq)

#ifdef HAVE_TOUCH_KEY
static u16 key = 0;
static int key_state_flag = 0;
struct key_data {
	u16 key;
	u16 x_min;
	u16 x_max;
	u16 y_min;
	u16 y_max;	
};


const u16 key_array[]={
	KEY_BACK,
	KEY_HOME,
	KEY_MENU,
	KEY_SEARCH,
}; 
#define MAX_KEY_NUM     (sizeof(key_array)/sizeof(key_array[0]))

struct key_data gsl_key_data[MAX_KEY_NUM] = {
	{KEY_BACK, 2048, 2048, 2048, 2048},
	{KEY_HOME, 2048, 2048, 2048, 2048},	
	{KEY_MENU, 2048, 2048, 2048, 2048},
	{KEY_SEARCH, 2048, 2048, 2048, 2048},
};
#endif

struct gsl_ts_data {
	u8 x_index;
	u8 y_index;
	u8 z_index;
	u8 id_index;
	u8 touch_index;
	u8 data_reg;
	u8 status_reg;
	u8 data_size;
	u8 touch_bytes;
	u8 update_data;
	u8 touch_meta_data;
	u8 finger_size;
};

static struct gsl_ts_data devices[] = {
	{
		.x_index = 6,
		.y_index = 4,
		.z_index = 5,
		.id_index = 7,
		.data_reg = GSL_DATA_REG,
		.status_reg = GSL_STATUS_REG,
		.update_data = 0x4,
		.touch_bytes = 4,
		.touch_meta_data = 4,
		.finger_size = 70,
	},
};

struct gsl_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct gsl_ts_data *dd;
	u8 *touch_data;
	u8 device_id;
	int irq;
};

static struct gsl_ts *g_gsl_ts;


#define GSL_DEBUG 

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
	do{                              \
		printk(fmt, ##args);     \
	}while(0)
#else
#define print_info(fmt, args...)
#endif


static u32 id_sign[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;

#ifdef CONFIG_GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,
		unsigned int addr, unsigned int len)
{
	int i;
	u8 buf[4];
	struct gsl_ts *ts = g_gsl_ts;

	if (ts == NULL || data == NULL) {
		printk(KERN_ERR"[GSL]: Invalid arguments.\n");
		return 0;
	}

	print_info("tp-gsl-gesture %s\n",__func__);

	for (i = 0; i < len / 2; i++) {
		buf[0] = ((addr+i*8)/0x80)&0xff;
		buf[1] = (((addr+i*8)/0x80)>>8)&0xff;
		buf[2] = (((addr+i*8)/0x80)>>16)&0xff;
		buf[3] = (((addr+i*8)/0x80)>>24)&0xff;
		i2c_smbus_write_i2c_block_data(ts->client,0xf0,4,buf);
		i2c_smbus_read_i2c_block_data(ts->client,(addr+i*8)%0x80,8,(char *)&data[i*2]);
		i2c_smbus_read_i2c_block_data(ts->client,(addr+i*8)%0x80,8,(char *)&data[i*2]);
	}

	if (len % 2) {
		buf[0] = ((addr+len*4 - 4)/0x80)&0xff;
		buf[1] = (((addr+len*4 - 4)/0x80)>>8)&0xff;
		buf[2] = (((addr+len*4 - 4)/0x80)>>16)&0xff;
		buf[3] = (((addr+len*4 - 4)/0x80)>>24)&0xff;
		i2c_smbus_write_i2c_block_data(ts->client,0xf0,4,buf);
		i2c_smbus_read_i2c_block_data(ts->client,(addr+len*4 - 4)%0x80,4,(char *)&data[len-1]);
		i2c_smbus_read_i2c_block_data(ts->client,(addr+len*4 - 4)%0x80,4,(char *)&data[len-1]);
	}

	for (i = 0; i < len; i++)
		print_info("data[%d]:0x%x\n",i, data[i]);

	return len;
}

static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	ssize_t len=0;
	sprintf(&buf[len],"%s\n","tp gesture is on/off:");
	len += (strlen("tp gesture is on/off:")+1);
	if(gsl_gesture_flag == 1){
		sprintf(&buf[len],"%s\n","  on  ");
		len += (strlen("  on  ")+1);
	}else if(gsl_gesture_flag == 0){
		sprintf(&buf[len],"%s\n","  off  ");
		len += (strlen("  off  ")+1);
	}	
	return len;
}
//wuhao start
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	char tmp_buf[16];

	if(copy_from_user(tmp_buf, buf, (count>16?16:count))){
		return -1;
	}
	if(buf[0] == '0'){
		gsl_gesture_flag = 0;
		print_info("[GSL_GESTURE] gsl_sysfs_tpgesturet_store off.\n");
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
		print_info("[GSL_GESTURE] gsl_sysfs_tpgesturet_store on.\n");
	}

	return count;
}
static DEVICE_ATTR(tpgesture, S_IRUGO|S_IWUSR, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);

static unsigned int gsl_gesture_init(void)
{
	int ret;
	struct kobject *gsl_debug_kobj;

	gsl_debug_kobj = kobject_create_and_add("gsl_gesture", NULL) ;
	if (gsl_debug_kobj == NULL) {
		printk("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_tpgesture.attr);
	if (ret) {
		printk("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	print_info("[GSL_GESTURE] gsl_gesture_init success.\n");

	return 1;
}

#endif 


static int gslX680_gpio_init(void)
{
	if (gpio_request(gpio_rst,"gsl-wake-port") != 0){
		gpio_free(gpio_rst);
		return -EIO;
	}
	gpio_direction_output(gpio_rst, 0);
	gpio_set_value(gpio_rst, 1);

	if (gpio_request(gpio_irq,"gsl-irq-port") != 0){
		gpio_free(gpio_irq);
		return -EIO;
	}
	gpio_direction_input(gpio_irq);

	return 0;
}

static void gslX680_gpio_free(void)
{
	gpio_free(gpio_rst);
	gpio_free(gpio_irq);
}

static int gslX680_shutdown_low(void)
{
	gpio_direction_output(gpio_rst, 0);
	gpio_set_value(gpio_rst, 0);
	return 0;
}

static int gslX680_shutdown_high(void)
{
	gpio_direction_output(gpio_rst, 1);
	gpio_set_value(gpio_rst, 1);
	return 0;
}

static inline u16 join_bytes(u8 a, u8 b)
{
	u16 ab = 0;
	ab = ab | a;
	ab = ab << 8 | b;
	return ab;
}
#if 0
static u32 gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;
#ifdef CONFIG_I2C_ROCKCHIP_COMPAT
	xfer_msg[0].scl_rate = 100*1000;
#endif

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags |= I2C_M_RD;
	xfer_msg[1].buf = buf;
#ifdef CONFIG_I2C_ROCKCHIP_COMPAT
	xfer_msg[1].scl_rate = 100*1000;
#endif

	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		msleep(5);
	}

	return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}
#endif
static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;
#ifdef CONFIG_I2C_ROCKCHIP_COMPAT
	xfer_msg[0].scl_rate = 100*1000;
#endif
	//xfer_msg[0].scl_rate = 400*1000; //RK3066 RK2926 I2C±\u0161\u017díÊ±\u017dò¿ªÕâ\u017eö

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;
	if (datalen > 125)
	{
		return -1;
	}

	tmp_buf[0] = addr;
	bytelen++;

	if (datalen != 0 && pdata != NULL)
	{
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}

	ret = i2c_master_send(client, tmp_buf, bytelen);
	return ret;
}

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen)
{
	int ret = 0;

	if (datalen > 126)
	{
		return -1;
	}

	ret = gsl_ts_write(client, addr, NULL, 0);
	if (ret < 0)
	{
		return ret;
	}

	return i2c_master_recv(client, pdata, datalen);
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len;
	const struct fw_data *ptr_fw;


	ptr_fw = GSLX680_FW;
	source_len = ARRAY_SIZE(GSLX680_FW);

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == ptr_fw[source_line].offset)
		{
			fw2buf(cur, &ptr_fw[source_line].val);
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
				buf[0] = (u8)ptr_fw[source_line].offset;

			fw2buf(cur, &ptr_fw[source_line].val);
			cur += 4;

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
				gsl_write_interface(client, buf[0], buf, cur - buf - 1);
				cur = buf + 1;
			}

			send_flag++;
		}
	}


}


static int test_i2c(struct i2c_client *client)
{
	u8 read_buf = 0;
	u8 write_buf = 0x12;
	int ret, rc = 1;

	ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
	if  (ret  < 0)  
		rc --;

	msleep(2);
	ret = gsl_ts_write(client, 0xf0, &write_buf, sizeof(write_buf));

	msleep(2);
	ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
	if(ret <  0 )
		rc --;

	return rc;
}


static void startup_chip(struct i2c_client *client)
{
	u8 tmp = 0x00;

#ifdef GSL_NOID_VERSION
	gsl_DataInit(gsl_config_data_id);
#endif
	gsl_ts_write(client, 0xe0, &tmp, 1);
	msleep(10);	
}

static void reset_chip(struct i2c_client *client)
{
	u8 tmp = 0x88;
	u8 buf[4] = {0x00};

	gsl_ts_write(client, 0xe0, &tmp, sizeof(tmp));
	msleep(20);
	tmp = 0x04;
	gsl_ts_write(client, 0xe4, &tmp, sizeof(tmp));
	msleep(10);
	gsl_ts_write(client, 0xbc, buf, sizeof(buf));
	msleep(10);
}

static void clr_reg(struct i2c_client *client)
{
	u8 write_buf[4]	= {0};

	write_buf[0] = 0x88;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1); 	
	msleep(20);
	write_buf[0] = 0x03;
	gsl_ts_write(client, 0x80, &write_buf[0], 1); 	
	msleep(5);
	write_buf[0] = 0x04;
	gsl_ts_write(client, 0xe4, &write_buf[0], 1); 	
	msleep(5);
	write_buf[0] = 0x00;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1); 	
	msleep(20);
}

static int init_chip(struct i2c_client *client)
{
	int rc;

	gslX680_shutdown_low();	
	msleep(20);
	gslX680_shutdown_high();
	msleep(20);

	rc = test_i2c(client);
	if (rc < 0) {
		return rc;
	}

	clr_reg(client);

	reset_chip(client);
	gsl_load_fw(client);
	startup_chip(client);

	reset_chip(client);
	startup_chip(client);

	return 0;
}

static void check_mem_data(struct i2c_client *client)
{
	u8 read_buf[4]  = {0};

	msleep(30);
	gsl_ts_read(client, 0xb0, read_buf, sizeof(read_buf));
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a) {
		init_chip(client);
	}
}



#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;
	u16 filter_step_x = 0, filter_step_y = 0;

	id_sign[id] = id_sign[id] + 1;
	if(id_sign[id] == 1)
	{
		x_old[id] = x;
		y_old[id] = y;
	}

	x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
	y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);

	if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
	{
		filter_step_x = x_err;
		filter_step_y = y_err;
	}
	else
	{
		if(x_err > FILTER_MAX)
			filter_step_x = x_err; 
		if(y_err> FILTER_MAX)
			filter_step_y = y_err;
	}

	if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
	{
		filter_step_x >>= 2; 
		filter_step_y >>= 2;
	}
	else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
	{
		filter_step_x >>= 1; 
		filter_step_y >>= 1;
	}	
	else if(x_err <= 4*FILTER_MAX && y_err <= 4*FILTER_MAX)
	{
		filter_step_x = filter_step_x*3/4; 
		filter_step_y = filter_step_y*3/4;
	}	

	x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
	y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);

	x_old[id] = x_new;
	y_old[id] = y_new;
}
#else
static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;

	id_sign[id]=id_sign[id]+1;

	if(id_sign[id]==1){
		x_old[id]=x;
		y_old[id]=y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;

	if(x>x_old[id]){
		x_err=x -x_old[id];
	}
	else{
		x_err=x_old[id]-x;
	}

	if(y>y_old[id]){
		y_err=y -y_old[id];
	}
	else{
		y_err=y_old[id]-y;
	}

	if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
		x_new = x;     x_old[id] = x;
		y_new = y;     y_old[id] = y;
	}
	else{
		if(x_err > 3){
			x_new = x;     x_old[id] = x;
		}
		else
			x_new = x_old[id];
		if(y_err> 3){
			y_new = y;     y_old[id] = y;
		}
		else
			y_new = y_old[id];
	}

	if(id_sign[id]==1){
		x_new= x_old[id];
		y_new= y_old[id];
	}

}
#endif

#ifdef HAVE_TOUCH_KEY
static void report_key(struct gsl_ts *ts, u16 x, u16 y)
{
	u16 i = 0;

	for(i = 0; i < MAX_KEY_NUM; i++) 
	{
		if((gsl_key_data[i].x_min < x) && (x < gsl_key_data[i].x_max)&&(gsl_key_data[i].y_min < y) && (y < gsl_key_data[i].y_max))
		{
			key = gsl_key_data[i].key;	
			input_report_key(ts->input, key, 1);
			input_sync(ts->input); 		
			key_state_flag = 1;
			break;
		}
	}
}
#endif

static void report_data(struct gsl_ts *ts, u16 x, u16 y, u8 pressure, u8 id)
{	
	print_info("#####id=%d,x=%d,y=%d######\n",id,x,y);
#if 1
	x = SCREEN_MAX_X - x;
	y = SCREEN_MAX_Y - y;
	//swap(x,y);
#else
	swap(x,y);
	if(x > SCREEN_MAX_X || y > SCREEN_MAX_Y)
	{
#ifdef HAVE_TOUCH_KEY
		report_key(ts,x,y);
#endif
		return;
	}

	y = (y*5)>>2;
	x = (x<<5)/25;

	x = SCREEN_MAX_X - x;
	y = SCREEN_MAX_Y - y;
	print_info("#####id=%d,x=%d,y=%d######\n",id,x,y);
#endif
#ifdef REPORT_DATA_ANDROID_4_0
	input_mt_slot(ts->input, id);		
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, pressure);
	input_report_abs(ts->input, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);	
	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 1);
#else
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, pressure);
	input_report_abs(ts->input, ABS_MT_POSITION_X,x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 1);
	input_mt_sync(ts->input);
#endif
}

static void gslX680_ts_worker(struct work_struct *work)
{
	int rc, i;
	u8 id, touches;
	u16 x, y;
#ifdef GSL_NOID_VERSION
	u32 tmp1;
	u8 buf[4] = {0};
	struct gsl_touch_info cinfo;
#endif

	struct gsl_ts *ts = container_of(work, struct gsl_ts,work);

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1)
		goto schedule;
#endif		 

#ifdef GSL_MONITOR
	if(i2c_lock_flag != 0)
		goto i2c_lock_schedule;
	else
		i2c_lock_flag = 1;
#endif


	rc = gsl_ts_read(ts->client, 0x80, ts->touch_data, ts->dd->data_size);
	if (rc < 0) 
	{
		dev_err(&ts->client->dev, "read failed\n");
		goto schedule;
	}

	touches = ts->touch_data[ts->dd->touch_index];
	print_info("-----touches: %d -----\n", touches);		
#ifdef GSL_NOID_VERSION
	cinfo.finger_num = touches;
	print_info("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(i = 0; i < (touches < MAX_CONTACTS ? touches : MAX_CONTACTS); i ++)
	{
		cinfo.x[i] = join_bytes( ( ts->touch_data[ts->dd->x_index  + 4 * i + 1] & 0xf),
				ts->touch_data[ts->dd->x_index + 4 * i]);
		cinfo.y[i] = join_bytes(ts->touch_data[ts->dd->y_index + 4 * i + 1],
				ts->touch_data[ts->dd->y_index + 4 * i ]);
		cinfo.id[i] = ((ts->touch_data[ts->dd->x_index  + 4 * i + 1] & 0xf0)>>4);
		print_info("tp-gsl  x = %d y = %d \n",cinfo.x[i],cinfo.y[i]);
	}
	cinfo.finger_num=(ts->touch_data[3]<<24)|(ts->touch_data[2]<<16)
		|(ts->touch_data[1]<<8)|(ts->touch_data[0]);
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
		gsl_ts_write(ts->client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
				tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_ts_write(ts->client,0x8,buf,4);
	}
	touches = cinfo.finger_num;
#endif
#ifdef CONFIG_GSL_GESTURE 
	//if(tpd_halt == 1 && gsl_gesture_flag == 1){ 
	if(suspend_flag && gsl_gesture_flag) {
		int tmp_c; 
		unsigned int report_key = 0;

		tmp_c = gsl_obtain_gesture(); 
		print_info("[GSL_GESTURE] tmp_c =%x\n",tmp_c); 
		print_info("[GSL_GESTURE] c format tmp_c =%c\n",tmp_c);
		switch(tmp_c)
		{
			case (int)'C':
				report_key = KEY_CTL_C;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break ;
			case (int)'E':
				report_key = KEY_CTL_E;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break ;
			case (int)'W':
				report_key = KEY_CTL_W;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break ;
			case (int)'O':
				report_key = KEY_CTL_O;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break ;
			case (int)'M':
				report_key = KEY_CTL_M;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break ;
			case (int)'Z':
				report_key = KEY_CTL_Z;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break ;
			case (int)'V':
				report_key = KEY_CTL_V;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break ;
			case (int)'*': /*double click*/
				report_key = KEY_CTL_DOUBLE_CLICK;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;	
			case (int)0xa1fa: /* right */
				tmp_c = 0;
				//tmp_c = 'R';
				report_key = KEY_CTL_RIGHT;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;		
			case (int)0xa1fd: /* down */
				tmp_c = 0;
				//tmp_c = 'D';
				report_key = KEY_CTL_DOWN;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;	
			case (int)0xa1fc: /* up */
				tmp_c = 0;
				//tmp_c = 'U';
				report_key = KEY_CTL_UP;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;	
			case (int)0xa1fb: /* left */
				tmp_c = 0;	
				//tmp_c = 'L';	
				report_key = KEY_CTL_LEFT;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;
			case (int)'<':
				tmp_c = 0;
				//report_key = KEY_CTL_LESS;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;
			case (int)'>':
				tmp_c = 0;
				//report_key = KEY_CTL_GREATER;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;
			case 4097:
				tmp_c = 0;
				report_key = KEY_CTL_RAISE;
				print_info("[GSL_GESTURE] report_key =%d \n",report_key);
				break;	
			default:
				print_info("default == [GSL_GESTURE] report_key =%d \n",report_key);
				tmp_c = 0;
				break;
		}
		gesture_key = (char)tmp_c;

		if(gesture_key > 0 && suspend_flag)
		{			    
			printk(KERN_ERR"[GSL_GESTURE] input report KEY_POWER\n");
			rk_send_wakeup_key();
			//rk_send_power_key(1);
			//rk_send_power_key(0);
			msleep(300);
			input_report_key(ts->input,report_key/*KEY_POWER*/,1);
			input_sync(ts->input);
			//udelay(10);
			//msleep(300);
			input_report_key(ts->input,report_key/*KEY_POWER*/,0);
			input_sync(ts->input);
			//rk_send_power_key(1);
			//rk_send_power_key(0);
		}

		return ;//goto schedule;

	} 
#endif

	for(i = 1; i <= MAX_CONTACTS; i ++)
	{
		if(touches == 0)
			id_sign[i] = 0;	
		id_state_flag[i] = 0;
	}
	for(i= 0;i < (touches > MAX_FINGERS ? MAX_FINGERS : touches);i ++)
	{
#ifdef GSL_NOID_VERSION
		id = cinfo.id[i];
		x =  cinfo.x[i];
		y =  cinfo.y[i];	
#else
		x = join_bytes( ( ts->touch_data[ts->dd->x_index  + 4 * i + 1] & 0xf),
				ts->touch_data[ts->dd->x_index + 4 * i]);
		y = join_bytes(ts->touch_data[ts->dd->y_index + 4 * i + 1],
				ts->touch_data[ts->dd->y_index + 4 * i ]);
		id = ts->touch_data[ts->dd->id_index + 4 * i] >> 4;
#endif

		if(1 <=id && id <= MAX_CONTACTS)
		{
#ifdef FILTER_POINT
			filter_point(x, y ,id);
#else
			record_point(x, y , id);
#endif
			report_data(ts, x_new, y_new, 10, id);		
			id_state_flag[id] = 1;
		}
	}
	for(i = 1; i <= MAX_CONTACTS; i ++)
	{	
		if( (0 == touches) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
		{
#ifdef REPORT_DATA_ANDROID_4_0
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
#endif
			id_sign[i]=0;
		}
		id_state_old_flag[i] = id_state_flag[i];
	}
#ifndef REPORT_DATA_ANDROID_4_0
	if(0 == touches)
	{	
		input_mt_sync(ts->input);
#ifdef HAVE_TOUCH_KEY
		if(key_state_flag)
		{
			input_report_key(ts->input, key, 0);
			input_sync(ts->input);
			key_state_flag = 0;
		}
#endif			
	}
#endif
	input_sync(ts->input);

schedule:
#ifdef GSL_MONITOR
	i2c_lock_flag = 0;
i2c_lock_schedule:
#endif
	enable_irq(ts->irq);

}

#ifdef GSL_MONITOR
static void gsl_monitor_worker(void)
{
	u8 write_buf[4] = {0};
	u8 read_buf[4]  = {0};
	char init_chip_flag = 0;

	print_info("----------------gsl_monitor_worker-----------------\n");	

	if(i2c_lock_flag != 0)
		goto queue_monitor_work;
	else
		i2c_lock_flag = 1;

	gsl_ts_read(gsl_client, 0xb0, read_buf, 4);
	if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter ++;
	else
		b0_counter = 0;

	if(b0_counter > 1)
	{
		init_chip_flag = 1;
		b0_counter = 0;
		goto queue_monitor_init_chip;
	}

	gsl_ts_read(gsl_client, 0xb4, read_buf, 4);	
	int_2nd[3] = int_1st[3];
	int_2nd[2] = int_1st[2];
	int_2nd[1] = int_1st[1];
	int_2nd[0] = int_1st[0];
	int_1st[3] = read_buf[3];
	int_1st[2] = read_buf[2];
	int_1st[1] = read_buf[1];
	int_1st[0] = read_buf[0];

	if(int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0]) 
	{
		init_chip_flag = 1;
		goto queue_monitor_init_chip;
	}

	gsl_ts_read(gsl_client, 0xbc, read_buf, 4);
	if(read_buf[3] != 0 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
		bc_counter++;
	else
		bc_counter = 0;
	if(bc_counter > 1)
	{
		init_chip_flag = 1;
		bc_counter = 0;
	}
queue_monitor_init_chip:
	if(init_chip_flag)
		init_chip(gsl_client);

	i2c_lock_flag = 0;

queue_monitor_work:	
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 100);
}
#endif

static irqreturn_t gsl_ts_irq(int irq, void *dev_id)
{	
	struct gsl_ts *ts = dev_id;			 

	disable_irq_nosync(ts->irq);

	if (!work_pending(&ts->work)) 
	{
#ifdef CONFIG_GSL_GESTURE
		if(suspend_flag) {
			wake_lock_timeout(&touch_wakelock,msecs_to_jiffies(3000));
			//queue_delayed_work(ts->wq, &ts->work, msecs_to_jiffies(20));
		} //else
#endif
		queue_work(ts->wq, &ts->work);
	}

	return IRQ_HANDLED;

}

static int gslX680_ts_init(struct i2c_client *client, struct gsl_ts *ts)
{
	int  rc = 0;
	//int j = 0;
	struct input_dev *input_device;

	ts->dd = &devices[ts->device_id];

	if (ts->device_id == 0) {
		ts->dd->data_size = MAX_FINGERS * ts->dd->touch_bytes + ts->dd->touch_meta_data;
		ts->dd->touch_index = 0;
	}

	ts->touch_data = kzalloc(ts->dd->data_size, GFP_KERNEL);
	if (!ts->touch_data) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}

	input_device = input_allocate_device();
	if (!input_device) {
		rc = -ENOMEM;
		goto error_alloc_dev;
	}

	ts->input = input_device;
	input_device->name = GSLX680_I2C_NAME;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

#ifdef REPORT_DATA_ANDROID_4_0
	__set_bit(EV_ABS, input_device->evbit);
	__set_bit(EV_KEY, input_device->evbit);
	__set_bit(EV_REP, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_mt_init_slots(input_device, (MAX_CONTACTS+1),0);
#else
	input_set_abs_params(input_device,ABS_MT_TRACKING_ID, 0, (MAX_CONTACTS+1), 0, 0);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_device->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#ifdef HAVE_TOUCH_KEY
	input_device->evbit[0] = BIT_MASK(EV_KEY);
	//input_device->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	for (i = 0; i < MAX_KEY_NUM; i++)
		set_bit(key_array[i], input_device->keybit);
#endif

#ifdef CONFIG_GSL_GESTURE
	for (j = 0; j < MAX_GESTURE_KEY_NUM; j++)
		set_bit(key_array_gesture[j], input_device->keybit);
#endif
	set_bit(ABS_MT_POSITION_X, input_device->absbit);
	set_bit(ABS_MT_POSITION_Y, input_device->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_device->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_device->absbit);

	input_set_abs_params(input_device,ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_device,ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_device,ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_device,ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	client->irq = GSL_IRQ;
	ts->irq = client->irq;

	ts->wq = create_singlethread_workqueue("kworkqueue_ts");
	if (!ts->wq) {
		dev_err(&client->dev, "Could not create workqueue\n");
		goto error_wq_create;
	}
	flush_workqueue(ts->wq);	

	INIT_WORK(&ts->work, gslX680_ts_worker);

	rc = input_register_device(input_device);
	if (rc)
		goto error_unreg_device;

	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
error_alloc_dev:
	kfree(ts->touch_data);
	return rc;
}

#ifdef CONFIG_PM_SLEEP
static int gsl_ts_suspend(struct device *dev)
{
#if 0
    struct gsl_ts *ts = dev_get_drvdata(dev);
    int i;
    printk("Lamson-->%s[%d]\n",__func__,__LINE__);

#ifdef GSL_MONITOR
    cancel_delayed_work_sync(&gsl_monitor_work);
#endif

    disable_irq_nosync(ts->irq);	

    gslX680_shutdown_low();

#ifdef SLEEP_CLEAR_POINT
    msleep(10); 		
#ifdef REPORT_DATA_ANDROID_4_0
    for(i = 1; i <= MAX_CONTACTS ;i ++)
    {	
	input_mt_slot(ts->input, i);
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
    }
#else	
    input_mt_sync(ts->input);
#endif
    input_sync(ts->input);
    msleep(10); 	
    report_data(ts, 1, 1, 10, 1);		
    input_sync(ts->input);	
#endif	
#endif

	return 0;
}

static int gsl_ts_resume(struct device *dev)
{
#if 0
    struct gsl_ts *ts = dev_get_drvdata(dev);
    int i;

    printk("Lamson-->%s[%d]\n",__func__,__LINE__);
    gslX680_shutdown_high();
    msleep(20); 	
    reset_chip(ts->client);
    startup_chip(ts->client);
    check_mem_data(ts->client);

#ifdef SLEEP_CLEAR_POINT
#ifdef REPORT_DATA_ANDROID_4_0
    for(i =1;i<=MAX_CONTACTS;i++)
    {	
	input_mt_slot(ts->input, i);
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
    }
#else	
    input_mt_sync(ts->input);
#endif
    input_sync(ts->input);	
#endif
#ifdef GSL_MONITOR
    queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
#endif	

    enable_irq(ts->irq);
#endif
	return 0;
}
#endif

static void gsl_ts_early_suspend(void)
{
	if (!suspend_flag) {
		gsl_ts_suspend(&g_gsl_ts->client->dev);
	}
}

static void gsl_ts_late_resume(void)
{
	if (suspend_flag) {
		gsl_ts_resume(&g_gsl_ts->client->dev);
	}
}

static ssize_t set_tp_rst(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	init_chip(gsl_client);

	return count;
}

static DEVICE_ATTR(tp_rst, S_IWUSR, NULL, set_tp_rst);

static struct attribute *tp_ctrl_attributes[] = {
	&dev_attr_tp_rst.attr,
	NULL,
};

static const struct attribute_group tp_ctrl_attr_group = {
	.attrs = tp_ctrl_attributes,
};

static int gsl_fb_event_notify(struct notifier_block *self,
		unsigned long action, void *data)
{

	struct fb_event *event = data;
	int blank_mode = *((int *)event->data);

	switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			gsl_ts_late_resume();
			break;
		case FB_BLANK_NORMAL:
			gsl_ts_early_suspend();
			break;
		default:
			gsl_ts_early_suspend();
			break;
	}

	return 0;
}

static struct notifier_block gsl_fb_notifier = {
	.notifier_call = gsl_fb_event_notify,
};

static int  gsl_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct gsl_ts *ts;
	int rc;
	struct device_node *node = NULL;
	enum of_gpio_flags rst_flags;
	unsigned long irq_flags;

	node = of_find_node_by_name(NULL, "gslX680_tp");
	gpio_irq = of_get_named_gpio_flags(node, "touch-gpio", 0, (enum of_gpio_flags *)&irq_flags);
	gpio_rst = of_get_named_gpio_flags(node, "reset-gpio", 0, &rst_flags);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts){
		return -ENOMEM;
	}

	ts->client = client;
	this_client = client;
	i2c_set_clientdata(client, ts);
	ts->device_id = id->driver_data;

#ifdef CONFIG_GSL_GESTURE
	wake_lock_init(&touch_wakelock, WAKE_LOCK_SUSPEND, "touch");
	//gsl_GestureExtern(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);
	gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);
	gsl_FunIICRead(gsl_read_oneframe_data);
	gsl_gesture_init();
#endif 
	rc = gslX680_ts_init(client, ts);
	if (rc < 0) {
		dev_err(&client->dev, "GSLX680 init failed\n");
		goto error_mutex_destroy;
	}	

	gsl_client = client;

	rc = gslX680_gpio_init();    	
	if (rc < 0) {
		gslX680_gpio_free();
		return -EIO;
	}

	rc = init_chip(ts->client);
	if (rc < 0) {
		gslX680_gpio_free();
		return rc;
	}

	check_mem_data(ts->client);

	rc = request_irq(client->irq, gsl_ts_irq, IRQF_TRIGGER_RISING, client->name, ts);
	if (rc < 0) {
		goto error_req_irq_fail;
	}

#ifdef CONFIG_GSL_GESTURE
	enable_irq_wake(client->irq);
#endif


#ifdef GSL_MONITOR

	INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
	gsl_monitor_workqueue = create_singlethread_workqueue("gsl_monitor_workqueue");
	queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1000);
#endif

	rc = sysfs_create_group(&client->dev.kobj, &tp_ctrl_attr_group);
	if (rc)
		dev_err(&client->dev, "could not create sysfs files\n");

	g_gsl_ts  = ts;

	fb_register_client(&gsl_fb_notifier);

	return 0;

	//exit_set_irq_mode:	
error_req_irq_fail:
	free_irq(ts->irq, ts);	

error_mutex_destroy:
	input_free_device(ts->input);
	kfree(ts);
	return rc;
}

static int  gsl_ts_remove(struct i2c_client *client)
{
	struct gsl_ts *ts = i2c_get_clientdata(client);

	fb_unregister_client(&gsl_fb_notifier);

#ifdef GSL_MONITOR
	cancel_delayed_work_sync(&gsl_monitor_work);
	destroy_workqueue(gsl_monitor_workqueue);
#endif

	device_init_wakeup(&client->dev, 0);
	cancel_work_sync(&ts->work);
	free_irq(ts->irq, ts);
	destroy_workqueue(ts->wq);
	input_unregister_device(ts->input);
	//device_remove_file(&ts->input->dev, &dev_attr_debug_enable);

	kfree(ts->touch_data);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id gsl_ts_id[] = {
	{GSLX680_I2C_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gsl_ts_id);

static struct of_device_id gslX680_ts_dt_ids[] = {
	{ .compatible = "gslX680" },
	{ /* sentinel */ }
};

static SIMPLE_DEV_PM_OPS(gsl_ts_pm_ops, gsl_ts_suspend, gsl_ts_resume);
static struct i2c_driver gsl_ts_driver = { 
	.driver = { 
		.name = "gsl_ts",
		.owner = THIS_MODULE,
		.pm = &gsl_ts_pm_ops,
		.of_match_table	= of_match_ptr(gslX680_ts_dt_ids),
	},
	.probe = gsl_ts_probe,
	.remove = gsl_ts_remove,
	.id_table = gsl_ts_id,
};


static int __init gsl_ts_init(void)
{
	return i2c_add_driver(&gsl_ts_driver);
}

static void __exit gsl_ts_exit(void)
{
	i2c_del_driver(&gsl_ts_driver);
}

module_init(gsl_ts_init);
module_exit(gsl_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GSLX680 touchscreen controller driver");
MODULE_AUTHOR("Guan Yuwei, guanyuwei@basewin.com");
MODULE_ALIAS("platform:gsl_ts");
