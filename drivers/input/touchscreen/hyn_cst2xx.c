/* 
 * drivers/input/touchscreen/hyn_cst2xx.c
 *
 * hynitron TouchScreen driver. 
 *
 * Copyright (c) 2015  hynitron
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *  1.0		    2015-10-12		    Tim
 *
 * note: only support mulititouch
 */ 

#include <linux/module.h>
#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
//#include <mach/gpio.h>
#include <linux/irq.h>
//#include <mach/board.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>

//#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include "tp_suspend.h"

#define HYN_DEBUG
#if defined (CONFIG_KP_AXP)
        extern int axp_gpio_set_value(int gpio, int io_state);
        extern int axp_gpio_set_io(int , int );
#if defined(CONFIG_BOARD_TYPE_ZM726CE_V12) 
        #define PMU_GPIO_NUM    3 
#endif
#endif

#if defined(CONFIG_TP_1680E_726_SD)
    //#define Y_POL
	//#define X_POL
	//#define SWAP_X_Y
	#define SCREEN_MAX_X 		1024
	#define SCREEN_MAX_Y 		600
    //#include "CST21680_F_WGJ10276.h"
        
#else
    #define Y_POL
  	#define X_POL
	//#define SWAP_X_Y
	#define SCREEN_MAX_X 		720
	#define SCREEN_MAX_Y 		1280
    #include "CST21680_F_WGJ10276.h"
#endif
	
#define ICS_SLOT_REPORT
//#define HAVE_TOUCH_KEY
#define SLEEP_CLEAR_POINT

#define CST2XX_I2C_NAME 	"cst2xx"
#define CST2XX_I2C_ADDR 	0x1A

//#define IRQ_PORT			RK2928_PIN1_PB0//RK30_PIN1_PB7
//#define WAKE_PORT			RK30_PIN0_PA1//RK30_PIN0_PB6

//#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
//static struct proc_dir_entry *hyn_config_proc = NULL;
#define HYN_CONFIG_PROC_FILE "hyn_config"
#define CONFIG_LEN 31
//static char hyn_read[CONFIG_LEN];
static u8 hyn_data_proc[8] = {0};
static u8 hyn_proc_flag = 0;
//static struct i2c_client *ts->client = NULL;
#endif

#define TRANSACTION_LENGTH_LIMITED
//#define HYN_MONITOR
#define PRESS_MAX    		255
#define MAX_FINGERS 		10
#define MAX_CONTACTS 		10
#define DMA_TRANS_LEN		0x20

#ifdef HYN_MONITOR
static struct workqueue_struct *hyn_monitor_workqueue = NULL;
static u8 int_1st[4] = {0};
static u8 int_2nd[4] = {0};
//static char dac_counter = 0;
static char b0_counter = 0;
static char bc_counter = 0;
static char i2c_lock_flag = 0;
#endif 

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

struct key_data hyn_key_data[MAX_KEY_NUM] = {
	{KEY_BACK, 2048, 2048, 2048, 2048},
	{KEY_HOME, 2048, 2048, 2048, 2048},	
	{KEY_MENU, 2048, 2048, 2048, 2048},
	{KEY_SEARCH, 2048, 2048, 2048, 2048},
};
#endif

struct hyn_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct hyn_ts_data *dd;
	u8 *touch_data;
	u8 device_id;
	int irq;
	int rst_pin;
	int irq_pin;
    struct delayed_work hyn_monitor_work;
	
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	struct  tp_device  tp;
};
int h_wake_pin = 0;

#ifdef HYN_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)
#endif

#if 0
static int cst2xx_init(struct hyn_ts *ts)
{
	struct device_node *np = ts->client->dev.of_node;
	enum of_gpio_flags rst_flags;
	unsigned long irq_flags;
	
	ts->irq = of_get_named_gpio_flags(np, "touch-gpio", 0, (enum of_gpio_flags *)&irq_flags);
	ts->rst = of_get_named_gpio_flags(np, "reset-gpio", 0, &rst_flags);
		
	//msleep(20);
	#if defined (CONFIG_BOARD_ZM71C)||defined (CONFIG_BOARD_ZM72CP)||defined (CONFIG_BOARD_ZM726C)||defined (CONFIG_BOARD_ZM726CE) || defined(CONFIG_BOARD_TYPE_ZM726CE_CK)||defined (CONFIG_BOARD_ZM72CP_NEW)
	        if(gpio_request(ts->rst,NULL) != 0){
               gpio_free(ts->rst);
              printk("cst2xx_init gpio_request error\n");
             return -EIO;
      }      
	#endif
	gpio_direction_output(ts->rst, 1);
	
	/*if(gpio_request(WAKE_PORT,NULL) != 0){
		gpio_free(WAKE_PORT);
		printk("zhongchu_init_platform_hw gpio_request error\n");
		return -EIO;
	}
	gpio_direction_output(WAKE_PORT, 0);
	gpio_set_value(WAKE_PORT,GPIO_HIGH);
	msleep(20);

	if(gpio_request(IRQ_PORT,NULL) != 0){
		gpio_free(IRQ_PORT);
		printk("zhongchu_init_platform_hw gpio_request error\n");
		return -EIO;
	}
	gpio_pull_updown(IRQ_PORT, 1);

	msleep(20);*/
	msleep(20);
	
	return 0;
}
#endif

/*static u32 hyn_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;
xfer_msg[0].scl_rate=300*1000;
  
	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags |= I2C_M_RD;
	xfer_msg[1].buf = buf;
xfer_msg[1].scl_rate=300*1000;
  
	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		msleep(5);
	}

	return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}*/

#if 0
static u32 hyn_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;
	xfer_msg[0].scl_rate=300*1000;

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}
#endif

static int cst2xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 
	
    //client->timing  = 370;
    //client->addr   |= I2C_ENEXT_FLAG;

	while(retries < 1) { 
		ret = i2c_master_recv(client, buf, len); 
		if(ret<=0) 
		    retries++;
        else
            break; 
	} 
	
	return ret; 
} 

static int cst2xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 
	
    //client->timing  = 370;
    //client->addr   |= I2C_ENEXT_FLAG;

	while(retries < 1) { 
		ret = i2c_master_send(client, buf, len); 
		if(ret<=0) 
		    retries++;
        else
            break; 
	} 
	
	return ret; 
}

static int cst2xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
    
    ret = cst2xx_i2c_write(client, buf, 2);

    ret = cst2xx_i2c_read(client, buf, len);
	
    return ret; 
}

static int cst2xx_test_i2c(struct i2c_client *client)
{
	u8 retry = 0;
	u8 ret;
	u8 buf[4];

	buf[0] = 0xD1;
	buf[1] = 0x06;
	while(retry++ < 5) {
		ret = cst2xx_i2c_write(client, buf, 2);
		if (ret > 0) 
			return ret;
		
		mdelay(2);		
	}

    if(retry==5) printk("hyn iic test error.ret:%d.\n", ret);
	
	return ret;
}

#if 0
static void soft_reset_chip(struct i2c_client *client)
{
	u8 buf[4];

	//printk("hyn software reset_chip\n");
	
	buf[0] = 0xD1;
	buf[1] = 0x02;
	cst2xx_i2c_write(client, buf, 2);
	
	msleep(50);
}
#endif

static void hard_reset_chip(struct hyn_ts *ts, u16 ms)
{
	 if(h_wake_pin != 0) {
         gpio_direction_output(ts->rst_pin, 0);
    	 msleep(10);
    	 gpio_direction_output(ts->rst_pin, 1);
	 }
	 
     msleep(ms); 
}


#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
    if(ch>='0' && ch<='9')
        return (ch-'0');
    else
        return (ch-'a'+10);
}

/*static int hyn_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *ptr = page;
	char temp_data[5] = {0};
	unsigned int tmp=0;
	
	if('v'==hyn_read[0]&&'s'==hyn_read[1])
	{
#ifdef HYN_NOID_VERSION
		tmp=hyn_version_id();
#else 
		tmp=0x20121215;
#endif
		ptr += sprintf(ptr,"version:%x\n",tmp);
	}
	else if('r'==hyn_read[0]&&'e'==hyn_read[1])
	{
		if('i'==hyn_read[3])
		{
#ifdef HYN_NOID_VERSION 
			tmp=(hyn_data_proc[5]<<8) | hyn_data_proc[4];
			ptr +=sprintf(ptr,"hyn_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<ARRAY_SIZE(hyn_config_data_id))
			{
					ptr +=sprintf(ptr,"%d\n",hyn_config_data_id[tmp]); 
			}
#endif
		}
		else 
		{
			hyn_ts_write(ts->client,0Xf0,&hyn_data_proc[4],4);
			if(hyn_data_proc[0] < 0x80)
				hyn_ts_read(ts->client,hyn_data_proc[0],temp_data,4);
			hyn_ts_read(ts->client,hyn_data_proc[0],temp_data,4);

			ptr +=sprintf(ptr,"offset : {0x%02x,0x",hyn_data_proc[0]);
			ptr +=sprintf(ptr,"%02x",temp_data[3]);
			ptr +=sprintf(ptr,"%02x",temp_data[2]);
			ptr +=sprintf(ptr,"%02x",temp_data[1]);
			ptr +=sprintf(ptr,"%02x};\n",temp_data[0]);
		}
	}
	*eof = 1;
	return (ptr - page);
}
static int hyn_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-hyn][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%ld]\n", CONFIG_LEN, count);
        return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
	}
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-hyn][%s][%s]\n",__func__,temp_buf);
	
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(hyn_read,temp_buf,4);
		printk("hyn version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
		hyn_proc_flag = 1;
		reset_chip(ts->client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		reset_chip(ts->client);
		startup_chip(ts->client);
		hyn_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(hyn_read,temp_buf,4);
		memcpy(hyn_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		hyn_ts_write(ts->client,buf[4],buf,4);
	}
#ifdef HYN_NOID_VERSION
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<ARRAY_SIZE(hyn_config_data_id))
		{
			hyn_config_data_id[tmp1] = tmp;
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}*/
#endif

#define CST2XX_BASE_ADDR		(24 * 1024)
static int cst2xx_enter_download_mode(struct hyn_ts *ts)
{
	int ret;
	int i;
	unsigned char buf[2];

    #if 0
	for (i = 0; i < 5; i++)
	{
		buf[0] = 0xD1;
		buf[1] = 0x11;
		ret = cst2xx_i2c_write(ts->client, buf, 2);
		if (ret > 0)
		{
			//printk("send first 0xD111 sucess.\r\n");
			break;
		}
		else
		{
			mdelay(2);
			continue;
		}
	}
	#endif

	hard_reset_chip(ts, 5);

	for(i=0; i<20; i++) {		
		buf[0] = 0xD1;
		buf[1] = 0x11;
		ret = cst2xx_i2c_write(ts->client, buf, 2);
		if (ret < 0) {
			mdelay(1);
			continue;
		}
		
		mdelay(4); //wait enter download mode
		
		buf[0] = 0xD0;
		buf[1] = 0x01;
		ret = cst2xx_i2c_read_register(ts->client, buf, 1);
		if(ret < 0) {
			mdelay(1);
			continue;
		}

		if (buf[0] == 0x55)
			break;
	}

	if(buf[0] != 0x55)
	{
		printk("hyn reciev 0x55 failed.\n");
		return -1;
	}

	buf[0] = 0xD1;
	buf[1] = 0x10;   //enter writer register mode
	ret = cst2xx_i2c_write(ts->client, buf, 2);
	if (ret < 0) {
		printk("hyn send cmd 0xD110 failed. \n");
		return -1;
	}

	return 0;
}

static int cst2xx_download_program(unsigned char *data, int len, struct hyn_ts *ts)
{	
	int ret;
	int i, j;
	unsigned int wr_addr;
	unsigned char *pData;
	unsigned char *pSrc;
	unsigned char *pDst;
	unsigned char  i2c_buf[8];

	pData = kmalloc(sizeof(unsigned char)*(1024 + 4), GFP_KERNEL);
	if(NULL == pData) {
		printk("hyn malloc data buffer failed.\n");
		return -1;
	}
	
	pSrc = data;
	//printk("hyn write program data begain:0x%x.------\n", len);
	
	for(i=0; i<(len/1024); i++) {
		wr_addr  = (i<<10) + CST2XX_BASE_ADDR;
		
		pData[0] = (wr_addr >> 24) & 0xFF;
		pData[1] = (wr_addr >> 16) & 0xFF;
		pData[2] = (wr_addr >> 8) & 0xFF;
		pData[3] =  wr_addr & 0xFF;

		pDst = pData + 4;
		
		for(j=0; j<256; j++) {
			*pDst       = *(pSrc + 3);
			*(pDst + 1) = *(pSrc + 2);
			*(pDst + 2) = *(pSrc + 1);
			*(pDst + 3) = *pSrc;
			
			pDst += 4;
			pSrc += 4;
		}

		#ifdef TRANSACTION_LENGTH_LIMITED
		for(j=0; j<256; j++) {
            i2c_buf[0] = (wr_addr >> 24) & 0xFF;
    		i2c_buf[1] = (wr_addr >> 16) & 0xFF;
    		i2c_buf[2] = (wr_addr >> 8) & 0xFF;
    		i2c_buf[3] =  wr_addr & 0xFF;

            i2c_buf[4] =  pData[j*4+4+0];
    		i2c_buf[5] =  pData[j*4+4+1];
    		i2c_buf[6] =  pData[j*4+4+2];
    		i2c_buf[7] =  pData[j*4+4+3];    
    		
    		ret = cst2xx_i2c_write(ts->client, i2c_buf, 8);
    		if(ret < 0) {
    			printk("hyn program failed.\n");
    			goto ERR_OUT;
    		}

			wr_addr += 4;
		}
		#else
		ret = cst2xx_i2c_write(ts->client, pData, 1024+4);
		if (ret < 0) {
			printk("hyn program failed.\n");
			goto ERR_OUT;
		}
		#endif
	}

    //clear update key
	pData[3] = 0x20000FFC & 0xFF;
	pData[2] = (0x20000FFC>>8)  & 0xFF;
	pData[1] = (0x20000FFC>>16) & 0xFF;	
	pData[0] = (0x20000FFC>>24) & 0xFF;
	pData[4] = 0x00;
	pData[5] = 0x00;
	pData[6] = 0x00;
	pData[7] = 0x00;	
	ret = cst2xx_i2c_write(ts->client, pData, 8);
	if (ret < 0) {
		printk("hyn clear update key failed.\r\n");
		goto ERR_OUT;
	}
	
	pData[3] = 0xD013D013 & 0xFF;
	pData[2] = (0xD013D013>>8)  & 0xFF;
	pData[1] = (0xD013D013>>16) & 0xFF;	
	pData[0] = (0xD013D013>>24) & 0xFF;
	ret = cst2xx_i2c_write(ts->client, pData, 4);
	if (ret < 0) {
		printk("hyn exit register read/write mode failed.\r\n");
		goto ERR_OUT;
	}
	
	//printk("--------write program data end--------.\r\n");

	if (pData != NULL) {
		kfree(pData);
		pData = NULL;
	}	
	return 0;
	
ERR_OUT:
	if (pData != NULL) {
		kfree(pData);
		pData = NULL;
	}
	return -1;	
}

static int cst2xx_read_checksum(struct hyn_ts *ts)
{
	int ret;
	int i;
	unsigned int  checksum;
	unsigned int  bin_checksum;
	unsigned char buf[4];
	unsigned char *pData;

	for(i=0; i<10; i++) {
		buf[0] = 0xD0;
		buf[1] = 0x00;
		ret = cst2xx_i2c_read_register(ts->client, buf, 1);
		if(ret < 0) {
			msleep(2);
			continue;
		}

		if((buf[0]==0x01) || (buf[0]==0x02))
			break;

		msleep(2);
	}

	if((buf[0]==0x01) || (buf[0]==0x02)) {
		buf[0] = 0xD0;
		buf[1] = 0x08;
		ret = cst2xx_i2c_read_register(ts->client, buf, 4);
		
		if(ret < 0)	return -1;
		
		//handle read data  --> checksum
		checksum = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);

        pData = fwbin + 6160; //6*1024+16
		bin_checksum = pData[0] + (pData[1]<<8) + (pData[2]<<16) + (pData[3]<<24);

		if(checksum!=bin_checksum) {
			printk("hyn check sum error.\r\n");
		}

        printk("hyn checksum ic:0x%x. bin:0x%x------\r\n", checksum, bin_checksum);
		
		buf[0] = 0xD0;
		buf[1] = 0x01;
		buf[2] = 0xA5;
		ret = cst2xx_i2c_write(ts->client, buf, 3);
		
		if(ret < 0) return -1;
	}
	else {
		printk("hyn No checksum.\r\n");
		return -1;
	}
	
	return 0;
}

static int cst2xx_update_firmware(struct i2c_client * client, struct hyn_ts *ts,
	unsigned char *pdata, int data_len)
{
	int ret;
	int retry;

	retry = 0;
	
start_flow:
	//disable i2c irq
	//if (g_ts_data->use_irq)
	//	cst2xx_irq_disable(g_ts_data);

	printk("hyn enter the update firmware.\r\n");

	ret = cst2xx_enter_download_mode(ts);
	if (ret < 0) {
		printk("hyn enter download mode failed.\r\n");
		goto fail_retry;
	}
	
	ret = cst2xx_download_program(pdata, data_len, ts);
	if (ret < 0) {
		printk("hyn download program failed.\r\n");
		goto fail_retry;
	}

	mdelay(3);
		
	ret = cst2xx_read_checksum(ts);
	if (ret < 0) {
		printk("hyn checksum failed.\r\n");
		goto fail_retry;
	}
	
	//enable i2c irq
	//if (g_ts_data->use_irq)
	//	cst2xx_irq_enable(g_ts_data);

	printk("hyn Download firmware succesfully.\r\n");

	return 0;
	
fail_retry:
	if (retry < 4) {
		retry++;
		goto start_flow;
	}
	
	//enable i2c irq
	//if (g_ts_data->use_irq)
	//	cst2xx_irq_enable(g_ts_data);	
	return -1;
}

static int cst2xx_boot_update_fw(struct i2c_client * client, struct hyn_ts *ts)
{
	return cst2xx_update_firmware(client, ts, fwbin, FW_BIN_SIZE);
}

static int cst2xx_check_code(struct hyn_ts *ts)
{
	int retry = 0;
	int ret;
	unsigned char buf[4];
	buf[0] = 0xD0;
	buf[1] = 0x4C;
	while(retry++ < 20) {
		ret = cst2xx_i2c_read_register(ts->client, buf, 1);
		if (ret > 0) break;
		mdelay(2);		
	}
 //  if(retry=3){
//return -1;
 //  }
	if((buf[0]==226)||(buf[0]==237)||(buf[0]==240)) {
		return 0;
	}
	else {
		printk("hyn check sum code error. buf[0]:%d.\n", buf[0]);
		ret = cst2xx_boot_update_fw(ts->client, ts);
		if(ret<0) return -2;
		else      return  0;
	}	
}

#ifdef HAVE_TOUCH_KEY
static void report_key(struct hyn_ts *ts, u16 x, u16 y)
{
	u16 i = 0;

	for(i = 0; i < MAX_KEY_NUM; i++) {
		if((hyn_key_data[i].x_min < x) && (x < hyn_key_data[i].x_max)&&(hyn_key_data[i].y_min < y) && (y < hyn_key_data[i].y_max)){
			key = hyn_key_data[i].key;	
			input_report_key(ts->input, key, 1);
			input_sync(ts->input); 		
			key_state_flag = 1;
			break;
		}
	}
}
#endif

static void cst2xx_touch_down(struct input_dev *input_dev, s32 id,s32 x,s32 y,s32 w)
{
    s32 temp_w = (w>>1);
	
#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_PRESSURE, temp_w);
#else
    input_report_key(input_dev, BTN_TOUCH, 1);
    input_report_abs(input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(input_dev);
#endif

}

static void cst2xx_touch_up(struct input_dev *input_dev, int id)
{
	
#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    //input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
#endif

}

static int report_flag = 0;
static void cst2xx_ts_worker(struct work_struct *work)
{
	//int rc, i;
	//u8 id, touches;
	//u16 x, y;
	u8 buf[30];
	u8 i2c_buf[8];
	u8 key_status, key_id, finger_id, sw;
	int  input_x = 0; 
	int  input_y = 0; 
	int  input_w = 0;
	int  temp;
    u8   i, cnt_up, cnt_down;
	int  ret, idx; 
	int  cnt, i2c_len, len_1, len_2;


#ifdef HYN_NOID_VERSION
	u32 tmp1;
	u8 buf[4] = {0};
	struct hyn_touch_info cinfo;
#endif

	struct hyn_ts *ts = container_of(work, struct hyn_ts,work);

	//print_info("=====cst2xx_ts_worker=====\n");				 

#ifdef TPD_PROC_DEBUG
	if(hyn_proc_flag == 1)
		goto schedule;
#endif
	
	//buf[0] = 0xD1;
	//buf[1] = 0x08;
	//ret = cst2xx_i2c_write(g_i2c_client, buf, 2);
	//if (ret < 0) 
	//{
	//	printk("send get finger point cmd failed.\r\n");
	//	goto END;
	//}

	buf[0] = 0xD0;
	buf[1] = 0x00;
	ret = cst2xx_i2c_read_register(ts->client, buf, 7);
	if(ret < 0) {
		printk("hyn iic read touch point data failed.\n");
		goto OUT_PROCESS;
	}
		
	if(buf[6] != 0xAB) {
		//printk("data is not valid..\r\n");
		goto OUT_PROCESS;
	}
	
	cnt = buf[5] & 0x7F;
	if(cnt>MAX_FINGERS) goto OUT_PROCESS;
	else if(cnt==0)     goto CLR_POINT;

	if(buf[5] == 0x80) {
		key_status = buf[0];
		key_id = buf[1];		
		goto KEY_PROCESS;
	} 
	else if(cnt == 0x01) {
		goto FINGER_PROCESS;
	} 
	else {
		#ifdef TRANSACTION_LENGTH_LIMITED
		if((buf[5]&0x80) == 0x80) { //key
			i2c_len = (cnt - 1)*5 + 3;
			len_1   = i2c_len;
			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst2xx_i2c_read_register(ts->client, i2c_buf, len_2);
    			if(ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}
			
			i2c_len   += 5;
			key_status = buf[i2c_len - 3];
			key_id     = buf[i2c_len - 2];
		} 
		else {			
			i2c_len = (cnt - 1)*5 + 1;
			len_1   = i2c_len;
			
			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst2xx_i2c_read_register(ts->client, i2c_buf, len_2);
    			if (ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}			
			i2c_len += 5;
		}
		#else
		if ((buf[5]&0x80) == 0x80) {
			buf[5] = 0xD0;
			buf[6] = 0x07;
			i2c_len = (cnt - 1)*5 + 3;
			ret = cst2xx_i2c_read_register(ts->client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
			key_status = buf[i2c_len - 3];
			key_id = buf[i2c_len - 2];
		} 
		else {			
			buf[5] = 0xD0;
			buf[6] = 0x07;			
			i2c_len = (cnt - 1)*5 + 1;
			ret = cst2xx_i2c_read_register(ts->client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
		}
		#endif

		if (buf[i2c_len - 1] != 0xAB) {
			goto OUT_PROCESS;
		}
	}	

	if((cnt>0)&&(key_status&0x80))  //both key and point
	{
        if(report_flag==0xA5) goto KEY_PROCESS; 
	}
	
FINGER_PROCESS:
	
	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst2xx_i2c_write(ts->client, i2c_buf, 3);
	if (ret < 0) {
		printk("hyn send read touch info ending failed.\r\n"); 
		hard_reset_chip(ts, 20);
	}

	idx = 0;
    cnt_up = 0;
    cnt_down = 0;
	for (i = 0; i < cnt; i++) {
		input_x = (unsigned int)((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0F));
		input_y = (unsigned int)((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0F));	
		input_w = (unsigned int)(buf[idx + 4]);
		sw = (buf[idx] & 0x0F) >> 1;
		finger_id = (buf[idx] >> 4) & 0x0F;

        #ifdef SWAP_X_Y
		temp    = input_x;
		input_x = input_y;
		input_y = temp;
		#endif

		#ifdef X_POL
		input_x = SCREEN_MAX_X - input_x;
		#endif
		
		#ifdef Y_POL
		input_y = SCREEN_MAX_Y - input_y;
		#endif
		
    // printk("Point x:%d, y:%d, id:%d, sw:%d.\r\n", input_x, input_y, finger_id, sw);

		if (sw == 0x03) {
			cst2xx_touch_down(ts->input, finger_id, input_x, input_y, input_w);
            cnt_down++;
        }
		else {
            cnt_up++;
            #ifdef ICS_SLOT_REPORT
			cst2xx_touch_up(ts->input, finger_id);
            #endif
        }
		idx += 5;
	}
    
    #ifndef ICS_SLOT_REPORT
    if((cnt_up>0) && (cnt_down==0))
        cst2xx_touch_up(ts->input, 0);
    #endif

	if(cnt_down==0)  report_flag = 0;
	else report_flag = 0xCA;

    input_sync(ts->input);

	goto END;

KEY_PROCESS:
	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst2xx_i2c_write(ts->client, i2c_buf, 3);
	if (ret < 0) {
		printk("hyn send read touch info ending failed.\r\n"); 
	}

    #ifdef HAVE_TOUCH_KEY
	if(key_status&0x80) {
        if((key_status&0x7F)==0x03) {
    	    i = (key_id>>4)-1;
			key = hyn_key_data[i].key;	
			input_report_key(ts->input, key, 1);
			
			report_flag = 0xA5;
		}
    	else {
			input_report_key(ts->input, key, 0);
			report_flag = 0;			
    	}
	}
	#endif
	
	input_sync(ts->input);

	goto END;

CLR_POINT:
#ifdef SLEEP_CLEAR_POINT
	#ifdef ICS_SLOT_REPORT
		for(i=0; i<=MAX_CONTACTS; i++) {	
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		}
	#else	
		input_mt_sync(ts->input);
	#endif
		input_sync(ts->input);
#endif	

OUT_PROCESS:
	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst2xx_i2c_write(ts->client, i2c_buf, 3);
	if (ret < 0) {
		printk("send read touch info ending failed.\n"); 
		hard_reset_chip(ts, 20);
	}
	
END:
#ifdef HYN_MONITOR
	if(i2c_lock_flag != 0)
		goto i2c_lock_schedule;
	else
		i2c_lock_flag = 1;
#endif

schedule:
#ifdef HYN_MONITOR
	i2c_lock_flag = 0;
i2c_lock_schedule:
#endif

	enable_irq(ts->irq);
		
}

#ifdef HYN_MONITOR
static void hyn_monitor_worker(struct work_struct *work)
{
	//u8 write_buf[4] = {0};
	u8 read_buf[4]  = {0};
	char init_chip_flag = 0;
	
//	print_info("----------------hyn_monitor_worker-----------------\n");	
   struct hyn_ts *ts = container_of(work, struct hyn_ts,hyn_monitor_work.work);
	if(i2c_lock_flag != 0) {
	    //i2c_lock_flag=1;
	    goto queue_monitor_work;
	}		
	else
		i2c_lock_flag = 1;

	//hyn_ts_read(ts->client, 0x80, read_buf, 4);
    	//printk("======read 0x80: %x %x %x %x ======tony0geshu\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
	
	hyn_ts_read(ts->client, 0xb0, read_buf, 4);
	if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		b0_counter ++;
	else
		b0_counter = 0;

	if(b0_counter > 1) {
		printk("======read 0xb0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		b0_counter = 0;
		goto queue_monitor_init_chip;
	}
	
	hyn_ts_read(ts->client, 0xb4, read_buf, 4);	
	int_2nd[3] = int_1st[3];
	int_2nd[2] = int_1st[2];
	int_2nd[1] = int_1st[1];
	int_2nd[0] = int_1st[0];
	int_1st[3] = read_buf[3];
	int_1st[2] = read_buf[2];
	int_1st[1] = read_buf[1];
	int_1st[0] = read_buf[0];

	//printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",int_1st[3], int_1st[2], int_1st[1], int_1st[0], int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);

	if(int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0])  {
		printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",int_1st[3], int_1st[2], int_1st[1], int_1st[0], int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
		init_chip_flag = 1;
		goto queue_monitor_init_chip;
	}
	

	hyn_ts_read(ts->client, 0xbc, read_buf, 4);
	if(read_buf[3] != 0 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
		bc_counter++;
	else
		bc_counter = 0;
	if(bc_counter > 1) {
		printk("======read 0xbc: %x %x %x %x======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		bc_counter = 0;
	}


/*
	write_buf[3] = 0x01;
	write_buf[2] = 0xfe;
	write_buf[1] = 0x10;
	write_buf[0] = 0x00;
	hyn_ts_write(ts->client, 0xf0, write_buf, 4);
	hyn_ts_read(ts->client, 0x10, read_buf, 4);
	hyn_ts_read(ts->client, 0x10, read_buf, 4);
	
	if(read_buf[3] < 10 && read_buf[2] < 10 && read_buf[1] < 10 && read_buf[0] < 10)
		dac_counter ++;
	else
		dac_counter = 0;

	if(dac_counter > 1) 
	{
		printk("======read DAC1_0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		init_chip_flag = 1;
		dac_counter = 0;
	}
*/
queue_monitor_init_chip:
	if(init_chip_flag)
		init_chip(ts->client,ts);

	i2c_lock_flag = 0;
	
queue_monitor_work:	
	queue_delayed_work(hyn_monitor_workqueue, &ts->hyn_monitor_work, 100);
}
#endif

static irqreturn_t hyn_ts_irq(int irq, void *dev_id)
{	
	///struct hyn_ts *ts = dev_id;
    struct hyn_ts *ts = (struct hyn_ts*)dev_id;
	//print_info("========cst2xx Interrupt=========\n");				 

	disable_irq_nosync(ts->irq);

	if (!work_pending(&ts->work)) 
	{
		queue_work(ts->wq, &ts->work);
	}
	
	return IRQ_HANDLED;
}

static int cst2xx_ts_init(struct i2c_client *client, struct hyn_ts *ts)
{
	struct input_dev *input_device;
	int rc = 0;
	
	printk("hyn cst2xx Enter %s\n", __func__);

    #if 0
	ts->dd = &devices[ts->device_id];
	if (ts->device_id == 0) {
		ts->dd->data_size = MAX_FINGERS * ts->dd->touch_bytes + ts->dd->touch_meta_data;
		ts->dd->touch_index = 0;
	}

	ts->touch_data = devm_kzalloc(&client->dev,ts->dd->data_size, GFP_KERNEL);
	if (!ts->touch_data) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}
	#endif

	input_device = devm_input_allocate_device(&ts->client->dev);
	if (!input_device) {
		rc = -ENOMEM;
		goto error_alloc_dev;
	}

	ts->input = input_device;
	input_device->name = CST2XX_I2C_NAME;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

#ifdef ICS_SLOT_REPORT
	__set_bit(EV_ABS, input_device->evbit);
	__set_bit(EV_KEY, input_device->evbit);
	__set_bit(EV_REP, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_mt_init_slots(input_device, (MAX_CONTACTS+1), 0);
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

	set_bit(ABS_MT_POSITION_X, input_device->absbit);
	set_bit(ABS_MT_POSITION_Y, input_device->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_device->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_device->absbit);

	input_set_abs_params(input_device,ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_device,ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_device,ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_device,ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	//client->irq = IRQ_PORT;
	//ts->irq = client->irq;

	ts->wq = create_singlethread_workqueue("kworkqueue_ts");
	if (!ts->wq) {
		dev_err(&client->dev, "hyn Could not create workqueue\n");
		goto error_wq_create;
	}
	flush_workqueue(ts->wq);	

	INIT_WORK(&ts->work, cst2xx_ts_worker);

	rc = input_register_device(input_device);
	if (rc)
		goto error_unreg_device;

	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
error_alloc_dev:
	//kfree(ts->touch_data);
	return rc;
}

static int hyn_ts_suspend(struct device *dev)
{
	struct hyn_ts *ts = dev_get_drvdata(dev);
	int i, rc;
	u8 buf[2];

  	printk("I'am in hyn_ts_suspend() start\n");

#ifdef HYN_MONITOR
	printk( "hyn_ts_suspend () : cancel hyn_monitor_work\n");
	cancel_delayed_work_sync(&ts->hyn_monitor_work);
#endif
	
	disable_irq_nosync(ts->irq);	

#if defined(CONFIG_BOARD_TYPE_ZM726CE_V12)
	axp_gpio_set_value(PMU_GPIO_NUM,0);
#endif

	//cst2xx_shutdown_low(ts);
	if(h_wake_pin != 0) {
	    gpio_direction_output(ts->rst_pin, 0);
	}

#ifdef SLEEP_CLEAR_POINT
	msleep(10); 		
	#ifdef ICS_SLOT_REPORT
	for(i=1; i<=MAX_CONTACTS; i++) {	
		input_mt_slot(ts->input, i);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	#else	
	input_mt_sync(ts->input);
	#endif
	input_sync(ts->input);
	//msleep(10); 	
	//report_data(ts, 1, 1, 10, 1);		
	//input_sync(ts->input);	
#endif	

    buf[0] = 0xD1;
	buf[1] = 0x05;
	rc = cst2xx_i2c_write(ts->client, buf, 2);

	return 0;
}

static int hyn_ts_resume(struct device *dev)
{
	struct hyn_ts *ts = dev_get_drvdata(dev);
	int i, rc;
    u8 buf[2];
	
  	printk("I'am in hyn_ts_resume() start\n");

#if defined(CONFIG_BOARD_TYPE_ZM726CE_V12)
	axp_gpio_set_value(PMU_GPIO_NUM,1);
#endif

	//cst2xx_shutdown_high(ts);
	//msleep(20); 	
	//reset_chip(ts->client);
	//startup_chip(ts->client);
	//check_mem_data(ts->client,ts);

	hard_reset_chip(ts, 30);

#ifdef SLEEP_CLEAR_POINT
	#ifdef ICS_SLOT_REPORT
	for(i=1; i<=MAX_CONTACTS; i++) {	
		input_mt_slot(ts->input, i);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	#else	
	input_mt_sync(ts->input);
	#endif
	input_sync(ts->input);	
#endif

#ifdef HYN_MONITOR
	printk( "hyn_ts_resume () : queue hyn_monitor_work\n");
	queue_delayed_work(hyn_monitor_workqueue, &ts->hyn_monitor_work, 300);
#endif	
	msleep(20); 	
    rc = cst2xx_check_code(ts);
	if(rc < 0){
		printk("hyn check code error.\n");
		return rc;
	}

	buf[0] = 0xD1;
	buf[1] = 0x06;
	rc = cst2xx_i2c_write(ts->client, buf, 2);

	msleep(100);

	//disable_irq_nosync(ts->irq);
	enable_irq(ts->irq);

	return 0;
}

static int hyn_ts_early_suspend(struct tp_device *tp_d)
{
	struct hyn_ts *ts = container_of(tp_d, struct hyn_ts, tp);
	printk("[CST2XX] Enter %s\n", __func__);
	hyn_ts_suspend(&ts->client->dev);
	return 0;
}

static int hyn_ts_late_resume(struct tp_device *tp_d)
{
	struct hyn_ts *ts = container_of(tp_d, struct hyn_ts, tp);
	printk("[CST2XX] Enter %s\n", __func__);
	hyn_ts_resume(&ts->client->dev);
	return 0;
}

#if 0//CONFIG_HAS_EARLYSUSPEND
static void hyn_ts_early_suspend(struct early_suspend *h)
{
	struct hyn_ts *ts = container_of(h, struct hyn_ts, early_suspend);
	printk("[CST2XX] Enter %s\n", __func__);
	hyn_ts_suspend(&ts->client->dev);
}

static void hyn_ts_late_resume(struct early_suspend *h)
{
	struct hyn_ts *ts = container_of(h, struct hyn_ts, early_suspend);
	printk("[CST2XX] Enter %s\n", __func__);
	hyn_ts_resume(&ts->client->dev);
}
#endif

static int  hyn_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct hyn_ts *ts;
	int rc;
	struct device_node *np = client->dev.of_node;
	enum of_gpio_flags wake_flags;
	unsigned long irq_flags;

	printk("cst2xx enter %s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "hyn I2C functionality not supported\n");
		return -ENODEV;
	}
 
	ts = devm_kzalloc(&client->dev,sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	printk("==kzalloc success=\n");
  
	ts->client = client;
	
	i2c_set_clientdata(client, ts);
	//ts->device_id = id->driver_data;

	//cst2xx_init(ts); 

	ts->irq_pin = of_get_named_gpio_flags(np, "irq-gpio", 0, (enum of_gpio_flags *)&irq_flags);
	ts->rst_pin = of_get_named_gpio_flags(np, "wake-gpio", 0, &wake_flags);
	
	if (gpio_is_valid(ts->rst_pin)) {
		rc = devm_gpio_request_one(&client->dev, ts->rst_pin, (wake_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH, "cst21680 wake pin");
		if (rc != 0) {
			dev_err(&client->dev, "cst21680 wake pin error\n");
			//return -EIO;
		}
		else
		{
		    h_wake_pin = ts->rst_pin;
		}
		//msleep(100);
	} else {
		dev_info(&client->dev, "wake pin invalid\n");
	}
		
	if (gpio_is_valid(ts->irq_pin)) {
		rc = devm_gpio_request_one(&client->dev, ts->irq_pin, (irq_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH, "gslX680 irq pin");
		if (rc != 0) {
			dev_err(&client->dev, "cst2xx irq pin error\n");
			return -EIO;
		}
	} else {
		dev_info(&client->dev, "irq pin invalid\n");
	}

	rc = cst2xx_test_i2c(client);
	if (rc < 0) {
		dev_err(&client->dev, "hyn cst2xx test iic error.\n");
		return rc;
	}	

    #if 0
	rc = cst2xx_boot_update_fw(client, ts);
	if(rc < 0){
		printk("hyn update fw failed.\n");
		return rc;
	}
	#endif

    msleep(40);	 //runing
	
    rc = cst2xx_check_code(ts);
	if(rc < 0){
		printk("hyn check code error.\n");
		return rc;
	}

	rc = cst2xx_ts_init(client, ts);
	if (rc < 0) {
		printk("hyn CST2XX init failed\n");
		goto error_mutex_destroy;
	}	
	
	//init_chip(ts->client,ts);
	//check_mem_data(ts->client,ts);

	ts->irq = gpio_to_irq(ts->irq_pin);	
	#if 0
	rc = request_irq(client->irq, hyn_ts_irq, IRQF_TRIGGER_RISING, client->name, ts);
	if (rc < 0) {
		printk( "hyn_probe: request irq failed\n");
		goto error_req_irq_fail;
	}
	#endif
	if(ts->irq)
	{
		rc = devm_request_threaded_irq(&client->dev, ts->irq, NULL, hyn_ts_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
		if (rc != 0) {
			printk(KERN_ALERT "Cannot allocate ts INT!ERRNO:%d\n", rc);
			goto error_req_irq_fail;
		}
		disable_irq(ts->irq);
	}
	else
	{
		printk("cst21680 irq req fail\n");
		goto error_req_irq_fail;
	}
	enable_irq(ts->irq);

	/* create debug attribute */
	//rc = device_create_file(&ts->input->dev, &dev_attr_debug_enable);
	ts->tp.tp_suspend = hyn_ts_early_suspend;
	ts->tp.tp_resume = hyn_ts_late_resume;
    tp_register_fb(&ts->tp);

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	//ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = hyn_ts_early_suspend;
	ts->early_suspend.resume = hyn_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif


#ifdef HYN_MONITOR
	printk( "hyn_ts_probe () : queue hyn_monitor_workqueue\n");
    
	INIT_DELAYED_WORK(&ts->hyn_monitor_work, hyn_monitor_worker);
	hyn_monitor_workqueue = create_singlethread_workqueue("hyn_monitor_workqueue");
	queue_delayed_work(hyn_monitor_workqueue, &ts->hyn_monitor_work, 1000);
#endif

#ifdef TPD_PROC_DEBUG
#if 0
    hyn_config_proc = create_proc_entry(HYN_CONFIG_PROC_FILE, 0666, NULL);
    printk("[tp-hyn] [%s] hyn_config_proc = %x \n",__func__,hyn_config_proc);
    if (hyn_config_proc == NULL)
    {
        print_info("create_proc_entry %s failed\n", HYN_CONFIG_PROC_FILE);
    }
    else
    {
        hyn_config_proc->read_proc = hyn_config_read_proc;
        hyn_config_proc->write_proc = hyn_config_write_proc;
    }
#else
  //  proc_create(HYN_CONFIG_PROC_FILE,0666,NULL,&hyn_seq_fops);
#endif
		hyn_proc_flag = 0;
#endif
    //disable_irq_nosync(->irq);
	printk("[CST2XX] End %s\n", __func__);
    
	return 0;

//exit_set_irq_mode:	
error_req_irq_fail:
    free_irq(ts->irq, ts);	

error_mutex_destroy:
	input_free_device(ts->input);
	kfree(ts);
	return rc;
}

static int hyn_ts_remove(struct i2c_client *client)
{
	struct hyn_ts *ts = i2c_get_clientdata(client);
	printk("==hyn_ts_remove=\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef HYN_MONITOR
	cancel_delayed_work_sync(&ts->hyn_monitor_work);
	destroy_workqueue(hyn_monitor_workqueue);
#endif

	device_init_wakeup(&client->dev, 0);
	cancel_work_sync(&ts->work);
	free_irq(ts->irq, ts);
	destroy_workqueue(ts->wq);
	input_unregister_device(ts->input);
	//device_remove_file(&ts->input->dev, &dev_attr_debug_enable);
	
	//kfree(ts->touch_data);
	kfree(ts);

	return 0;
}

#if 1
static struct of_device_id hyn_ts_ids[] = {
	{ .compatible = "HYN,CST2XX" },
	{ }
};
#endif

static const struct i2c_device_id hyn_ts_id[] = {
	{CST2XX_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, hyn_ts_id);

static struct i2c_driver hyn_ts_driver = {
	.driver = {
		.name = CST2XX_I2C_NAME,
		.owner = THIS_MODULE,
        .of_match_table = of_match_ptr(hyn_ts_ids),
	},
	
#ifndef CONFIG_HAS_EARLYSUSPEND
	//.suspend	= hyn_ts_suspend,
	//.resume	= hyn_ts_resume,
#endif
	.probe		= hyn_ts_probe,
	.remove		= hyn_ts_remove,
	.id_table	= hyn_ts_id,
};

static int __init hyn_ts_init(void)
{
    int ret;
	printk("==hyn_ts_init==\n");
	ret = i2c_add_driver(&hyn_ts_driver);
	printk("ret=%d\n",ret);
	return ret;
}
static void __exit hyn_ts_exit(void)
{
	printk("==hyn_ts_exit==\n");
	i2c_del_driver(&hyn_ts_driver);
	return;
}

module_init(hyn_ts_init);
module_exit(hyn_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HYNCST2XX touchscreen controller driver");
MODULE_AUTHOR("Tim.Tan");
MODULE_ALIAS("platform:hyn_ts");

