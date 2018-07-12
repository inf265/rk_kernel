
#include <linux/rk_fb.h>
#include <linux/device.h>
#include "lcd.h"
#include "../hdmi/rockchip-hdmi.h"
#include <linux/of_gpio.h>
#include <linux/delay.h>


static struct rk_screen *rk_screen;
#define GPIO_HIGH 1
#define GPIO_LOW 0


/* define lcd command */
#define ENTER_SLEEP_MODE        0x10
#define EXIT_SLEEP_MODE         0x11
#define SET_COLUMN_ADDRESS      0x2a
#define SET_PAGE_ADDRESS        0x2b
#define WRITE_MEMORY_START      0x2c
#define SET_DISPLAY_ON          0x29
#define SET_DISPLAY_OFF         0x28
#define SET_ADDRESS_MODE        0x36
#define SET_PIXEL_FORMAT        0x3a


#define SIMULATION_SPI 1
#ifdef SIMULATION_SPI
//	&gpio1 GPIO_B1 0		&gpio1 GPIO_B0 0		&gpio1 GPIO_B3 0		&gpio0 GPIO_D0 0

    #define TXD_PORT        41
	#define CLK_PORT        40
	#define CS_PORT         43
	#define LCD_RST_PORT    24

	#define CS_OUT()        gpio_direction_output(CS_PORT, 0)
	#define CS_SET()        gpio_set_value(CS_PORT, GPIO_HIGH)
	#define CS_CLR()        gpio_set_value(CS_PORT, GPIO_LOW)
	#define CLK_OUT()       gpio_direction_output(CLK_PORT, 0)
	#define CLK_SET()       gpio_set_value(CLK_PORT, GPIO_HIGH)
	#define CLK_CLR()       gpio_set_value(CLK_PORT, GPIO_LOW)
	#define TXD_OUT()       gpio_direction_output(TXD_PORT, 0)
	#define TXD_SET()       gpio_set_value(TXD_PORT, GPIO_HIGH)
	#define TXD_CLR()       gpio_set_value(TXD_PORT, GPIO_LOW)
    #define LCD_RST_OUT()  gpio_direction_output(LCD_RST_PORT, 1)
    #define LCD_RST(i)      gpio_set_value(LCD_RST_PORT, i)

	//#define bits_9
	#ifdef bits_9  //9bits
	#define LCDSPI_InitCMD(cmd)    spi_write_9bit(0, cmd)
	#define LCDSPI_InitDAT(dat)    spi_write_9bit(1, dat)
	#else  //16bits
	#define LCDSPI_InitCMD(cmd)    spi_write_16bit(0, cmd)
	#define LCDSPI_InitDAT(dat)    spi_write_16bit(1, dat)
	#endif
	#define Lcd_EnvidOnOff(i)

#else

	#define bits_9 1
	#ifdef bits_9  //9bits
	#define LCDSPI_InitCMD(cmd)
	#define LCDSPI_InitDAT(dat)
	#else  //16bits
	#define LCDSPI_InitCMD(cmd)
	#define LCDSPI_InitDAT(dat)
	#endif

#endif

int rk_fb_get_extern_screen(struct rk_screen *screen)
{
	if (unlikely(!rk_screen) || unlikely(!screen))
		return -1;

	memcpy(screen, rk_screen, sizeof(struct rk_screen));
	screen->dsp_lut = NULL;
	screen->cabc_lut = NULL;
	screen->type = SCREEN_NULL;

	return 0;
}

int  rk_fb_get_prmry_screen(struct rk_screen *screen)
{
	if (unlikely(!rk_screen) || unlikely(!screen))
		return -1;

	memcpy(screen, rk_screen, sizeof(struct rk_screen));
	return 0;
}

int rk_fb_set_prmry_screen(struct rk_screen *screen)
{
	if (unlikely(!rk_screen) || unlikely(!screen))
		return -1;

	rk_screen->lcdc_id = screen->lcdc_id;
	rk_screen->screen_id = screen->screen_id;
	rk_screen->x_mirror = screen->x_mirror;
	rk_screen->y_mirror = screen->y_mirror;
	rk_screen->overscan.left = screen->overscan.left;
	rk_screen->overscan.top = screen->overscan.left;
	rk_screen->overscan.right = screen->overscan.left;
	rk_screen->overscan.bottom = screen->overscan.left;
	return 0;
}

size_t get_fb_size(u8 reserved_fb)
{
	size_t size = 0;
	u32 xres = 0;
	u32 yres = 0;

	if (unlikely(!rk_screen))
		return 0;

	xres = rk_screen->mode.xres;
	yres = rk_screen->mode.yres;

	/* align as 64 bytes(16*4) in an odd number of times */
	xres = ALIGN_64BYTE_ODD_TIMES(xres, ALIGN_PIXEL_64BYTE_RGB8888);
        if (reserved_fb == 1) {
                size = (xres * yres << 2) << 1;/*two buffer*/
        } else {
#if defined(CONFIG_THREE_FB_BUFFER)
		size = (xres * yres << 2) * 3;	/* three buffer */
#else
		size = (xres * yres << 2) << 1; /* two buffer */
#endif
	}
	return ALIGN(size, SZ_1M);
}

static int rk_screen_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}
	rk_screen = devm_kzalloc(&pdev->dev,
			sizeof(struct rk_screen), GFP_KERNEL);
	if (!rk_screen) {
		dev_err(&pdev->dev, "kmalloc for rk screen fail!");
		return  -ENOMEM;
	}
	ret = rk_fb_prase_timing_dt(np, rk_screen);
	dev_info(&pdev->dev, "rockchip screen probe %s\n",
				ret ? "failed" : "success");
	return ret;
}

static const struct of_device_id rk_screen_dt_ids[] = {
	{ .compatible = "rockchip,screen", },
	{}
};

static struct platform_driver rk_screen_driver = {
	.probe		= rk_screen_probe,
	.driver		= {
		.name	= "rk-screen",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rk_screen_dt_ids),
	},
};
#define DRVDelayUs(i)   udelay(i*2)


/* spi write a data frame,type mean command or data */
int spi_write_9bit(u32 type, u32 value)
{
    u32 i = 0;

    if(type != 0 && type != 1)
    {
    	return -1;
    }
    /*make a data frame of 9 bits,the 8th bit  0:mean command,1:mean data*/
    value &= 0xff;
    value &= (type << 8);

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    DRVDelayUs(2);
    DRVDelayUs(2);

    CS_SET();
    TXD_SET();
    CLK_SET();
    DRVDelayUs(2);

	CS_CLR();
	for(i = 0; i < 9; i++)  //reg
	{
		if(value & (1 << (8-i)))
        {
			TXD_SET();
		}
        else
        {
			TXD_CLR();
        }

		CLK_CLR();
		DRVDelayUs(2);
		CLK_SET();
		DRVDelayUs(2);
	}

	CS_SET();
	CLK_CLR();
	TXD_CLR();
	DRVDelayUs(2);
    return 0;
}


/* spi write a data frame,type mean command or data */
int spi_write_16bit(u32 type, u32 value)
{
    u32 i = 0;
    u32 data = 0;
	
    if(type != 0 && type != 1)
    {
    	return -1;
    }
    /*make a data frame of 16 bits,the 8th bit  0:mean command,1:mean data*/
    data = (type << 8)|value; 

    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    DRVDelayUs(2);
    DRVDelayUs(2);

    CS_SET();
    TXD_SET();
    CLK_SET();
    DRVDelayUs(2);

	CS_CLR();
	for(i = 0; i < 16; i++)  //reg
	{
		if(data & (1 << (15-i)))
        {
			TXD_SET();
		}
        else
        {
			TXD_CLR();
        }

		CLK_CLR();
		DRVDelayUs(2);
		CLK_SET();
		DRVDelayUs(2);
	}

	CS_SET();
	CLK_CLR();
	TXD_CLR();
	DRVDelayUs(2);
    return 0;
}


void SPI_SendData(unsigned char i)
{  
   unsigned char n;
   
   for(n=0; n<8; n++)			
   {  
	  if(i&0x80) TXD_SET();
      	else TXD_CLR();
      i<<= 1;

	  CLK_CLR();
	  DRVDelayUs(2);
	  CLK_SET();
	  DRVDelayUs(2);

   }
}
void SPI_WriteComm(unsigned char i)
{
    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    DRVDelayUs(2);
    DRVDelayUs(2);

    CS_CLR();
    TXD_CLR();
    CLK_SET();
    DRVDelayUs(2);


	SPI_SendData(i);

   CS_SET();
   CLK_CLR();
   TXD_CLR();
   DRVDelayUs(2);

}


void SPI_WriteData(unsigned char i)
{
    TXD_OUT();
    CLK_OUT();
    CS_OUT();
    DRVDelayUs(2);
    DRVDelayUs(2);

    CS_CLR();
    TXD_SET();
    CLK_SET();
    DRVDelayUs(2);


	SPI_SendData(i);

	CS_SET();
	CLK_CLR();
	TXD_CLR();
	DRVDelayUs(2);

}


static int __init rk_screen_init(void)
{
	    printk("lcd_init...\n");
/* reset lcd to start init lcd by software if there is no hardware reset circuit for the lcd */
#ifdef LCD_RST_PORT
	gpio_request(LCD_RST_PORT, NULL);
#endif

CS_CLR();

LCD_RST_OUT();
mdelay(50);
LCD_RST(0);
mdelay(150);
LCD_RST(1);
mdelay(150);



#if 0
		TXD_OUT();
		CLK_OUT();
		CS_OUT();
		CS_SET();
		TXD_SET();
		CLK_SET();
		LCD_RST_OUT();
		LCD_RST(1);
		msleep(10);
		LCD_RST(0);
		msleep(100);
		LCD_RST(1);
		msleep(100);
#endif
#if 1
	
		SPI_WriteComm(0xFF);
		SPI_WriteData(0x77);
		SPI_WriteData(0x01);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x10);
		SPI_WriteComm(0xC0);
		SPI_WriteData(0x64);///63
		SPI_WriteData(0x00);
		
		
		 
		
		SPI_WriteComm(0xC1);
		SPI_WriteData(0x0A);
		SPI_WriteData(0x02);
		SPI_WriteComm(0xC2);
		SPI_WriteData(0x31);
		SPI_WriteData(0x08);
		SPI_WriteComm(0xCC);
		SPI_WriteData(0x10);
		//-------------------------------------Gamma Cluster Setting-------------------------------------------//
		SPI_WriteComm(0xB0);
		
		
		SPI_WriteData(0x00);
		SPI_WriteData(0x11);
		SPI_WriteData(0x19);
		SPI_WriteData(0x0C);
		SPI_WriteData(0x10);
		SPI_WriteData(0x06);
		SPI_WriteData(0x07);
		SPI_WriteData(0x0A);
		SPI_WriteData(0x09);
		SPI_WriteData(0x22);
		SPI_WriteData(0x04);
		SPI_WriteData(0x10);
		SPI_WriteData(0x0E);
		SPI_WriteData(0x28);
		SPI_WriteData(0x30);
		SPI_WriteData(0x1C);
		SPI_WriteComm(0xB1);
		SPI_WriteData(0x00);
		SPI_WriteData(0x12);
		SPI_WriteData(0x19);
		SPI_WriteData(0x0D);
		SPI_WriteData(0x10);
		SPI_WriteData(0x04);
		SPI_WriteData(0x06);
		SPI_WriteData(0x07);
		SPI_WriteData(0x08);
		SPI_WriteData(0x23);
		SPI_WriteData(0x04);
		SPI_WriteData(0x12);
		SPI_WriteData(0x11);
		SPI_WriteData(0x28);
		SPI_WriteData(0x30);
		SPI_WriteData(0x1C);
		//---------------------------------------End Gamma Setting----------------------------------------------//
		//------------------------------------End Display Control setting----------------------------------------//
		//-----------------------------------------Bank0 Setting End---------------------------------------------//
		//-------------------------------------------Bank1 Setting---------------------------------------------------//
		//-------------------------------- Power Control Registers Initial --------------------------------------//
		SPI_WriteComm(0xFF);
		SPI_WriteData(0x77);
		
		
		
		SPI_WriteData(0x01);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x11);
		SPI_WriteComm(0xB0);
		SPI_WriteData(0x4D);
		//-------------------------------------------Vcom Setting---------------------------------------------------//
		SPI_WriteComm(0xB1);
		SPI_WriteData(0x3E);
		//-----------------------------------------End Vcom Setting-----------------------------------------------//
		SPI_WriteComm(0xB2);
		SPI_WriteData(0x07);
		SPI_WriteComm(0xB3);
		SPI_WriteData(0x80);
		SPI_WriteComm(0xB5);
		SPI_WriteData(0x47);
		SPI_WriteComm(0xB7);
		SPI_WriteData(0x85);
		SPI_WriteComm(0xB8);
		SPI_WriteData(0x21);
		SPI_WriteComm(0xB9);
		SPI_WriteData(0x10);
		SPI_WriteComm(0xC1);
		SPI_WriteData(0x78);
		SPI_WriteComm(0xC2);
		SPI_WriteData(0x78);
		SPI_WriteComm(0xD0);
		SPI_WriteData(0x88);
		//---------------------------------End Power Control Registers Initial -------------------------------//
		//Delayms (100);
		//---------------------------------------------GIP Setting----------------------------------------------------//
		SPI_WriteComm(0xE0);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x02);
		SPI_WriteComm(0xE1);
		SPI_WriteData(0x04);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x05);
		
		
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x20);
		SPI_WriteData(0x20);
		SPI_WriteComm(0xE2);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xE3);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x33);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xE4);
		SPI_WriteData(0x22);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xE5);
		SPI_WriteData(0x04);
		SPI_WriteData(0x34);
		SPI_WriteData(0xAA);
		SPI_WriteData(0xAA);
		SPI_WriteData(0x06);
		SPI_WriteData(0x34);
		SPI_WriteData(0xAA);
		SPI_WriteData(0xAA);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		
		
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xE6);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x33);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xE7);
		SPI_WriteData(0x22);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xE8);
		SPI_WriteData(0x05);
		SPI_WriteData(0x34);
		SPI_WriteData(0xAA);
		SPI_WriteData(0xAA);
		SPI_WriteData(0x07);
		SPI_WriteData(0x34);
		SPI_WriteData(0xAA);
		SPI_WriteData(0xAA);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xEB);
		SPI_WriteData(0x02);
		SPI_WriteData(0x00);
		SPI_WriteData(0x40);
		SPI_WriteData(0x40);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xEC);
		SPI_WriteData(0x00);
		SPI_WriteData(0x00);
		SPI_WriteComm(0xED);
		
		
		SPI_WriteData(0xFA);
		SPI_WriteData(0x45);
		SPI_WriteData(0x0B);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xFF);
		SPI_WriteData(0xB0);
		SPI_WriteData(0x54);
		SPI_WriteData(0xAF);
		
		
		
		SPI_WriteComm(0x00);//0x36
		SPI_WriteData(0x08); 
		
		SPI_WriteComm(0x3A);
		SPI_WriteData(0x55); //77
		
		
		
		SPI_WriteComm(0x11);
		SPI_WriteData(0x00);				  // Sleep-Out
		 mdelay(200);
		
		SPI_WriteComm(0x29);
		SPI_WriteData(0x00);				  // Display On
#endif

#if 1//test
SPI_WriteComm(0xFF);
SPI_WriteData(0x77);
SPI_WriteData(0x01);
SPI_WriteData(0x00);
SPI_WriteData(0x00);
SPI_WriteData(0x12);

SPI_WriteComm(0xD1);
SPI_WriteData(0x81);

SPI_WriteComm(0xD2);
SPI_WriteData(0x06);		
#endif


	return platform_driver_register(&rk_screen_driver);
}

static void __exit rk_screen_exit(void)
{
	platform_driver_unregister(&rk_screen_driver);
}

fs_initcall(rk_screen_init);
module_exit(rk_screen_exit);

