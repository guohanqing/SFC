
#include <stdint.h>
#include "stm32f4xx.h"
#include "board.h"
#include "lcd_st7735s.h"
#include <drivers/spi.h>
#include "stm32f20x_40x_spi.h"

// Compatible list:
// ili9320 ili9325 ili9328
// LG4531

//内联函数定义,用以提高性能
#ifdef __CC_ARM                			 /* ARM Compiler 	*/
#define lcd_inline   				static __inline
#elif defined (__ICCARM__)        		/* for IAR Compiler */
#define lcd_inline 					inline
#elif defined (__GNUC__)        		/* GNU GCC Compiler */
#define lcd_inline 					static __inline
#else
#define lcd_inline                 static
#endif

//#define rw_data_prepare()               write_cmd(34)



#define BRIGHT_MAX		255		/* 最大亮度 */
#define BRIGHT_MIN		0		/* 最小亮度,本例设置为0 */
#define BRIGHT_DEFAULT	200		/* 缺省亮度 */

#define BRIGHT_STEP		5	   	/* PWM值调节步长 */

struct spi_lcd_device spi_lcd_device;

static void lcd_lock(struct spi_lcd_device * lcd_device);
static void lcd_unlock(struct spi_lcd_device * lcd_device);


//输出重定向.当不进行重定向时.
#define printf               rt_kprintf //使用rt_kprintf来输出
//#define printf(...)                       //无输出

/* LCD is connected to the FSMC_Bank1_NOR/SRAM2 and NE2 is used as ship select signal */
/* RS <==> A2 */
//#define LCD_REG              (*((volatile unsigned short *) 0x64000000)) /* RS = 0 */
//#define LCD_RAM              (*((volatile unsigned short *) 0x64000008)) /* RS = 1 */
/*
static void delay(int cnt)
{
    volatile unsigned int dl;
    while(cnt--)
    {
        for(dl=0; dl<500; dl++);
    }
}*/

 void write_cmd(uint8_t cmd)
{
		/*
		LCD_A0 --- PA12
		*/
		/* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_lcd_device.rt_spi_device, &cfg);
    }	
    
		{   
							
        lcd_lock(&spi_lcd_device);
			
				GPIO_ResetBits(GPIOA,GPIO_Pin_12);/* command:A0=0;data or parameter:A0=1*/

        rt_spi_send(spi_lcd_device.rt_spi_device, &cmd, 1);
			
				lcd_unlock(&spi_lcd_device);
		}
}

 void write_data8(uint8_t dat )
{
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_lcd_device.rt_spi_device, &cfg);
    }			
		{
        lcd_lock(&spi_lcd_device);
			
				GPIO_SetBits(GPIOA,GPIO_Pin_12);/* command:A0=0;data or parameter:A0=1*/

        rt_spi_send(spi_lcd_device.rt_spi_device, &dat, 1);
			
				lcd_unlock(&spi_lcd_device);
		}
}
 void write_data16(uint16_t dat )
{
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 16;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_lcd_device.rt_spi_device, &cfg);
    }			
		{
        lcd_lock(&spi_lcd_device);
			
				GPIO_SetBits(GPIOA,GPIO_Pin_12);/* command:A0=0;data or parameter:A0=1*/

        rt_spi_send(spi_lcd_device.rt_spi_device, &dat, 1);
			
				lcd_unlock(&spi_lcd_device);
		}
}
 uint8_t read_data8(void)
{
		uint8_t dat = 0;
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_lcd_device.rt_spi_device, &cfg);
    }			
		{
        lcd_lock(&spi_lcd_device);
			
				GPIO_SetBits(GPIOA,GPIO_Pin_12);/* command:A0=0;data or parameter:A0=1*/

        rt_spi_recv(spi_lcd_device.rt_spi_device, &dat, 1);
			
				lcd_unlock(&spi_lcd_device);
		}
		return dat;
}
 uint16_t read_data16(void)
{
		uint16_t dat = 0;
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 16;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_lcd_device.rt_spi_device, &cfg);
    }			
		{
        lcd_lock(&spi_lcd_device);
			
				GPIO_SetBits(GPIOA,GPIO_Pin_12);/* command:A0=0;data or parameter:A0=1*/

        rt_spi_recv(spi_lcd_device.rt_spi_device, &dat, 1);
			
				lcd_unlock(&spi_lcd_device);
		}
		return dat;
}
lcd_inline void lcd_write_ram_prepare(void)
{
		write_cmd(0x2c);/*RAMWR (2Ch): Memory Write*/
}
lcd_inline void lcd_read_ram_prepare(void)
{
		write_cmd(0x2e);/*RAMRD (2Eh): Memory Read*/
}


/********* control <只移植以上函数即可> ***********/

static unsigned short deviceid = 0;//设置一个静态变量用来保存LCD的ID

//返回LCD的ID
unsigned int lcd_getdeviceid(void)
{
    return deviceid;
}
/*
static uint16_t BGR2RGB(unsigned short c)
{
    u16  r, g, b, rgb;

    b = (c>>0)  & 0x1f;
    g = (c>>5)  & 0x3f;
    r = (c>>11) & 0x1f;

    rgb =  (b<<11) + (g<<5) + (r<<0);

    return( rgb );
}*/

static void lcd_set_cursor(uint16_t xs,uint16_t ys,uint16_t xe,uint16_t ye)
{
    write_cmd(0x2a);	/*Column Address Set*/
		write_data16(xs);
		write_data16(xe);
		write_cmd(0x2b);	/*Row Address Set*/
		write_data16(ys);
		write_data16(ye);
	
}

/*
static void lcd_clear(uint16_t color)
{
    unsigned int index = 0;
	
		lcd_set_cursor(0,0,LCD_WIDTH,LCD_HEIGHT);
	
		// Prepare to write GRAM 
		lcd_write_ram_prepare();

    for (index = 0; index < (LCD_WIDTH*LCD_HEIGHT); index++)
    {
        write_data16(color);
    }
}*/


void lcd_Initializtion(void)
{
    

    //数据总线测试,用于测试硬件连接是否正常.
    //lcd_data_bus_test();

    //清屏
    //lcd_clear( Blue );
		//RST=0;					//??
		//delay(200);
		//RST=1;
		//delay(20);
	
    write_cmd(0x11);   /* sleep out    */
	
		/* ST7735R Frame Rate */
    write_cmd(0xb1);   /* FRMCTR1 (B1h): Frame Rate Control (In normal mode/ Full colors) */  
    write_data8(0x01);
    write_data8(0x2c);
    write_data8(0x2d);

    write_cmd(0xb2);  /* FRMCTR2 (B2h): Frame Rate Control (In Idle mode/ 8-colors) */  
    write_data8(0x01);
    write_data8(0x2c);
    write_data8(0x2d);
   
    write_cmd(0xb3); /* FRMCTR3 (B3h): Frame Rate Control (In Partial mode/ full colors) */   
    write_data8(0x01);
    write_data8(0x2c);
    write_data8(0x2d);
    write_data8(0x01);
    write_data8(0x2d);  
    write_data8(0x2d);
    
			
    write_cmd(0xb4);  /* INVCTR (B4h): Display Inversion Control */  
    write_data8(0x07);  

		/*ST7735R Power Sequence */  
    write_cmd(0xc0);  /* PWCTR1 (C0h): Power Control 1 */  
    write_data8(0xa2);
    write_data8(0x02);
		write_data8(0x84);
    

    write_cmd(0xc1); /* PWCTR2 (C1h): Power Control 2 */   
    write_data8(0xc5);
 
    write_cmd(0xc2); /* PWCTR3 (C2h): Power Control 3 (in Normal mode/ Full colors) */   
    write_data8(0x0a);
    write_data8(0x00);
    

    write_cmd(0xc3); /* PWCTR4 (C3h): Power Control 4 (in Idle mode/ 8-colors) */   
    write_data8(0x8a);
    write_data8(0x2a);
    
    write_cmd(0xc4); /* PWCTR5 (C4h): Power Control 5 (in Partial mode/ full-colors) */   
    write_data8(0x8a);
    write_data8(0xee);
    
		/* VCOM Control */
    write_cmd(0xc5); /* VMCTR1 (C5h): VCOM Control 1 */   
    write_data8(0x0e);

		/* MX, MY, RGB mode */
		write_cmd(0x36);   /* MADCTL (36h): Memory Data Access Control */
    write_data8(0xC8);   //MX=1(Column Address Order:????),MY=1(Row Address Order:????),MV=0(Row/Column Exchange),
												//ML=0(Vertical Refresh Order:????),RGB = 0(RGB-BGR ORDER RGB),MH=0(Horizontal Refresh Order:????)
       
    /* ST7735R Gamma Sequence */
    write_cmd(0xe0); /* GMCTRP1 (E0h): Gamma (+ Polarity) Correction Characteristics Setting */   
    write_data8(0x02);
    write_data8(0x1c);
    write_data8(0x07);
    write_data8(0x12);
    write_data8(0x37);
    write_data8(0x32);
    write_data8(0x29);
    write_data8(0x2d);
    write_data8(0x29);
    write_data8(0x25);
    write_data8(0x2b);
    write_data8(0x39);
    write_data8(0x00);
    write_data8(0x01);
    write_data8(0x03);    
    write_data8(0x10);

    write_cmd(0xe1);  /* GMCTRN1 (E1h): Gamma (- polarity) Correction Characteristics Setting */  
    write_data8(0x0b);
    write_data8(0x14);
    write_data8(0x09);
    write_data8(0x26);
    write_data8(0x27);
    write_data8(0x22);
    write_data8(0x1c);
    write_data8(0x20);
    write_data8(0x1d);
    write_data8(0x1a);
    write_data8(0x25);
    write_data8(0x2d);
    write_data8(0x06);
    write_data8(0x06);
    write_data8(0x02);    
    write_data8(0x0f);

		 
    write_cmd(0x3a);  /* COLMOD (3Ah): Interface Pixel Format */  
    write_data8(0x05); //65K Mode


    write_cmd(0x2a);  /* CASET (2Ah): Column Address Set */
    write_data8(0x00);
    write_data8(0x00);
    write_data8(0x00);	
    write_data8(0x7F);  
    
    write_cmd(0x2b);   /*  RASET (2Bh): Row Address Set */
    write_data8(0x00);
    write_data8(0x00);  
		write_data8(0x00);
    write_data8(0x9F);

    
    write_cmd(0x29);		/* Display On */
}

/*  设置像素点 颜色,X,Y */
void rt_hw_lcd_set_pixel(const char *pixel, int x, int y)
{
    lcd_set_cursor(x,y,x+1,y+1);

    /* Prepare to write GRAM */
		lcd_write_ram_prepare();
	
    write_data16(*(rt_uint16_t*)pixel);
}

/* 获取像素点颜色 */
void rt_hw_lcd_get_pixel(char *pixel, int x, int y)
{
		lcd_set_cursor(x,y,x+1,y+1);
	
	  lcd_read_ram_prepare();
	
    *(rt_uint16_t*)pixel = read_data16(); //BGR2RGB( lcd_read_gram(x,y) ); 
		
}

/* 画水平线 */
void rt_hw_lcd_draw_hline( const char *pixel, int x1, int y1, int x2)
{
	  lcd_set_cursor(x1,y1,x2,y1+1);
		
		lcd_write_ram_prepare();
	
    while (x1 < x2)
    {
        write_data16( *(rt_uint16_t*)pixel );
        x1++;
    }
}

/* 垂直线 */
void rt_hw_lcd_draw_vline(const char *pixel, int x1, int y1, int y2)
{
		lcd_set_cursor(x1,y1,x1+1,y2);
	
		lcd_write_ram_prepare();
	
    while (y1 < y2)
    {
        write_data16( *(rt_uint16_t*)pixel );
        y1++;
    }
}

/* draw blit dot */
void rt_hw_lcd_draw_blit_dot(const char *pixels, int x, int y, rt_size_t size)
{
    rt_uint16_t *ptr;

    ptr = (rt_uint16_t*)pixels;
	
		lcd_set_cursor(x,y,x+1,y+1);
	
		lcd_write_ram_prepare();

    	
    while (size)
    {
        write_data16(*ptr ++);
        size --;
    }
}

struct rt_device_graphic_ops lcd_st7735_ops =
{
    rt_hw_lcd_set_pixel,
    rt_hw_lcd_get_pixel,
    rt_hw_lcd_draw_hline,
    rt_hw_lcd_draw_vline,
    rt_hw_lcd_draw_blit_dot
};

struct rt_device _lcd_device;
static rt_err_t lcd_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case RTGRAPHIC_CTRL_GET_INFO:
    {
        struct rt_device_graphic_info *info;

        info = (struct rt_device_graphic_info*) args;
        RT_ASSERT(info != RT_NULL);

        info->bits_per_pixel = 16;
        info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
        info->framebuffer = RT_NULL;
        info->width = LCD_WIDTH;
        info->height = LCD_HEIGHT;
    }
    break;

    case RTGRAPHIC_CTRL_RECT_UPDATE:
        /* nothong to be done */
        break;

    default:
        break;
    }

    return RT_EOK;
}
/*
static void lcd_lock(struct spi_lcd_device * lcd_device)
{
    rt_mutex_take(&lcd_device->lock, RT_WAITING_FOREVER);
}

static void lcd_unlock(struct spi_lcd_device * lcd_device)
{
    rt_mutex_release(&lcd_device->lock);
}*/


int rt_hw_spi_init(void)
{
	

		/*
		SPI1_SCK 	--- PA5
		SPI1_MISO --- PA6
		SPI1_MOSI	--- PB5	     
		SPI1_NSS 	--- PA4
		*/
		{	
        //static struct stm32_spi_bus stm32_spi;
        GPIO_InitTypeDef GPIO_InitStructure;
				SPI_InitTypeDef   SPI_InitStructure;

        /* Enable GPIO CRC clock */
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
			
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB ,ENABLE);
			  /* 开启 SPI1 外设时钟 */
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
        /* 配置 PA5、PA6，用于 SCK, MISO */
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5|GPIO_Pin_6;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
				/* 配置 PB5，用于 MOSI */
				GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 ;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
				/* 配置 PA4用于 SPI1 CS */
				GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 ;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
				
				/* 配置 SPI1工作模式 */
				SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
				SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
				SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
				SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
				SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
				SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 		/* 软件控制片选 */
				/*
					SPI_BaudRatePrescaler_64 对应SCK时钟频率约1M

					示波器实测频率
					SPI_BaudRatePrescaler_64 时，SCK时钟频率约 1.116M
					SPI_BaudRatePrescaler_32 时，SCK时钟频率月 2.232M
				*/
				SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
				SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
				SPI_InitStructure.SPI_CRCPolynomial = 7;
				SPI_Init(SPI1,&SPI_InitStructure);

				/* 使能 SPI1 */
				SPI_Cmd(SPI1,ENABLE);
				
      
    }
   		
		return 0;

}
INIT_DEVICE_EXPORT(rt_hw_spi_init);

/*
*********************************************************************************************************
*	函 数 名: set_backlight
*	功能说明: 初始化GPIO,配置为PWM模式
*	形    参：_bright 亮度，0是灭，255是最亮
*	返 回 值: 无
*********************************************************************************************************
*/
static void set_backlight(uint8_t bright)
{
	/*
		背光口线是 PA11, 复用功能选择 TIM1_CH4
	
		当关闭背光时，将CPU IO设置为浮动输入模式（推荐设置为推挽输出，并驱动到低电平)
		将TIM3关闭以节约功耗
	*/

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* 第1步：打开GPIOB RCC_APB2Periph_AFIO 的时钟	*/
	
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
			
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	if (bright == 0)
	{
		/* 配置背光GPIO为输入模式 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* 关闭TIM1 */
		TIM_Cmd(TIM1, DISABLE);
		return;
	}
	else if (bright == BRIGHT_MAX)	/* 最大亮度 */
	{
		/* 配置背光GPIO为推挽输出模式 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_SetBits(GPIOA, GPIO_Pin_11);

		/* 关闭TIM1 */
		TIM_Cmd(TIM1, DISABLE);
		return;
	}

	/* 配置背光GPIO为复用推挽输出模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* 开启 TIM1 外设时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	/*
		TIM1 配置: 产生1路PWM信号;
		TIM1CLK = 168 MHz, Prescaler = 0(不分频), TIM1 counter clock = 168 MHz
		计算公式：
		PWM输出频率 = TIM1 counter clock /(ARR + 1)

		我们期望设置为100Hz

		如果不对TIM3CLK预分频，那么不可能得到100Hz低频。
		我们设置分频比 = 1000， 那么  TIM1 counter clock = 168KHz
		TIM_Period = 1680 - 1;
		频率下不来。
	 */
	TIM_TimeBaseStructure.TIM_Period = 1680 - 1;	/* TIM_Period = TIM1 ARR Register */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channe4 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	/*
		_bright = 1 时, TIM_Pulse = 1
		_bright = 255 时, TIM_Pulse = TIM_Period
	*/
	TIM_OCInitStructure.TIM_Pulse = (TIM_TimeBaseStructure.TIM_Period * bright) / BRIGHT_MAX;	/* 改变占空比 */

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);
}

rt_err_t rt_hw_lcd_init(const char * lcd_device_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;
	
		/*initialize spi1 bus and register lcd cs*/
		rt_hw_spi_init();

    /* initialize mutex */
    if (rt_mutex_init(&spi_lcd_device.lock, spi_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init lcd lock mutex failed\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_lcd_device.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_lcd_device.rt_spi_device, &cfg);
    }

    /* LCD RESET */
    /* PD14 : LCD RESET */
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

        GPIO_ResetBits(GPIOD,GPIO_Pin_14);
        GPIO_SetBits(GPIOD,GPIO_Pin_14);
        /* wait for lcd reset */
        rt_thread_delay(1);
    }
		
		/* PA11 : LCD backlight */
		set_backlight(BRIGHT_MAX);
		
		
    /* register lcd device */
    _lcd_device.type  = RT_Device_Class_Graphic;
    _lcd_device.init  = lcd_init;
    _lcd_device.open  = lcd_open;
    _lcd_device.close = lcd_close;
    _lcd_device.control = lcd_control;
    _lcd_device.read  = RT_NULL;
    _lcd_device.write = RT_NULL;

    _lcd_device.user_data = &lcd_st7735_ops;
		
    //lcd_Initializtion();
		/* init lcd */
    
		

    /* register graphic device driver */
    rt_device_register(&_lcd_device, "lcd",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
		
		return RT_EOK;
}
