/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtthread.h>
#include <led.h>

#include "GUI.h"
#include "arm_math.h"
#include "lcd_st7735s.h"

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth_driver.h"
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif
#if 0
ALIGN(RT_ALIGN_SIZE)
static char thread_led2_stack[256];
struct rt_thread thread_led2;
static void rt_thread_entry_led2(void* parameter)
{
   
    //char str_buffer[256];
		//rt_hw_led_init();
		
    while(1)
    {
         /* led2 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n",count);
#endif       
        //rt_hw_led_on(1);
        rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* sleep 0.5 second and switch to other thread */

        /* led2 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        //rt_hw_led_off(1);
        rt_thread_delay( RT_TICK_PER_SECOND/2 );
    }
}
#endif
void rt_init_thread_entry(void* parameter)
{
    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif

    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif
		
		rt_hw_lcd_init("lcd", "spi1");
		//GUI_Init();            /* ≥ı ºªØST-emwin */ 
	while (1)
    {
			  rt_kprintf("thread1\r\n");
			  MainTask();

    }
}

int rt_application_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);
#if 0		
		//------- init led2 thread
    rt_thread_init(&thread_led2,
                   "led2",
                   rt_thread_entry_led2,
                   RT_NULL,
                   &thread_led2_stack[0],
                   sizeof(thread_led2_stack),11,5);
    rt_thread_startup(&thread_led2);
#endif
    return 0;
}

/*@}*/
