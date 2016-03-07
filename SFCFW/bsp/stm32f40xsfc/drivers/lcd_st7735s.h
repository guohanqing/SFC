#ifndef LCD_ST7735S_H_INCLUDED
#define LCD_ST7735S_H_INCLUDED

/*
 Compatible list:
 ili9320 ili9325 ili9328
 LG4531
*/

/* LCD color */
#define White            0xFFFF
#define Black            0x0000
#define Grey             0xF7DE
#define Blue             0x001F
#define Blue2            0x051F
#define Red              0xF800
#define Magenta          0xF81F
#define Green            0x07E0
#define Cyan             0x7FFF
#define Yellow           0xFFE0

/*---------------------- Graphic LCD size definitions ------------------------*/
#define LCD_WIDTH       128                /* Screen Width (in pixels)           */
#define LCD_HEIGHT      160                 /* Screen Hight (in pixels)           */
#define BPP             16                  /* Bits per pixel                     */
#define BYPP            ((BPP+7)/8)         /* Bytes per pixel                    */

extern void lcd_Initializtion(void);
extern unsigned int lcd_getdeviceid(void);

#include <stdint.h>
#include <rtthread.h>


#include <drivers/spi.h>

struct spi_lcd_device
{
    struct rt_device                lcd_device;
    struct rt_spi_device *          rt_spi_device;
    struct rt_mutex                 lock;
};

void write_cmd(uint8_t cmd);
void write_data8(uint8_t dat );
void write_data16(uint16_t dat );
uint8_t read_data8(void);
uint16_t read_data16(void);
void lcd_Initializtion(void);
void rt_hw_lcd_set_pixel(const char *pixel, int x, int y);
void rt_hw_lcd_get_pixel(char *pixel, int x, int y);

rt_err_t rt_hw_lcd_init(const char * lcd_device_name, const char * spi_device_name);

//extern void rt_hw_lcd_update(rtgui_rect_t *rect);
//extern rt_uint8_t * rt_hw_lcd_get_framebuffer(void);
//extern void rt_hw_lcd_set_pixel(rtgui_color_t *c, rt_base_t x, rt_base_t y);
//extern void rt_hw_lcd_get_pixel(rtgui_color_t *c, rt_base_t x, rt_base_t y);
//extern void rt_hw_lcd_draw_hline(rtgui_color_t *c, rt_base_t x1, rt_base_t x2, rt_base_t y);
//extern void rt_hw_lcd_draw_vline(rtgui_color_t *c, rt_base_t x, rt_base_t y1, rt_base_t y2);
//extern void rt_hw_lcd_draw_raw_hline(rt_uint8_t *pixels, rt_base_t x1, rt_base_t x2, rt_base_t y);

#endif // LCD_ST7735S_H_INCLUDED
