#ifndef __BSP_LCD_ST7565_H
#define __BSP_LCD_ST7565_H

#include "main.h"

#define BSP_LCD_SPI hspi1
#define BSP_LCD_BL_PWM_TIM htim3
#define BSP_LCD_BL_PWM_CH TIM_CHANNEL_1

#define BSP_LCD_X_PIXELS 128
#define BSP_LCD_Y_PIXELS 64

void bsp_lcd_init(void);
void bsp_lcd_clear(void);
void bsp_lcd_6x8_str(uint8_t y, uint8_t x, const char *str);
void bsp_lcd_8x16_str(uint8_t y, uint8_t x, const char *str);
void bsp_lcd_bitmap(uint8_t x0, uint8_t y0, uint8_t x_lenth, uint8_t y_lenth, const uint8_t *bmp_tab);

#endif
