#ifndef __BSP_LCD_ST7565_H
#define __BSP_LCD_ST7565_H

#include "main.h"

#define BSP_LCD_SPI_INST hspi1

#define BSP_LCD_X_PIXELS 128
#define BSP_LCD_Y_PIXELS 64

void lcd_init(void);
void lcd_clear(void);
void lcd_6x8_str(uint8_t y, uint8_t x, const char *str);
void lcd_8x16_str(uint8_t y, uint8_t x, const char *str);
void lcd_bitmap(uint8_t x0, uint8_t y0, uint8_t x_lenth, uint8_t y_lenth, const uint8_t *bmp_tab);

#endif
