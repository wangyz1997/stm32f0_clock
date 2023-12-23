#include "app_main.h"
#include "main.h"
#include "app_main.h"
#include "bsp_lcd_st7565.h"
#include "bsp_ds3231.h"
#include "bsp_ds18b20.h"

#include <stdio.h>

#define LCD_8X16_STR_CENTERED(y, str) lcd_8x16_str((y), (BSP_LCD_X_PIXELS - (sizeof(str) - 1) * 8) / 2, str)
#define LCD_6X8_STR_CENTERED(y, str) lcd_6x8_str((y), (BSP_LCD_X_PIXELS - (sizeof(str) - 1) * 6) / 2, str)

void app_init(void)
{
  lcd_init();
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, GPIO_PIN_SET);
  
  LCD_8X16_STR_CENTERED(0, "LCD Display Test");
  LCD_6X8_STR_CENTERED(4, "By Wangyz");
  
  HAL_StatusTypeDef res = bsp_ds3231_init();
  if (res == HAL_OK) {
    LCD_6X8_STR_CENTERED(7, "DS3231 init ok    ");
  } else {
    LCD_6X8_STR_CENTERED(7, "DS3231 init error!");
  }
	
  HAL_Delay(1000);
	lcd_clear();
}

void app_main(void)
{
  
  ds3231_date_time_info_t date_time_info;
  char str[32];
  uint32_t counter = 0;
  
  while (1)
  {
		bsp_ds3231_get_time_date(&date_time_info);

    sprintf(str, "20%02d.%02d.%02d  %02d:%02d:%02d", date_time_info.year, date_time_info.month, date_time_info.date,
                                               date_time_info.hour, date_time_info.minute, date_time_info.second);
    lcd_6x8_str(1, 0, str);
		
    if (counter % 1000 == 0) {
      bsp_ds18b20_start_conv();
    } else if (counter % 1000 == 800) {
      float temp;
      bsp_ds18b20_get_temp(&temp);
      sprintf(str, "%5.1fC", temp);
      lcd_6x8_str(2, 0, str);
    }

    if (HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET) {
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    } else if (HAL_GPIO_ReadPin(KEY_OK_GPIO_Port, KEY_OK_Pin) == GPIO_PIN_RESET) {
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		} else if (HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET) {
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		} else {
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
    
    counter ++;
    HAL_Delay(1);
  }
}
