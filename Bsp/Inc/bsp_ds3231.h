#ifndef __BSP_DS3231_H
#define __BSP_DS3231_H

#include "stm32f0xx_hal.h"

typedef struct {
  uint8_t second; //0-59
  uint8_t minute; //0-59
  uint8_t hour; //0-23
  uint8_t day_of_week; //1-7
  uint8_t date; //1-31
  uint8_t month; //1-12
  uint8_t year; //00-99
} ds3231_date_time_info_t;

HAL_StatusTypeDef bsp_ds3231_update_time(uint8_t hour, uint8_t minute, uint8_t second);
HAL_StatusTypeDef bsp_ds3231_update_date(uint8_t day_of_week, uint8_t year, uint8_t month, uint8_t date);
HAL_StatusTypeDef bsp_ds3231_update_time_date(const ds3231_date_time_info_t *info);
HAL_StatusTypeDef bsp_ds3231_get_time_date(ds3231_date_time_info_t *info);
HAL_StatusTypeDef bsp_ds3231_init(void);

#endif
