#ifndef __BSP_DS18B20_H
#define __BSP_DS18B20_H

#include "stm32f0xx_hal.h"

#define ONEWIRE_DELAY_TIMER htim6

HAL_StatusTypeDef bsp_ds18b20_start_conv(void);//开始温度转换
HAL_StatusTypeDef bsp_ds18b20_get_temp(float *temp_out);

#endif
