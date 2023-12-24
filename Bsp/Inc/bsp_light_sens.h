#ifndef __BSP_LIGHT_SENS_H
#define __BSP_LIGHT_SENS_H

#include "stm32f0xx_hal.h"

#define BSP_LIGHT_SENS_ADC hadc
#define BSP_ADC_TRIGGER_TIM htim15

void bsp_light_sens_start_convert(void);
uint8_t bsp_light_sens_get_level(void);

#endif
