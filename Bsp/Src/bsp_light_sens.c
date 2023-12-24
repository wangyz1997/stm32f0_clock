#include "bsp_light_sens.h"
#include "main.h"

volatile uint16_t adc_value = 0;

void bsp_light_sens_start_convert(void)
{
  HAL_ADCEx_Calibration_Start(&BSP_LIGHT_SENS_ADC); //ADC校准
  HAL_ADC_Start_IT(&BSP_LIGHT_SENS_ADC);

  HAL_TIM_Base_Start(&BSP_ADC_TRIGGER_TIM);
}

uint8_t bsp_light_sens_get_level(void)
{
  return 100.0f * adc_value / 0x1000;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  adc_value = HAL_ADC_GetValue(hadc);
}
