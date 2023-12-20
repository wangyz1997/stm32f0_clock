#include "bsp_ds18b20.h"
#include "main.h"

static void bsp_ds18b20_delay_us(uint16_t delay_us)
{
  __HAL_TIM_SET_COUNTER(&ONEWIRE_DELAY_TIMER, 0);
  __HAL_TIM_ENABLE(&ONEWIRE_DELAY_TIMER); //启动定时器
  while(__HAL_TIM_GET_COUNTER(&ONEWIRE_DELAY_TIMER) < delay_us);
  __HAL_TIM_DISABLE(&htim6);
}

static void bsp_ds18b20_reset_bus(void)
{
  HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET); //拉低DQ
	bsp_ds18b20_delay_us(750); //拉低750us 复位总线
  HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
	bsp_ds18b20_delay_us(15); //15us
}

static HAL_StatusTypeDef bsp_ds18b20_check_present(void) 	   
{   
  uint16_t retry = 0;
  
  HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
  
  while(HAL_GPIO_ReadPin(ONEWIRE_GPIO_Port, ONEWIRE_Pin) == GPIO_PIN_SET && retry < 200) {
    retry ++;
    bsp_ds18b20_delay_us(1);
  }
	
  if(retry >= 200) {
    return HAL_ERROR;
  } else {
    retry = 0;
  }
  
  while(HAL_GPIO_ReadPin(ONEWIRE_GPIO_Port, ONEWIRE_Pin) == GPIO_PIN_RESET && retry < 300) {
    retry ++;
    bsp_ds18b20_delay_us(1);
	}
  
  if(retry>=240) {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

static uint8_t bsp_ds18b20_read_bit(void) 	 
{
  uint8_t bit;

  HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET);
  bsp_ds18b20_delay_us(2); //拉低2us 开始读取
  HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);

  bsp_ds18b20_delay_us(12);
  if(HAL_GPIO_ReadPin(ONEWIRE_GPIO_Port, ONEWIRE_Pin) == GPIO_PIN_SET) {
    bit = 0x80; //最高位为1
  } else {
    bit = 0;//最高位为0
  }

  return bit;
}

static uint8_t bsp_ds18b20_read_byte(void)     
{        
  uint8_t byte = 0;
  
  for(uint8_t i = 0; i < 8; i ++)
  {
    byte = bsp_ds18b20_read_bit() | (byte >> 1);//高位在前
    bsp_ds18b20_delay_us(50);
  }
  
  return byte;
}

static void bsp_ds18b20_write_byte(uint8_t byte)     
{  
  for(uint8_t i = 0; i < 8; i ++) {
    if(byte & 0x01) {
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET);
      bsp_ds18b20_delay_us(2);
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
      bsp_ds18b20_delay_us(60);
    } else {
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET);
      bsp_ds18b20_delay_us(60);
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
      bsp_ds18b20_delay_us(2);
    }
    byte >>= 1;
  }
}

HAL_StatusTypeDef bsp_ds18b20_start_conv(void) //开始温度转换
{
  bsp_ds18b20_reset_bus();
  
  HAL_StatusTypeDef res = bsp_ds18b20_check_present();
  if (res != HAL_OK) {
    return res;
  }
  
  bsp_ds18b20_write_byte(0xCC); //跳过ROM
  bsp_ds18b20_write_byte(0x44); //开始温度转换
  
  return HAL_OK;
}

HAL_StatusTypeDef bsp_ds18b20_get_temp(float *temp_out)
{
	uint8_t temp_flag;
	uint8_t temp_lsb, temp_msb;
  
	bsp_ds18b20_reset_bus();
  
  HAL_StatusTypeDef res = bsp_ds18b20_check_present();
  if (res != HAL_OK) {
    return res;
  }
  
	bsp_ds18b20_write_byte(0xCC); //跳过ROM
	bsp_ds18b20_write_byte(0xBE); //读Scratchpad
	temp_lsb = bsp_ds18b20_read_byte(); //Scratchpad第0字节 温度LSB
	temp_msb = bsp_ds18b20_read_byte(); //Scratchpad第1字节 温度MSB

	if(temp_msb & 0x80) { //温度为负 
    int16_t temp = (~temp_msb) << 8 | (~temp_lsb); //转换为正数
    *temp_out = -0.0625 * temp; //输出负数
	} else { //温度为正
    int16_t temp = temp_msb << 8 | temp_lsb;
    *temp_out = 0.0625f * temp;
	}
	
  return HAL_OK;
}
