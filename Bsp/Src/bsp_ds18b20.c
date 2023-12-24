#include "bsp_ds18b20.h"
#include "main.h"

static void bsp_ds18b20_delay_us(uint16_t delay_us) //实际延时时间会比delay_us长约2us
{
  __HAL_TIM_SET_COUNTER(&ONEWIRE_DELAY_TIMER, 0);
  __HAL_TIM_ENABLE(&ONEWIRE_DELAY_TIMER); //启动定时器
  while(__HAL_TIM_GET_COUNTER(&ONEWIRE_DELAY_TIMER) < delay_us);
  __HAL_TIM_DISABLE(&htim6);
}

static void bsp_ds18b20_reset_bus(void)
{
  HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET); //拉低DQ
	bsp_ds18b20_delay_us(500); //拉低500us 复位总线
  HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef bsp_ds18b20_check_present(void)
{
  uint16_t retry = 0;

  while(HAL_GPIO_ReadPin(ONEWIRE_GPIO_Port, ONEWIRE_Pin) == GPIO_PIN_SET && retry < 100) { //等待最长60us
    retry ++;
    bsp_ds18b20_delay_us(1);
  }

  if(retry >= 100) {
    return HAL_ERROR;
  }

  bsp_ds18b20_delay_us(480); //检测存在Slot必须大于480us

  return HAL_OK;
}

static uint8_t bsp_ds18b20_read_byte(void)
{
  uint8_t byte = 0;

  for(uint8_t i = 0; i < 8; i ++) {
    byte >>= 1;
    
    HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET);
    bsp_ds18b20_delay_us(2); //拉低2us 开始读取
    HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);

    bsp_ds18b20_delay_us(13);
    if(HAL_GPIO_ReadPin(ONEWIRE_GPIO_Port, ONEWIRE_Pin) == GPIO_PIN_SET) {
      byte |= 0x80; //最高位为1
    }
    
    bsp_ds18b20_delay_us(45);
    bsp_ds18b20_delay_us(1); //1us recover time
  }

  return byte;
}

static void bsp_ds18b20_write_byte(uint8_t byte)
{
  for(uint8_t i = 0; i < 8; i ++) {
    if(byte & 0x01) {
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET);
      bsp_ds18b20_delay_us(10);
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
      bsp_ds18b20_delay_us(50);
    } else {
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_RESET);
      bsp_ds18b20_delay_us(50);
      HAL_GPIO_WritePin(ONEWIRE_GPIO_Port, ONEWIRE_Pin, GPIO_PIN_SET);
      bsp_ds18b20_delay_us(10);
    }
    bsp_ds18b20_delay_us(1); //1us recover time
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
	bsp_ds18b20_reset_bus();

  HAL_StatusTypeDef res = bsp_ds18b20_check_present();
  if (res != HAL_OK) {
    return res;
  }

	bsp_ds18b20_write_byte(0xCC); //跳过ROM
	bsp_ds18b20_write_byte(0xBE); //读Scratchpad
	uint8_t temp_lsb = bsp_ds18b20_read_byte(); //Scratchpad第0字节 温度LSB
	uint8_t temp_msb = bsp_ds18b20_read_byte(); //Scratchpad第1字节 温度MSB

	if(temp_msb & 0x80) { //温度为负
    int16_t temp = (~temp_msb) << 8 | (~temp_lsb); //转换为正数
    *temp_out = -0.0625f * temp; //输出负数
	} else { //温度为正
    int16_t temp = temp_msb << 8 | temp_lsb;
    *temp_out = 0.0625f * temp; //12位转换精度 每LSB等于0.0625度
	}

  return HAL_OK;
}
