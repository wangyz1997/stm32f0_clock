#include "bsp_ds3231.h"
#include "main.h"

HAL_StatusTypeDef bsp_ds3231_update_time(uint8_t hour, uint8_t minute, uint8_t second)
{
  uint8_t write_buffer[3];
  
  if (second >= 60 || minute >= 60 || hour >= 24) {
    return HAL_ERROR;
  }
  
  write_buffer[0] = (second % 10) * 0x01 | (second / 10) * 0x10;
  write_buffer[1] = (minute % 10) * 0x01 | (minute / 10) * 0x10;
  write_buffer[2] = (hour % 10) * 0x01 | (hour / 10) * 0x10; //24小时模式

  return HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x00, 1, write_buffer, sizeof(write_buffer), 200); //写寄存器0x00-0x02
}

HAL_StatusTypeDef bsp_ds3231_update_date(uint8_t day_of_week, uint8_t year, uint8_t month, uint8_t date)
{
  uint8_t write_buffer[4];
  
  if (day_of_week > 7 || day_of_week == 0 || date > 31 || date == 0 || month > 12 || month == 0 || year > 99) {
    return HAL_ERROR;
  }
  
  write_buffer[0] = day_of_week;
  write_buffer[1] = (date % 10) * 0x01 | (date / 10) * 0x10;
  write_buffer[2] = (1 << 7) | ((month % 10) * 0x01 | (month / 10) * 0x10); //月份寄存器的最高位是世纪标志位
  write_buffer[3] = (year % 10) * 0x01 | (year / 10) * 0x10;
  
  return HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x03, 1, write_buffer, sizeof(write_buffer), 200); //写寄存器0x03-0x06
}

HAL_StatusTypeDef bsp_ds3231_update_time_date(const ds3231_date_time_info_t *info)
{
  uint8_t write_buffer[7];
  
  if (info->second >= 60 || info->minute >= 60 || info->hour >= 24 || info->day_of_week > 7 || info->day_of_week == 0 ||
      info->date > 31 || info->date == 0 || info->month > 12 || info->month == 0 || info->year > 99) {
    return HAL_ERROR;
  }
  
  write_buffer[0] = (info->second % 10) * 0x01 | (info->second / 10) * 0x10;
  write_buffer[1] = (info->minute % 10) * 0x01 | (info->minute / 10) * 0x10;
  write_buffer[2] = (info->hour % 10) * 0x01 | (info->hour / 10) * 0x10; //24小时模式
  write_buffer[3] = info->day_of_week;
  write_buffer[4] = (info->date % 10) * 0x01 | (info->date / 10) * 0x10;
  write_buffer[5] = (1 << 7) | ((info->month % 10) * 0x01 | (info->month / 10) * 0x10); //月份寄存器的最高位是世纪标志位
  write_buffer[6] = (info->year % 10) * 0x01 | (info->year / 10) * 0x10;
  
  return HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x00, 1, write_buffer, sizeof(write_buffer), 200); //写寄存器0x00-0x06
}

HAL_StatusTypeDef bsp_ds3231_get_time_date(ds3231_date_time_info_t *info)
{
  uint8_t read_buffer[7];
  uint8_t bcd_high, bcd_low;

  HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x00, 1, read_buffer, sizeof(read_buffer), 200);
  if (res != HAL_OK) {
    return res;
  }
  
  //秒
  bcd_low = read_buffer[0] & 0x0F; bcd_high = (read_buffer[0] & 0x70) >> 4;
  info->second = bcd_low + bcd_high * 10;
  //分
  bcd_low = read_buffer[1] & 0x0F; bcd_high = (read_buffer[1] & 0x70) >> 4;
  info->minute = bcd_low + bcd_high * 10;
  //时
  if (read_buffer[2] & 0x40) { //12小时模式
    bcd_low = read_buffer[2] & 0x0F;
    info->hour = bcd_low + (read_buffer[2] & 0x10) ? 10 : 0 + (read_buffer[2] & 0x20) ? 12 : 0;
  } else { //24小时模式
    bcd_low = read_buffer[2] & 0x0F; bcd_high = (read_buffer[2] & 0x30) >> 4;
    info->hour = bcd_low + bcd_high * 10;
  }
  //星期
  info->day_of_week = read_buffer[3] & 0x07;
  //日
  bcd_low = read_buffer[4] & 0x0F; bcd_high = (read_buffer[4] & 0x30) >> 4;
  info->date = bcd_low + bcd_high * 10;
  //月
  bcd_low = read_buffer[5] & 0x0F; bcd_high = (read_buffer[5] & 0x10) >> 4;
  info->month = bcd_low + bcd_high * 10;
  //年
  bcd_low = read_buffer[6] & 0x0F; bcd_high = (read_buffer[6] & 0xF0) >> 4;
  info->year = bcd_low + bcd_high * 10;
  
  return HAL_OK;
}

HAL_StatusTypeDef bsp_ds3231_init(void)
{
  uint8_t read_write_buffer[2];
  
  read_write_buffer[0] = 0x00; //电池供电时使能振荡器 方波输出频率1Hz 关闭电池供电时的时钟输出 关闭闹钟中断输出
  read_write_buffer[1] = 0x00; //关闭时钟输出
      
  HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x0E, 1, read_write_buffer, 2, 200); //写寄存器0x0E-0x0F
  if (res != HAL_OK) {
    return res;
  }
  
  res = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x02, 1, read_write_buffer, 1, 200); //读寄存器0x02
    if (res != HAL_OK) {
    return res;
  }
  if (read_write_buffer[0] & (1 << 6)) {
    read_write_buffer[0] &= ~(1 << 6); //切换至默认的24小时制
    
    res = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x02, 1, read_write_buffer, 1, 200); //写回寄存器0x02
      if (res != HAL_OK) {
      return res;
    }
  }
  
  res = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x05, 1, read_write_buffer, 1, 200); //读寄存器0x05
    if (res != HAL_OK) {
    return res;
  }
  if ((read_write_buffer[0] & (1 << 7)) == 0x00) {
    read_write_buffer[0] |= (1 << 7); //切换世纪位为1
    
    res = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x05, 1, read_write_buffer, 1, 200); //写回寄存器0x05
      if (res != HAL_OK) {
      return res;
    }
  }
  
  return HAL_OK;
}
