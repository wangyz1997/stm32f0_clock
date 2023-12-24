#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include "stm32f0xx_hal.h"

#define BSP_BUZZER_PWM_TIM htim2
#define BSP_BUZZER_PWM_CH TIM_CHANNEL_1

typedef enum {
  L1 = 0, LS1 = 1, L2 = 2, LS2 = 3, L3 = 4, L4 = 5,
  LS4 = 6, L5 = 7, LS5 = 8, L6 = 9, LS6 = 10, L7 = 11,
  M1 = 12, MS1 = 13, M2 = 14, MS2 = 15, M3 = 16, M4 = 17,
  MS4 = 18, M5 = 19, MS5 = 20, M6 = 21, MS6 = 22, M7 = 23,
  H1 = 24, HS1 = 25, H2 = 26, HS2 = 27, H3 = 28, H4 = 29,
  HS4 = 30, H5 = 31, HS5 = 32, H6 = 33, HS6 = 34, H7 = 35,
  BEEP = 36, STOP = 0xFF
} bsp_buzzer_key;

extern const uint8_t song_happy_birthday[58];

void bsp_buzzer_start(void);
void bsp_buzzer_play_note(bsp_buzzer_key key);
void bsp_buzzer_play_song(const uint8_t *note_table, uint32_t note_count);

#endif
