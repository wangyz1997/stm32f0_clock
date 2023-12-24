#include "bsp_buzzer.h"
#include "main.h"

const uint16_t note_frequency_table[] = {
//Low  do    do#   re    re#   mi    fa    fa#   so    so#   ra    ra#   si
       262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466,  494,
//Med  do    do#   re    re#   mi    fa    fa#   so    so#   ra    ra#   si
       523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932,  988,
//High do    do#   re    re#   mi    fa    fa#   so    so#   ra    ra#   si
       1046, 1109, 1175, 1245, 1318, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
//     Beep
       2700,
};

const uint8_t song_happy_birthday[58] = {
  M5, 50, STOP, 1, M5, 20, M6, 60, M5, 60, H1, 60, M7, 120,
  M5, 40, STOP, 1, M5, 20, M6, 60, M5, 60, H2, 60, H1, 120,
  M5, 40, STOP, 1, M5, 20, H5, 60, H3, 60, H1, 60, M7,  60, M6, 120,
  H4, 40, STOP, 1, H4, 20, H3, 60, H1, 60, H2, 60, H1, 120,
};


//unsigned char code SkyCastle[] = {
//l6,t8, l7,t8, m1,t4_l, l7,t8, m1,t4, m3,t4, l7,t2_l,
//l3,t4, l6,t4_l, l5,t8, l6,t4, m1,t4, l5,t2_l,
//l3,t4, l4,t8, l3,t8, m1,t4, m1,t4, l3,t2_l,
//STOP
//};

static uint8_t bsp_buzzer_volume_percent = 30;

void bsp_buzzer_start(void)
{
  HAL_TIM_Base_Start(&BSP_BUZZER_PWM_TIM);
}

void bsp_buzzer_play_note(bsp_buzzer_key key)
{
  if (key == STOP) {
    HAL_TIM_PWM_Stop(&BSP_BUZZER_PWM_TIM, BSP_BUZZER_PWM_CH);
    return;
  } else {
    uint32_t psc = SystemCoreClock / note_frequency_table[key];
    uint32_t oc = psc * bsp_buzzer_volume_percent / 100;
    
    __HAL_TIM_SET_AUTORELOAD(&BSP_BUZZER_PWM_TIM, psc); //32位定时器
    __HAL_TIM_SET_COMPARE(&BSP_BUZZER_PWM_TIM, BSP_BUZZER_PWM_CH, oc);
    
    HAL_TIM_PWM_Start(&BSP_BUZZER_PWM_TIM, BSP_BUZZER_PWM_CH);
  }
}

void bsp_buzzer_play_song(const uint8_t *note_table, uint32_t note_count)
{
  for (uint32_t i = 0; i < note_count; i ++) {
    bsp_buzzer_play_note(note_table[i * 2]);
    HAL_Delay(note_table[i * 2 + 1] * 8);
  }
  
  bsp_buzzer_play_note(STOP);
}

//#define TICK 60

//#define t1 TICK*4//全音符
//#define t2_l TICK*3//长二分音符
//#define t2 TICK*2//二分音符
//#define t4_l TICK*1.5//长四分音符
//#define t4 TICK//四分音符
//#define t8 TICK/2//八分音符
//#define t16 TICK/4//十六分音符


