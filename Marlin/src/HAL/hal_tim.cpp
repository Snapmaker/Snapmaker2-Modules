/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "stdio.h"
#include "hal_tim.h"
#include <src/HAL/std_library/inc/stm32f10x_tim.h>
#include <src/HAL/std_library/inc/stm32f10x.h>
//                           1     2     3     4
TIM_CB_F tim_cb_table[4] = {NULL, NULL, NULL, NULL};
static TIM_TypeDef * tim_table[] = {
  TIM1, TIM2, TIM3, TIM4
};
uint8_t tim_nvic_iqr_channel[] = {
  TIM1_UP_IRQn, TIM2_IRQn, TIM3_IRQn, TIM4_IRQn
};
uint32_t rcc_APB_Periph_tim[] = {
  RCC_APB2Periph_TIM1, RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4
};

static uint8_t tim_check(uint8_t tim) {
  if (tim >= sizeof(tim_table)/sizeof(tim_table[1])) {
    return 0;
  }
  return 1;
}
/*
uint8_t         tim;            // STM32F10X_MD TIM 2\3\4
uint16_t        u16Prescaler;    // 分频系数,不用减1,如72就是72分频
uint16_t        u16Period;       // 装入活动的自动重装载寄存器周期的值(设定周期)
*/
void HAL_timer_init(uint8_t tim, uint16_t u16Prescaler, uint16_t u16Period) {
  TIM_TimeBaseInitTypeDef tim_TimeBaseInitTypeDef;
  tim -= 1;
  if (!tim_check(tim)) {
    return ;
  }
  if (tim == 0) {
    RCC_APB2PeriphClockCmd(rcc_APB_Periph_tim[tim], ENABLE);
  } else {
    RCC_APB1PeriphClockCmd(rcc_APB_Periph_tim[tim], ENABLE);
  }
  tim_TimeBaseInitTypeDef.TIM_ClockDivision = TIM_CKD_DIV1;
  tim_TimeBaseInitTypeDef.TIM_CounterMode = TIM_CounterMode_Up;
  tim_TimeBaseInitTypeDef.TIM_Period = u16Period - 1;
  tim_TimeBaseInitTypeDef.TIM_Prescaler = u16Prescaler - 1;

  TIM_ARRPreloadConfig(tim_table[tim], ENABLE);
  TIM_TimeBaseInit(tim_table[tim], &tim_TimeBaseInitTypeDef);
  TIM_Cmd(tim_table[tim], DISABLE);
}

/*
uint8_t  PreemptionPriority;   // 抢占优先级(0,1,2,3) 数值越小优先级越高
uint8_t  SubPriority;          // 响应优先级(0,1,2,3) 数值越小优先级越高
*/
void HAL_timer_nvic_init(uint8_t tim, uint8_t PreemptionPriority, uint8_t SubPriority) {
  NVIC_InitTypeDef NVIC_InitStructure;
  tim -= 1;
  if (!tim_check(tim)) {
    return ;
  }
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = tim_nvic_iqr_channel[tim] ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ClearITPendingBit(tim_table[tim], TIM_IT_Update);
  TIM_ITConfig(tim_table[tim], TIM_IT_Update, ENABLE);
}
void HAL_timer_cb_init(uint8_t tim, TIM_CB_F cb) {
  tim -= 1;
  if (!tim_check(tim)) {
    return ;
  }
  tim_cb_table[tim] = cb;
}

void HAL_timer_enable(uint8_t tim) {
  TIM_Cmd(tim_table[tim - 1], ENABLE);
}

void HAL_timer_disable(uint8_t tim) {
  TIM_Cmd(tim_table[tim - 1], DISABLE);
}

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif

void __irq_tim1_up() {
  if (tim_cb_table[0]) {
    tim_cb_table[0]();
  }
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

void __irq_tim2() {
  if (tim_cb_table[1]) {
    tim_cb_table[1]();
  }
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void __irq_tim3() {
  if (tim_cb_table[2]) {
    tim_cb_table[2]();
  }
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void __irq_tim4() {
  if (tim_cb_table[3]) {
    tim_cb_table[3]();
  }
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

#ifdef __cplusplus
}
#endif