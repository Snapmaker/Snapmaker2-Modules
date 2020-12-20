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

#ifndef FIRMWARE_USER_HARDWARE_TIM_H_
#define FIRMWARE_USER_HARDWARE_TIM_H_

typedef void(*TIM_CB_F) (void);  // 中断回调函数

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif
void __irq_tim2(void);
void __irq_tim3(void);
void __irq_tim4(void);
#ifdef __cplusplus
}
#endif
/*
uint8_t         tim;            // STM32F10X_MD TIM 2\3\4
uint16_t        u16Prescaler;    // 分频系数,不用减1,如72就是72分频
uint16_t        u16Period;       // 装入活动的自动重装载寄存器周期的值(设定周期)
*/
extern void HAL_timer_init(uint8_t tim, uint16_t u16Prescaler, uint16_t u16Period);
/*
uint8_t  PreemptionPriority;   // 抢占优先级(0,1,2,3) 数值越小优先级越高
uint8_t  SubPriority;          // 响应优先级(0,1,2,3) 数值越小优先级越高
*/
extern void HAL_timer_nvic_init(uint8_t tim, uint8_t PreemptionPriority, uint8_t SubPriority);
extern void HAL_timer_cb_init(uint8_t tim, TIM_CB_F cb);
extern void HAL_timer_enable(uint8_t tim);
#endif  // FIRMWARE_USER_HARDWARE_TIM_H_

