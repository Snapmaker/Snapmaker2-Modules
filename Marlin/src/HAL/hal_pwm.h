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

/******************************************************************************
// 定时器引脚：

定时器	引脚重映像		通道      	1	  2	   3	4

TIM1	0 没重映像 				PA8   PA9  PA10  PA11 
		1 部分重映射		        PA7   PB0  PB1
		2 完全重映相			    PE9	  PE11 PE13	 PE14

TIM2	0 没重映相				PA0   PA1  PA2   PA3 
		1 部分重映相1			PA15  PB3  PA2	 PA3
		2 部分重映像2			PA0   PA1  PB10	 PB11
		3 完全从映像			    PA15  PB3  PB10  PB11

TIM3	0 没重映相				PA6   PA7  PB0   PB1
		1 部分重映像			    PB4   PB5  PB0   PB1
		2 完全重映像			    PC6   PC7  PC8   PC9

TIM4	0 没重映像				PB6   PB7  PB8   PB9
		1 完全重映像			    PD12  PD13 PD14  PD15

TIM5    0 没重映像				PA0   PA1  PA2   PA3

定时器	通道	1	    2	  3	    4
TIM1 		PA8  PA9  PA10  PA11 
TIM2 		PA0  PA1  PA2   PA3 
TIM3 		PA6  PA7  PB0   PB1 
TIM4 		PB6  PB7  PB8   PB9
TIM5		PA0  PA1  PA2   PA3
***************************************************************************/

#ifndef FIRMWARE_USER_HARDWARE_PWM_H_
#define FIRMWARE_USER_HARDWARE_PWM_H_

typedef enum {
	PWM_TIM1_CH1,
	PWM_TIM1_CH2,
	PWM_TIM1_CH3,
	PWM_TIM1_CH4,
	PWM_TIM2_CH1,
	PWM_TIM2_CH2,
	PWM_TIM2_CH3,
	PWM_TIM2_CH4,
	PWM_TIM3_CH1,
	PWM_TIM3_CH2,
	PWM_TIM3_CH3,
	PWM_TIM3_CH4,
	PWM_TIM4_CH1,
	PWM_TIM4_CH2,
	PWM_TIM4_CH3,
	PWM_TIM4_CH4,
	PWM_TIM5_CH1,
	PWM_TIM5_CH2,
	PWM_TIM5_CH3,
	PWM_TIM5_CH4,
} PWM_TIM_CHN_E;

typedef enum {
	PWM_TIM1,
	PWM_TIM2,
	PWM_TIM3,
	PWM_TIM4,
	PWM_TIM5,
	PWM_TIM1_PARTIAL,
	PWM_TIM2_PARTIAL1,
	PWM_TIM2_PARTIAL2,
	PWM_TIM3_PARTIAL,
    PWM_TIM1_FULL,
	PWM_TIM2_FULL,
	PWM_TIM3_FULL,
	PWM_TIM4_FULL,
} PWM_TIM_E;

typedef enum {
	PWM_CH1,
	PWM_CH2,
	PWM_CH3,
	PWM_CH4,
} PWM_CHN_E;

void HAL_PwnConfig(uint8_t tim, uint8_t chn, uint32_t freq, uint16_t period) ;
void HAL_PwmInit(PWM_TIM_CHN_E tim_chn, uint8_t pin, uint32_t freq, uint16_t period);
void HAL_PwmInit(uint8_t tim, uint8_t chn, uint8_t pin, uint32_t freq, uint16_t period);
void HAL_PwmSetPulse(PWM_TIM_CHN_E tim_chn, uint16_t pulse);
void HAL_PwmSetPulse(uint8_t tim, uint8_t chn, uint16_t pulse);

#endif  // FIRMWARE_USER_HARDWARE_PWM_H_
