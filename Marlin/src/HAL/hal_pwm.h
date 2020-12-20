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
//返回值：0成功，1失败


注意：TIM2 映射3 与 TIM3 重映射 2 ，需要自己调用下边的函数。否则不能输出波形
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//解决TIM2 复用功能部分映射3没有波形的问题
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//解决TIM3 复用功能部分映射1没有波形的问题

	TIM1输出仅测试了部分重映射的2通道，PB0输出
*/
/***************************************************************************/

#ifndef FIRMWARE_USER_HARDWARE_PWM_H_
#define FIRMWARE_USER_HARDWARE_PWM_H_


//ERR_E PWM_Init(PWM_PARM_S stPwmParm);

// 功能：改变PWM周期
// 参数1：选用的定时器；(范围 1,2,3,4)
// 参数2：设定周期
#define PWM_ChangePeriod(u8Tim, u16Period) TIM##u8Tim->ARR = u16Period-1;

// 功能：改变PWM脉冲宽度
// 参数1：选用的定时器；(范围 1,2,3,4)
// 参数2：设定通道； (范围 1,2,3,4)
// 参数3：设定脉冲宽度(高电平时间)
void HAL_pwm_set_pulse(uint8_t tim_num, uint8_t u8Channel, uint16_t u16Pulse);
// please init pin to GPIO_Mode_AF_PP mode before  HAL_pwn_config
void HAL_pwn_config(uint8_t tim_num, uint16_t u16TimChannel, uint32_t u32TimFrequency,
                        uint16_t u16TimPeriod, uint16_t u16TimPulse, uint8_t u8PinRemapflag);



#endif  // FIRMWARE_USER_HARDWARE_PWM_H_
