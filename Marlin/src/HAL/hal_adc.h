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


#ifndef _ADC_H_
#define _ADC_H_

#include <stdio.h>
/*
ADC 通道	    	 对应IO口
    ADC_Channel_0		  PA0
    ADC_Channel_1		  PA1
    ADC_Channel_2		  PA2
    ADC_Channel_3		  PA3
    ADC_Channel_4		  PA4
    ADC_Channel_5		  PA5
    ADC_Channel_6		  PA6
    ADC_Channel_7		  PA7
    ADC_Channel_8		  PB0
    ADC_Channel_9		  PB1
    ADC_Channel_10		PC0
    ADC_Channel_11		PC1
    ADC_Channel_12		PC2
    ADC_Channel_13		PC3
    ADC_Channel_14		PC4
    ADC_Channel_15		PC5	
*/

#define POWER_MV (3.3 * 1000)  // 电压 mv


typedef void(*ADC_CB_F) (void);  // 中断回调函数


// if tim_num != 0 , used dma
// please init pin to GPIO_Mode_AIN mode before  Adc_Init
void HAL_adc_init(uint16_t ADC_channel, uint8_t tim_numm);
void HAL_adc_tim_init(uint8_t tim_num, uint32_t u32TimFrequency, uint16_t u16Period);
void HAL_adc_dma_init(void * voBuf, uint32_t u32BufSize, ADC_CB_F cb);

void ADC_CaptureEnable();
void ADC_CaptureDisable();

// // Rank：规则组采样顺序。取值范围 1 到 16
// uint16_t ADC_GetVal(ADC_TypeDef * stADC, uint8_t Adc_Channel, uint8_t Rank);

// // Rank：规则组采样顺序。取值范围 1 到 16
// // 获取电压值， 单位mv
// uint16_t ADC_GetVal_mv(ADC_TypeDef * stADC, uint8_t Adc_Channel, uint8_t Rank);

#endif

