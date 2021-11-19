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

#include <stdio.h>
#include <src/HAL/std_library/inc/stm32f10x.h>
#include <src/HAL/std_library/inc/system_stm32f10x.h>
#include "hal_adc.h"
#include "hal_pwm.h"
#include "hal_gpio.h"
/*********使用方法***********************************************************************************
    初始化ADC通道(取值范围：0到15)
    ADC 通道          对应IO口
    ADC_Channel_0     PA0
    ADC_Channel_1     PA1
    ADC_Channel_2     PA2
    ADC_Channel_3     PA3
    ADC_Channel_4     PA4
    ADC_Channel_5     PA5
    ADC_Channel_6     PA6
    ADC_Channel_7     PA7
    ADC_Channel_8     PB0
    ADC_Channel_9     PB1
    ADC_Channel_10    PC0
    ADC_Channel_11    PC1
    ADC_Channel_12    PC2
    ADC_Channel_13    PC3
    ADC_Channel_14    PC4
    ADC_Channel_15    PC5
****************************************************************************************************/
ADC_CB_F adc_cb_f = NULL;
TIM_TypeDef * AdcTim = NULL;
uint16_t adc_cache[ADC_CACHE_SIZE];
uint32_t adc_cusum[ADC_MAX_DEV_COUNT];
uint8_t ADC_NbrOfChannel = 0;
uint8_t adc_status = 0;
static ADC_TIM_E adc_tim;
static uint16_t adc_period_us;
static bool adc_started = false;

// ADC1 CH0-CH15        PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PB0 PB1 PC0 PC1 PC2 PC3 PC4 PC5
const uint8_t adc_pin_map[] = {0,  1,  2,  3,  4,  5,  6,  7,  16, 17, 32, 33, 34, 35, 36, 37};

static void AdcCaptureDeal() {
  uint32_t size = ADC_NbrOfChannel * ADC_DEEP;
  uint32_t  temp[ADC_MAX_DEV_COUNT] = {0};
  for (uint8_t i = 0; i < size; i += ADC_NbrOfChannel) {
    for (uint8_t j = 0; j < ADC_NbrOfChannel; j++) {
      temp[j] += adc_cache[i + j];
    }
  }
  for (uint8_t i = 0; i < ADC_NbrOfChannel; i++) {
    adc_cusum[i] = temp[i];
  }
  adc_status = 1;
}

void hal_AddRegularChanne(uint16_t ADC_channel) {
  ADC_Cmd(ADC1, DISABLE);
  ADC1->SQR1 &= (~(0xf << 20));
  ADC1->SQR1 |= (ADC_NbrOfChannel << 20);
  ADC_NbrOfChannel++;
  ADC_RegularChannelConfig(ADC1, ADC_channel, ADC_NbrOfChannel, ADC_SampleTime_55Cycles5);
  ADC_Cmd(ADC1, ENABLE);
}

uint8_t hal_adc_status() {
  uint8_t ret = adc_status;
  adc_status = 0;
  return ret;
}

// if tim_num != 0 , used dma
// please init pin to GPIO_Mode_AIN mode before  Adc_Init
void HAL_adc_init_reg(uint16_t ADC_channel, uint8_t tim_numm) {
    ADC_InitTypeDef ADC_InitStructure;
    if (ADC_NbrOfChannel > 0) {
      hal_AddRegularChanne(ADC_channel);
      return ;
    }
    ADC_DeInit(ADC1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    switch (tim_numm) {
        case ADC_TIM_1:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
            break;
        case ADC_TIM_2:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
            break;
        case ADC_TIM_3:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
            break;
        case ADC_TIM_4:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
            break;
        default:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    }

    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  // ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    hal_AddRegularChanne(ADC_channel);
    ADC_ResetCalibration(ADC1);                    // 使能复位校准
    while (ADC_GetResetCalibrationStatus(ADC1));   // 等待复位校准结束
    ADC_StartCalibration(ADC1);                    // 开启AD校准
    while (ADC_GetCalibrationStatus(ADC1));   // 等待校准结束
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
}


void HAL_adc_tim_init(uint8_t tim_num, uint32_t u32TimFrequency, uint16_t u16Period) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    switch (tim_num) {
      case ADC_TIM_1:
        AdcTim = TIM1;
        HAL_PwnConfig(PWM_TIM1, PWM_CH1, u32TimFrequency, u16Period);
        HAL_PwmSetPulse(PWM_TIM1, PWM_CH1, 1);
        return;
      case ADC_TIM_2:
        AdcTim = TIM2;
        HAL_PwnConfig(PWM_TIM2, PWM_CH2, u32TimFrequency, u16Period);
        HAL_PwmSetPulse(PWM_TIM2, PWM_CH2, 1);
        return;
      case ADC_TIM_4:
        AdcTim = TIM4;
        HAL_PwnConfig(PWM_TIM4, PWM_CH4, u32TimFrequency, u16Period);
        HAL_PwmSetPulse(PWM_TIM4, PWM_CH4, 1);
        return;
    }

    AdcTim = TIM3;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Prescaler = SystemCoreClock / u32TimFrequency - 1;
    TIM_TimeBaseInitStruct.TIM_Period = u16Period;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM3->CNT = 0;
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    TIM_Cmd(AdcTim, ENABLE);
}


void HAL_adc_dma_init() {
    DMA_InitTypeDef DMA_InitStruct;

    DMA_DeInit(DMA1_Channel1);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct.DMA_BufferSize = ADC_DEEP * ADC_NbrOfChannel;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)adc_cache;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // DMA1_Channel1
    NVIC_InitTypeDef NVicInit;

    NVicInit.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVicInit.NVIC_IRQChannelSubPriority = 0;
    NVicInit.NVIC_IRQChannelPreemptionPriority = 2;
    NVicInit.NVIC_IRQChannelCmd = ENABLE;
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    NVIC_Init(&NVicInit);
}

void ADC_CaptureEnable() {
    DMA_Cmd(DMA1_Channel1, ENABLE);
    TIM_Cmd(AdcTim, ENABLE);
}

void ADC_CaptureDisable() {
    DMA_Cmd(DMA1_Channel1, DISABLE);
    TIM_Cmd(AdcTim, DISABLE);
}

uint16_t ADC_Get(uint8_t index) {
  if (index < ADC_NbrOfChannel) {
    return adc_cusum[index] / ADC_DEEP;
  }
  return 0;
}

uint16_t ADC_GetCusum(uint8_t index) {
  if (index < ADC_NbrOfChannel) {
    return adc_cusum[index];
  }
  return 0;
}

extern "C" void __irq_dma1_channel1() {
  if (DMA_GetITStatus(DMA1_IT_TC1) != RESET) {
        AdcTim->CR1 &= ~TIM_CR1_CEN;                // 失能定时器
        AdcTim->CNT = 0;                            // 定时器重新计数
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        AdcCaptureDeal();
        AdcTim->CR1 |= TIM_CR1_CEN;                 // 开启定时器
    }
}

uint8_t HAL_adc_init_chn(ADC_CHN_E chn, ADC_TIM_E tim, uint16_t period_us) {
    adc_tim = tim;
    adc_period_us = period_us;
    uint8_t ret_index = ADC_NbrOfChannel;
    HAL_adc_init_reg(chn, tim);
    return ret_index;
}

void hal_start_adc() {
  if (adc_started == false) {
    adc_started = true;
  } else {
    return;
  }

  if (ADC_NbrOfChannel > 0) {
    HAL_adc_tim_init(adc_tim, 1000000, adc_period_us);
    HAL_adc_dma_init();
  }
}

uint8_t HAL_adc_init(uint8_t pin, ADC_TIM_E tim, uint16_t period_us) {
  for (uint8_t i = 0; i < sizeof(adc_pin_map); i++) {
    if (adc_pin_map[i] == pin) {
      GpioInit(pin, GPIO_Mode_AIN);
      return HAL_adc_init_chn((ADC_CHN_E)i, tim, period_us);
    }
  }
  return ADC_ERROR;
}


