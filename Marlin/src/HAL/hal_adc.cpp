
#include <stdio.h>
#include <src/HAL/std_library/inc/stm32f10x.h>
#include <src/HAL/std_library/inc/system_stm32f10x.h>
#include "hal_adc.h"
#include "hal_pwm.h"
/*********使用方法***********************************************************************************
    初始化ADC通道(取值范围：0到15)
    ADC 通道	    	 对应IO口
    ADC_Channel_0		PA0
    ADC_Channel_1		PA1
    ADC_Channel_2		PA2
    ADC_Channel_3		PA3
    ADC_Channel_4		PA4
    ADC_Channel_5		PA5
    ADC_Channel_6		PA6
    ADC_Channel_7		PA7
    ADC_Channel_8		PB0
    ADC_Channel_9		PB1
    ADC_Channel_10		PC0
    ADC_Channel_11		PC1
    ADC_Channel_12		PC2
    ADC_Channel_13		PC3
    ADC_Channel_14		PC4
    ADC_Channel_15		PC5		
****************************************************************************************************/
ADC_CB_F adc_cb_f = NULL;
TIM_TypeDef * AdcTim = NULL;

// if tim_num != 0 , used dma
// please init pin to GPIO_Mode_AIN mode before  Adc_Init
void HAL_adc_init(uint16_t ADC_channel, uint8_t tim_numm) {
    ADC_InitTypeDef ADC_InitStructure;
    ADC_DeInit(ADC1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // ADC工作模式:ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;  // 模数转换工作在单通道模式 多通道用 ENABLE
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // 模数转换工作在单次转换模式
    switch (tim_numm) {
        case 1:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
            break;
        case 2:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
            break;
        case 3:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
            break;
        case 4:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
            break;
        default:
            ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    }

    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  // ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;         // 顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_channel, 1, ADC_SampleTime_55Cycles5);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);                    // 使能复位校准
    while (ADC_GetResetCalibrationStatus(ADC1));   // 等待复位校准结束
    ADC_StartCalibration(ADC1);                    // 开启AD校准
    while (ADC_GetCalibrationStatus(ADC1));   // 等待校准结束
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
}


void HAL_adc_tim_init(uint8_t tim_num, uint32_t u32TimFrequency, uint16_t u16Period) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    switch (tim_num) {
      case 1:
        AdcTim = TIM1;
        HAL_pwn_config(1, 1, u32TimFrequency, u16Period, 1, 0);
        return;
      case 2:
        AdcTim = TIM2;
        HAL_pwn_config(2, 2, u32TimFrequency, u16Period, 1, 0);
        return;
      case 4:
        AdcTim = TIM4;
        HAL_pwn_config(4, 4, u32TimFrequency, u16Period, 1, 0);
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


void HAL_adc_dma_init(void * voBuf, uint32_t u32BufSize, ADC_CB_F cb) {
    DMA_InitTypeDef DMA_InitStruct;

    DMA_DeInit(DMA1_Channel1);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct.DMA_BufferSize = u32BufSize;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)voBuf;
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
    adc_cb_f = cb;
}

void ADC_CaptureEnable() {
    DMA_Cmd(DMA1_Channel1, ENABLE);
    TIM_Cmd(AdcTim, ENABLE);
}


void ADC_CaptureDisable() {
    DMA_Cmd(DMA1_Channel1, DISABLE);
    TIM_Cmd(AdcTim, DISABLE);
}
extern "C" void __irq_dma1_channel1() {
  if (DMA_GetITStatus(DMA1_IT_TC1) != RESET) {
        AdcTim->CR1 &= ~TIM_CR1_CEN;                // 失能定时器
        AdcTim->CNT = 0;                            // 定时器重新计数
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        if (adc_cb_f != NULL)
            adc_cb_f();
        AdcTim->CR1 |= TIM_CR1_CEN;                 // 开启定时器
    }
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET) {
        AdcTim->CR1 &= ~TIM_CR1_CEN;                // 失能定时器
        AdcTim->CNT = 0;                            // 定时器重新计数
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        if (adc_cb_f != NULL)
            adc_cb_f();
        AdcTim->CR1 |= TIM_CR1_CEN;                 // 开启定时器
    }
}


// Rank：规则组采样顺序。取值范围 1 到 16
uint16_t ADC_GetVal(ADC_TypeDef * stADC, uint8_t Adc_Channel, uint8_t Rank) {
    u32 temp_val = 0;

    // 设置指定ADC的规则组通道，一个序列，采样时间
    ADC_RegularChannelConfig(stADC, Adc_Channel, Rank, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(stADC, ENABLE);  // 使能指定的ADC1的软件转换启动功能

    while (!ADC_GetFlagStatus(stADC, ADC_FLAG_EOC));  // 等待转换结束
    temp_val = ADC_GetConversionValue(stADC);

    return temp_val;  // 返回最近一次ADC1规则组的转换结果
}


// Rank：规则组采样顺序。取值范围 1 到 16
// 获取电压值,单位mv
uint16_t ADC_GetVal_mv(ADC_TypeDef * stADC, uint8_t Adc_Channel, uint8_t Rank) {
    float f32Val = ADC_GetVal(stADC, Adc_Channel, Rank);

    return (uint16_t) (f32Val * POWER_MV / 0x1000);
}


