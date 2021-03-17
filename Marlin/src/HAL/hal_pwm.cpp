#include <src/HAL/std_library/inc/stm32f10x.h>
#include <src/HAL/std_library/inc/system_stm32f10x.h>
#include "hal_pwm.h"
#include "hal_gpio.h"
/*
定时器	引脚重映像		通道	       1	  2	   3	4

TIM1	0 没重映像 			PA8   PA9  PA10  PA11 
        1 部分重映射         PA7   PB0  PB1
        2 完全重映相			PE9	  PE11 PE13	 PE14

TIM2	0 没重映相			PA0   PA1  PA2   PA3 
        1 部分重映相1		PA15  PB3  PA2	 PA3
        2 部分重映像2		PA0   PA1  PB10	 PB11
        3 完全从映像			PA15  PB3  PB10  PB11

TIM3	0 没重映相			PA6   PA7  PB0   PB1
        1 部分重映像			PB4   PB5  PB0   PB1
        2 完全重映像			PC6   PC7  PC8   PC9

TIM4	0 没重映像			PB6   PB7  PB8   PB9
        1 完全重映像			PD12  PD13 PD14  PD15

TIM5    0 没重映像			PA0   PA1  PA2   PA3

*/
/***************************************************************************/
#define TO_TIM(t) (t <= PWM_TIM5 ? t : \
            (t <= PWM_TIM3_PARTIAL ? (t - PWM_TIM1_PARTIAL) : t - PWM_TIM1_FULL))

#define assert_tim(tim, chn) if(tim > PWM_TIM4_FULL || chn > PWM_CH4) while(1); 

static TIM_TypeDef * tim_table[] = {TIM1, TIM2, TIM3, TIM4, TIM5};
static const uint32_t RCC_tim[] = {RCC_APB2Periph_TIM1, RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4, RCC_APB1Periph_TIM5};

const uint32_t pwm_remap[] = {
    GPIO_PartialRemap_TIM1,
    GPIO_PartialRemap1_TIM2,
    GPIO_PartialRemap2_TIM2,
    GPIO_PartialRemap_TIM3,
    GPIO_FullRemap_TIM1,
    GPIO_FullRemap_TIM2,
    GPIO_FullRemap_TIM3,
    GPIO_Remap_TIM4,
};

static void PwmOcInit(TIM_TypeDef * tim, uint16_t chn, TIM_OCInitTypeDef * TIM_OCInitStruct) {
    switch (chn) {
        case PWM_CH1: TIM_OC1Init(tim, TIM_OCInitStruct); TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);break;
        case PWM_CH2: TIM_OC2Init(tim, TIM_OCInitStruct); TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);break;
        case PWM_CH3: TIM_OC3Init(tim, TIM_OCInitStruct); TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);break;
        case PWM_CH4: TIM_OC4Init(tim, TIM_OCInitStruct); TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);break;
    }
}

static void PwmPinInit(uint8_t tim, uint8_t pin) {
    GpioInit(pin, GPIO_Mode_AF_PP);
    if (tim >= PWM_TIM1_PARTIAL) {
        GPIO_PinRemapConfig(pwm_remap[tim - PWM_TIM1_PARTIAL],ENABLE);
    }
}

void HAL_PwnConfig(uint8_t tim, uint8_t chn, uint32_t freq, uint16_t period) {
    TIM_TimeBaseInitTypeDef tim_TimeBaseInitTypeDef;
    TIM_OCInitTypeDef Tim_OCInitTypeDef;
    TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
    uint8_t tim_num = (PWM_TIM_E)TO_TIM(tim);

    assert_tim(tim, chn);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


    if (tim_num == PWM_TIM1)
        RCC_APB2PeriphClockCmd(RCC_tim[tim_num], ENABLE);
    else
        RCC_APB1PeriphClockCmd(RCC_tim[tim_num], ENABLE);

    tim_TimeBaseInitTypeDef.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_TimeBaseInitTypeDef.TIM_CounterMode = TIM_CounterMode_Up;
    tim_TimeBaseInitTypeDef.TIM_Period = period - 1;
    tim_TimeBaseInitTypeDef.TIM_Prescaler = 72000000 / freq - 1;
    tim_TimeBaseInitTypeDef.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim_table[tim_num], &tim_TimeBaseInitTypeDef);

    TIM_OCStructInit(&Tim_OCInitTypeDef);
    Tim_OCInitTypeDef.TIM_OCMode = TIM_OCMode_PWM1;
    Tim_OCInitTypeDef.TIM_Pulse = 0;
    if (tim == PWM_TIM1_PARTIAL) {
        Tim_OCInitTypeDef.TIM_OutputNState 	= TIM_OutputNState_Enable;
        Tim_OCInitTypeDef.TIM_OCNPolarity 	= TIM_OCNPolarity_High;
        Tim_OCInitTypeDef.TIM_OCIdleState 	= TIM_OCIdleState_Reset;     
        Tim_OCInitTypeDef.TIM_OCNIdleState 	= TIM_OCNIdleState_Reset;
        Tim_OCInitTypeDef.TIM_OutputState = TIM_OutputState_Disable;
        Tim_OCInitTypeDef.TIM_OCPolarity = TIM_OCPolarity_Low;
    } else {
        Tim_OCInitTypeDef.TIM_OutputState = TIM_OutputState_Enable;
        Tim_OCInitTypeDef.TIM_OCPolarity = TIM_OCPolarity_High;
    }

    PwmOcInit(tim_table[tim_num], chn, &Tim_OCInitTypeDef);
    TIM_ARRPreloadConfig(tim_table[tim_num], ENABLE);
    if (tim_num == PWM_TIM1) {
        TIM_BDTRStructInit(&TIM_BDTRInitStruct);
        TIM_BDTRInitStruct.TIM_OSSRState = ENABLE;
        TIM_BDTRInitStruct.TIM_OSSIState = ENABLE;
        TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
        TIM_BDTRInitStruct.TIM_DeadTime = 0;
        TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
        TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
        TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
    }
    TIM_Cmd(tim_table[tim_num], ENABLE);
}

void HAL_PwmInit(PWM_TIM_CHN_E tim_chn, uint8_t pin, uint32_t freq, uint16_t period) {
    uint8_t tim = tim_chn / 4;
    uint8_t chn = tim_chn % 4;
    HAL_PwmInit(tim, chn, pin, freq, period);
}

void HAL_PwmInit(uint8_t tim, uint8_t chn, uint8_t pin, uint32_t freq, uint16_t period) {
    PwmPinInit(tim, pin);
    HAL_PwnConfig(tim, chn, freq, period);
}

void HAL_PwmSetPulse(uint8_t tim, uint8_t chn, uint16_t pulse) {
    assert_tim(tim, chn);
    uint8_t tim_num = (PWM_TIM_E)TO_TIM(tim);
    switch (chn) {
        case PWM_CH1: tim_table[tim_num]->CCR1 = pulse; break;
        case PWM_CH2: tim_table[tim_num]->CCR2 = pulse; break;
        case PWM_CH3: tim_table[tim_num]->CCR3 = pulse; break;
        case PWM_CH4: tim_table[tim_num]->CCR4 = pulse; break;
    }
}

void HAL_PwmSetPulse(PWM_TIM_CHN_E tim_chn, uint16_t pulse) {
    uint8_t tim = tim_chn / 4;
    uint8_t chn = tim_chn % 4;
    HAL_PwmSetPulse(tim, chn, pulse);
}