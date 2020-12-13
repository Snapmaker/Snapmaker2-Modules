#include <src/HAL/std_library/inc/stm32f10x.h>
#include <src/HAL/std_library/inc/system_stm32f10x.h>
#include "hal_pwm.h"
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
static TIM_TypeDef * tim_table[] = {TIM1, TIM2, TIM3, TIM4, TIM5};
uint32_t RCC_APB1Periph_tim_table[] = {RCC_APB2Periph_TIM1, RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4, RCC_APB1Periph_TIM5};


static void TIM_OCInit(TIM_TypeDef * TIMx, uint16_t TIM_channel, TIM_OCInitTypeDef * TIM_OCInitStruct) {
    switch (TIM_channel) {
        case 1:
            TIM_OC1Init(TIMx, TIM_OCInitStruct);
            break;

        case 2:
            TIM_OC2Init(TIMx, TIM_OCInitStruct);
            break;

        case 3:
            TIM_OC3Init(TIMx, TIM_OCInitStruct);
            break;

        case 4:
            TIM_OC4Init(TIMx, TIM_OCInitStruct);
            break;
    }
}

// please used pinMode(tim_pin, PWM) init pin before call HAL_pwn_config()
void HAL_pwn_config(uint8_t tim_num, uint16_t u16TimChannel, uint32_t u32TimFrequency,
                        uint16_t u16TimPeriod, uint16_t u16TimPulse, uint8_t u8PinRemapflag) {
    TIM_TimeBaseInitTypeDef tim_TimeBaseInitTypeDef;
    TIM_OCInitTypeDef Tim_OCInitTypeDef;
    TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
    tim_num -= 1;
    if (tim_num >= (sizeof(tim_table)/sizeof(tim_table[0])))
      return ;
    // 开启时钟及配置复用引脚
    RCC_APB2PeriphClockCmd(RCC_APB1Periph_tim_table[tim_num], ENABLE);

    tim_TimeBaseInitTypeDef.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_TimeBaseInitTypeDef.TIM_CounterMode = TIM_CounterMode_Up;
    tim_TimeBaseInitTypeDef.TIM_Period = u16TimPeriod - 1;
    tim_TimeBaseInitTypeDef.TIM_Prescaler = SystemCoreClock / u32TimFrequency - 1;
    tim_TimeBaseInitTypeDef.TIM_RepetitionCounter = 0;  // 配置重复计数寄存器，高级定时器才有
    TIM_TimeBaseInit(tim_table[tim_num], &tim_TimeBaseInitTypeDef);

    TIM_OCStructInit(&Tim_OCInitTypeDef);
    Tim_OCInitTypeDef.TIM_OCMode = TIM_OCMode_PWM1;
    Tim_OCInitTypeDef.TIM_Pulse = u16TimPulse;
    Tim_OCInitTypeDef.TIM_OCPolarity = TIM_OCPolarity_High;

    if (tim_num == 1 && u8PinRemapflag == 1) {
        Tim_OCInitTypeDef.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        Tim_OCInitTypeDef.TIM_OutputNState = TIM_OutputNState_Enable;  // N的通道输出使能
    } else {
        Tim_OCInitTypeDef.TIM_OutputState = TIM_OutputState_Enable;
    }
    TIM_OCInit(tim_table[tim_num], u16TimChannel, &Tim_OCInitTypeDef);
    TIM_ARRPreloadConfig(tim_table[tim_num], DISABLE);
    if (tim_table[tim_num] == TIM1) {
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


// ERR_E PWM_Init(PWM_PARM_S stPwmParm) {
//     TIM_PwmGpioInit(stPwmParm.stTIM, stPwmParm.u16TimChannel, stPwmParm.stGPIOX, stPwmParm.u16Pin,
//          stPwmParm.u8PinRemapflag);
//     TIM_PwmConfig(stPwmParm.stTIM, stPwmParm.u16TimChannel, stPwmParm.u32TimFrequency, stPwmParm.u16TimPeriod,
//          stPwmParm.u16TimPulse, stPwmParm.u8PinRemapflag);
//     return E_TRUE;
// }



void HAL_pwm_set_pulse(uint8_t tim_num, uint8_t u8Channel, uint16_t u16Pulse) {
    tim_num -= 1;
    switch (u8Channel) {
        case 1:
            tim_table[tim_num]->CCR1 = u16Pulse;
            break;

        case 2:
            tim_table[tim_num]->CCR2 = u16Pulse;
            break;

        case 3:
            tim_table[tim_num]->CCR3 = u16Pulse;
            break;

        case 4:
            tim_table[tim_num]->CCR4 = u16Pulse;
            break;
    }
}


