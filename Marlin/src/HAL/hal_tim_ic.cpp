#include "stdio.h"
#include <src/HAL/std_library/inc/stm32f10x_tim.h>
#include <src/HAL/std_library/inc/stm32f10x.h>

#include "hal_tim_ic.h"
#include "hal_gpio.h"
#include "hal_adc.h"

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
#include <src/HAL/std_library/inc/stm32f10x_tim.h>
#include <src/HAL/std_library/inc/stm32f10x.h>

typedef struct {
    TIM_TypeDef *tim;
    TIM_CB_F update_cb;
    TIM_CB_F ic_cb[4];
} tim_ic_t;

tim_ic_t tim_ic[4] = {
    [0] = {.tim = TIM1, .update_cb = NULL, .ic_cb = {NULL, NULL, NULL, NULL}},
    [1] = {.tim = TIM2, .update_cb = NULL, .ic_cb = {NULL, NULL, NULL, NULL}},
    [2] = {.tim = TIM3, .update_cb = NULL, .ic_cb = {NULL, NULL, NULL, NULL}},
    [3] = {.tim = TIM4, .update_cb = NULL, .ic_cb = {NULL, NULL, NULL, NULL}},
};

static uint16_t tim_channel[] = {
    TIM_Channel_1, TIM_Channel_2, TIM_Channel_3, TIM_Channel_4
};

static uint8_t tim_nvic_iqr_channel[] = {
    TIM1_UP_IRQn, TIM2_IRQn, TIM3_IRQn, TIM4_IRQn
};


static void HAL_TIM_ICInit(uint8_t timx, uint8_t ch) {
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    if (timx >= TIM_MAX && ch > TIM_CH4)
        return;
    timx -= 1;
    ch -= 1;

	TIM_ICInitStructure.TIM_Channel = tim_channel[ch];
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(tim_ic[timx].tim, &TIM_ICInitStructure);
}


static void HAL_timer_ic_cb_init(uint8_t timx, uint8_t ch, TIM_CB_F cb) {
    if (timx >= TIM_MAX && ch > TIM_CH4)
        return;

    timx -= 1;
    ch -= 1;

    if ( cb) {
        tim_ic[timx].ic_cb[ch] = cb;
    }
}


static void HAL_timer_update_cb_init(uint8_t timx, TIM_CB_F cb) {
    if (timx >= TIM_MAX)
        return;

    timx -= 1;

    if (cb) {
        tim_ic[timx].update_cb = cb;
    }
}


static void tim_ic_irq(uint8_t timx) {
    if (timx >= TIM_MAX)
        return;
    timx -= 1;

    if (TIM_GetITStatus(tim_ic[timx].tim, TIM_IT_Update) == 1) {
        if (tim_ic[timx].update_cb)
            tim_ic[timx].update_cb();
        TIM_ClearITPendingBit(tim_ic[timx].tim, TIM_IT_Update);
    }

    if (TIM_GetITStatus(tim_ic[timx].tim, TIM_IT_CC1) == 1) {
        if (tim_ic[timx].ic_cb[0])
            tim_ic[timx].ic_cb[0]();
        TIM_ClearITPendingBit(tim_ic[timx].tim, TIM_IT_CC1);
    }

    if (TIM_GetITStatus(tim_ic[timx].tim, TIM_IT_CC2) == 1) {
        if (tim_ic[timx].ic_cb[1])
            tim_ic[timx].ic_cb[1]();
        TIM_ClearITPendingBit(tim_ic[timx].tim, TIM_IT_CC2);
    }

    if (TIM_GetITStatus(tim_ic[timx].tim, TIM_IT_CC3) == 1) {
        if (tim_ic[timx].ic_cb[2])
            tim_ic[timx].ic_cb[2]();
        TIM_ClearITPendingBit(tim_ic[timx].tim, TIM_IT_CC3);
    }

    if (TIM_GetITStatus(tim_ic[timx].tim, TIM_IT_CC4) == 1) {
        if (tim_ic[timx].ic_cb[3])
            tim_ic[timx].ic_cb[3]();
        TIM_ClearITPendingBit(tim_ic[timx].tim, TIM_IT_CC4);
    }
}


static void tim1_ic_irq() {
    tim_ic_irq(TIM_1);
}


static void tim2_ic_irq() {
    tim_ic_irq(TIM_2);
}


static void tim3_ic_irq() {
    tim_ic_irq(TIM_3);
}


static void tim4_ic_irq() {
    tim_ic_irq(TIM_4);
}


void HAL_timer_nvic_ic_init(uint8_t timx, uint8_t PreemptionPriority, uint8_t SubPriority, uint16_t type) {
    NVIC_InitTypeDef NVIC_InitStructure;

    if (timx >= TIM_MAX)
        return;
    timx -= 1;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = tim_nvic_iqr_channel[timx];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ClearITPendingBit(tim_ic[timx].tim, type);
    TIM_ITConfig(tim_ic[timx].tim, type, ENABLE);
}


void HAL_timer_ic_init(uint8_t timx, uint8_t ch, TIM_CB_F ic_cb, TIM_CB_F update_cb) {
    if (timx >= TIM_MAX && ch > TIM_CH4)
        return;

    switch (timx) {
        case TIM_1:
            HAL_timer_cb_init(timx, tim1_ic_irq);
            break;
        case TIM_2:
            HAL_timer_cb_init(timx, tim2_ic_irq);
            break;
        case TIM_3:
            HAL_timer_cb_init(timx, tim3_ic_irq);
            break;
        case TIM_4:
            HAL_timer_cb_init(timx, tim4_ic_irq);
            break;
        default: break;
    }

    HAL_TIM_ICInit(timx, ch);
    if (ic_cb)
        HAL_timer_ic_cb_init(timx, ch, ic_cb);

    if (update_cb)
        HAL_timer_update_cb_init(timx, update_cb);
}
