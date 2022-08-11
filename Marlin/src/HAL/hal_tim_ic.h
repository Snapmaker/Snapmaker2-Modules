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

#ifndef FIRMWARE_USER_HARDWARE_TIM_IC_H_
#define FIRMWARE_USER_HARDWARE_TIM_IC_H_

#include "hal_tim.h"

enum tim_it_ch_t : uint16_t {
    TIM_IT_UPDATE = 0x0001,
    TIM_IT_CH1 = 0x0002,
    TIM_IT_CH2 = 0x0004,
    TIM_IT_CH3 = 0x0008,
    TIM_IT_CH4 = 0x0010
};

enum tim_t : uint8_t { TIM_1 = 1, TIM_2, TIM_3, TIM_4, TIM_MAX };
enum tim_ch_t : uint8_t { TIM_CH1 = 1, TIM_CH2, TIM_CH3, TIM_CH4 };

void HAL_timer_nvic_ic_init(uint8_t timx, uint8_t PreemptionPriority, uint8_t SubPriority, uint16_t type);
void HAL_timer_ic_init(uint8_t timx, uint8_t ch, TIM_CB_F ic_cb, TIM_CB_F update_cb);

#endif

