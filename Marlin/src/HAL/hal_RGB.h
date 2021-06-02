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
#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_HAL_RGB_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_HAL_RGB_H_

#include <stdio.h>
#include "hal_exti.h"
typedef struct{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB_T;

void HAL_RGBInit(uint8_t pin, SOFT_EXTI_LINE_E exti);  // pin: eg. PA1
void HAL_SetAllRGB(uint8_t pin, RGB_T *rgb, uint8_t light_count);

#endif
