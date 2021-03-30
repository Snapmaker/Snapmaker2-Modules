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
#ifndef _EXTI_H_
#define _EXTI_H_
#include <stdint.h>
typedef void(*EXTI_CB_F)(uint8_t pin_source);

typedef enum {
    EXTI_Rising = 0x08,
    EXTI_Falling = 0x0C,  
    EXTI_Rising_Falling = 0x10
}EXTI_MODE_E;

typedef enum {
    SOFT_EXTI_LINE0 = 0,
    SOFT_EXTI_LINE1 = 1,
    SOFT_EXTI_LINE2 = 2,
    SOFT_EXTI_LINE3 = 3,
    SOFT_EXTI_LINE4 = 4,
    SOFT_EXTI_LINE5 = 5,
    SOFT_EXTI_LINE6 = 6,
    SOFT_EXTI_LINE7 = 7,
    SOFT_EXTI_LINE8 = 8,
    SOFT_EXTI_LINE9 = 9,
    SOFT_EXTI_LINE10 = 10,
    SOFT_EXTI_LINE11 = 11,
    SOFT_EXTI_LINE12 = 12,
    SOFT_EXTI_LINE13 = 13,
    SOFT_EXTI_LINE14 = 14,
    SOFT_EXTI_LINE15 = 15,
}SOFT_EXTI_LINE_E;


uint8_t ExtiInit(uint8_t pin, EXTI_MODE_E exti_mode, EXTI_CB_F cb);
uint8_t SoftExtiInit(SOFT_EXTI_LINE_E exti_line, EXTI_CB_F cb, uint8_t pro=1, uint8_t sub_pro=1);
void SoftExtiTrigger(SOFT_EXTI_LINE_E exti_line);
#endif
