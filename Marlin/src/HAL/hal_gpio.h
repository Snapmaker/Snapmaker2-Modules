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
#ifndef GPIO_H
#define GPIO_H
#include "std_library/inc/stm32f10x.h"

void GpioRemap(void);
void GpioInit(uint8_t Port, GPIOMode_TypeDef Mode);
void GpioWrite(uint8_t Port, uint8_t IOLevel);
uint8_t GpioRead(uint8_t Port);
GPIO_TypeDef * GpioGetPort(uint8_t pin);
#endif
