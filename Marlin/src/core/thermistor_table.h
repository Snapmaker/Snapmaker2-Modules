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


/*热敏电子阻值与温度转换*/
#ifndef _THERMISTOR_TABLE_H_
#define _THERMISTOR_TABLE_H_
#include "common_type.h"

#define OVERSAMPLENR 16  // 采集数据的累计个数

// 参数 u32Raw: 累计 OVERSAMPLENR 次的采样值
extern float32 TempTableCalcCurTemp(uint32_t u32Raw);

#endif

