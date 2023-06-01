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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_UTILS_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_UTILS_H_

#include <stdint.h>
#include "../core/millis_t.h"

int Number36To10(uint8_t *data, uint8_t len);
int Number10To36str(uint32_t num, uint8_t *OutBuf, uint8_t BufLen);
uint16_t CalcChecksum(uint8_t *data, uint16_t len);

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CORE_UTILS_H_
