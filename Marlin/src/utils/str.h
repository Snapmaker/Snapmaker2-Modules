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

#ifndef SM2_STR_H
#define SM2_STR_H

#include "stdint.h"
#include "string.h"

#define IS_NUMBER(ch) (((ch) <= '9') && ((ch) >= '0'))
#define IS_LETTER(ch) ((((ch) <= 'z') && ((ch) >= 'a')) || (((ch) <= 'Z') && ((ch) >= 'A')))
uint8_t ToLower(uint8_t c);
uint16_t ToLowers(uint8_t *s, uint16_t max_len);
uint8_t ToCapital(uint8_t c);
uint16_t ToCapitals(uint8_t *s, uint16_t max_len);
int16_t FindCharIndex(uint8_t * str, uint8_t ch);
bool IsBeginWith(uint8_t * str,uint8_t *begin);
bool IfStringFLoat(uint8_t *str, uint8_t &decimal_index);
bool IfStringFLoat(uint8_t *str);
bool StringToFloat(uint8_t *str, float &out);
uint8_t StringToInt(uint8_t *str, int32_t &out);
bool StringToBool(uint8_t *str, bool &out);
void StringCopy(uint8_t *dst, uint8_t *src, uint8_t len);

#endif
