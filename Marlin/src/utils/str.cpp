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

#include "str.h"
#include <stdio.h>
#include "string.h"
uint8_t ToLower(uint8_t c) {
  #define TOLOWER_DIFF  ('a' - 'A')
  if (c <= 'Z' && c >=  'A') {
    c += TOLOWER_DIFF;
  }
  return c;
}

uint16_t ToLowers(uint8_t *s, uint16_t max_len) {
  uint16_t i = 0;
  for ( ; s[i] != '\0' && i < max_len; i++) {
    s[i] = ToLower(s[i]);
  }
  return i;
}

uint8_t ToCapital(uint8_t c) {
  if (c <= 'z' && c >=  'a') {
    c = c - 'a' + 'A';
  }
  return c;
}

uint16_t ToCapitals(uint8_t *s, uint16_t max_len) {
  uint16_t i = 0;
  for ( ; s[i] != '\0' && i < max_len; i++) {
    s[i] = ToCapital(s[i]);
  }
  return i;
}

int16_t FindCharIndex(uint8_t * str, uint8_t ch) {
  int16_t ret  = -1;
  if (str != NULL) {
    for (uint16_t index = 0; str[index] != '\0'; index++) {
      if (str[index] == ch) {
        ret = index;
        break;
      }
    }
  }
  return ret;
}

bool IsBeginWith(uint8_t * str,uint8_t *begin) {
  if(str == NULL || begin == NULL)
    return false;
  int len1 = strlen((char *)str);
  int len2 = strlen((char *)begin);
  if((len1 < len2) || (len1 == 0 || len2 == 0))
    return false;
  uint8_t *p = begin;
  int i = 0;
  while(*p != '\0') {
    if(*p != str[i])
      return false;
    p++;
    i++;
  }
  return true;
}

bool IfStringFLoat(uint8_t *str, uint8_t &decimal_index) {
  uint8_t index = 0;
  if (!IS_NUMBER(str[index])) {
    return false;
  }
  while(IS_NUMBER(str[index])) index++;
  if (str[index] == '.' && IS_NUMBER(str[index+1])) {
    decimal_index = index+1;
    return true;
  }
  return false;
}

bool IfStringFLoat(uint8_t *str) {
  uint8_t index;
  return IfStringFLoat(str, index);
}

bool StringToFloat(uint8_t *str, float &out) {
  int32_t num = 0;
  float decimal = 0;
  out = 0;
  if (StringToInt(str, num)) {
    out = num;
    uint8_t decimal_index;
    if (IfStringFLoat(str, decimal_index)) {
      if (StringToInt(&str[decimal_index], num)) {
        while (num) {
          decimal += (num % 10);
          decimal /= 10;
          num /= 10;
        }
      }
    }
  } else {
    return false;
  }
  out += decimal;
  return true;
}

uint8_t StringToInt(uint8_t *str, int32_t &out) {
  int32_t num = 0;
  int8_t sign = -1;
  uint8_t bit = 0;
  out = 0;
  if (str == NULL) {
    return false;
  }

  while ((*str) == ' ') {
    str++;
  }

  if ((!IS_NUMBER(*str)) && ((*str) != '-')) {
    return false;
  }

  if ((*str) == '-') {
    sign = 1;
    str++;
  }

  while (IS_NUMBER(*str)) {
    num = num * 10 + ((*str) - '0');
    str++;
    bit++;
  }
  out = num * (sign * (-1));
  return bit;
}

bool StringToBool(uint8_t *str, bool &out) {

  if (str == NULL) {
    return false;
  }

  while (!IS_NUMBER(*str) && (*str != '\0')) {
    str++;
  }

  if (IS_NUMBER(*str)) {
    out = ((*str) - '0') > 0;
    return true;
  }
  return false;
}

void StringCopy(uint8_t *dst, uint8_t *src, uint8_t len) {
  if (dst && src) {
    for (uint8_t i = 0; i < len; i++) {
      dst[i] = src[i];
    }
  }
}
