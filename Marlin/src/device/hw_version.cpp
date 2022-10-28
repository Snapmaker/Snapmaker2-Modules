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

#include "hw_version.h"


struct VersionADCRange {
  uint16_t lower;
  uint16_t upper;
};

// unit: mV
static struct VersionADCRange ver_adc_range[HW_VER_MAX] = {
  {207, 447},   // 0
  {497, 737},   // 1
  {823, 1063},  // 2
  {1078, 1318}, // 3
  {1377, 1617}, // 4
  {1687, 1927}, // 5
  {2006, 2246}, // 6
  {2291, 2531}, // 7
  {2602, 2842}, // 8
  {2880, 3120}  // 9
};

uint32_t HWVersion::Init(uint32_t adc_pin, ADC_TIM_E adc_tim) {
  version_ = (uint8_t)HW_VER_MAX;

  adc_index_ = HAL_adc_init(adc_pin, adc_tim, ADC_PERIOD_DEFAULT);

  version_ = HW_VER_MAX;

  return adc_index_;
}

void HWVersion::UpdateVersion() {
  struct   VersionADCRange *range;
  uint32_t ver_adc;

  int i;

  if (version_ < HW_VER_MAX)
    return;

  range   = ver_adc_range;
  ver_adc = ADC_Get(adc_index_);

  // get voltage from raw ADC with unit mV
  ver_adc = (uint32_t)(ver_adc * 3300 / 4096);

  for (i = 0; i < HW_VER_MAX; i++) {
    if ((ver_adc >= range->lower) && (ver_adc <= range->upper)) {
      break;
    }

    range++;
  }

  version_ = (uint8_t)i;
}
