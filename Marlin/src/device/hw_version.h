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

#ifndef DEVICE_HW_VERSION_H_
#define DEVICE_HW_VERSION_H_

#include <stdint.h>
#include "device_base.h"
#include "src/HAL/hal_adc.h"

enum HWVersionList {
  HW_VER_0,
  HW_VER_1,
  HW_VER_2,
  HW_VER_3,
  HW_VER_4,
  HW_VER_5,
  HW_VER_6,
  HW_VER_7,
  HW_VER_8,
  HW_VER_9,
  HW_VER_MAX
};

class HWVersion {
  public:
    uint32_t Init(uint32_t adc_pin, ADC_TIM_E adc_tim);
    uint32_t GetVersion() { return version_; }
    void UpdateVersion();

  private:
    uint8_t adc_index_;
    uint8_t version_;
};

#endif  // #ifndef DEVICE_HW_VERSION_H_
