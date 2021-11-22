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

#include <stdint.h>
#include <src/core/can_bus.h>
#include <wirish.h>
#include "nozzle_identify.h"
#include "src/HAL/hal_adc.h"
#include "src/HAL/hal_flash.h"
#include "src/configuration.h"
#include "src/registry/registry.h"

void NozzleIdentify::Init(uint8_t adc_pin, ADC_TIM_E adc_tim) {
  adc_index_ = HAL_adc_init(adc_pin, adc_tim, 2400);
}

nozzle_type_t NozzleIdentify::CheckNozzleType(uint16_t adc) {
  uint32_t i;

  for (i = 0; i < NOZZLE_TYPE_IDLE; i++) {
    if (adc >= nozzle_type_array[i].min && adc <= nozzle_type_array[i].max) {
      return (nozzle_type_t)i;
    }
  }

  return NOZZLE_TYPE_INVALID;
}

nozzle_type_t NozzleIdentify::GetNozzleType() {
  return nozzle_type_;
}

void NozzleIdentify::ReportNozzle(uint8_t nozzle) {
  uint8_t buf[8];
  uint8_t index = 0;

  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_NOZZLE_TYPE);
  if (msgid != INVALID_VALUE) {
    buf[index++] = nozzle;
    buf[index++] = (uint8_t)nozzle_type_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

bool NozzleIdentify::CheckLoop() {
  uint16_t raw_adc_tmp = ADC_Get(adc_index_);
  uint16_t raw_adc_diff = 0;

  if (raw_adc_tmp >= raw_adc_value_) {
    raw_adc_diff = raw_adc_tmp - raw_adc_value_;
  } else {
    raw_adc_diff = raw_adc_value_ - raw_adc_tmp;
  }

  if (raw_adc_diff > 20) {
    adc_filter_count_++;
  } else {
    adc_filter_count_ = 0;
    return false;
  }

  if (adc_filter_count_ == 2) {
    adc_filter_count_ = 0;
    raw_adc_value_ = raw_adc_tmp;
  } else {
    return false;
  }

  nozzle_type_t nozzle_type = CheckNozzleType(raw_adc_value_);

  if (nozzle_type_ != nozzle_type) {
    nozzle_type_ = nozzle_type;
    return true;
  }

  return false;
}
