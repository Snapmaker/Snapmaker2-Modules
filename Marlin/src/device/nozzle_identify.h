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

#ifndef _NOZZLE_IDENTIFY_H_
#define _NOZZLE_IDENTIFY_H_

#include "device_base.h"
#include "../HAL/hal_adc.h"

typedef enum {
  NOZZLE_TYPE_0,
  NOZZLE_TYPE_1,
  NOZZLE_TYPE_2,
  NOZZLE_TYPE_3,
  NOZZLE_TYPE_4,
  NOZZLE_TYPE_5,
  NOZZLE_TYPE_6,
  NOZZLE_TYPE_7,
  NOZZLE_TYPE_8,
  NOZZLE_TYPE_9,
  NOZZLE_TYPE_10,

  NOZZLE_TYPE_IDLE,
  NOZZLE_TYPE_INVALID,
}nozzle_type_t;

typedef struct {
  uint16_t min;
  uint16_t max;
}nozzle_adc_domain_t;

const nozzle_adc_domain_t nozzle_type_array[NOZZLE_TYPE_IDLE] = {{.min = 0,    .max = 124},  \
                                                                 {.min = 248,  .max = 496},  \
                                                                 {.min = 614,  .max = 861},  \
                                                                 {.min = 1024,  .max = 1273}, \
                                                                 {.min = 1345, .max = 1593}, \
                                                                 {.min = 1721, .max = 1969}, \
                                                                 {.min = 2109, .max = 2357}, \
                                                                 {.min = 2507, .max = 2756}, \
                                                                 {.min = 2863, .max = 3112}, \
                                                                 {.min = 3252, .max = 3500}, \
                                                                 {.min = 3598, .max = 3846}};

class NozzleIdentify {
 public:
  NozzleIdentify() {
    nozzle_type_ = NOZZLE_TYPE_IDLE;
    is_assigned_message_id_ = false;
  }
  void Init(uint8_t adc_pin, ADC_TIM_E adc_tim);
  nozzle_type_t CheckNozzleType(uint16_t adc);
  nozzle_type_t GetNozzleType();
  void ReportNozzle(uint8_t nozzle);
  void IdentifyProcess(uint8_t nozzle);

 private:
  uint8_t adc_index_;
  uint16_t raw_adc_value_;
  nozzle_type_t nozzle_type_;
  bool is_assigned_message_id_;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
