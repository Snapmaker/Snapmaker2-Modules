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
#include "src/core/thermistor_table.h"

#define PT100_NOZZLE_TYPE_BASE_COUNT    0
#define NTC3950_NOZZLE_TYPE_BASE_COUNT  20

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
  NOZZLE_TYPE_IDLE,
  NOZZLE_TYPE_INVALID = 0xff,
}nozzle_type_t;

typedef struct {
  uint16_t min;
  uint16_t max;
}nozzle_adc_domain_t;

const nozzle_adc_domain_t pt100_nozzle_type_array[NOZZLE_TYPE_IDLE] = {{.min = 143,  .max = 392},  \
                                                                       {.min = 483,  .max = 732},  \
                                                                       {.min = 866,  .max = 1114}, \
                                                                       {.min = 1383, .max = 1508}, \
                                                                       {.min = 1564, .max = 1813}, \
                                                                       {.min = 1923, .max = 2172}, \
                                                                       {.min = 2296, .max = 2546}, \
                                                                       {.min = 2660, .max = 2909}, \
                                                                       {.min = 2993, .max = 3242}, \
                                                                       {.min = 3598, .max = 3847}};

const nozzle_adc_domain_t ntc3950_nozzle_type_array[NOZZLE_TYPE_IDLE] = {{.min = 235,    .max = 360},    \
                                                                         {.min = 96,     .max = 159},    \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}, \
                                                                         {.min = 0xffff, .max = 0xffff}};

class NozzleIdentify {
 public:
  NozzleIdentify() {
    nozzle_type_base_count_ = NOZZLE_TYPE_IDLE;
    nozzle_type_ = NOZZLE_TYPE_IDLE;
    adc_filter_count_ = 0;
    raw_adc_value_ = 0xffff;
  }
  uint8_t Init(uint8_t adc_pin, ADC_TIM_E adc_tim);
  void SetAdcIndex(uint8_t index) { adc_index_ = index; }
  void SetNozzleTypeCheckArray(thermistor_type_e type);
  nozzle_type_t CheckNozzleType(uint16_t adc);
  nozzle_type_t GetNozzleType();
  void ReportNozzle(uint8_t nozzle);
  bool CheckLoop();

 private:
  uint8_t adc_index_;
  uint16_t raw_adc_value_;
  nozzle_type_t nozzle_type_;
  uint8_t adc_filter_count_;
  uint8_t nozzle_type_base_count_;
  const nozzle_adc_domain_t *nozzle_type_array_;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
