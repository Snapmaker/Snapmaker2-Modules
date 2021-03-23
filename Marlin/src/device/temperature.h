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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_

#include <src/core/pid.h>
#include "device_base.h"
#include "../HAL/hal_adc.h"
#include "src/HAL/hal_pwm.h"

class Temperature {
 public:
  static uint8_t TempertuerStatus();
  void InitCapture(uint8_t adc_pin, ADC_TIM_E adc_tim);
  void InitOutCtrl(uint8_t tim_num, uint8_t tim_chn, uint8_t tim_pin);
  void ReportTemprature();
  void ReportPid();
  void SetPID(uint8_t pid_index, float val);
  void TemperatureOut();
  void Maintain();
  void ChangeTarget(uint32_t target);
  uint16_t GetCurTemprature() {return detect_celsius_ * 10;}
  uint16_t GetTargetTemprature() {return pid_.getTarget();}
  bool isEnabled();

  float detect_celsius_;
  bool detect_ready_;
 private:
  int last_time_;
  uint8_t adc_index_;
  uint8_t pwm_tim_num_;
  uint8_t pwm_tim_chn_;
  Pid  pid_;
  uint8_t pid_set_flag_ = 0;
  int count_;
  bool enabled_;
  void InitPID();
  void SavePID();
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
