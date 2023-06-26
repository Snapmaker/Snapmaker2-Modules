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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_

#include <stdint.h>
#include "device_base.h"
#include "../HAL/hal_pwm.h"

#define FAN_MAX_THRESHOLD 255

typedef struct {
  PWM_TIM_CHN_E  tim_chn;
  uint8_t  fan_pin;
  uint16_t ocpolarity;
  uint16_t period;
  uint32_t freq;
} fan_hardware_pwm_info;

class Fan {
 public:
  void Init(uint8_t fan_pin);
  void Init(uint8_t fan_pin, uint32_t threshold);
  void InitUseHardwarePwm(PWM_TIM_CHN_E tim_chn, uint8_t pin, uint32_t freq, uint16_t period, uint16_t ocpolarity=0xFFFF);
  void Loop();
  void ChangePwm(uint8_t threshold, uint16_t delay_close_time_s);


 private:
  uint8_t fan_index_;
  uint32_t delay_close_time_;
  uint32_t delay_start_time_;
  bool  delay_close_enadle_;
  bool  is_hardware_pwm = false;
  fan_hardware_pwm_info pwm_info;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_
