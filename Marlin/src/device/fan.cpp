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

#include <src/HAL/hal_can.h>
#include <vector>
#include <src/core/can_bus.h>
#include <stdexcept>
#include <board/board.h>
#include <io.h>
#include <src/HAL/hal_tim.h>
#include "fan.h"
#include "src/device/soft_pwm.h"
#include <stdint.h>
#include <wirish_time.h>

void Fan::Loop() {
  if (this->delay_close_enadle_) {
    if ((this->delay_start_time_ + this->delay_close_time_) <= millis()) {
      this->delay_close_enadle_ = false;
      if (!is_hardware_pwm)
        soft_pwm_g.ChangeSoftPWM(this->fan_index_, 0);
      else
        HAL_PwmSetPulse(this->pwm_info.tim_chn, 0);
    }
  }
}

void Fan::Init(uint8_t fan_pin) {
  this->fan_index_ =  soft_pwm_g.AddPwm(fan_pin, FAN_MAX_THRESHOLD);
}

void Fan::Init(uint8_t fan_pin, uint32_t threshold) {
  fan_index_ =  soft_pwm_g.AddPwm(fan_pin, threshold);
}

void Fan::InitUseHardwarePwm(PWM_TIM_CHN_E tim_chn, uint8_t pin, uint32_t freq, uint16_t period, uint16_t ocpolarity) {
  this->pwm_info.tim_chn = tim_chn;
  this->pwm_info.period = period;
  this->pwm_info.freq = freq;
  this->pwm_info.ocpolarity = ocpolarity;
  HAL_PwmInitEx(tim_chn, pin, freq, period, ocpolarity);
  is_hardware_pwm = true;
}

void Fan::ChangePwm(uint8_t threshold, uint16_t delay_close_time_s) {
  this->delay_close_time_ = delay_close_time_s * 1000;
  if (threshold == 0) {
    this->delay_start_time_ = millis();
    this->delay_close_enadle_ = true;
  } else {
    this->delay_close_enadle_ = false;
    if (!is_hardware_pwm)
      soft_pwm_g.ChangeSoftPWM(this->fan_index_, threshold);
    else
      HAL_PwmSetPulse(this->pwm_info.tim_chn, threshold);
  }
}
