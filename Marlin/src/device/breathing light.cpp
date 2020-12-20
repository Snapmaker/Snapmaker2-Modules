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

#include <io.h>
#include <src/HAL/hal_tim.h>
#include "src/device/soft_pwm.h"
#include "src/device/breathing light.h"
#include <wirish_time.h>

#define BREATH_ONCE_TIME_MS 5

void BreathingLight::Loop() {
  if ((this->execute_time_ + BREATH_ONCE_TIME_MS) < millis()) {
    this->execute_time_ = millis();
    uint32_t pwm_out = (uint32_t)(this->cur_step_ * this->one_step_);
    soft_pwm_g.ChangeSoftPWM(this->index_, pwm_out);
    if (this->dir_) {
      this->cur_step_++;
      if (this->cur_step_ >= (this->total_step_ / 2)) {
        this->dir_ = false;
      }
    } else {
      this->cur_step_--;
      if (this->cur_step_ == 0) {
        this->dir_ = true;
      }
    }
  }
}

void BreathingLight::Init(uint8_t pin, uint16_t hold_time_ms) {
  uint32_t period = SOFT_PWM_MS(BREATH_ONCE_TIME_MS);
  this->index_ =  soft_pwm_g.AddPwm(pin, period);
  this->total_step_ = hold_time_ms / BREATH_ONCE_TIME_MS;
  this->one_step_ = (float)this->total_step_ * 2 / period;
}

void BreathingLight::Set(uint16_t frequency) {

}
