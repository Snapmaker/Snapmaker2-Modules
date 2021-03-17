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
#include "cnc_head.h"
#include <io.h>
#include <wirish_time.h>

void CncHead::Init() {
  this->speed_.InitCapture(PA6, SPEED_TIM_MUN);
  this->speed_.InitOut(PA1, PWM_TIM2, PWM_CH2);
  this->speed_.InitDir(PA5, 0);
}

void CncHead::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_MOTOR_SPEED:
      this->speed_.ReportSpeed();
      break;
    case FUNC_SET_MOTOR_SPEED:
      this->speed_.SetSpeed(data[0]);
      break;
  }
}

void CncHead::EmergencyStop() {
  speed_.SetSpeed(0);
}

void CncHead::Loop() {
  this->speed_.SpeedOutCtrl();
  if ((this->time_ + 500) < millis()) {
    this->time_ = millis();
    this->speed_.ReportSpeed();
  }
}