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
#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SPEED_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SPEED_H_

#include <stdint.h>

#define CNC_MAX_RPM (12000)
#define MAX_SPEED_OUT (1000 - 1)
#define SPEED_CAPTURE_FREQUENCY 5
#define DEFAULT_MOTOR_LAP_PULSE 4
#define SPEED_TO_RPM_RATE ((SPEED_CAPTURE_FREQUENCY * 60)  / motor_Lap_pulse)
class Speed {
 public:
  void InitOut(uint8_t pwm_pin, uint8_t tim_num, uint8_t tim_chn);
  void InitDir(uint8_t dir_pin, uint8_t dir);
  void InitCapture(uint8_t fg_pin, uint8_t tim_num);
  uint16_t ReadCurSpeed();
  void SetSpeed(uint8_t percent);
  void ReportSpeed();
  void SpeedOutCtrl();  // need loop
  bool SpeedStatuCheck();  // true:normal  false:fail
  void GetMotorLapPulse();
 private:
  void ExtiCallBack();
  uint32_t target_speed_ =0;
  uint32_t cur_set_percent_ = 0;
  uint32_t cur_speed = 0;
  uint32_t change_time_ = 0;
  uint32_t set_speed_time_ = 0;
  bool speed_fail_flag_ = false;
  uint8_t pwm_tim_num_;
  uint8_t pwm_tim_chn_;
  uint32_t exti_count_ = 0;
};

#endif