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
#include "speed.h"
#include <board/board.h>
#include <io.h>
#include "src/HAL/hal_exti.h"
#include <wirish_time.h>
#include <ext_interrupts.h>
#include "src/HAL/hal_tim.h"
#include "src/core/can_bus.h"
#include "src/registry/registry.h"
#include "src/configuration.h"
uint32_t speed_exti_count = 0; 
uint32_t speed_tim_count = 0;
uint8_t motor_Lap_pulse = 0;

void Speed::GetMotorLapPulse() {
    ModuleMacInfo * param = (ModuleMacInfo *)(FLASH_MODULE_PARA);
    if (param->other_parm[0] > 0 && param->other_parm[0] <=4) {
      SetLapPulse(param->other_parm[0]);
    } else {
      SetLapPulse(DEFAULT_MOTOR_LAP_PULSE);
    }
}

void Speed::SetLapPulse(uint8_t pulse) {
  motor_Lap_pulse = pulse;
}

void Speed::InitOut(uint8_t pwm_pin, uint8_t tim_num, uint8_t tim_chn, uint32_t freq) {
  this->pwm_tim_chn_ = tim_chn;
  this->pwm_tim_num_ = tim_num;
  uint32_t tim_freq = freq * MAX_SPEED_OUT;
  HAL_PwmInit(tim_num, tim_chn, pwm_pin, tim_freq, MAX_SPEED_OUT);
}


void Speed::InitDir(uint8_t dir_pin, uint8_t dir) {
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, dir);
}

void FgExtiCallBack(uint8_t exti_line) {
  speed_exti_count++;
}

void FgTimCallBack() {
  speed_tim_count = speed_exti_count;
  speed_exti_count = 0;
}

void Speed::InitCapture(uint8_t fg_pin, uint8_t tim_num) {
  ExtiInit(fg_pin, EXTI_Rising, FgExtiCallBack);
  HAL_timer_init(tim_num, 7200, 10000 / SPEED_CAPTURE_FREQUENCY);
  HAL_timer_nvic_init(tim_num, 1, 1);
  HAL_timer_cb_init(tim_num, FgTimCallBack);
  HAL_timer_enable(tim_num);
  GetMotorLapPulse();
}

uint32_t Speed::ReadCurSpeed() {
  return (speed_tim_count * SPEED_TO_RPM_RATE);
}

void Speed::SetSpeed(uint8_t percent) {
  if (percent > 100) {
    percent = 100;
  }
  if (is_pwm_inverter_) {
    percent = MAX_SPEED_OUT - percent;
  }
  this->target_speed_ = percent;
  this->speed_fail_flag_ = false;
  this->set_speed_time_ =  millis();
}

void Speed::ReportSpeed() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_MOTOR_SPEED);

  if (msgid != INVALID_VALUE) {
    uint16_t cur_speed = this->ReadCurSpeed();
    uint8_t data[8];
    uint8_t index = 0;
    data[index++] = cur_speed >> 8;
    data[index++] = cur_speed;
    if (this->SpeedStatuCheck() == true) {
      data[index++] = 0;  // normal
    } else {
      data[index++] = 1;  // fail
    }
    canbus_g.PushSendStandardData(msgid, data, index);
  }
}


bool Speed::SpeedStatuCheck() {
  uint32_t normal_speed = CNC_MAX_RPM * this->target_speed_ / 100;
  uint32_t min_speed = normal_speed * 60 / 100;

  if ((this->target_speed_ > 0) && this->ReadCurSpeed() < min_speed) {
    if (this->speed_fail_flag_ == false) {
      this->speed_fail_flag_ = true;
      this->set_speed_time_ =  millis();
    }
    if (this->speed_fail_flag_ == true) {
      if ((this->set_speed_time_ + 3000) < millis()) {
        HAL_PwmSetPulse(pwm_tim_num_, pwm_tim_chn_, 0);
        return false;
      }
    }
  } else {
    this->speed_fail_flag_ = false;
  }
  return true;
}

// need loop
void Speed::SpeedOutCtrl() {
  uint16_t out_pwm = 0;
  if ((this->change_time_ + 10) < millis()) {
    this->change_time_ = millis();
    if (this->target_speed_ != this->cur_set_percent_) {
      if (this->target_speed_ > this->cur_set_percent_) {
        this->cur_set_percent_ = this->target_speed_;
      } else if (this->target_speed_ < this->cur_set_percent_) {
        this->cur_set_percent_--;
      }
      out_pwm = (this->cur_set_percent_ * MAX_SPEED_OUT) / 100;
      HAL_PwmSetPulse(pwm_tim_num_, pwm_tim_chn_, out_pwm);
    }
  }
}