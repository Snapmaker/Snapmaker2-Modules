//
// Created by David Chen on 2019-07-16.
//

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
      soft_pwm_g.ChangeSoftPWM(this->fan_index_, 0);
    }
  }
}

void Fan::Init(uint8_t fan_pin) {
  this->fan_index_ =  soft_pwm_g.AddPwm(fan_pin, FAN_MAX_THRESHOLD);
}

void Fan::ChangePwm(uint8_t threshold, uint16_t delay_close_time_s) {
  this->delay_close_time_ = delay_close_time_s * 1000;
  if (threshold == 0) {
    this->delay_start_time_ = millis();
    this->delay_close_enadle_ = true;
  } else {
    this->delay_close_enadle_ = false;
    soft_pwm_g.ChangeSoftPWM(this->fan_index_, threshold);
  }
}
