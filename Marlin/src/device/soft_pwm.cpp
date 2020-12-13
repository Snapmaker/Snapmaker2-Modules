//
// Created by David Chen on 2019-07-16.
//

#include <stdexcept>
#include <board/board.h>
#include <io.h>
#include <src/HAL/hal_tim.h>
#include "src/device/soft_pwm.h"


SoftPwm soft_pwm_g;
void PwmTimIsrCallBack() {
  soft_pwm_g.Isr();
}

void SoftPwm::HalTimInit() {
  if (this->tim_init_falg_ == true) {
    return ;
  }
  HAL_timer_init(SOFT_PWM_TIM, 72, 10);
  HAL_timer_nvic_init(SOFT_PWM_TIM, 3, 3);
  HAL_timer_cb_init(SOFT_PWM_TIM, PwmTimIsrCallBack);
  HAL_timer_enable(SOFT_PWM_TIM);
  this->tim_init_falg_ = true;
}

void SoftPwm::Isr() {
  for (int i = 0; i < this->used_count_; i++) {
    if (this->cnt_[i] < (this->threshold_[i])) {
      digitalWrite(this->pin_list_[i], HIGH);
    } else {
      digitalWrite(this->pin_list_[i], LOW);
    }
    this->cnt_[i] = (this->cnt_[i] + 1) % this->period_[i];
  }
}

int SoftPwm::AddPwm(uint8_t pwm_pin, uint32_t period) {
  uint8_t cur_index = this->used_count_;
  if (cur_index >= PWM_MAX_COUNT) {
    return -1;
  }
  HalTimInit();
  this->pin_list_[cur_index] = pwm_pin;
  this->threshold_[cur_index] = 0;
  this->period_[cur_index] = period;
  this->cnt_[cur_index] = 0;
  pinMode(this->pin_list_[cur_index], OUTPUT);
  this->used_count_++;
  return cur_index;
}

void SoftPwm::ChangeSoftPWM(uint8_t pwm_index, uint32_t threshold) {
  if (pwm_index >= PWM_MAX_COUNT) {
    return ;
  }
  this->threshold_[pwm_index] = (threshold > this->period_[pwm_index]) ? this->period_[pwm_index] : threshold;
}
