
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
