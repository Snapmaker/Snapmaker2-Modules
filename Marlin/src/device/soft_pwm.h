//
// Created by David Chen on 2019-07-16.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_SOFT_PWM_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_SOFT_PWM_H_

#include <stdint.h>
#include "device_base.h"

#define PWM_MAX_COUNT 5
#define SOFT_PWM_TIM 3

#define SOFT_PWM_US(us) (us / 10) 
#define SOFT_PWM_MS(ms) SOFT_PWM_US(ms * 1000)

class SoftPwm {
 public:
  void Isr();
  void ChangeSoftPWM(uint8_t pwm_index, uint32_t threshold);
  int AddPwm(uint8_t pwm_pin, uint32_t period) ;

 private:
  void HalTimInit();
  bool tim_init_falg_ = false;
  uint8_t used_count_ = 0;
  uint32_t pin_list_[PWM_MAX_COUNT];
  uint32_t  threshold_[PWM_MAX_COUNT];
  uint32_t  period_[PWM_MAX_COUNT];
  uint32_t  cnt_[PWM_MAX_COUNT];
  uint32_t delay_close_time[PWM_MAX_COUNT];
  uint32_t delay_start_time[PWM_MAX_COUNT];
};

extern SoftPwm soft_pwm_g;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_SOFT_PWM_H_
