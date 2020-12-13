//
// Created by David Chen on 2019-07-16.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_BREATHING_LIGHT_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_BREATHING_LIGHT_H_

#include <stdint.h>

#define TIM_FREQUENCY 100000

class BreathingLight {
 public:
  void Init(uint8_t pin, uint16_t frequency);
  void Loop();
  void Enable(bool enable);
  void Set(uint16_t frequency);


 private:
  uint8_t index_;
  bool enable_ = false;
  uint32_t execute_time_ = 0;
  float one_step_= 0;
  uint32_t total_step_= 0;
  uint32_t cur_step_ = 0;
  bool dir_ = true;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_BREATHING_LIGHT_H_
