//
// Created by David Chen on 2019-07-16.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_

#include <stdint.h>
#include "device_base.h"

#define FAN_MAX_THRESHOLD 255

class Fan {
 public:
  void Init(uint8_t fan_pin);
  void Loop();
  void ChangePwm(uint8_t threshold, uint16_t delay_close_time_s);


 private:
  uint8_t fan_index_;
  uint32_t delay_close_time_;
  uint32_t delay_start_time_;
  bool  delay_close_enadle_;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FAN_H_
