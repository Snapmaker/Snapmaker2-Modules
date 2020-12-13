//
// Created by David Chen on 2019-07-16.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_

#include <stdint.h>
#include "device_base.h"
#include <io.h>

class SwitchInput {
 public:
  void Init(uint8_t pin, WiringPinMode mode=INPUT_PULLUP);
  uint8_t Read();
  bool CheckStatuLoop();
  void ReportStatu(uint16_t funcid);
 private:
  uint8_t pin_;
  uint8_t status_;
  uint8_t cur_statu;
  uint8_t last_statu_;
  uint32_t time_;
};

class SwitchOutput {
 public:
  void Init(uint8_t pin, uint8_t out_val, WiringPinMode mode=OUTPUT_OPEN_DRAIN);
  void Out(uint8_t out);
  void DelayOut(uint8_t out, uint32_t delay_time_ms);
  void ReastOut(uint32_t reset_time_ms);
  void OutCtrlLoop();
 private:
  uint8_t pin_;
  uint32_t time_;
  uint32_t delay_time_;
  uint8_t out_val_;  // bit0 out val, bit1 out flag
};
#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_
