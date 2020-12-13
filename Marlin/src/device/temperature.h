//
// Created by David Chen on 2019-07-17.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_

#include <src/core/pid.h>
#include "device_base.h"

class Temperature {
 public:
  Temperature();
  void InitCapture(uint8_t adc_chn, uint8_t adc_pin, uint8_t adc_tim);
  void InitOutCtrl(uint8_t tim_num, uint8_t tim_chn, uint8_t tim_pin);
  void ReportTemprature();
  void ReportPid();
  void SetPID(uint8_t pid_index, float val);

  void Maintain();
  void ChangeTarget(uint32_t target);

  bool isEnabled();

  float detect_celsius_;
  bool detect_ready_;
 private:
  int last_time_;
  uint8_t pwm_tim_num_;
  uint8_t pwm_tim_chn_;
  Pid  pid_;
  uint8_t pid_set_flag_ = 0;
  int count_;
  bool enabled_;
  void InitPID();
  void SavePID();
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_TEMPERATURE_H_
