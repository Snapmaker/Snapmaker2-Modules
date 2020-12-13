//
// Created by David Chen on 2019-07-19.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PRINTHEAD_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PRINTHEAD_H_

#include <src/device/switch.h>
#include <src/device/temperature.h>
#include <src/device/fan.h>
#include "module_base.h"
#include <src/configuration.h>

#define FAN_1_PIN PA4
#define FAN_2_PIN PA5
// If we have composite logic in module, we should implement this moudle layer
// For now, we should keep it simple, let it group device together. But let the logic in route.cpp
class PrintHead : public ModuleBase {
 public:
  void Init();
  void PeriphInit();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();

  Fan fan_1_;
  Fan fan_2_;
  SwitchInput switch_probe_;
  SwitchInput switch_cut_;
  Temperature  temperature_;

 private:
  uint32_t temp_report_time_ = 0;
  uint32_t cut_report_time_ = 0;
  bool is_report_cut_ = false;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PRINTHEAD_H_
