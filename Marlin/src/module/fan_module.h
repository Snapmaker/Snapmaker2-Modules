//
// Created by David Chen on 2019-08-03.
//

#ifndef SNAPMAKERMODULES_MARLIN_SRC_FAN_MODULE_H_
#define SNAPMAKERMODULES_MARLIN_SRC_FAN_MODULE_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "module_base.h"

#define FAN_MODULE_PIN PA4

class FanModule : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();
  Fan fan_;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_FAN_MODULE_H_
