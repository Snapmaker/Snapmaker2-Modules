//
// Created by David Chen on 2019-08-03.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PURIFIER_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PURIFIER_H_

#include "src/configuration.h"
#include "src/device/fan.h"
#include "src/device/breathing light.h"
#include "module_base.h"

#define ENCLOSURE_CLOSE_STATU 0
#define PURIFIER_FAN_PIN PB5
#define PURIFIER_LIGHT_PIN PB4
class PurifierModule : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();

 private:
  Fan fan_;
  BreathingLight breathing_light_;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PURIFIER_H_
