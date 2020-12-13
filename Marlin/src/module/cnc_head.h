//
// Created by David Chen on 2019-08-03.
//

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_CNC_HEAD_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_CNC_HEAD_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "src/device/speed.h"
#include "module_base.h"

#define SPEED_TIM_MUN 3

class CncHead : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();

 private:
  Speed speed_;
  uint32_t time_ = 0;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_LINEAR_MODULE_H_
