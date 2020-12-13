//
// Created by David Chen on 2019-08-03.
//

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_PROOFREAD_KNIFE_MODULE_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_PROOFREAD_KNIFE_MODULE_H_

#include <src/device/switch.h>
#include "src/configuration.h"
#include "module_base.h"

class CncToolSetting : public ModuleBase {

 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();

 private:
  SwitchInput switch_;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_PROOFREAD_KNIFE_MODULE_H_
