#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_ROTATE_MODULE_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_ROTATE_MODULE_H_

#include "src/device/switch.h"
#include "src/configuration.h"
#include "module_base.h"

class RotateModule : public ModuleBase {

 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_ROTATE_MODULE_H_
