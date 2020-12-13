//
// Created by David Chen on 2019-08-03.
//

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_MODULE_BASE_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_MODULE_BASE_H_

class ModuleBase {
 public:
  virtual void Init() {};
  virtual void Loop() {};
  virtual void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {};
  virtual void EmergencyStop() {};
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_MODULE_BASE_H_
