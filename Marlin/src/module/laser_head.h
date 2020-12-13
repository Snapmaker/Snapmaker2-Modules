//
// Created by David Chen on 2019-08-03.
//

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_LASER_HEAD_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_LASER_HEAD_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "module_base.h"

#define LASER_FAN_PIN PA4
#define LASER_CAMERA_POWER_PIN PA8
class LaserHead : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();
  void LaserSaveFocus(uint8_t type, uint16_t foch);
  void LaserReportFocus(uint8_t type);
  Fan fan_;
  SwitchOutput camera_power_;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_LINEAR_MODULE_H_
