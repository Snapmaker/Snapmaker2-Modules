//
// Created by David Chen on 2019-08-03.
//

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_ENCLOSURE_HEAD_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_ENCLOSURE_HEAD_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/module/light_module.h"
#include "src/device/fan.h"
#include "module_base.h"

#define ENCLOSURE_CLOSE_STATU 0
#define ENCLOSURE_FAN_PIN PA4
class EnclosureModule : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();
  void StartingUpLight();
 private:
  void ReportStatu();
  SwitchInput enclosure_;
  LightModule light_;
  Fan fan_;
  uint8_t last_statu_ = 0;
  uint8_t report_statu_;
  bool is_report_ = false;
  uint32_t time_ =  0;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_LINEAR_MODULE_H_
