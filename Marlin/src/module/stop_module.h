#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_STOP_MODULE_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_STOP_MODULE_H_

#include "src/device/switch.h"
#include "src/configuration.h"
#include "module_base.h"

#define SWITCH_CHECK_PIN PA3
#define GREEN_LIGHT_PIN PA0
#define RED_LIGHT_PIN PA1

#define LIGHT_ON 1
#define LIGHT_OFF (!LIGHT_ON)

#define SWITCH_DOWN 0
#define SWITCH_UP (!SWITCH_DOWN)

class StopModule : public ModuleBase {

 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();

 private:
  void LightStateDown();
  void LightStateUp();
  void LightStateDisconnect();

 private:
  SwitchInput switch_;
  SwitchOutput green_;
  SwitchOutput red_;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_STOP_MODULE_H_
