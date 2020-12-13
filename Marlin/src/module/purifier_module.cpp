#include "src/module/purifier_module.h"
#include "io.h"
#include <wirish_time.h>
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
#include "src/HAL/hal_reset.h"
void PurifierModule::Init() {
  this->fan_.Init(PURIFIER_FAN_PIN);
  this->fan_.ChangePwm(255, 0);
  HAL_JTAGDisable();
  this->breathing_light_.Init(PURIFIER_LIGHT_PIN, 3500);
}

void PurifierModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_SET_PURIFIER_FUN:
      this->fan_.ChangePwm(data[1], data[0]);
      break;
  }
}

void PurifierModule::Loop() {
  this->fan_.Loop();
  this->breathing_light_.Loop();
}
