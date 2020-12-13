
#include "fan_module.h"
#include <board/board.h>
#include "src/HAL/hal_flash.h"
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
void FanModule::Init() {
  this->fan_.Init(FAN_MODULE_PIN);
}

void FanModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_SET_FAN_MODULE:
      this->fan_.ChangePwm(data[1], data[0]);
      break;
  }
}

void FanModule::EmergencyStop() {
  fan_.ChangePwm(0, 0);
}

void FanModule::Loop() {
  this->fan_.Loop();
}