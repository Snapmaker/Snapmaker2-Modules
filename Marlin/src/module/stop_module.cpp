#include <board/board.h>
#include "stop_module.h"
#include "src/configuration.h"
#include "src/registry/registry.h"
#include "wirish_time.h"

void StopModule::Init() {
  switch_.Init(SWITCH_CHECK_PIN);
  green_.Init(GREEN_LIGHT_PIN, LIGHT_OFF, OUTPUT);
  red_.Init(RED_LIGHT_PIN, LIGHT_OFF, OUTPUT);
}

void StopModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_STOP_SWITCH:
      switch_.ReportStatu(FUNC_REPORT_STOP_SWITCH);
      break;
  }
}

void StopModule::LightStateDown() {
  static uint8_t last_status = 0;
  static uint32_t last_time = 0;
  if ((last_time + 250) > millis()) {
    return ;
  }
  last_time = millis();
  if (last_status) {
    red_.Out(LIGHT_ON);
    last_status = 0;
  } else {
    red_.Out(LIGHT_OFF);
    last_status = 1;
  }
  green_.Out(LIGHT_OFF);
}

void StopModule::LightStateUp() {
  green_.Out(LIGHT_ON);
  red_.Out(LIGHT_OFF);
}

void StopModule::LightStateDisconnect() {
  static uint8_t last_status = 0;
  static uint32_t last_time = 0;
  if ((last_time + 200) > millis()) {
    return ;
  }
  last_time = millis();
  if (last_status) {
    green_.Out(LIGHT_ON);
    red_.Out(LIGHT_OFF);
    last_status = 0;
  } else {
    green_.Out(LIGHT_OFF);
    red_.Out(LIGHT_ON);
    last_status = 1;
  }
}

void StopModule::Loop() {
  if (switch_.CheckStatuLoop()) {
    switch_.ReportStatu(FUNC_REPORT_STOP_SWITCH);
  }

  if (!registryInstance.IsConnect()) {
    LightStateDisconnect();
  } else if (switch_.Read() == SWITCH_DOWN) {
    LightStateDown();
  } else {
    LightStateUp();
  }
}
