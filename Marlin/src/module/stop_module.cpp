/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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
      switch_.ReportStatus(FUNC_REPORT_STOP_SWITCH);
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
  if (switch_.CheckStatusLoop()) {
    switch_.ReportStatus(FUNC_REPORT_STOP_SWITCH);
  }

  if (!registryInstance.IsConnect()) {
    LightStateDisconnect();
  } else if (switch_.Read() == SWITCH_DOWN) {
    LightStateDown();
  } else {
    LightStateUp();
  }
}
