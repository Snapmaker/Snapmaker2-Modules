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
    case FUNC_SET_PURIFIER:
      this->fan_.ChangePwm(data[1], data[0]);
      break;
  }
}

void PurifierModule::Loop() {
  this->fan_.Loop();
  this->breathing_light_.Loop();
}
