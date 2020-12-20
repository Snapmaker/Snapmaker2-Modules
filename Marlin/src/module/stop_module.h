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
