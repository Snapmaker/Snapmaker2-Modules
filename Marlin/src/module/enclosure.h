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
  void ReportStatus();
  SwitchInput enclosure_;
  LightModule light_;
  Fan fan_;
  uint8_t last_statu_ = 0;
  uint8_t report_statu_;
  bool is_report_ = false;
  uint32_t time_ =  0;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_LINEAR_MODULE_H_
