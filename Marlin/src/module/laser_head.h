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

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_LASER_HEAD_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_LASER_HEAD_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "module_base.h"
#include "laser_hw_version.h"

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
