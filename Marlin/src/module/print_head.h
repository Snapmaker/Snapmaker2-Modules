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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PRINTHEAD_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PRINTHEAD_H_

#include <src/device/switch.h>
#include <src/device/temperature.h>
#include <src/device/fan.h>
#include "module_base.h"
#include <src/configuration.h>

#define FAN_1_PIN PA4
#define FAN_2_PIN PA5
// If we have composite logic in module, we should implement this moudle layer
// For now, we should keep it simple, let it group device together. But let the logic in route.cpp
class PrintHead : public ModuleBase {
 public:
  void Init();
  void PeriphInit();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();

  Fan fan_1_;
  Fan fan_2_;
  SwitchInput switch_probe_;
  SwitchInput switch_cut_;
  Temperature  temperature_;

 private:
  uint32_t temp_report_time_ = 0;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_MODULE_PRINTHEAD_H_
