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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_

#include <src/configuration.h>
#include <src/device/temperature.h>
#include <src/module/print_head.h>
#include <src/device/fan.h>
#include <src/module/linear_module.h>
#include "registry.h"
#include "context.h"
#include "src/module/print_head.h"
#include "src/module/laser_head.h"
#include "src/module/cnc_head.h"
#include "src/module/light_module.h"
#include "src/module/cnc_tool_setting.h"
#include "src/module/enclosure.h"
#include "src/module/fan_module.h"
#include "src/module/purifier_module.h"
#include "src/module/stop_module.h"
#include "src/module/rotate_module.h"
#include "src/module/laser_head_10w.h"
#include "src/module/cnc_head_200w.h"
#include "src/module/enclosure_a400.h"
#include "src/module/dual_extruder.h"
#include "src/module/drybox.h"
#include "src/module/calibrator.h"
class Route {
 public:
  void Invoke();
  void Init();
  void ModuleLoop();
  // eg. v1.10.2 -> SetBaseVersions(1, 10, 2)
  void SetBaseVersions(uint8_t level_1, uint8_t level_2, uint8_t level_3) {
    base_version[0] = level_1;
    base_version[1] = level_2;
    base_version[2] = level_3;
  };

  bool VersionComparison(uint8_t level_1, uint8_t level_2, uint8_t level_3) {
    if (level_1 >= base_version[0]) {
      if (level_2 >= base_version[1]) {
        if (level_3 >= base_version[2]) {
          return true;
        }
      }
    }
    return false;
  }

 public:
  ModuleBase * module_;
  uint16_t const * func_list_ = NULL;
  uint8_t func_count_ = 0;
  uint8_t base_version[3] = {0, 0, 0};
 private:
};

extern Route routeInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_
