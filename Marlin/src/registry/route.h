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
#include "registry.h"
#include "context.h"
#include "src/module/module_base.h"
class Route {
 public:
  void Invoke();
  void Init();
  void ModuleLoop();

 public:
  ModuleBase * module_;
  uint16_t const * func_list_ = NULL;
  uint8_t func_count_ = 0;
 private:
};

extern Route routeInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_
