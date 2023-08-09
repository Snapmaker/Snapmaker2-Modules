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

#include <src/configuration.h>
#include <src/core/utils.h>
#include <src/registry/registry.h>
#include <src/core/can_bus.h>
#include <src/registry/route.h>
#include "startup.h"

uint32_t Startup::SelfDetct() {
  registryInstance.Init();
  return 0;
}

void Startup::BasePeriphInit() {
  canbus_g.Init(registryInstance.ModuleCanId());
}

void Startup::PeriphInit() {
  routeInstance.Init();
}

void Startup::FuncIdListInit() {
  registryInstance.InitlizeFuncIds();
}

Startup startupInstance;
