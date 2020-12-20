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

#include <src/core/can_bus.h>
#include "route.h"

#define FUNC_LIST_INIT(list)  func_list_ = list;\
                              func_count_ = sizeof(list) / sizeof(list[0]);

Route routeInstance;
void Route::Init() {
  uint32_t moduleType = registryInstance.module();

  switch (moduleType) {
  }
}

void Route::Invoke() {
  uint16_t func_id = contextInstance.funcid_;
  uint8_t * data = contextInstance.data_;
  uint8_t   data_len = contextInstance.len_;
  module_->HandModule(func_id, data, data_len);
}

void Route::ModuleLoop() {
  module_->Loop();
}

