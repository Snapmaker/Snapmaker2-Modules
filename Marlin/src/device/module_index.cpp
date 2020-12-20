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
  #include <io.h>
  #include "src/device/module_index.h"
  void ModuleIndex::Init() {
    pinMode(PIN_MODULE_INDEX, INPUT_FLOATING);
  }
  uint8_t ModuleIndex::SetModuleIndex(uint8_t axis_index) {
    module_index_ = MODULE_INDEX_NONE;
    if (digitalRead(PIN_MODULE_INDEX) != 0) {
      module_index_ = axis_index;
    }
    return module_index_ != MODULE_INDEX_NONE;
  }
  ModuleIndex moduleIndex;