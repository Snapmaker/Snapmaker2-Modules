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

#include <board/board.h>
#include "src/module/cnc_tool_setting.h"
#include "src/configuration.h"


void CncToolSetting::Init() {
  this->switch_.Init(PA7);
}

void CncToolSetting::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_TOOL_SETTING:
      this->switch_.ReportStatus(FUNC_REPORT_TOOL_SETTING);
      break;
  }
}

void CncToolSetting::Loop() {
  if (this->switch_.CheckStatusLoop()) {
    this->switch_.ReportStatus(FUNC_REPORT_TOOL_SETTING);
  }
}