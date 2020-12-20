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
#include "linear_module.h"
#include "src/configuration.h"
#include <io.h>

void LinearModule::Init() {
  this->limit_.Init(PB1);

  // set the tmc2209 to low power mode
  pinMode(PA9, OUTPUT);
  pinMode(PA10, OUTPUT);
  digitalWrite(PA9, 0);
  digitalWrite(PA10, 0);
}

void LinearModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_LIMIT:
      this->limit_.ReportStatus(FUNC_REPORT_LIMIT);
      break;
  }
}

void LinearModule::Loop() {
  if (this->limit_.CheckStatusLoop()) {
    this->limit_.ReportStatus(FUNC_REPORT_LIMIT);
  }
}