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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_

#include <stdint.h>
#include "device_base.h"
#include <io.h>

class SwitchInput {
 public:
  SwitchInput () {
    input_io_need_reverse_ = false;
  }
  void Init(uint8_t pin, WiringPinMode mode=INPUT_PULLUP);
  void Init(uint8_t pin, bool input_io_reverse, WiringPinMode mode);
  uint8_t Read();
  bool CheckStatusLoop();
  void ReportStatus(uint16_t funcid);
 private:
  uint8_t pin_;
  uint8_t status_;
  uint8_t cur_statu;
  uint8_t last_statu_;
  uint32_t time_;
  bool input_io_need_reverse_;
};

class SwitchOutput {
 public:
  void Init(uint8_t pin, uint8_t out_val, WiringPinMode mode=OUTPUT_OPEN_DRAIN);
  void Out(uint8_t out);
  void DelayOut(uint8_t out, uint32_t delay_time_ms);
  void ReastOut(uint32_t reset_time_ms);
  void OutCtrlLoop();
 private:
  uint8_t pin_;
  uint32_t time_;
  uint32_t delay_time_;
  uint8_t out_val_;  // bit0 out val, bit1 out flag
};
#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_
