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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_BREATHING_LIGHT_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_BREATHING_LIGHT_H_

#include <stdint.h>

#define TIM_FREQUENCY 100000

class BreathingLight {
 public:
  void Init(uint8_t pin, uint16_t frequency);
  void Loop();
  void Enable(bool enable);
  void Set(uint16_t frequency);


 private:
  uint8_t index_;
  bool enable_ = false;
  uint32_t execute_time_ = 0;
  float one_step_= 0;
  uint32_t total_step_= 0;
  uint32_t cur_step_ = 0;
  bool dir_ = true;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_BREATHING_LIGHT_H_
