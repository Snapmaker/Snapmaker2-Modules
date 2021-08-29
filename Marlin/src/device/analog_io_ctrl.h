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

#ifndef ANALOG_IO_CTRL_H_
#define ANALOG_IO_CTRL_H_

#include <stdint.h>

#define REFERENCE_VOLTAGE 3.0

class Dac {
 public:
  void Init();
  void Output(uint16_t dac_val);

 private:
  uint8_t pin_;
  uint16_t dac_val_;
};

class Adc {
 public:
  void Init(uint8_t pin);
  uint16_t ReadVoltage();
  uint16_t ReadAdcVal();

 private:
  uint8_t pin_;
};
#endif //ANALOG_IO_CTRL_H_
