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

#ifndef MODULES_SOFT_I2C_H_
#define MODULES_SOFT_I2C_H_

#include <stdint.h>
#include "device_base.h"

class SoftI2C {
 public:
  void Init(uint8_t sda, uint8_t scl);
  void IICStart();
  void IICStop();
  bool IICWaitAck();
  void IICAck();
  void IICNAck();
  void IICSendByte(uint8_t data);
  uint8_t IICReadByte(bool send_ack);

 private:
  uint8_t i2c_sda_;
  uint8_t i2c_scl_;

};

#endif //MODULES_SOFT_I2C_H_
