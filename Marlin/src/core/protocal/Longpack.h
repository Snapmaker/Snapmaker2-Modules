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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PROTOCAL_LONGPACK_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PROTOCAL_LONGPACK_H_

#include <stdint.h>
#include <src/core/common_type.h>

#define MAX_SYS_CMD_LEN 1024

struct PackHead {
  uint8_t magic1;
  uint8_t magic2;
  uint8_t lenHigh;
  uint8_t lenLow;
  uint8_t version;
  uint8_t lenCheck;
  uint8_t dataCheckHigh;
  uint8_t dataCheckLow;
};



class Longpack {
 public:
  ERR_E parseCmd();
  void sendLongpack(uint8_t* data, uint16_t len);
  void sendLongpack(uint16_t* data, uint16_t len);
  void cmd_clean();
  public:
    uint8_t packData_[MAX_SYS_CMD_LEN];
    uint8_t * cmd = packData_ + sizeof(PackHead);
    uint16_t len_ = 0;

  private:
    uint16_t recv_index_ = 0;
};

extern Longpack longpackInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PROTOCAL_LONGPACK_H_
