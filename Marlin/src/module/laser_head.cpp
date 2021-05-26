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

#include "laser_head.h"
#include <board/board.h>
#include "src/HAL/hal_flash.h"
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
void LaserHead::Init() {
  this->fan_.Init(LASER_FAN_PIN);
  this->camera_power_.Init(LASER_CAMERA_POWER_PIN, 0);
}

void LaserHead::LaserReportFocus(uint8_t type) {
    AppParmInfo *param = &registryInstance.cfg_;;
    uint8_t u8DataBuf[8], u8Index = 0;
    uint16_t u16Focu = 0;
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_LASER_FOCUS);
    if (msgid != INVALID_VALUE) {
      if (type) {
        u16Focu = param->laser_high_4_axis;
      } else {
        u16Focu = param->laser_high;
      }
      if (!(param->parm_mark[0] == 0xaa && param->parm_mark[1] == 0x55) || (u16Focu == 0xffff)) {
          u16Focu = (uint16_t)LASER_DEFAULT_HIGH;
      }
      u8DataBuf[u8Index++] = u16Focu >> 8;
      u8DataBuf[u8Index++] = u16Focu;
      canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
    }
}
void LaserHead::LaserSaveFocus(uint8_t type, uint16_t foch) {
    AppParmInfo *param = &registryInstance.cfg_;;
    if (type) {
      param->laser_high_4_axis = foch;
    } else {
      param->laser_high = foch;
    }
    registryInstance.SaveCfg();
}
void LaserHead::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  uint8_t focus_type;
  switch (func_id) {
    case FUNC_SET_FAN:
      this->fan_.ChangePwm(data[1], data[0]);
      break;
    case FUNC_SET_CAMERA_POWER:
      this->camera_power_.ReastOut(data[0]<<8 | data[1]);
      break;
    case FUNC_SET_LASER_FOCUS:
      focus_type = data_len > 2 ? data[2] : 0;
      this->LaserSaveFocus(focus_type, data[0]<<8 | data[1]);
      break;
    case FUNC_REPORT_LASER_FOCUS:
      focus_type = data_len ? data[0] : 0;
      this->LaserReportFocus(focus_type);
      break;
  }
}

void LaserHead::EmergencyStop() {
  fan_.ChangePwm(0, 0);
}

void LaserHead::Loop() {
  this->camera_power_.OutCtrlLoop();
  this->fan_.Loop();
}