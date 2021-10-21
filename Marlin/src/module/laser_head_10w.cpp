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
#include "src/HAL/hal_flash.h"
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
#include "src/device/icm4xxxx/icm4xxxx_driver.h"
#include <wirish_time.h>
#include "../registry/route.h"
#include <src/HAL/hal_tim.h>
#include <math.h>
#include "laser_head_10w.h"

void LaserHead10W::Init() {
    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

    camera_power_.Init(LASER10W_CAMERA_POWER_PIN, 0, OUTPUT);
    autofocus_light_.Init(LASER10W_AUTOFOCUS_LIGHT_CTRL_PIN, 0, OUTPUT);
    laser_power_ctrl_.Init(LASER10W_ENBLE_PIN, 0, OUTPUT);
    fan_.Init(LASER10W_FAN_PIN);
    temperature_.InitCapture(LASER10W_TEMP_PIN, ADC_TIM_4);

    AppParmInfo parm;
    HAL_flash_read(FLASH_APP_PARA, (uint8_t*)&parm, sizeof(parm));
    sync_id_ = parm.module_sync_id;
    if (parm.laser_protect_temp == 0xff) {
        protect_temp_ = LASER_TEMP_LIMIT;
    } else {
        protect_temp_ = parm.laser_protect_temp;
    }

    if (parm.laser_protect_temp == 0xff) {
        recovery_temp_ = LASER_TEMP_RECOVERY;
    } else {
        recovery_temp_ = parm.laser_recovery_temp;
    }

    if (icm42670.ChipInit() == false) {
        security_status_ |= FAULT_IMU_CONNECTION;
    }
}

void LaserHead10W::Loop() {
    camera_power_.OutCtrlLoop();
    fan_.Loop();
    SecurityStatusCheck();
}

void LaserHead10W::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
    uint8_t focus_type;
    switch (func_id) {
        case FUNC_SET_FAN:
            fan_.ChangePwm(data[1], data[0]);
            break;
        case FUNC_SET_CAMERA_POWER:
            camera_power_.ReastOut(data[0]<<8 | data[1]);
            break;
        case FUNC_SET_LASER_FOCUS:
            focus_type = data_len > 2 ? data[2] : 0;
            LaserSaveFocus(focus_type, data[0]<<8 | data[1]);
            break;
        case FUNC_REPORT_LASER_FOCUS:
            focus_type = data_len ? data[0] : 0;
            LaserReportFocus(focus_type);
            break;
        case FUNC_SET_AUTOFOCUS_LIGHT:
            SetAutoFocusLight(data[0]);
            break;
        case FUNC_REPORT_SECURITY_STATUS:
            ReportSecurityStatus();
            break;
        case FUNC_MODULE_ONLINE_SYNC:
            LaserOnlineStateSync(data);
            break;
        case FUNC_MODULE_SET_TEMP:
            LaserSetProtectTemp(data);
            break;
        default:
            break;
  }
}

void LaserHead10W::SecurityStatusCheck() {
    // wait message id to be asigned
    if (registryInstance.FuncId2MsgId(FUNC_REPORT_SECURITY_STATUS) == INVALID_VALUE) {
        return;
    }

    temperature_.GetTemperature(laser_celsius_);

    if ((security_status_ & FAULT_IMU_CONNECTION) == 0) {
        if (icm42670.AttitudeSolving() == true) {
            icm42670.GetGesture(yaw_, pitch_, roll_);
        }
    }

    if (laser_celsius_ > protect_temp_) {
        security_status_ |= FAULT_LASER_TEMP;
    } else if (laser_celsius_ < recovery_temp_) {
        security_status_ &= ~FAULT_LASER_TEMP;
    }

    if ((roll_ <= roll_min_) || (roll_ >= roll_max_) || (pitch_ <= pitch_min_) || (pitch_ >= pitch_max_)) {
        security_status_ |= FAULT_LASER_GESTURE;
    } else {
        security_status_ &= ~FAULT_LASER_GESTURE;
    }

    if (security_status_ != security_status_pre_) {
        security_status_pre_ = security_status_;
        if (security_status_ != 0) {
            laser_power_ctrl_.Out(0);
            autofocus_light_.Out(0);
        } else {
            laser_power_ctrl_.Out(1);
        }
        ReportSecurityStatus();
    }
}

void LaserHead10W::SetAutoFocusLight(uint8_t state) {
  autofocus_light_.Out(state ? 1 : 0);

  uint8_t u8DataBuf[1], u8Index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_SET_AUTOFOCUS_LIGHT);
  if (msgid != INVALID_VALUE) {
    u8DataBuf[u8Index++] = state;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

void LaserHead10W::ReportSecurityStatus() {
  uint8_t buf[8];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_SECURITY_STATUS);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE) {
    icm42670.GetGesture(yaw_, pitch_, roll_);
    int16_t pitch_int16, roll_int16;
    int8_t celsius_int8;
    pitch_int16   = (int16_t)pitch_;
    roll_int16    = (int16_t)roll_;
    celsius_int8  = (signed char)laser_celsius_;

    buf[index++] = security_status_;
    buf[index++] = (pitch_int16 >> 8) & 0xff;
    buf[index++] = pitch_int16 & 0xff;
    buf[index++] = (roll_int16 >> 8) & 0xff;;
    buf[index++] = roll_int16 & 0xff;
    buf[index++] = celsius_int8;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead10W::LaserSaveFocus(uint8_t type, uint16_t foch) {
    AppParmInfo parm;
    HAL_flash_read(FLASH_APP_PARA, (uint8_t*)&parm, sizeof(parm));
    parm.parm_mark[0] = 0xaa;
    parm.parm_mark[1] = 0x55;
    if (type) {
      parm.laser_high_4_axis = foch;
    } else {
      parm.laser_high = foch;
    }

    HAL_flash_erase_page(FLASH_APP_PARA, 1);
    HAL_flash_write(FLASH_APP_PARA, (uint8_t *)&parm, sizeof(parm));
}

void LaserHead10W::LaserReportFocus(uint8_t type) {
    AppParmInfo parm;
    uint8_t u8DataBuf[8], u8Index = 0;
    uint16_t u16Focu = 0;
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_LASER_FOCUS);
    if (msgid != INVALID_VALUE) {
      HAL_flash_read(FLASH_APP_PARA, (uint8_t*)&parm, sizeof(parm));
      if (type) {
        u16Focu = parm.laser_high_4_axis;
      } else {
        u16Focu = parm.laser_high;
      }
      if (!(parm.parm_mark[0] == 0xaa && parm.parm_mark[1] == 0x55) || (u16Focu == 0xffff)) {
          u16Focu = (uint16_t)LASER_DEFAULT_HIGH;
      }
      u8DataBuf[u8Index++] = u16Focu >> 8;
      u8DataBuf[u8Index++] = u16Focu;
      canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
    }
}

void LaserHead10W::LaserOnlineStateSync(uint8_t *data) {
    if (data[0] == 1) {
        // set module sync id
        sync_id_ = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        AppParmInfo parm;
        HAL_flash_read(FLASH_APP_PARA, (uint8_t*)&parm, sizeof(parm));
        parm.parm_mark[0] = 0xaa;
        parm.parm_mark[1] = 0x55;
        parm.module_sync_id = sync_id_;
        HAL_flash_erase_page(FLASH_APP_PARA, 1);
        HAL_flash_write(FLASH_APP_PARA, (uint8_t *)&parm, sizeof(parm));
    } else if (data[0] == 0) {
        // report module sync id
        uint8_t buf[8];
        uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_ONLINE_SYNC);
        uint8_t index = 0;
        if (msgid != INVALID_VALUE) {
            buf[index++] = sync_id_ & 0xff;
            buf[index++] = (sync_id_ >> 8) & 0xff;
            buf[index++] = (sync_id_ >> 16) & 0xff;
            buf[index++] = (sync_id_ >> 24) & 0xff;
            canbus_g.PushSendStandardData(msgid, buf, index);
        }
    }
}

void LaserHead10W::LaserSetProtectTemp(uint8_t *data) {
    protect_temp_ = data[0];
    recovery_temp_ = data[1];

    AppParmInfo parm;
    HAL_flash_read(FLASH_APP_PARA, (uint8_t*)&parm, sizeof(parm));
    parm.parm_mark[0] = 0xaa;
    parm.parm_mark[1] = 0x55;
    parm.laser_protect_temp = protect_temp_;
    parm.laser_recovery_temp = recovery_temp_;
    HAL_flash_erase_page(FLASH_APP_PARA, 1);
    HAL_flash_write(FLASH_APP_PARA, (uint8_t *)&parm, sizeof(parm));
}


