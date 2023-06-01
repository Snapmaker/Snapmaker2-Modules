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
// #include "../registry/route.h"
#include <src/HAL/hal_tim.h>
#include <math.h>
#include "laser_head_20w_40W.h"

void LaserHead20W40W::Init() {
    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
    laser_power_ctrl_.Init(LASER_20W_40W_ENBLE_PIN, 0, OUTPUT);
    fan_.Init(LASER_20W_40W_FAN_PIN, LSAER_FAN_FB_IC_TIM, LSAER_FAN_FB_CH, LSAER_FAN_FB_IT_CH, FAN_FEEDBACK_THRESHOLD);
    temperature_.InitCapture(LASER_20W_40W_TEMP_PIN, ADC_TIM_4);
    hw_version_.index = HAL_adc_init(LASER_20W_40W_HW_VERSION_PIN, ADC_TIM_4, ADC_PERIOD_DEFAULT);
    pwm_detect_.Init(LASER_20W_40W_PWM_DETECT, INPUT_PULLUP);
    cross_light_.Init(LASER_20W_40W_CROSS_LIGHT, OUTPUT);

    AppParmInfo *param = &registryInstance.cfg_;
    sync_id_ = param->module_sync_id;
    if ((uint8_t)param->laser_protect_temp == 0xff) {
        protect_temp_ = LASER_20W_40W_TEMP_LIMIT;
    } else {
        protect_temp_ = param->laser_protect_temp;
    }

    if ((uint8_t)param->laser_protect_temp == 0xff) {
        recovery_temp_ = LASER_20W_40W_TEMP_LIMIT;
    } else {
        recovery_temp_ = param->laser_recovery_temp;
    }

    security_status_ |= FAULT_LASER_PWM_PIN;

    if (icm42670.ChipInit() == false) {
        security_status_ |= FAULT_IMU_CONNECTION;
    }
}

void LaserHead20W40W::GetHwVersion() {
    hw_version_.adc_value = ADC_Get(hw_version_.index);

    if (hw_version_.adc_value == 0)
        return;

    if (hw_version_.number != 0xAA)
        return;

    if (hw_version_.adc_value > 3700) { //3.3v
        hw_version_.number = 0xff;
        fan_.set_feed_back_enable(false);
    } else if (hw_version_.adc_value > 670 && hw_version_.adc_value < 770) {    //0.57v
        hw_version_.number = 0;
        fan_.set_feed_back_enable(true);
    }
}

void LaserHead20W40W::Loop() {
    GetHwVersion();
    fan_.Loop();
    laser_power_ctrl_.OutCtrlLoop();
    cross_light_.OutCtrlLoop();
    SecurityStatusCheck();
}

void LaserHead20W40W::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
    switch (func_id) {
        case FUNC_SET_FAN:
            fan_.ChangePwm(data[1], data[0]);
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
        case FUNC_MODULE_LASER_CTRL:
            LaserCtrl(data);
            break;
        case FUNC_MODULE_GET_HW_VERSION:
            LaserReportHWVersion();
            break;
        case FUNC_REPORT_PIN_STATUS:
            LaserReportPinState();
            break;
        case FUNC_CONFIRM_PIN_STATUS:
            LaserConfirmPinState();
            break;
        default:
            break;
  }
}

void LaserHead20W40W::EmergencyStop() {
    laser_power_ctrl_.Out(0);
    fan_.ChangePwm(0, 0);
}

void LaserHead20W40W::SecurityStatusCheck() {
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

    if (fan_.get_feed_back_state() == false) {
        security_status_ |= FAULT_LASER_FAN_RUN;
    } else {
        security_status_ &= ~FAULT_LASER_FAN_RUN;
    }

    if (security_status_ != security_status_pre_) {
        security_status_pre_ = security_status_;
        if (security_status_ != 0) {
            laser_power_ctrl_.Out(0);
        }
        ReportSecurityStatus();
    }
}

void LaserHead20W40W::ReportSecurityStatus() {
  uint8_t buf[8];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_SECURITY_STATUS);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE) {
    icm42670.GetGesture(yaw_, pitch_, roll_);
    imu_celsius_ = (int8_t)icm42670.GetTemperature();
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
    buf[index++] = (uint8_t)imu_celsius_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserOnlineStateSync(uint8_t *data) {
    AppParmInfo *param = &registryInstance.cfg_;
    if (data[0] == 1) {
        // set module sync id
        sync_id_ = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        param->module_sync_id = sync_id_;
        registryInstance.SaveCfg();
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

void LaserHead20W40W::LaserSetProtectTemp(uint8_t *data) {
    AppParmInfo *param = &registryInstance.cfg_;
    protect_temp_ = data[0];
    recovery_temp_ = data[1];

    param->laser_protect_temp = protect_temp_;
    param->laser_recovery_temp = recovery_temp_;
    registryInstance.SaveCfg();
}

void LaserHead20W40W::LaserCtrl(uint8_t *data) {
    switch (data[0]) {
        case 0:
            laser_power_ctrl_.Out(0);
            break;
        case 1:
            laser_power_ctrl_.Out(1);
            break;
    }

  uint8_t buf[1];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_LASER_CTRL);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE) {
    buf[index++] = data[0];
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserReportHWVersion() {
  ModuleMacInfo * mac = (ModuleMacInfo *)FLASH_MODULE_PARA;

  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_GET_HW_VERSION);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE) {
    if (hw_version_.number == 0xAA)
        buf[index++] = mac->hw_version;
    else
        buf[index++] = hw_version_.number;
    // to have a simple checksum
    buf[index++] = ~buf[0];
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserReportPinState() {
  uint8_t buf[1];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PIN_STATUS);
  if (msgid != INVALID_VALUE) {
    buf[index++] = digitalRead(LASER_20W_40W_PWM_DETECT);
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserConfirmPinState() {
  security_status_ &= ~FAULT_LASER_PWM_PIN;
}

