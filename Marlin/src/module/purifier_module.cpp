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
#include "src/module/purifier_module.h"
#include "io.h"
#include <wirish_time.h>
#include "src/HAL/hal_adc.h"
#include "src/HAL/hal_flash.h"
#include "src/registry/registry.h"
#include "src/core/can_bus.h"

#define ERR_ON(e) (err_ |= (e))
#define ERR_CLN(e) (err_ &= ~(e))
#define IS_ERR(e) (err_ & (e))

#define IS_POWER_ERR(power) ((power) > 26000 || (power) < 23000)
#define IS_POWER_OFF(power) (power < 4000)
#define ELEC_TO_LIFETIME(elec) ((elec) <= lifetime_low_limit_[fan_gears_] ? LIFETIME_LOW : \
                                ((elec) <= lifetime_normal_limit_[fan_gears_]) ? LIFETIME_MEDIUM : LIFETIME_NORMAL)

static uint8_t fan_power_table[4] = {0, 30, 75, 100};

void PurifierModule::Init() {
  AppParmInfo *parm = &registryInstance.cfg_;
  registryInstance.SetConnectTimeout(CONNECT_TIMEOUT_MS);
  InitFan();
  filter_switch_.Init(FILTER_SWITCH_PIN, INPUT_PULLUP);
  InitAdcCheck();
  InitLight();
  LoadLifetime();
  is_forced_run_ = parm->purifier_forced_run == 1;
  fan_gears_ = parm->purifier_fan_gears;
  if (fan_gears_ > FAN_GEARS_3) {
    fan_gears_ = FAN_GEARS_3;
  }
  fan_state_ = FAN_STA_IDLE;
  fan_out_ = 0;
}

static uint16_t AdcToMv(uint16_t adc_val) {
  return 3.0 * adc_val * 11 * 1000 / 4096;
}

void PurifierModule::InitFan() {
  fan_power_.Init(PURIFIER_FAN_POWER_PIN, 0, OUTPUT);
  fan_power_.Out(0);
  fan_.InitOut(PURIFIER_FAN_PWM_PIN, PURIFIER_FAN_PWM_TIM);
  fan_.InitCapture(FAN_SPEED_CAPTURE_PARM);
  fan_.SetLapPulse(4);  // There are four pulses per turn
  FanOut(0);
}

void PurifierModule::InitAdcCheck() {
  addon_power_index_ = HAL_adc_init(ADDON_POWER_ADC);
  extend_power_index_ = HAL_adc_init(EXTEND_POWER_ADC);
  elec_adc_index_ = HAL_adc_init(FAN_ELEC_ADC);
}

void PurifierModule::InitLight() {
  light_.Init(PURIFIER_LIGHT_PIN, PURIFIER_LIGHT_COUNT, PURIFIER_LIGHT_EXTI);
  RGB_T color = {0, 0, 0};
  light_.StaticLight(color);
}

uint32_t PurifierModule::GetFanElectricity() {
  uint16_t elec_adc = ADC_Get(elec_adc_index_);
  float v = 3.0 * elec_adc / 4096;
  // 0.02 * 25 * I = v  --> I = v / 0.02 / 25
  return v / 0.02 / 25 * 1000;  // mA
}

uint16_t PurifierModule::GetAddonPower() {
  uint16_t adc = ADC_Get(addon_power_index_);
  return AdcToMv(adc);  // mv
}

uint16_t PurifierModule::GetExtendPower() {
  uint16_t adc = ADC_Get(extend_power_index_);
  return AdcToMv(adc);  // mv
}

void PurifierModule::FanOut(uint8_t power_gears) {
  if (is_debug_) {  // M1011 P control
    fan_power_.Out(1);
    fan_.SetSpeed(fan_out_);
    return;
  }
  if (power_gears > 0) {
    fan_power_.Out(1);
    if (fan_out_ != fan_power_table[power_gears]) {
      fan_out_ = fan_power_table[power_gears];
    }
    if (fan_last_out_ != fan_out_) {
      fan_.SetSpeed(fan_out_);
      fan_start_time_ = millis() + FAN_STABLE_TIME_MS;
    }
  } else {
    fan_out_ = 0;
    fan_power_.Out(0);
    fan_.SetSpeed(0);
  }
  fan_last_out_ = fan_out_;

}

void PurifierModule::LightCtrl(PURIFIER_LIGHT_STA_E sta) {
  static PURIFIER_LIGHT_STA_E last_sta = LT_INVALID;
  if ((sta >= LT_INVALID) || (last_sta == sta) || is_debug_) {
    return;
  }
  last_sta = sta;
  RGB_T c[PURIFIER_LIGHT_COUNT] = {0};
  switch (sta) {
    case LT_WAIT_CONNECT:
      c[0] = WHITE_WATERFALL_LIGHT;
      light_.WaterfallLight(c, 100);
      break;
    case LT_LIFETIME_NORMAL:
      light_.BreathLight(BLACK_LIGHT, WHITE_LIGHT, 6000);
      break;
    case LT_LIFETIME_MEDIUM:
      light_.BreathLight(BLACK_LIGHT, WHITE_LIGHT, 6000);
      break;
    case LT_LIFETIME_LOW:
      light_.StaticLight(ORANGE_LIGHT);
      break;
    case LT_NO_FILTER_ERR:
      light_.FlickeringLight(YELLOW_LIGHT, 500);
      break;
    case LT_FAN_SPEED_ERR:
      light_.FlickeringLight(RED_LIGHT, 500);
      break;
    case LT_ELEC_TOO_HIGH_ERR:
      light_.FlickeringLight(RED_LIGHT, 500);
      break;
    case LT_POWER_ERR:
      light_.FlickeringLight(RED_LIGHT, 500);
      break;
    case LT_FORCED_RUN:
      light_.BreathLight(BLACK_LIGHT, BLUE_LIGHT, 3000);
      break;
    case LT_POWER_OFF:
      light_.StaticLight(BLACK_LIGHT);
      break;
    default:
      light_.StaticLight(BLACK_LIGHT);
      break;
  }
}

bool PurifierModule::IsFanStable() {
  return (ELAPSED(millis(), fan_start_time_) && (fan_out_ > 0));
}

void PurifierModule::ReportErrStatus() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PURIFIER);
  if (msgid != INVALID_VALUE) {
    uint8_t err = WorkExceptionCheck();
    uint8_t data[2] = {PURIFIER_REPORT_ERR, err};
    canbus_g.PushSendStandardData(msgid, data, sizeof(data));
  }
}

void PurifierModule::ReportFanSpeed() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PURIFIER);
  if (msgid != INVALID_VALUE) {
    uint8_t data[8];
    uint8_t index = 0;
    uint16_t val = fan_.ReadCurSpeed();
    data[index++] = PURIFIER_REPORT_FAN_STA;
    data[index++] = fan_state_;
    data[index++] = val >> 8;
    data[index++] = val;
    data[index++] = fan_out_;
    data[index++] = fan_gears_;

    canbus_g.PushSendStandardData(msgid, data, index);
  }
}

void PurifierModule::ReportPower() {
  uint16_t addon_power = GetAddonPower();
  uint16_t extent_power = GetExtendPower();
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PURIFIER);
  if (msgid != INVALID_VALUE) {
    uint8_t data[5];
    data[0] = PURIFIER_REPORT_POWER;
    data[1] = addon_power >> 8;
    data[2] = addon_power;
    data[3] = extent_power >> 8;
    data[4] = extent_power;
    canbus_g.PushSendStandardData(msgid, data, sizeof(data));
  }
}

void PurifierModule::ReportFanElec() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PURIFIER);
  if (msgid != INVALID_VALUE) {
    uint8_t data[3];
    uint16_t val = GetFanElectricity();
    data[0] = PURIFIER_REPORT_ELEC;
    data[1] = val >> 8;
    data[2] = val;
    canbus_g.PushSendStandardData(msgid, data, sizeof(data));
  }
}

void PurifierModule::ReportLifetime() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PURIFIER);
  if (msgid != INVALID_VALUE) {
    uint8_t data[2];
    data[0] = PURIFIER_REPORT_LIFETIME;
    data[1] = cur_lifetime_;
    canbus_g.PushSendStandardData(msgid, data, sizeof(data));
  }
}

void PurifierModule::ReportSysStatus() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PURIFIER);
  if (msgid != INVALID_VALUE) {
    uint8_t data[2];
    data[0] = PURIFIER_REPORT_STATUS;
    data[1] = sys_status_;
    canbus_g.PushSendStandardData(msgid, data, sizeof(data));
  }
}

void PurifierModule::LoadLifetime() {
  cur_lifetime_ = registryInstance.cfg_.purifier_lifetime;
  cur_lifetime_ = (cur_lifetime_ > LIFETIME_NORMAL) ? LIFETIME_NORMAL :cur_lifetime_;
}

void PurifierModule::SaveLifetime() {
  registryInstance.cfg_.purifier_lifetime = cur_lifetime_;
  registryInstance.SaveCfg();
}

void PurifierModule::CheckLifetime() {
  static uint32_t last_time = 0;
  static uint32_t elec_sum = 0;
  static uint8_t times = 0;

  if (!IsFanStable()) {
    if (elec_sum > 0) {
      elec_sum = times = last_time = 0;
    }
    return;
  }

  if (PENDING(millis(), last_time))
    return;
  last_time = millis() + LIFETIME_CAPTURE_INTERVAL;
  elec_sum += GetFanElectricity();
  times++;
  if (times == LIFETIME_CUMULATIVE_NUMBER) {
    __IO uint32_t elec = elec_sum / times;
    times = elec_sum = 0;
    uint8_t lifetime = ELEC_TO_LIFETIME(elec);

    if (test_lifetime_ == lifetime) {
      same_lifetime_count_++;
    } else {
      same_lifetime_count_ = 0;
    }
    test_lifetime_ = lifetime;

    if (same_lifetime_count_ >= LIFETIME_STABLE_TIMES) {
      same_lifetime_count_ = 0;
      if (cur_lifetime_ > test_lifetime_ || fan_reopen_) {
        // The fan is re-run to refresh the lifetime state
        fan_reopen_ = false;
        // The lifetime can only get lower and lower as it runs
        cur_lifetime_ = test_lifetime_;
        ReportLifetime();
        SaveLifetime();
      }
    }
  }
}

uint8_t PurifierModule::WorkExceptionCheck() {
  uint32_t temp=0;
  if (filter_switch_.Read() == 1) {
    ERR_ON(ERR_NO_FILTER);
  } else {
    ERR_CLN(ERR_NO_FILTER);
    if (!err_ && IsFanStable()) {
      temp = fan_.ReadCurSpeed();
      if (temp < FAN_MIN_SPEED) {
        ERR_ON(ERR_FAN_SPEED_TOO_LOW);
      } else {
        ERR_CLN(ERR_FAN_SPEED_TOO_LOW);
      }

      temp = GetFanElectricity();
      if (temp >= OVERCURRENT_PROTECTION) {
        ERR_ON(ERR_ELEC_TOO_HIGH);
      } else {
        ERR_CLN(ERR_ELEC_TOO_HIGH);
      }
    }
  }

  temp = GetExtendPower();
  if (IS_POWER_ERR(temp)) {
    ERR_ON(ERR_EXTEND_POWER);
  } else {
    ERR_CLN(ERR_EXTEND_POWER);
    ERR_CLN(ERR_EXTEND_POWER_OFF);
  }
  return err_;
}

void PurifierModule::WorkProcess() {
  if (!WorkExceptionCheck()) {
    LightCtrl(PURIFIER_LIGHT_STA_E(LT_LIFETIME_LOW + cur_lifetime_));
    CheckLifetime();
    FanOut(fan_gears_);
  } else {
    if (IS_ERR(ERR_NO_FILTER)) {
      LightCtrl(LT_NO_FILTER_ERR);
    } else if (IS_ERR(ERR_FAN_SPEED_TOO_LOW)) {
      LightCtrl(LT_FAN_SPEED_ERR);
    }
    else if (IS_ERR(ERR_ELEC_TOO_HIGH)) {
      LightCtrl(LT_ELEC_TOO_HIGH_ERR);
    }
    FanOut(0);
  }
  sys_status_ = STA_WORKING;
}

void PurifierModule::SetFanStatus(uint8_t is_open, uint8_t is_forced) {
  err_ = 0;
  if (is_open) {
    fan_state_ = FAN_STA_WORKING;
    fan_reopen_ = true;
    WorkExceptionCheck();
    if (err_) {
      ReportErrStatus();
    }
  } else {
    fan_state_ = FAN_STA_IDLE;
  }
  is_debug_ = false;
  is_forced_run_ = is_forced;
  registryInstance.cfg_.purifier_forced_run = is_forced_run_;
  registryInstance.SaveCfg();
}

void PurifierModule::SetFanGears(uint8_t gears) {
   gears = gears > FAN_GEARS_3 ? FAN_GEARS_3 : gears;
  if (gears) {
    fan_gears_ = gears;
    registryInstance.cfg_.purifier_fan_gears = fan_gears_;
    registryInstance.SaveCfg();
    fan_start_time_ = millis() + FAN_STABLE_TIME_MS;
  } else {
    fan_state_ = FAN_STA_IDLE;
  }
}

void PurifierModule::FanPowerOut(uint8_t power) {
  fan_out_ = power;
  is_debug_ = true;
}

void PurifierModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_SET_PURIFIER: {
      switch (data[0]) {
        case PURIFIER_SET_FAN_STA:
          SetFanStatus(data[1], data[2]);
          break;
        case PURIFIER_SET_FAN_GEARS:
          SetFanGears(data[1]);
          break;
        case PURIFIER_SET_FAN_POWER:
          FanPowerOut(data[1]);
          break;
        case PURIFIER_SET_LIGHT:
          light_.StaticLight({data[1], data[2], data[3]});
          is_debug_ = true;
          break;
      }
      break;
    }
    case FUNC_REPORT_PURIFIER: {
      switch (data[0]) {
        case PURIFIER_REPORT_LIFETIME:
          ReportLifetime();
          break;
        case PURIFIER_REPORT_ERR:
          ReportErrStatus();
          break;
        case PURIFIER_REPORT_FAN_STA:
          ReportFanSpeed();
          break;
        case PURIFIER_REPORT_ELEC:
          ReportFanElec();
          break;
        case PURIFIER_REPORT_POWER:
          ReportPower();
          break;
        case PURIFIER_REPORT_STATUS:
          ReportSysStatus();
          break;
        case PURIFIER_INFO_ALL:
          ReportLifetime();
          ReportErrStatus();
          ReportFanSpeed();
          ReportFanElec();
          ReportPower();
          ReportSysStatus();
          break;
      }
    }
  }
  ERR_CLN(ERR_EMERGENCY_STOP);
}

void PurifierModule::EmergencyStop() {
  FanOut(0);
  LightCtrl(LT_POWER_OFF);
  fan_state_ = FAN_STA_IDLE;
  ERR_ON(ERR_EMERGENCY_STOP);
  sys_status_ = STA_EMERGENCY_STOP;
}

void PurifierModule::PeripheralLoopCtrl() {
  fan_.SpeedOutCtrl();
  light_.Loop();
  filter_switch_.CheckStatusLoop();
  if (last_err_ != err_) {
    last_err_ = err_;
    ReportErrStatus();
  }
}

void PurifierModule::ExtendPowerErrEvent(uint16_t power) {
  // Since the voltage drops slowly when the power is turned off,
  // a delay is needed to determine if the voltage is abnormal
  static uint32_t err_time = 0;
  if (sys_status_ != STA_POWER_OFF && (!IS_ERR(ERR_EXTEND_POWER))) {
    err_time = millis() + 10000;
  }
  if (PENDING(millis(), err_time)|| IS_POWER_OFF(power)) {
    LightCtrl(LT_POWER_OFF);
    err_ = ERR_EXTEND_POWER_OFF;
    sys_status_ = STA_POWER_OFF;
  } else {
    LightCtrl(LT_POWER_ERR);
    ERR_ON(ERR_EXTEND_POWER);
    sys_status_ = STA_POWER_ERR;
  }
  FanOut(0);
}

void PurifierModule::WaitConnectHandling() {
  LightCtrl(LT_WAIT_CONNECT);
  err_ = 0;
  FanOut(0);
  fan_state_ = FAN_STA_IDLE;
  sys_status_ = STA_WAIT_CONNECT;
}

void PurifierModule::StandbyMode() {
  LightCtrl((PURIFIER_LIGHT_STA_E)(LT_LIFETIME_LOW + cur_lifetime_));
  err_ = 0;
  FanOut(0);
  sys_status_ = STA_IDLE;
}

void PurifierModule::ForcedRun() {
  LightCtrl(LT_FORCED_RUN);
  FanOut(fan_gears_);
  sys_status_ = STA_FORCED_RUN;
}

void PurifierModule::Loop() {
  uint16_t extent_power = GetExtendPower();
  bool is_online = registryInstance.IsConnect();
  if (IS_ERR(ERR_EMERGENCY_STOP)) {
    EmergencyStop();
  } else if (is_forced_run_) {
    ForcedRun();
  } else if (IS_POWER_ERR(extent_power)) {
    ExtendPowerErrEvent(extent_power);
  } else if (is_online) {
    if (fan_state_ == FAN_STA_WORKING) {
      WorkProcess();
    } else {
      StandbyMode();
    }
  } else {
    WaitConnectHandling();
  }

  PeripheralLoopCtrl();
}
