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

#include <src/device/fan.h>
#include <board/board.h>
#include "print_head.h"
#include "src/registry/context.h"
#include "src/core/can_bus.h"
#include <wirish_time.h>
#include "io.h"
#include "dual_extruder_print_head.h"

#define DUAL_EXTRUDER_FAN_0_PIN PA6
#define DUAL_EXTRUDER_FAN_1_PIN PA5
#define DUAL_EXTRUDER_FAN_2_PIN PB0
#define DUAL_EXTRUDER_FAN_3_PIN PA7

#define SWITCH_0_PIN PA8
#define SWITCH_1_PIN PA1

#define LEVELING_0 PA10
#define LEVELING_1 PA2

#define E_DET_PIN PB4
#define TEMP_1_PIN PA3
#define TEMP_0_PIN PB1

#define HEATER_1_TIM PWM_TIM2, PWM_CH1
#define HEATER_1_PIN PA0
#define HEATER_0_TIM PWM_TIM1, PWM_CH2
#define HEATER_0_PIN PA9

#define PROXIMITY_SWITCH_PROBE_PIN  PB6
#define MOTOR_CS_PIN PB5

//  Periph initialization according schema
void DualExtruderPrintHead::PeriphInit() {

}
void DualExtruderPrintHead::Init() {
  // Fan
  fan_0_.Init(DUAL_EXTRUDER_FAN_0_PIN);
  fan_1_.Init(DUAL_EXTRUDER_FAN_1_PIN);
  fan_2_.Init(DUAL_EXTRUDER_FAN_2_PIN);
  fan_3_.Init(DUAL_EXTRUDER_FAN_3_PIN);

  switch_probe_0_.Init(LEVELING_0);
  switch_probe_1_.Init(LEVELING_1);
  switch_probe_.Init(PROXIMITY_SWITCH_PROBE_PIN);
  switch_cut_0_.Init(SWITCH_0_PIN);
  switch_cut_1_.Init(SWITCH_1_PIN);
  motor_cs_.Init(MOTOR_CS_PIN, 1, OUTPUT);
  temperature_0_.InitCapture(TEMP_0_PIN, ADC_TIM_4);
  temperature_0_.InitOutCtrl(HEATER_0_TIM, HEATER_0_PIN);
  temperature_1_.InitCapture(TEMP_1_PIN, ADC_TIM_4);
  temperature_1_.InitOutCtrl(HEATER_1_TIM, HEATER_1_PIN);
}

void ReportSwitch(uint8_t funcid, uint8_t s1, uint8_t s2) {
  uint8_t buf[8];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(funcid);
  if (msgid != INVALID_VALUE) {
    buf[index++] = s1;
    buf[index++] = s2;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void ReportProbe(uint8_t funcid, uint8_t s1, uint8_t s2, uint8_t s3) {
  uint8_t buf[8];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(funcid);
  if (msgid != INVALID_VALUE) {
    buf[index++] = s1;
    buf[index++] = s2;
    buf[index++] = s3;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void ReportMotorCsStatus() {
  uint8_t buf[8];
  uint8_t index = 0;
  buf[index++] = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_SWITCH_EXTRUDER);
  if (msgid != INVALID_VALUE) {
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DualExtruderPrintHead::ReportTemprature() {
  int16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMPEARTURE);
  if (msgid != INVALID_VALUE) {
    int16_t temp = temperature_0_.GetCurTemprature();
    int16_t target = temperature_0_.GetTargetTemprature();
    uint8_t u8DataBuf[8], u8Index = 0;
    u8DataBuf[u8Index++] = temp >> 8;
    u8DataBuf[u8Index++] = temp;
    u8DataBuf[u8Index++] = target >> 8;
    u8DataBuf[u8Index++] = target;

    temp = temperature_1_.GetCurTemprature();
    target = temperature_1_.GetTargetTemprature();
    u8DataBuf[u8Index++] = temp >> 8;
    u8DataBuf[u8Index++] = temp;
    u8DataBuf[u8Index++] = target >> 8;
    u8DataBuf[u8Index++] = target;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

void DualExtruderPrintHead::StartNozzleSwitching(uint8_t *data) {
  uint8_t nozzle_index = data[0];

  target_nozzle = nozzle_index;
  motor_cs_.Out(!target_nozzle);
  active_nozzle = target_nozzle;
  ReportMotorCsStatus();
}

void DualExtruderPrintHead::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  // float val = 0.0;
  switch ((uint32_t)func_id) {
    case FUNC_REPORT_CUT:
      ReportSwitch(FUNC_REPORT_CUT, switch_cut_0_.Read(), switch_cut_1_.Read());
      break;
    case FUNC_REPORT_PROBE:
      ReportProbe(FUNC_REPORT_PROBE, switch_probe_.Read(), switch_probe_0_.Read(), switch_probe_1_.Read());
      break;
    case FUNC_SET_FAN:
      fan_0_.ChangePwm(data[1], data[0]);
      fan_3_.ChangePwm(data[3], data[2]);
      break;
    case FUNC_SET_FAN2:
      fan_1_.ChangePwm(data[1], data[0]);
      fan_2_.ChangePwm(data[3], data[2]);
      break;
    case FUNC_SET_TEMPEARTURE:
      temperature_0_.ChangeTarget((contextInstance.data_[0] << 8) | contextInstance.data_[1]);
      temperature_1_.ChangeTarget((contextInstance.data_[2] << 8) | contextInstance.data_[3]);
      break;
    case FUNC_REPORT_TEMPEARTURE:
      ReportTemprature();
      break;
    case FUNC_REPORT_TEMP_PID:
      // temperature_.ReportPid();
      break;
    case FUNC_SET_PID:
      // val = (float)(((data[1]) << 24) | ((data[2]) << 16) | ((data[3]) << 8 | (data[4]))) / 1000;
      // temperature_.SetPID(data[0], val);
      break;
    case FUNC_SWITCH_EXTRUDER:
      StartNozzleSwitching(data);
      break;
  }
}

void DualExtruderPrintHead::EmergencyStop() {
  temperature_0_.ChangeTarget(0);
  temperature_1_.ChangeTarget(0);
  fan_0_.ChangePwm(0, 0);
  fan_1_.ChangePwm(0, 0);
  fan_2_.ChangePwm(0, 0);
  fan_3_.ChangePwm(0, 0);
}

void DualExtruderPrintHead::Loop() {
  if (temperature_0_.TempertuerStatus()) {
    temperature_0_.TemperatureOut();
    temperature_1_.TemperatureOut();
  }
  if ((temp_report_time_ + 500) < millis()) {
    temp_report_time_ = millis();
    ReportTemprature();
  }

  if (switch_cut_0_.CheckStatusLoop() || switch_cut_1_.CheckStatusLoop()) {
    if (is_report_cut_) {
      is_report_cut_ = false;
    } else {
      is_report_cut_ = true;
    }
    cut_report_time_ = millis();
  }
  if (is_report_cut_ && ((cut_report_time_ + 500) > millis())) {
    is_report_cut_ = false;
    ReportSwitch(FUNC_REPORT_CUT, switch_cut_0_.Read(), switch_cut_1_.Read());
  }

  bool probe2_status = switch_probe_.CheckStatusLoop();
  bool probe0_status = switch_probe_0_.CheckStatusLoop();
  bool probe1_status = switch_probe_1_.CheckStatusLoop();
  if (probe0_status || probe1_status || probe2_status) {
    probe_switch_status = switch_probe_.Read();
    probe0_status = switch_probe_0_.Read();
    probe1_status = switch_probe_1_.Read();
    ReportProbe(FUNC_REPORT_PROBE, probe_switch_status, probe0_status, probe1_status);
  }

  fan_0_.Loop();
  fan_1_.Loop();
  fan_2_.Loop();
  fan_3_.Loop();
}
