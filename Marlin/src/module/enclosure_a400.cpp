
/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2022 Snapmaker [https://github.com/Snapmaker]
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

#include "enclosure_a400.h"
#include "io.h"
#include <wirish_time.h>
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
#include "src/HAL/hal_adc.h"

#define ENCLOSURE_A400_REPORT_DOOR_1_MASK           (1 << 0)
#define ENCLOSURE_A400_REPORT_DOOR_2_MASK           (1 << 1)
#define ENCLOSURE_A400_REPORT_LIGHT_LIMIT_MASK      (1 << 2)

#define ENCLOSURE_A400_CLOSE_STA                0
#define ENCLOSURE_A400_FAN_PIN                  PA4
#define ENCLOSURE_A400_HALL_1_PIN               PA7
#define ENCLOSURE_A400_HALL_2_PIN               PB0
#define ENCLOSURE_A400_LIGHT_ADC_PIN            PB1
#define ENCLOSURE_A400_OPEN_DOOR_VERIFY         10      // ms 
#define ENCLOSURE_A400_CLOSE_DOOR_VERIFY        1000    // ms 
#define ENCLOSURE_A400_LIGHT_ADC_STA_VERIFY     10     // ms
#define ENCLOSURE_A400_LOOP_REPORT_INTERVAL     500     // ms
#define ENCLOSURE_A400_LIGHT_ADC_LOWER_LIMIT    300  
#define ENCLOSURE_A400_LIGHT_ADC_UPPER_LIMIT    1500 

void EnclosureA400Module::Init() {
  hall_switch_1_.Init(ENCLOSURE_A400_HALL_1_PIN);
  hall_switch_2_.Init(ENCLOSURE_A400_HALL_2_PIN);
  light_.Init();
  fan_.Init(ENCLOSURE_A400_FAN_PIN);
  adc_index_ = HAL_adc_init(ENCLOSURE_A400_LIGHT_ADC_PIN , ADC_TIM_4, 1000);
  loop_next_time_ = millis();
  light_next_time_ = 0;
}

// Process before reuse to keep the effect consistent
void EnclosureA400Module::StartLightEffect() {
  if (!start_flag_) {
    return;
  }
  if (start_flag_ && (light_next_time_ < millis())) {
    light_next_time_ = millis() + 3;
    if (start_flag_ == 1) {
      light_out_ += 1;
      if (light_out_ == 200) {
        start_flag_ = 2;
      }
    } else if (start_flag_ == 2) {
      light_out_ -= 1;
      if (light_out_ == 0) {
        start_flag_ = 0;
      }
    }
    light_.SetRGB(light_out_, light_out_, light_out_);
  }
}

void EnclosureA400Module::EmergencyStop() {
  start_flag_ = 0;
  fan_.ChangePwm(0, 0);
  light_.SetRGB(0, 0, 0);
}

void EnclosureA400Module::ReportEnclosureStatus() {
  uint8_t buf[8];
  uint8_t index = 0;
  uint16_t light_adc = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_ENCLOSURE);
  if (msgid != INVALID_VALUE) {
    light_adc = ADC_Get(adc_index_);
    buf[index++] = door_1_sta_ || door_2_sta_;
    buf[index++] = light_limit_sta_;
    buf[index++] = door_1_sta_;
    buf[index++] = door_2_sta_;
    buf[index++] = (light_adc >> 8) & 0xff;
    buf[index++] = (light_adc >> 0) & 0xff;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void EnclosureA400Module::ReportConfigResult(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  uint16_t msgid = INVALID_VALUE;
  if (!data)
    return;

  msgid = registryInstance.FuncId2MsgId(func_id); 
  if (msgid != INVALID_VALUE) {
    data_len = data_len > 8 ? 8 : data_len;
    canbus_g.PushSendStandardData(msgid, data, data_len);
  }
}

void EnclosureA400Module::EnclosureReportHWVersion() {
  ModuleMacInfo *mac_info = (ModuleMacInfo *)FLASH_MODULE_PARA;
  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_GET_HW_VERSION);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE) {
    buf[index++] = mac_info->hw_version;
    // to have a simple checksum
    buf[1] = ~buf[0];
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void EnclosureA400Module::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  uint8_t buf[8];
  uint8_t index = 0;

  switch (func_id) {
    case FUNC_REPORT_ENCLOSURE:
      ReportEnclosureStatus();
      break;
    case FUNC_SET_ENCLOSURE_LIGHT:
      if (!start_flag_)
        light_.HandModule(func_id, data, data_len);
      buf[index++] = !!start_flag_;   // 0: success, 1: fail
      ReportConfigResult(FUNC_SET_ENCLOSURE_LIGHT, buf, index);
      break;
    case FUNC_SET_FAN_MODULE:
      fan_.ChangePwm(data[1], data[0]);
      buf[index++] = 0;    // 0: success, 1: fail
      ReportConfigResult(FUNC_SET_FAN_MODULE, buf, index);
      break;
    case FUNC_MODULE_GET_HW_VERSION:
      EnclosureReportHWVersion();
      break;
  }
}

void EnclosureA400Module::Loop() {

  uint8_t hall_1_sta = 0;
  uint8_t hall_2_sta = 0;
  uint8_t light_limit_sta = 0;
  uint8_t is_report = 0;
  uint16_t light_adc = 0;
  
  // Take a breath when the light is startint up
  StartLightEffect();
  fan_.Loop();
  hall_switch_1_.CheckStatusLoop();
  hall_switch_2_.CheckStatusLoop();

  // get door detection status
  hall_1_sta = hall_switch_1_.Read();
  hall_2_sta = hall_switch_2_.Read();
  
  light_adc = ADC_Get(adc_index_);
  // after the light bar is initialized, it is detected
  if (light_adc >= ENCLOSURE_A400_LIGHT_ADC_LOWER_LIMIT && \
      light_adc < ENCLOSURE_A400_LIGHT_ADC_UPPER_LIMIT && !start_flag_) {
    light_limit_sta = 0;
  }
  else {
    light_limit_sta = 1;
  }

  // new door 1 state detected
  if (hall_1_sta != door_1_sta_) {
    if (!(is_report_ & ENCLOSURE_A400_REPORT_DOOR_1_MASK)) {
      is_report_ |= ENCLOSURE_A400_REPORT_DOOR_1_MASK;
      door_1_check_time_ = millis();
    }
  }
  else {
    is_report_ &= (~ENCLOSURE_A400_REPORT_DOOR_1_MASK);
  }

  // new door 2 state detected
  if (hall_2_sta != door_2_sta_) {
    if (!(is_report_ & ENCLOSURE_A400_REPORT_DOOR_2_MASK)) {
      is_report_ |= ENCLOSURE_A400_REPORT_DOOR_2_MASK;
      door_2_check_time_ = millis();
    }
  }
  else {
    is_report_ &= (~ENCLOSURE_A400_REPORT_DOOR_2_MASK);
  }

  // light bar limit state change
  if (light_limit_sta != light_limit_sta_) {
    if (!(is_report_ & ENCLOSURE_A400_REPORT_LIGHT_LIMIT_MASK)) {
      is_report_ |= ENCLOSURE_A400_REPORT_LIGHT_LIMIT_MASK;
      light_limit_time_ = millis();
    }
  }
  else{
    is_report_ &= (~ENCLOSURE_A400_REPORT_LIGHT_LIMIT_MASK);
  }

  if (is_report_) {
    uint32_t time_out = 0;
    if (is_report_ & ENCLOSURE_A400_REPORT_DOOR_1_MASK) {
      time_out = ENCLOSURE_A400_CLOSE_DOOR_VERIFY;
      if (door_1_sta_ == ENCLOSURE_A400_CLOSE_STA){
        time_out = ENCLOSURE_A400_OPEN_DOOR_VERIFY;
      }
      if (((int32_t)(millis() - (door_1_check_time_ + time_out))) > 0) {
        is_report_ &= (~ENCLOSURE_A400_REPORT_DOOR_1_MASK);
        door_1_sta_ = hall_1_sta;
        is_report = true;
      }
    }

    if (is_report_ & ENCLOSURE_A400_REPORT_DOOR_2_MASK) {
      time_out = ENCLOSURE_A400_CLOSE_DOOR_VERIFY;
      if (door_2_sta_ == ENCLOSURE_A400_CLOSE_STA){
        time_out = ENCLOSURE_A400_OPEN_DOOR_VERIFY;
      }
      if (((int32_t)(millis() - (door_2_check_time_ + time_out))) > 0) {
        is_report_ &= (~ENCLOSURE_A400_REPORT_DOOR_2_MASK);
        door_2_sta_ = hall_2_sta;
        is_report = true;
      }
    }
    
    if (is_report_ & ENCLOSURE_A400_REPORT_LIGHT_LIMIT_MASK) {
      if (((int32_t)(millis() - (light_limit_time_ + ENCLOSURE_A400_LIGHT_ADC_STA_VERIFY))) > 0) { 
        is_report_ &= (~ENCLOSURE_A400_REPORT_LIGHT_LIMIT_MASK);
        light_limit_sta_ = light_limit_sta;
        is_report = true;
      }
    }
  }

  if (is_report || ((int)(millis() - (loop_next_time_ + ENCLOSURE_A400_LOOP_REPORT_INTERVAL))) > 0) {
    ReportEnclosureStatus();
    loop_next_time_ = millis();
  }
}

