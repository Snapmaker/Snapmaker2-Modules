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
#include <wirish_time.h>
#include "../registry/route.h"
#include <src/HAL/hal_tim.h>
#include <math.h>
#include "drybox.h"

void DryBox::Init() {
  fan_.Init(FAN_PIN);
  heater_.InitCapture(THERMISTOR_TEMP_PIN, ADC_TIM_4);
  heater_.SetThermistorType(THERMISTOR_NTC3590);
  heater_.InitOutCtrl(PWM_TIM2, PWM_CH4, HEATING_BLOCK_PIN, 25500);  // 100Hz
  chamber_.InitOutCtrl(PWM_TIM2, PWM_CH4, HEATING_BLOCK_PIN, 25500); // 100Hz
  temp_humidity_sensor_.Init(I2C_SDA, I2C_SCL);
  power_source_detect_.Init(POWER_SOURCE_DETECT_PIN);
  heater_power_monitor_.Init(HEATER_POWER_MONITOR_PIN);
  cover_detect_.Init(COVER_DET_PIN, INPUT_FLOATING);
  power_select_.Init(POWER_SELECT_PIN, 0, OUTPUT);
  InitLight();
  ResetGXHT3X();
  StartCyclicConvert(0x27, 0x37);
}

void DryBox::InitLight() {
  light_.Init(DRYBOX_LIGHT_PIN, DRYBOX_LIGHT_COUNT, DRYBOX_LIGHT_EXTI);
  RGB_T color = {0, 0, 0};
  light_.StaticLight(color);
}

void DryBox::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  float val = 0.0;
  switch ((uint32_t)func_id) {
    case FUNC_SET_FAN:
      fan_.ChangePwm(data[1], data[0]);
      break;
    case FUNC_SET_TEMPEARTURE:
      heater_target_temp_  = contextInstance.data_[0] << 8 | contextInstance.data_[1];
      chamber_target_temp_ = contextInstance.data_[2] << 8 | contextInstance.data_[3];
      SetTargetTemp(heater_target_temp_, chamber_target_temp_);
      break;
    case FUNC_SET_HEAT_TIME:
      SetTargetHeatingTime((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
      break;
    case FUNC_REPORT_TEMP_HUMIDITY:
      ReportTempHumidity();
      break;
    case FUNC_REPORT_TEMP_PID:
        chamber_.ReportPid();
      break;
    case FUNC_SET_PID:
      val = (float)(((data[1]) << 24) | ((data[2]) << 16) | ((data[3]) << 8 | (data[4]))) / 1000;
      chamber_.SetPID(data[0], val);
      break;
    case FUNC_MODULE_START:
      StartHeating(data[0]);
      break;
    case FUNC_SET_MAINCTRL_TYPE:
      mainctrl_type_ = data[0] << 8 | data[1];
      break;
    case FUNC_REPORT_HEATER_POWER_STATE:
      ReportHeaterPowerState();
      break;
    case FUNC_REPORT_COVER_STATE:
      ReportCoverState();
      break;
    case FUNC_REPORT_DRYBOX_STATE:
      ReportDryBoxState();
      break;
    default:
      break;
  }
}

void DryBox::ReportTempHumidity() {
  uint8_t buf[8];
  uint8_t index = 0;
  int16_t heater_temp = heater_temp_;
  int16_t chamber_temp = chamber_temp_;
  int16_t chamber_humidity = chamber_humidity_;

  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMP_HUMIDITY);
  if (msgid != INVALID_VALUE) {
      buf[index++] = heater_temp >> 8;
      buf[index++] = heater_temp & 0xff;
      buf[index++] = chamber_temp >> 8;
      buf[index++] = chamber_temp & 0xff;
      buf[index++] = chamber_humidity >> 8;
      buf[index++] = chamber_humidity & 0xff;
      canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DryBox::ReportHeaterPowerState() {
  uint8_t buf[8];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_HEATER_POWER_STATE);
  if (msgid != INVALID_VALUE) {
      buf[index++] = heater_power_state_;
      canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DryBox::ReportCoverState() {
  uint8_t buf[8];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_COVER_STATE);
  if (msgid != INVALID_VALUE) {
      buf[index++] = cover_state_;
      canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DryBox::ReportDryBoxState() {
  uint8_t buf[8];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_DRYBOX_STATE);
  if (msgid != INVALID_VALUE) {
      buf[index++] = (uint8_t)drybox_state_;
      canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

bool DryBox::ResetGXHT3X() {
    bool ret = true;
    uint8_t wirte_header = (ADDR_GXHT3X<<1) & (~0x01);
    temp_humidity_sensor_.IICStart();
    temp_humidity_sensor_.IICSendByte(wirte_header);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        ret = false;
        goto EXIT;
    }
    temp_humidity_sensor_.IICSendByte(0x30);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        ret = false;
        goto EXIT;
    }
    temp_humidity_sensor_.IICSendByte(0xA2);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        ret = false;
        goto EXIT;
    }

EXIT:
    uint32_t time = millis();
    while(time + 1000 > millis());
    return ret;
}

bool DryBox::StartSingleConvert(uint8_t clock_stretching, uint8_t accuracy) {
    bool ret = true;
    uint8_t wirte_header = (ADDR_GXHT3X<<1) & (~0x01);
    temp_humidity_sensor_.IICStart();
    temp_humidity_sensor_.IICSendByte(wirte_header);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        ret = false;
        goto EXIT;
    }
    temp_humidity_sensor_.IICSendByte(clock_stretching);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        ret = false;
        goto EXIT;
    }
    temp_humidity_sensor_.IICSendByte(accuracy);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        ret = false;
        goto EXIT;
    }

EXIT:
    return ret;
}

void DryBox::StartCyclicConvert(uint8_t freq, uint8_t accuracy) {
    uint8_t wirte_header = (ADDR_GXHT3X<<1) & (~0x01);
    temp_humidity_sensor_.IICStart();
    temp_humidity_sensor_.IICSendByte(wirte_header);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return;
    }
    temp_humidity_sensor_.IICSendByte(freq);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return;
    }
    temp_humidity_sensor_.IICSendByte(accuracy);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return;
    }

    temp_humidity_time_elaspe_ = millis();
    is_cyclic_convert_start_ = true;
}

void DryBox::StopCyclicConvert() {

}

bool DryBox::GetTempHumidity() {
    uint8_t read_header;
    uint8_t temp_h, temp_l;
    uint8_t humidity_h, humidity_l;
    read_header = (ADDR_GXHT3X<<1) | 0x01;
    temp_humidity_sensor_.IICStart();
    temp_humidity_sensor_.IICSendByte(read_header);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return false;
    }
    temp_h = temp_humidity_sensor_.IICReadByte(true);
    temp_l = temp_humidity_sensor_.IICReadByte(true);
    temp_humidity_sensor_.IICReadByte(true);                  // read crc
    humidity_h = temp_humidity_sensor_.IICReadByte(true);
    humidity_l = temp_humidity_sensor_.IICReadByte(true);
    temp_humidity_sensor_.IICReadByte(false);                 // read crc
    chamber_temp_ = ((temp_h<<8)|temp_l)*175/65535 - 45;
    chamber_humidity_ = ((humidity_h<<8)|humidity_l)*100/65535;
    return true;
}

bool DryBox::GetTempHumidityCyclic() {
    uint8_t read_header, write_header;
    uint8_t temp_h, temp_l;
    uint8_t humidity_h, humidity_l;

    write_header = (ADDR_GXHT3X<<1) & (~0x01);
    temp_humidity_sensor_.IICStart();
    temp_humidity_sensor_.IICSendByte(write_header);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return false;
    }
    temp_humidity_sensor_.IICSendByte(0xE0);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return false;
    }
    temp_humidity_sensor_.IICSendByte(0x00);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return false;
    }

    read_header = (ADDR_GXHT3X<<1) | 0x01;
    temp_humidity_sensor_.IICStart();
    temp_humidity_sensor_.IICSendByte(read_header);
    if (temp_humidity_sensor_.IICWaitAck() == false) {
        return false;
    }
    temp_h = temp_humidity_sensor_.IICReadByte(true);
    temp_l = temp_humidity_sensor_.IICReadByte(true);
    temp_humidity_sensor_.IICReadByte(true);                  // read crc
    humidity_h = temp_humidity_sensor_.IICReadByte(true);
    humidity_l = temp_humidity_sensor_.IICReadByte(true);
    temp_humidity_sensor_.IICReadByte(false);                 // read crc
    chamber_temp_ = ((temp_h<<8)|temp_l)*175/65535 - 45;
    chamber_humidity_ = ((humidity_h<<8)|humidity_l)*100/65535;
    return true;
}

void DryBox::ReadTempHumidityCyclic() {
    if (is_cyclic_convert_start_ == false) {
        return;
    }

    if (temp_humidity_time_elaspe_ + 110 < millis()) {
        temp_humidity_time_elaspe_ = millis();
    } else {
        return;
    }

    if (GetTempHumidityCyclic() == true) {
        chamber_temp_ready_ = true;
        // uint8_t buf[8] = {0};
        // uint8_t index = 0;
        // uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMP_HUMIDITY);
        // if (msgid != INVALID_VALUE) {
        //     buf[index++] = ((uint8_t *)(&chamber_temp_))[0];
        //     buf[index++] = ((uint8_t *)(&chamber_temp_))[1];
        //     buf[index++] = ((uint8_t *)(&chamber_temp_))[2];
        //     buf[index++] = ((uint8_t *)(&chamber_temp_))[3];
        //     buf[index++] = ((uint8_t *)(&chamber_humidity_))[0];
        //     buf[index++] = ((uint8_t *)(&chamber_humidity_))[1];
        //     buf[index++] = ((uint8_t *)(&chamber_humidity_))[2];
        //     buf[index++] = ((uint8_t *)(&chamber_humidity_))[3];
        //     canbus_g.PushSendStandardData(msgid, buf, index);
        // }
    }
}

void DryBox::TempAndHumidityProcess() {
  // get the newest temperature
  ReadTempHumidityCyclic();
  heater_.GetTemperature(heater_temp_);
}

void DryBox::clear_heating_time() {
  target_heating_time_ = 0;
  accumulate_dry_time_ = 0;
  timing_heating_time_ = 0;
  remain_heating_time_ = 0;
}

void DryBox::StartHeating(uint8_t state) {
  if (state == 1) {
    if (target_heating_time_ > 0) {
      RequestSystemEnterState(DRYBOX_STATE_HEATING_TIMING);
    } else {
      RequestSystemEnterState(DRYBOX_STATE_HEATING_FREE);
    }
  } else if (state == 0) {
    RequestSystemEnterState(DRYBOX_STATE_IDLE);
  }
}

void DryBox::SetTargetTemp(int16_t heater_temp, int16_t chamber_temp) {
  heater_target_temp_ = heater_temp;
  chamber_target_temp_ = chamber_temp;
  heater_.ChangeTarget(heater_temp);
  chamber_.ChangeTarget(chamber_temp);
}

void DryBox::SetTargetHeatingTime(uint32_t time) {
  target_heating_time_ = time;
  remain_heating_time_ = target_heating_time_ - timing_heating_time_;

  if (drybox_state_ == DRYBOX_STATE_HEATING_FREE) {
    drybox_state_ = DRYBOX_STATE_HEATING_TIMING;
  } else if (drybox_state_ == DRYBOX_STATE_HEATING_TIMING) {
    if (target_heating_time_ == 0) {
      timing_heating_time_ = 0;
      remain_heating_time_ = 0;
      drybox_state_ = DRYBOX_STATE_HEATING_FREE;
    }
  }
}

void DryBox::CoverDetect() {
  bool cover_state_changed = cover_detect_.CheckStatusLoop();
  if (cover_state_ == 0xff) {
    cover_state_changed = true;
  }

  cover_state_ = cover_detect_.Read();

  if (cover_state_changed) {
    ReportCoverState();

    if (cover_state_ == 0) {                 // close
      CLR_BIT(drybox_fault_state_, DRYBOX_FAULT_COVER_OPEN);
      RequestSystemEnterState(drybox_prev_state_);
    } else if (cover_state_ == 1) {          // open
      SET_BIT(drybox_fault_state_, DRYBOX_FAULT_COVER_OPEN);
      RequestSystemEnterState(DRYBOX_STATE_FAULT);
    }
  }
}

void DryBox::HeaterTempMonitor() {
  if (heater_temp_ > HEATER_PROTECT_TEMP) {
    if (!GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_OVER_TEMP)) {
      SET_BIT(drybox_fault_state_, DRYBOX_FAULT_OVER_TEMP);
      RequestSystemEnterState(DRYBOX_STATE_FAULT);
    }
  } else if (heater_temp_ < HEATER_RECOVER_TEMP) {
    if (GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_OVER_TEMP)) {
      CLR_BIT(drybox_fault_state_, DRYBOX_FAULT_OVER_TEMP);
      RequestSystemEnterState(DRYBOX_STATE_IDLE);
    }
  }
}

void DryBox::TempCtrl() {
  switch (temp_ctrl_stage_) {
    case TEMP_CTRL_PRE_HEAT:
      heater_.SetPwmDutyLimitAndThreshold(204, 0);
      heater_.PrfetchTempMaintain();
      if ((heater_temp_ >= heater_target_temp_) && (heater_target_temp_ > 70)) {
        temp_ctrl_stage_ = TEMP_CTRL_CHAMBER_HEAT;
      }
      break;
    case TEMP_CTRL_CHAMBER_HEAT:
      heater_.SetPwmDutyLimitAndThreshold(255, 0);
      // if heater temp lower than target, switch to heater for heating
      if (heater_temp_ < heater_target_temp_) {
        temp_ctrl_stage_ = TEMP_CTRL_PRE_HEAT;
      } else if (chamber_temp_ready_) {
        chamber_temp_ready_ = false;
        chamber_.TempMaintain(chamber_temp_);
      }
      break;
    default:
      break;
  }
}

void DryBox::HeatingTimeProcess() {
  if (dry_time_process_time_elaspe_ + 1000 > millis()) {
    return;
  } else {
    dry_time_process_time_elaspe_ = millis();
  }

  if (drybox_state_ == DRYBOX_STATE_HEATING_FREE) {
    accumulate_dry_time_++;
  } else if (drybox_state_ == DRYBOX_STATE_HEATING_TIMING) {
    accumulate_dry_time_++;
    timing_heating_time_++;
    remain_heating_time_ = target_heating_time_ - timing_heating_time_;
    if (remain_heating_time_ <= 0) {
      RequestSystemEnterState(DRYBOX_STATE_IDLE);
    }
  }
}

void DryBox::HeatingProcess() {
  TempCtrl();
  HeatingTimeProcess();
}

void DryBox::ExternalPowerDetection() {
  if (mainctrl_type_ == BOARD_SM_NULL) {
    return;
  }

  if (power_source_state_ != 0xff) {
    return;
  }

  power_select_.Out(0);
  delay(1000);
  power_source_state_ = power_source_detect_.Read();
  if (power_source_state_ == 1) {
    CLR_BIT(drybox_fault_state_, DRYBOX_FAULT_NO_EXTERNAL_POWER);
  } else {
    if (mainctrl_type_ == BOARD_SM_CONTROLLER2019_V1) {
      SET_BIT(drybox_fault_state_, DRYBOX_FAULT_NO_EXTERNAL_POWER);
      RequestSystemEnterState(DRYBOX_STATE_FAULT);
    }
  }
  power_select_.Out(1);
}

void DryBox::HeaterPowerMonitor() {
  bool heater_power_changed = heater_power_monitor_.CheckStatusLoop();
  if (heater_power_state_ == 0xff) {
    heater_power_changed = true;
  }
  heater_power_state_ = heater_power_monitor_.Read();

  if (heater_power_changed) {
    ReportHeaterPowerState();

    if (heater_power_state_ == 1) {   // normal
      // set to previous state
      CLR_BIT(drybox_fault_state_, DRYBOX_FAULT_HEATER_POWER);
      RequestSystemEnterState(drybox_prev_state_);
    } else if (heater_power_state_ == 0) {    // error
      // set to fault state
      SET_BIT(drybox_fault_state_, DRYBOX_FAULT_HEATER_POWER);
      RequestSystemEnterState(DRYBOX_STATE_FAULT);
    }
  }
}

void DryBox::ReportTimeInfo() {
  uint8_t buf[8];
  uint8_t index = 0;

  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_HEATING_TIME_INFO);
  if (msgid != INVALID_VALUE) {
      index = 0;
      buf[index++] = 0;
      buf[index++] = (target_heating_time_ >> 24) & 0xff;
      buf[index++] = (target_heating_time_ >> 16) & 0xff;
      buf[index++] = (target_heating_time_ >> 8) & 0xff;
      buf[index++] = target_heating_time_ & 0xff;
      canbus_g.PushSendStandardData(msgid, buf, index);

      index = 0;
      buf[index++] = 1;
      buf[index++] = (accumulate_dry_time_ >> 24) & 0xff;
      buf[index++] = (accumulate_dry_time_ >> 16) & 0xff;
      buf[index++] = (accumulate_dry_time_ >> 8) & 0xff;
      buf[index++] = accumulate_dry_time_ & 0xff;
      canbus_g.PushSendStandardData(msgid, buf, index);

      index = 0;
      buf[index++] = 2;
      buf[index++] = (remain_heating_time_ >> 24) & 0xff;
      buf[index++] = (remain_heating_time_ >> 16) & 0xff;
      buf[index++] = (remain_heating_time_ >> 8) & 0xff;
      buf[index++] = remain_heating_time_ & 0xff;
      canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DryBox::ReportInfoProcess() {
  if ((info_report_time_ + 2000) < millis()) {
    info_report_time_ = millis();

    ReportTempHumidity();
    ReportTimeInfo();
  }
}

void DryBox::MainCtrlOnlineMonitor() {
  bool is_online = registryInstance.IsConnect();

  if (GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_MAINCTRL_DISCONNECT)) {
    if (is_online) {
      CLR_BIT(drybox_fault_state_, DRYBOX_FAULT_MAINCTRL_DISCONNECT);
      RequestSystemEnterState(drybox_prev_state_);
    }
  } else {
    if (!is_online) {
      SET_BIT(drybox_fault_state_, DRYBOX_FAULT_MAINCTRL_DISCONNECT);
      RequestSystemEnterState(DRYBOX_STATE_FAULT);
    }
  }
}

bool DryBox::RequestSystemEnterState(drybox_state_e state) {
  bool ret = true;

  // enter idle mode
  if (state == DRYBOX_STATE_IDLE) {
    if (drybox_fault_state_ == 0) {
      heater_.ShutDown();
      chamber_.ShutDown();
      fan_.ChangePwm(0, 0);
      clear_heating_time();
      light_.StaticLight(WHITE_LIGHT);
      drybox_prev_state_ = drybox_state_;
      drybox_state_ = DRYBOX_STATE_IDLE;
      goto EXIT;
    } else {
      state = DRYBOX_STATE_FAULT;
      ret = false;
    }
  }

  // enter free heating mode
  if (state == DRYBOX_STATE_HEATING_FREE) {
    if (drybox_fault_state_ == 0) {
      fan_.ChangePwm(255, 0);
      drybox_prev_state_ = drybox_state_;
      drybox_state_ = DRYBOX_STATE_HEATING_FREE;
      light_.BreathLight(BLACK_LIGHT, WHITE_LIGHT, 6000);
      goto EXIT;
    } else {
      state = DRYBOX_STATE_FAULT;
      ret = false;
    }
  }

  // enter timing heating mode
  if (state == DRYBOX_STATE_HEATING_TIMING) {
    if (drybox_fault_state_ == 0) {
      fan_.ChangePwm(255, 0);
      drybox_prev_state_ = drybox_state_;
      drybox_state_ = DRYBOX_STATE_HEATING_TIMING;
      light_.BreathLight(BLACK_LIGHT, WHITE_LIGHT, 6000);
      goto EXIT;
    } else {
      state = DRYBOX_STATE_FAULT;
      ret = false;
    }
  }

  // enter fault mode
  if (state == DRYBOX_STATE_FAULT) {
    heater_.ShutDown();
    chamber_.ShutDown();
    fan_.ChangePwm(0, 0);
    if (drybox_state_ != DRYBOX_STATE_FAULT) {
      drybox_prev_state_ = drybox_state_;
    }
    drybox_state_ = DRYBOX_STATE_FAULT;

    if (GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_HEATER_POWER)) {
      light_color_ = BLACK_LIGHT;
      if (!GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_MAINCTRL_DISCONNECT)) {
        light_color_ = YELLOW_LIGHT;
      }
      light_.StaticLight(light_color_);

      if (GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_COVER_OPEN)) {
        light_.FlickeringLight(light_color_, 500);
      }
    } else {
      if ((drybox_prev_state_ == DRYBOX_STATE_HEATING_FREE) || (drybox_prev_state_ == DRYBOX_STATE_HEATING_TIMING)) {
        light_color_ = RED_LIGHT;
      } else {
        light_color_ = WHITE_LIGHT;
      }

      if (GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_MAINCTRL_DISCONNECT)) {
        light_color_ = YELLOW_LIGHT;
        light_.StaticLight(light_color_);
      }

      if (GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_OVER_TEMP)) {
        clear_heating_time();
        light_color_ = RED_LIGHT;
        light_.StaticLight(RED_LIGHT);
      }

      if (GET_STATUS(drybox_fault_state_, DRYBOX_FAULT_COVER_OPEN)) {
        light_.FlickeringLight(light_color_, 500);
      }
    }

    goto EXIT;
  }

EXIT:
  ReportDryBoxState();
  return ret;
}

void DryBox::EmergencyStop() {
  heater_.ShutDown();
  chamber_.ShutDown();
  drybox_state_ = DRYBOX_STATE_IDLE;
}

void DryBox::Loop() {
  ExternalPowerDetection();

  // routine services
  fan_.Loop();
  light_.Loop();
  ReportInfoProcess();
  TempAndHumidityProcess();

  // exception check
  HeaterTempMonitor();
  CoverDetect();
  HeaterPowerMonitor();
  MainCtrlOnlineMonitor();

  switch (drybox_state_) {
    case DRYBOX_STATE_IDLE:
      {
        // do nothing
      }
      break;

    case DRYBOX_STATE_HEATING_FREE:
    case DRYBOX_STATE_HEATING_TIMING:
      {
        HeatingProcess();
      }
      break;

    case DRYBOX_STATE_FAULT:
      {
        // do nothing
      }
      break;

    default:
      break;
  }
}
