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

#include <stdint.h>
#include <src/core/can_bus.h>
#include <wirish.h>
#include "temperature.h"
#include "src/core/thermistor_table.h"
#include "src/HAL/hal_adc.h"
#include "src/HAL/hal_pwm.h"
#include "src/HAL/hal_flash.h"
#include "src/configuration.h"
#include "src/registry/registry.h"
#define TEMP_BUF_SIZE OVERSAMPLENR
static uint16_t u16TempBuf_g[TEMP_BUF_SIZE];
static uint32_t temp_cusum_g = 0;
Temperature * p_temperature = NULL;

static void TemperatureCaptureDeal() {
    uint32_t temp = 0, i = 0;

    for (i = 0; i < TEMP_BUF_SIZE; i++) {
        temp += u16TempBuf_g[i];
    }
    if (p_temperature) {
      temp_cusum_g = temp;
      p_temperature->detect_ready_ = true;
    }
}

void Temperature::InitPID() {
  AppParmInfo *parm = &registryInstance.cfg_;
  if (parm->parm_mark[0] == 0xaa && parm->parm_mark[1] == 0x55) {
    this->pid_.Init(parm->temp_P, parm->temp_I, parm->temp_D);
  } else {
    this->pid_.Init(TEMP_DEFAULT_KP, TEMP_DEFAULT_KI, TEMP_DEFAULT_KD);
  }
}

void Temperature::SavePID() {
  AppParmInfo * parm = &registryInstance.cfg_;
  if ((parm->temp_P != this->pid_.k_p_) ||
      (parm->temp_I != this->pid_.k_i_)||
      (parm->temp_D != this->pid_.k_d_)) {
      parm->temp_P = this->pid_.k_p_;
      parm->temp_I = this->pid_.k_i_;
      parm->temp_D = this->pid_.k_d_;
      registryInstance.SaveCfg();
  }
}

void Temperature::InitOutCtrl(uint8_t tim_num, uint8_t tim_chn, uint8_t tim_pin) {
  this->InitPID();
  this->pwm_tim_chn_ = tim_chn;
  this->pwm_tim_num_ = tim_num;
  pinMode(tim_pin, PWM);
  HAL_pwn_config(tim_num, tim_chn, 1000000, 255, 0, 0);
}

void Temperature::ReportTemprature() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMPEARTURE);
  if (msgid != INVALID_VALUE) {
    int16_t temp = (int16_t)(this->detect_celsius_ * 10);
    int16_t target = (int16_t)this->pid_.getTarget();

    uint8_t u8DataBuf[8], u8Index = 0;
    u8DataBuf[u8Index++] = temp >> 8;
    u8DataBuf[u8Index++] = temp;
    u8DataBuf[u8Index++] = target >> 8;
    u8DataBuf[u8Index++] = target;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

void Temperature::ReportPid() {
    float pid[3];
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMP_PID);
    if (msgid == INVALID_VALUE) 
      return ;
    uint8_t u8DataBuf[8], i, j,u8Index = 0;
    pid[0] = this->pid_.k_p_;
    pid[1] = this->pid_.k_i_;
    pid[2] = this->pid_.k_d_;

    for (i = 0; i < 3; i++) {
        u8DataBuf[0] = i;
        for (j = 0, u8Index = 1; j < 4; j ++) {
            u8DataBuf[u8Index++] = ((uint32_t)(pid[i] * 1000)) >> (8 * (3 - j));
        }
        canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
    }
}

void Temperature::Maintain() {
  if (this->detect_ready_) {
    detect_ready_ = false;
    this->detect_celsius_ = TempTableCalcCurTemp(temp_cusum_g);
    uint32_t pwmOutput = pid_.output(detect_celsius_);
    HAL_pwm_set_pulse(this->pwm_tim_num_, this->pwm_tim_chn_, pwmOutput);
  }
}

void Temperature::ChangeTarget(uint32_t target) {
  pid_.target(target);
}
void Temperature::SetPID(uint8_t pid_index, float val) {
  switch (pid_index) {
    case SET_P_INDEX :
        this->pid_.k_p(val);
        this->pid_set_flag_ |= 0x1;
        break;
    case SET_I_INDEX :
        this->pid_.k_i(val);
        this->pid_set_flag_ |= 0x2;
        break;
    case SET_D_INDEX :
        this->pid_.k_d(val);
        this->pid_set_flag_ |= 0x4;
        break;
    default :
        return ;
  }

  if (this->pid_set_flag_ == 0x07) {
      this->pid_set_flag_ = 0;
      this->SavePID();
  }
}

bool Temperature::isEnabled() {
  return pid_.getTarget() > 0 ;
}

