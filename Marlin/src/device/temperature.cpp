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
#include "src/HAL/hal_flash.h"
#include "src/configuration.h"
#include "src/registry/registry.h"


void Temperature::InitCapture(uint8_t adc_pin, ADC_TIM_E adc_tim) {
  adc_index_ = HAL_adc_init(adc_pin, adc_tim, 2400);
}

void Temperature::InitPID() {
  AppParmInfo *param = &registryInstance.cfg_;
  float p=0, i=0, d=0;
  if (param->parm_mark[0] == 0xaa && param->parm_mark[1] == 0x55) {
    p = param->temp_P;
    i = param->temp_I;
    d = param->temp_D;
  }
  if ((p == 0) && (i == 0) && (d == 0)) {
    param->temp_P = TEMP_DEFAULT_KP;
    param->temp_I = TEMP_DEFAULT_KI;
    param->temp_D = TEMP_DEFAULT_KD;
    registryInstance.SaveCfg();
  }
  this->pid_.Init(param->temp_P, param->temp_I, param->temp_D);
}

void Temperature::SavePID() {
  AppParmInfo * param = &registryInstance.cfg_;
  if ((param->temp_P != this->pid_.k_p_) ||
      (param->temp_I != this->pid_.k_i_)||
      (param->temp_D != this->pid_.k_d_)) {
      param->temp_P = this->pid_.k_p_;
      param->temp_I = this->pid_.k_i_;
      param->temp_D = this->pid_.k_d_;
      registryInstance.SaveCfg();
  }
}

void Temperature::InitOutCtrl(uint8_t tim_num, uint8_t tim_chn, uint8_t tim_pin) {
  this->InitPID();
  this->pwm_tim_chn_ = tim_chn;
  this->pwm_tim_num_ = tim_num;
  HAL_PwmInit(tim_num, tim_chn, tim_pin, 1000000, 255);
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

uint8_t Temperature::TempertuerStatus() {
  return hal_adc_status();
}

void Temperature::TemperatureOut() {
  detect_celsius_ = TempTableCalcCurTemp(ADC_GetCusum(adc_index_));
  uint32_t pwmOutput = pid_.output(detect_celsius_);
  HAL_PwmSetPulse(pwm_tim_num_, pwm_tim_chn_, pwmOutput);
}

void Temperature::Maintain() {
  if (TempertuerStatus()) {
    TemperatureOut();
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

