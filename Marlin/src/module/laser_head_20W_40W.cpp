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

void LaserHead20W40W::Init()
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  laser_power_ctrl_.Init(LASER_20W_40W_ENBLE_PIN, 0, OUTPUT);
  fan_.Init(LASER_20W_40W_FAN_PIN, LSAER_FAN_FB_IC_TIM, LSAER_FAN_FB_CH, LSAER_FAN_FB_IT_CH, FAN_FEEDBACK_THRESHOLD);
  temperature_.InitCapture(LASER_20W_40W_TEMP_PIN, ADC_TIM_4);
  hw_version_.index = HAL_adc_init(LASER_20W_40W_HW_VERSION_PIN, ADC_TIM_4, ADC_PERIOD_DEFAULT);
  pwm_detect_.Init(LASER_20W_40W_PWM_DETECT, INPUT_PULLUP);
  cross_light_.Init(LASER_20W_40W_CROSS_LIGHT, 0, OUTPUT);
  fire_sensor_adc_index_ = HAL_adc_init(LASER_20W_40W_FIRE_SENSOR_PIN, LASER_20W_40W_FIRE_SENSOR_ADC_TIMER, LASER_20W_402_FIRE_SENSOR_ADC_PERIOD_US);

  AppParmInfo *param = &registryInstance.cfg_;
  if (param->parm_mark[0] != 0xaa || param->parm_mark[1] != 0x55)
  {
    param->laser_protect_temp = LASER_20W_40W_TEMP_LIMIT;
    param->laser_recovery_temp = LASER_20W_40W_TEMP_RECOVERY;
    param->fire_sensor_sensitivity = FIRE_DETECT_SENSITIVITY_HIGHT;
    if (MODULE_LASER_20W == registryInstance.module())
    {
      param->laser_crosslight_offset_x = LASER_20W_CL_OFFSET_X;
      param->laser_crosslight_offset_y = LASER_20W_CL_OFFSET_Y;
    }
    else
    {
      param->laser_crosslight_offset_x = LASER_40W_CL_OFFSET_X;
      param->laser_crosslight_offset_y = LASER_40W_CL_OFFSET_Y;
    }
    registryInstance.SaveCfg();
  }

  sync_id_ = param->module_sync_id;
  protect_temp_ = param->laser_protect_temp;
  recovery_temp_ = param->laser_recovery_temp;
  fire_sensor_sensitivity_ = param->fire_sensor_sensitivity;
  fire_sensor_trigger_ = false;
  fire_sensor_raw_data_report_tick_ms_ = millis();
  fire_sensor_raw_data_report_interval_ms_ = 0;
  crosslight_offset_x_ = param->laser_crosslight_offset_x;
  crosslight_offset_y_ = param->laser_crosslight_offset_y;

  security_status_ |= FAULT_LASER_PWM_PIN;
  if (icm42670.ChipInit() == false)
  {
    security_status_ |= FAULT_IMU_CONNECTION;
  }
}

void LaserHead20W40W::GetHwVersion()
{
  hw_version_.adc_value = ADC_Get(hw_version_.index);

  if (hw_version_.adc_value == 0)
    return;

  if (hw_version_.number != 0xAA)
    return;

  if (hw_version_.adc_value > 3700)
  { // 3.3v
    hw_version_.number = 0xff;
    fan_.set_feed_back_enable(false);
  }
  else if (hw_version_.adc_value > 670 && hw_version_.adc_value < 770)
  { // 0.57v
    hw_version_.number = 0;
    fan_.set_feed_back_enable(true);
  }
}

void LaserHead20W40W::Loop()
{
  GetHwVersion();
  fan_.Loop();
  laser_power_ctrl_.OutCtrlLoop();
  cross_light_.OutCtrlLoop();
  SecurityStatusCheck();
  LaserFireSensorLoop();
  LaserFireSensorReportLoop();
}

void LaserHead20W40W::HandModule(uint16_t func_id, uint8_t *data, uint8_t data_len)
{
  uint8_t focus_type;
  uint16_t rp_itv;
  float x_offset, y_offset;

  switch (func_id)
  {
  case FUNC_SET_FAN:
    fan_.ChangePwm(data[1], data[0]);
    break;
  case FUNC_SET_CAMERA_POWER:
    //
    break;
  case FUNC_SET_LASER_FOCUS:
    focus_type = data_len > 2 ? data[2] : 0;
    LaserSaveFocus(focus_type, data[0] << 8 | data[1]);
    break;
  case FUNC_REPORT_LASER_FOCUS:
    focus_type = data_len ? data[0] : 0;
    LaserReportFocus(focus_type);
    break;
  case FUNC_SET_AUTOFOCUS_LIGHT:
    //
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
  case FUNC_SET_CROSSLIGHT:
    LaserSetCrossLight(!!data[0]);
    break;
  case FUNC_GET_CROSSLIGHT_STATE:
    LaserGetCrossLightState();
    break;
  case FUNC_SET_FIRE_SENSOR_SENSITIVITY:
    LaserSetFireSensorSensitivity(data[0]);
    break;
  case FUNC_GET_FIRE_SENSOR_SENSITIVITY:
    LaserGetFireSensorSensitivity();
    break;
  case FUNC_SET_FIRE_SENSOR_REPORT_TIME:
    rp_itv = (data[1] << 8) | data[0];
    LaserSetFireSensorRawDataReportTime(rp_itv);
    break;
  case FUNC_REPORT_FIRE_SENSOR_RAW_DATA:
    LaserReportFireSensorRawData();
    break;
  case FUNC_SET_CROSSLIGHT_OFFSET:
    x_offset = *((float *)(&data[0]));
    y_offset = *((float *)(&data[4]));
    LaserSetCrosslightOffset(x_offset, y_offset);
    break;
  case FUNC_GET_CROSSLIGHT_OFFSET:
    LaserGetCrosslightOffset();
    break;
  default:
    break;
  }
}

void LaserHead20W40W::EmergencyStop()
{
  laser_power_ctrl_.Out(0);
  fan_.ChangePwm(0, 0);
}

void LaserHead20W40W::SecurityStatusCheck()
{
  temperature_.GetTemperature(laser_celsius_);

  if ((security_status_ & FAULT_IMU_CONNECTION) == 0)
  {
    if (icm42670.AttitudeSolving() == true)
    {
      icm42670.GetGesture(yaw_, pitch_, roll_);
    }
  }

  if (laser_celsius_ > protect_temp_)
  {
    security_status_ |= FAULT_LASER_TEMP;
  }
  else if (laser_celsius_ < recovery_temp_)
  {
    security_status_ &= ~FAULT_LASER_TEMP;
  }

  if ((roll_ <= roll_min_) || (roll_ >= roll_max_) || (pitch_ <= pitch_min_) || (pitch_ >= pitch_max_))
  {
    security_status_ |= FAULT_LASER_GESTURE;
  }
  else
  {
    security_status_ &= ~FAULT_LASER_GESTURE;
  }

  if (fan_.get_feed_back_state() == false)
  {
    security_status_ |= FAULT_LASER_FAN_RUN;
  }
  else
  {
    security_status_ &= ~FAULT_LASER_FAN_RUN;
  }

  if (fire_sensor_trigger_)
  {
    security_status_ |= FAULT_FIRE_DECT;
  }
  else
  {
    security_status_ &= ~FAULT_FIRE_DECT;
  }

  if (security_status_ != 0)
  {
    laser_power_ctrl_.Out(0);
  }

  if (security_status_ != security_status_pre_)
  {
    security_status_pre_ = security_status_;
    if (registryInstance.FuncId2MsgId(FUNC_REPORT_SECURITY_STATUS) != INVALID_VALUE)
    {
      ReportSecurityStatus();
    }
  }
}

void LaserHead20W40W::ReportSecurityStatus()
{
  uint8_t buf[8];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_SECURITY_STATUS);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    icm42670.GetGesture(yaw_, pitch_, roll_);
    imu_celsius_ = (int8_t)icm42670.GetTemperature();
    int16_t pitch_int16, roll_int16;
    int8_t celsius_int8;
    pitch_int16 = (int16_t)pitch_;
    roll_int16 = (int16_t)roll_;
    celsius_int8 = (signed char)laser_celsius_;

    buf[index++] = security_status_;
    buf[index++] = (pitch_int16 >> 8) & 0xff;
    buf[index++] = pitch_int16 & 0xff;
    buf[index++] = (roll_int16 >> 8) & 0xff;
    ;
    buf[index++] = roll_int16 & 0xff;
    buf[index++] = celsius_int8;
    buf[index++] = (uint8_t)imu_celsius_;
    // buf[index++] = !fire_dect_sensor_.Read();
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserSaveFocus(uint8_t type, uint16_t foch)
{
  AppParmInfo *param = &registryInstance.cfg_;
  if (type)
  {
    param->laser_high_4_axis = foch;
  }
  else
  {
    param->laser_high = foch;
  }
  registryInstance.SaveCfg();
}

void LaserHead20W40W::LaserReportFocus(uint8_t type)
{
  AppParmInfo *param = &registryInstance.cfg_;
  uint8_t u8DataBuf[8], u8Index = 0;
  uint16_t u16Focu = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_LASER_FOCUS);
  if (msgid != INVALID_VALUE)
  {
    if (type)
    {
      u16Focu = param->laser_high_4_axis;
    }
    else
    {
      u16Focu = param->laser_high;
    }
    if (!(param->parm_mark[0] == 0xaa && param->parm_mark[1] == 0x55) || (u16Focu == 0xffff))
    {
      u16Focu = (uint16_t)LASER_DEFAULT_HIGH;
    }
    u8DataBuf[u8Index++] = u16Focu >> 8;
    u8DataBuf[u8Index++] = u16Focu;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

void LaserHead20W40W::LaserOnlineStateSync(uint8_t *data)
{
  AppParmInfo *param = &registryInstance.cfg_;
  if (data[0] == 1)
  {
    // set module sync id
    sync_id_ = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    param->module_sync_id = sync_id_;
    registryInstance.SaveCfg();
  }
  else if (data[0] == 0)
  {
    // report module sync id
    uint8_t buf[8];
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_ONLINE_SYNC);
    uint8_t index = 0;
    if (msgid != INVALID_VALUE)
    {
      buf[index++] = sync_id_ & 0xff;
      buf[index++] = (sync_id_ >> 8) & 0xff;
      buf[index++] = (sync_id_ >> 16) & 0xff;
      buf[index++] = (sync_id_ >> 24) & 0xff;
      canbus_g.PushSendStandardData(msgid, buf, index);
    }
  }
}

void LaserHead20W40W::LaserSetProtectTemp(uint8_t *data)
{
  AppParmInfo *param = &registryInstance.cfg_;
  protect_temp_ = data[0];
  recovery_temp_ = data[1];

  param->laser_protect_temp = protect_temp_;
  param->laser_recovery_temp = recovery_temp_;
  registryInstance.SaveCfg();
}

void LaserHead20W40W::LaserCtrl(uint8_t *data)
{
  switch (data[0])
  {
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
  if (msgid != INVALID_VALUE)
  {
    buf[index++] = data[0];
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserReportHWVersion()
{
  ModuleMacInfo *mac = (ModuleMacInfo *)FLASH_MODULE_PARA;

  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_GET_HW_VERSION);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE)
  {
    if (hw_version_.number == 0xAA)
      buf[index++] = mac->hw_version;
    else
      buf[index++] = hw_version_.number;
    // to have a simple checksum
    buf[index++] = ~buf[0];
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserReportPinState()
{
  uint8_t buf[1];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PIN_STATUS);
  if (msgid != INVALID_VALUE)
  {
    buf[index++] = digitalRead(LASER_20W_40W_PWM_DETECT);
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserConfirmPinState()
{
  security_status_ &= ~FAULT_LASER_PWM_PIN;
}

void LaserHead20W40W::LaserSetCrossLight(bool onoff)
{
  cross_light_.Out(onoff);
}

void LaserHead20W40W::LaserGetCrossLightState(void)
{
  uint8_t buf[1];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_GET_CROSSLIGHT_STATE);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    buf[index++] = digitalRead(LASER_20W_40W_CROSS_LIGHT);
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserSetFireSensorSensitivity(uint8_t fds)
{
  fire_sensor_sensitivity_ = fds;
  AppParmInfo *param = &registryInstance.cfg_;
  param->fire_sensor_sensitivity = fire_sensor_sensitivity_;
  registryInstance.SaveCfg();
}

void LaserHead20W40W::LaserGetFireSensorSensitivity(void)
{
  uint8_t buf[1];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_GET_FIRE_SENSOR_SENSITIVITY);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    buf[index++] = fire_sensor_sensitivity_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserSetFireSensorRawDataReportTime(uint16_t rp_itv_ms)
{
  fire_sensor_raw_data_report_interval_ms_ = rp_itv_ms;
  // Turn on report, reset time start tick
  if (0 != fire_sensor_raw_data_report_interval_ms_)
    fire_sensor_raw_data_report_tick_ms_ = millis();
}

void LaserHead20W40W::LaserSetCrosslightOffset(float x, float y)
{
  crosslight_offset_x_ = x;
  crosslight_offset_y_ = y;
  AppParmInfo *param = &registryInstance.cfg_;
  param->laser_crosslight_offset_x = crosslight_offset_x_;
  param->laser_crosslight_offset_y = crosslight_offset_y_;
  registryInstance.SaveCfg();
}

void LaserHead20W40W::LaserReportFireSensorRawData(void)
{
  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_FIRE_SENSOR_RAW_DATA);
  uint8_t index = 0;

  if (msgid != INVALID_VALUE)
  {
    buf[index++] = fire_sensor_raw_adc_ & 0xff;
    buf[index++] = (fire_sensor_raw_adc_ >> 8) & 0xff;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void LaserHead20W40W::LaserGetCrosslightOffset(void)
{
  uint8_t buf[8];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_GET_CROSSLIGHT_OFFSET);

  if (msgid != INVALID_VALUE)
  {
    float *t = (float *)(&buf[0]);
    *t = crosslight_offset_x_;
    t = (float *)(&buf[4]);
    *t = crosslight_offset_y_;
    canbus_g.PushSendStandardData(msgid, buf, 8);
  }
}

void LaserHead20W40W::LaserFireSensorReportLoop(void)
{
  if (0 == fire_sensor_raw_data_report_interval_ms_)
    return;

  if (ELAPSED(millis(), fire_sensor_raw_data_report_tick_ms_ + fire_sensor_raw_data_report_interval_ms_))
  {
    fire_sensor_raw_data_report_tick_ms_ = millis();
    LaserReportFireSensorRawData();
  }
}

void LaserHead20W40W::LaserFireSensorLoop(void)
{
  bool trigger = false;
  if (PENDING(millis(), fire_sensor_maf_last_ms_))
    return;

  fire_sensor_maf_last_ms_ = millis() + 1000 / LASER_FIRE_SENSOR_SAMPLE_FREQ;
  fire_sensor_raw_adc_ = ADC_Get(fire_sensor_adc_index_);
  fire_sensor_maf_.addValue(fire_sensor_raw_adc_);

  switch (fire_sensor_sensitivity_)
  {
  case FIRE_DETECT_SENSITIVITY_HIGHT:
    if (fire_sensor_maf_.getMovingAverage() < FIRE_DETECT_SENSITIVITY_HIGHT_ADC_VALUE)
      trigger = true;
    break;

  case FIRE_DETECT_SENSITIVITY_MID:
    if (fire_sensor_maf_.getMovingAverage() < FIRE_DETECT_SENSITIVITY_MID_ADC_VALUE)
      trigger = true;
    break;

  case FIRE_DETECT_SENSITIVITY_LOW:
    if (fire_sensor_maf_.getMovingAverage() < FIRE_DETECT_SENSITIVITY_MID_ADC_VALUE)
      trigger = true;
    break;

  case FIRE_DETECT_SENSITIVITY_DIS:
    /* code */
    break;

  default:
    break;
  }

  if (trigger) {
    fire_sensor_trigger_ = 1;
    fire_sensor_trigger_reset_delay_ = 5 * LASER_FIRE_SENSOR_SAMPLE_FREQ;// 5 second delay
  }    
  else {
    if (fire_sensor_trigger_reset_delay_)
      fire_sensor_trigger_reset_delay_--;
    else
      fire_sensor_trigger_ = 0;
  }
}
