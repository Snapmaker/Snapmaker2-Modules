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

#ifndef PURIFIER_H_
#define PURIFIER_H_

#include "src/configuration.h"
#include "src/device/speed.h"
#include "src/device/switch.h"
#include "src/device/rgb_light.h"
#include "module_base.h"

#define PURIFIER_FAN_POWER_PIN PA4

#define PURIFIER_FAN_PWM_TIM PWM_TIM3, PWM_CH1
#define PURIFIER_FAN_PWM_PIN PA6

#define FAN_SPEED_CAPTURE_PARM PA7, 2  // TIM2
#define FILTER_SWITCH_PIN PB3

#define PURIFIER_LIGHT_PIN PA8
#define PURIFIER_LIGHT_COUNT 12
#define PURIFIER_LIGHT_EXTI SOFT_EXTI_LINE2  // Using software interrupts

#define PURIFIER_ADC_PERIOD_US 1000
#define PURIFIER_ADC_TIM ADC_TIM_4
#define ADDON_POWER_ADC  PA2 , PURIFIER_ADC_TIM, PURIFIER_ADC_PERIOD_US
#define EXTEND_POWER_ADC PA3 , PURIFIER_ADC_TIM, PURIFIER_ADC_PERIOD_US
#define FAN_ELEC_ADC     PB0 , PURIFIER_ADC_TIM, PURIFIER_ADC_PERIOD_US
#define PURIFIER_ADC_DEEP 8
#define PURIFIER_ADC_COUNT 3
#define PURIFIER_ADC_CACHE_SIZE (PURIFIER_ADC_DEEP * PURIFIER_ADC_COUNT)

#define CONNECT_TIMEOUT_MS 3000
#define FAN_STABLE_TIME_MS 10000
#define FAN_MIN_SPEED 1000  // rpm/min
#define FAN_MAX_WORK_ELEC 3700  // mA

#define LIFETIME_TO_ELEC_LOW {0, 390, 1200, 1500}
#define LIFETIME_TO_ELEC_NORMAL {0, 460,1750, 2220}
#define OVERCURRENT_PROTECTION 5000

#define LIFETIME_STABLE_TIMES 5
#define LIFETIME_CAPTURE_INTERVAL 20  // ms
#define LIFETIME_CUMULATIVE_NUMBER 50

#define WHITE_LIGHT {255, 255, 255}
#define WHITE_WATERFALL_LIGHT {255, 255, 255}
#define YELLOW_LIGHT {255, 189, 19}
#define RED_LIGHT {255, 0, 0}
#define BLUE_LIGHT {0, 0, 255}
#define ORANGE_LIGHT {226, 97, 14}
#define BLACK_LIGHT {0, 0, 0}
#define GREEN_LIGHT {0, 255, 0}


typedef enum  : uint8_t {
  ERR_NONE = 0x0,
  ERR_ADDON_POWER = 0x1,
  ERR_EXTEND_POWER_OFF = 0x2,
  ERR_FAN_SPEED_TOO_LOW = 0x4,
  ERR_NO_FILTER = 0x8,
  ERR_ELEC_TOO_HIGH = 0x10,
  ERR_EMERGENCY_STOP = 0x20,
  ERR_EXTEND_POWER = 0x40,
}PURIFIER_ERR_E;

typedef enum : uint8_t {
  FAN_STA_IDLE,
  FAN_STA_WORKING,
}PURIFIER_WORK_STATUS_E;

typedef enum : uint8_t{
  STA_IDLE,
  STA_WORKING,
  STA_POWER_OFF,
  STA_POWER_ERR,
  STA_WAIT_CONNECT,
  STA_FORCED_RUN,
  STA_EMERGENCY_STOP,
}PURIFIER_STATUS_E;

typedef enum : uint8_t {  // LT -> light
  LT_WAIT_CONNECT,
  LT_LIFETIME_LOW,
  LT_LIFETIME_MEDIUM,
  LT_LIFETIME_NORMAL,
  LT_FAN_SPEED_ERR,
  LT_NO_FILTER_ERR,
  LT_ELEC_TOO_HIGH_ERR,
  LT_POWER_ERR,
  LT_FORCED_RUN,
  LT_POWER_OFF,
  LT_INVALID
}PURIFIER_LIGHT_STA_E;

typedef enum : uint8_t {
  FAN_GEARS_0,  // off
  FAN_GEARS_1,
  FAN_GEARS_2,
  FAN_GEARS_3,
  FAN_GEARS_INVALID,
}FAN_GEARS_E;

typedef enum : uint8_t {
  LIFETIME_LOW,
  LIFETIME_MEDIUM,
  LIFETIME_NORMAL,
  LIFETIME_INVALID,
}PURIFIER_LIFETIME_E;

enum : uint8_t {
  PURIFIER_SET_FAN_STA,
  PURIFIER_SET_FAN_GEARS,
  PURIFIER_SET_FAN_POWER,
  PURIFIER_SET_LIGHT,
};

enum : uint8_t {
  PURIFIER_REPORT_LIFETIME,
  PURIFIER_REPORT_ERR,
  PURIFIER_REPORT_FAN_STA,
  PURIFIER_REPORT_ELEC,
  PURIFIER_REPORT_POWER,
  PURIFIER_REPORT_STATUS,
  PURIFIER_INFO_ALL,
};

class PurifierModule : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();

 private:
  void InitFan();
  void FanOut(uint8_t power_gears);
  void InitAdcCheck();
  void InitLight();
  void CheckLifetime();
  void ReportLifetime();
  void LightCtrl(PURIFIER_LIGHT_STA_E sta);
  void PeripheralLoopCtrl();
  bool IsFanStable();
  void LoadLifetime();
  void SaveLifetime();
  void ReportErrStatus();
  void ReportFanSpeed();
  void ReportFanElec();
  void ReportPower();
  void ReportSysStatus();
  uint32_t GetFanElectricity();
  uint16_t GetAddonPower();
  uint16_t GetExtendPower();
  uint8_t WorkExceptionCheck();
  void SetFanStatus(uint8_t is_open, uint8_t is_forced);
  void SetFanGears(uint8_t gears);
  void FanPowerOut(uint8_t power);
  void WaitCfgProcess();
  void IdleProcess();
  void WorkProcess();
  void ExceptionProcess();
  void ExtendPowerErrEvent(uint16_t extent_power);
  void ForcedRun();
  void WorkExceptionHandling();
  void WaitConnectHandling();
  void StandbyMode();

 private:
  Speed fan_;
  SwitchOutput fan_power_;
  SwitchInput filter_switch_;
  RGBLight light_;
  uint8_t fan_out_ = 0;
  uint8_t fan_last_out_ = 0xff;
  bool fan_reopen_ = false;
  uint8_t fan_gears_ = 0;
  uint8_t cur_lifetime_ = LIFETIME_NORMAL;
  uint8_t test_lifetime_= 255;
  uint8_t same_lifetime_count_ = 0;
  PURIFIER_WORK_STATUS_E fan_state_ = FAN_STA_IDLE;
  uint32_t fan_start_time_ = 0;  //
  uint8_t err_ = 0;
  uint8_t last_err_ = err_;
  uint8_t addon_power_index_ = 0;
  uint8_t extend_power_index_ = 0;
  uint8_t elec_adc_index_ = 0;
  uint8_t is_forced_run_ = 0;
  uint16_t lifetime_normal_limit_[FAN_GEARS_INVALID] = LIFETIME_TO_ELEC_NORMAL;
  uint16_t lifetime_low_limit_[FAN_GEARS_INVALID] = LIFETIME_TO_ELEC_LOW;
  PURIFIER_STATUS_E sys_status_ = STA_IDLE;
  bool is_debug_ = false;
};

#endif
