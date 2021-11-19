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

#ifndef __DUAL_DRYBOX_H_
#define __DUAL_DRYBOX_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "src/device/analog_io_ctrl.h"
#include "module_base.h"
#include "src/device/temperature.h"
#include "../device/soft_i2c.h"

#define BOARD_SM_NULL                    (0xffff)
#define BOARD_SM_CONTROLLER2019_V1       (4300)
#define BOARD_SM_CONTROLLER2022_V1       (4301)

#define CAN_DATA_FRAME_LENGTH    (8)

#define I2C_SDA                  PB7
#define I2C_SCL                  PB6
#define THERMISTOR_TEMP_PIN      PA2
#define HEATING_BLOCK_PIN        PA3
#define POWER_SOURCE_DETECT_PIN  PA5
#define HEATER_POWER_MONITOR_PIN PA6
#define COVER_DET_PIN            PB0
#define FAN_PIN                  PB1
#define POWER_SELECT_PIN         PB5

#define ADDR_GXHT3X           0x44

#define DRYBOX_LIGHT_PIN PA7
#define DRYBOX_LIGHT_COUNT 12
#define DRYBOX_LIGHT_EXTI SOFT_EXTI_LINE2  // Using software interrupts

#define HEATER_PROTECT_TEMP  130
#define HEATER_RECOVER_TEMP  80

#define GET_STATUS(S, B) ((S >> B) & 0x01)
#define SET_BIT(S, B) (S |= (0x01 << B))
#define CLR_BIT(S, B) (S &= ~(0x01 << B))

typedef enum {
  TEMP_CTRL_PRE_HEAT,
  TEMP_CTRL_CHAMBER_HEAT,
}temp_ctrl_e;

typedef enum {
  DRYBOX_STATE_IDLE,
  DRYBOX_STATE_HEATING_FREE,
  DRYBOX_STATE_HEATING_TIMING,
  DRYBOX_STATE_FAULT,
}drybox_state_e;

typedef enum {
  DRYBOX_FAULT_OVER_TEMP,
  DRYBOX_FAULT_COVER_OPEN,
  DRYBOX_FAULT_HEATER_POWER,
  DRYBOX_FAULT_NO_EXTERNAL_POWER,
  DRYBOX_FAULT_MAINCTRL_DISCONNECT,
}drybox_fault_state_e;

class DryBox : public ModuleBase {
  public:
    DryBox () {
      temp_ctrl_stage_ = TEMP_CTRL_PRE_HEAT;
      chamber_temp_ready_ = false;
      is_cyclic_convert_start_ = false;
      temp_humidity_time_elaspe_ = 0;
      info_report_time_ = 0;

      mainctrl_type_ = BOARD_SM_NULL;
      power_source_state_ = 0xff;
      cover_state_ = 0xff;
      heater_power_state_ = 0xff;
      drybox_prev_state_ = DRYBOX_STATE_IDLE;
      drybox_state_ = DRYBOX_STATE_IDLE;
      drybox_fault_state_ = 0;
      light_color_ = WHITE_LIGHT;
      target_heating_time_ = 0;
      accumulate_dry_time_ = 0;
      timing_heating_time_ = 0;
      remain_heating_time_ = 0;

      dry_time_process_time_elaspe_ = 0;
    }
    void Init();
    void InitLight();
    void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
    void ReportTempHumidity();
    void ReportHeaterPowerState();
    void ReportCoverState();
    void ReportDryBoxState();
    bool ResetGXHT3X();
    bool StartSingleConvert(uint8_t clock_stretching, uint8_t accuracy);
    void StartCyclicConvert(uint8_t freq, uint8_t accuracy);
    void StopCyclicConvert();
    bool GetTempHumidity();
    bool GetTempHumidityCyclic();
    void ReadTempHumidityCyclic();
    void TempAndHumidityProcess();
    void clear_heating_time();
    void StartHeating(uint8_t state);
    void SetTargetTemp(int16_t heater_temp, int16_t chamber_temp);
    void SetTargetHeatingTime(uint32_t time);
    void CoverDetect();
    void HeatingTimeProcess();
    void HeaterTempMonitor();
    void TempCtrl();
    void HeatingProcess();
    void ExternalPowerDetection();
    void HeaterPowerMonitor();
    void ReportTimeInfo();
    void ReportInfoProcess();
    void MainCtrlOnlineMonitor();
    bool RequestSystemEnterState(drybox_state_e state);
    void EmergencyStop();
    void Loop();

    Temperature heater_;
    Temperature chamber_;
    Fan fan_;
    SwitchInput power_source_detect_;
    SwitchInput heater_power_monitor_;
    SwitchInput  cover_detect_;
    SwitchOutput power_select_;
    RGBLight light_;

  private:
    SoftI2C temp_humidity_sensor_;
    uint16_t heater_target_temp_;
    uint16_t chamber_target_temp_;
    float heater_temp_;
    float chamber_temp_;
    float chamber_humidity_;
    temp_ctrl_e temp_ctrl_stage_;
    bool chamber_temp_ready_;
    bool is_cyclic_convert_start_;
    uint32_t temp_humidity_time_elaspe_;
    uint32_t info_report_time_;

    uint16_t mainctrl_type_;

    drybox_state_e drybox_prev_state_;
    drybox_state_e drybox_state_;
    volatile uint32_t drybox_fault_state_;
    uint8_t cover_state_;                  // 0: close, 1: open
    uint8_t power_source_state_;
    uint8_t heater_power_state_;
    uint8_t temp_protection_state_;
    uint8_t external_power_state_;
    RGB_T light_color_;

    int32_t target_heating_time_;
    int32_t accumulate_dry_time_;
    int32_t timing_heating_time_;
    int32_t remain_heating_time_;
    uint32_t dry_time_process_time_elaspe_;
};


#endif
