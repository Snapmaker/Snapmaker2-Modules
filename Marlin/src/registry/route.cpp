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

#include <src/core/can_bus.h>
#include "route.h"

#define FUNC_LIST_INIT(list)  func_list_ = list;\
                              func_count_ = sizeof(list) / sizeof(list[0]);

const uint16_t print_func_list_[] = {
  FUNC_SET_FAN,
  FUNC_SET_FAN2,
  FUNC_REPORT_TEMPEARTURE,
  FUNC_SET_TEMPEARTURE,
  FUNC_REPORT_PROBE,
  FUNC_SET_PID,
  FUNC_REPORT_CUT,
  FUNC_REPORT_TEMP_PID,
};

const uint16_t dual_extruder_func_list_[] = {
  FUNC_SET_FAN,
  FUNC_SET_FAN2,
  FUNC_REPORT_TEMPEARTURE,
  FUNC_SET_TEMPEARTURE,
  FUNC_REPORT_PROBE,
  FUNC_SET_PID,
  FUNC_REPORT_CUT,
  FUNC_REPORT_TEMP_PID,
  FUNC_SWITCH_EXTRUDER,
  FUNC_REPORT_NOZZLE_TYPE,
  FUNC_SET_FAN_NOZZLE,
  FUNC_REPORT_EXTRUDER_INFO,
  FUNC_SET_EXTRUDER_CHECK,
  FUNC_SET_HOTEND_OFFSET,
  FUNC_REPORT_HOTEND_OFFSET,
  FUNC_SET_PROBE_SENSOR_COMPENSATION,
  FUNC_REPORT_PROBE_SENSOR_COMPENSATION,
  FUNC_MOVE_TO_DEST,
  FUNC_SET_RIGHT_EXTRUDER_POS,
  FUNC_REPORT_RIGHT_EXTRUDER_POS,
  FUNC_PROXIMITY_SWITCH_POWER_CTRL,
  FUNC_MODULE_GET_HW_VERSION,
  FUNC_SET_RIGHT_LEVEL_MODE,
  FUNC_REPORT_RIGHT_LEVEL_MODE_INFO,
};

const uint16_t laser_func_list_[] = {
  FUNC_SET_FAN,
  FUNC_SET_CAMERA_POWER,
  FUNC_SET_LASER_FOCUS,
  FUNC_REPORT_LASER_FOCUS,
};

const uint16_t laser_10w_func_list_[] = {
  FUNC_SET_FAN,
  FUNC_SET_CAMERA_POWER,
  FUNC_SET_LASER_FOCUS,
  FUNC_REPORT_LASER_FOCUS,
  FUNC_SET_AUTOFOCUS_LIGHT,
  FUNC_REPORT_SECURITY_STATUS,
  FUNC_MODULE_ONLINE_SYNC,
  FUNC_MODULE_SET_TEMP,
  FUNC_MODULE_LASER_CTRL,
  FUNC_MODULE_GET_HW_VERSION,
  FUNC_REPORT_PIN_STATUS,
  FUNC_CONFIRM_PIN_STATUS,
};

const uint16_t cnc_func_list_[] = {
  FUNC_REPORT_MOTOR_SPEED,
  FUNC_SET_MOTOR_SPEED,
};

const uint16_t enclosure_func_list_[] = {
  FUNC_REPORT_ENCLOSURE,
  FUNC_SET_ENCLOSURE_LIGHT,
  FUNC_SET_FAN_MODULE,
};

const uint16_t tool_setting_func_list_[] = {
    FUNC_REPORT_TOOL_SETTING,
};

const uint16_t light_func_list_[] = {
  FUNC_SET_LIGHT_COLOR,
};

const uint16_t linear_func_list_[] = {
  FUNC_REPORT_LIMIT,
};

const uint16_t stop_func_list_[] = {
  FUNC_REPORT_STOP_SWITCH,
};

const uint16_t purifier_func_list_[] = {
  FUNC_SET_PURIFIER,
  FUNC_REPORT_PURIFIER,
};

const uint16_t fan_func_list_[] = {
  FUNC_SET_FAN_MODULE,
};

const uint16_t cnc_200w_func_list_[] = {
  FUNC_SET_MOTOR_SPEED,
  FUNC_SET_FAN,
  FUNC_SET_PID,
  FUNC_MODULE_GET_HW_VERSION,
  FUNC_SET_MOTOR_SPEED_RPM,
  FUNC_SET_MOTOR_CTR_MODE,
  FUNC_SET_MOTOR_RUN_DIRECTION,
  FUNC_REPORT_MOTOR_STATUS_INFO,
  FUNC_REPORT_MOTOR_SENSOR_INFO,
  FUNC_REPORT_MOTOR_SELF_TEST_INFO,
};

const uint16_t enclosure_a400_func_list_[] = {
  FUNC_REPORT_ENCLOSURE,
  FUNC_SET_ENCLOSURE_LIGHT,
  FUNC_SET_FAN_MODULE,
  FUNC_MODULE_GET_HW_VERSION,
};

const uint16_t drybox_func_list_[] = {
  FUNC_SET_FAN,
  FUNC_SET_TEMPEARTURE,
  FUNC_REPORT_TEMP_HUMIDITY,
  FUNC_REPORT_TEMP_PID,
  FUNC_SET_PID,
  FUNC_SET_HEAT_TIME,
  FUNC_REPORT_HEATING_TIME_INFO,
  FUNC_SET_MAINCTRL_TYPE,
  FUNC_MODULE_START,
  FUNC_REPORT_HEATER_POWER_STATE,
  FUNC_REPORT_COVER_STATE,
  FUNC_REPORT_DRYBOX_STATE,
};

const uint16_t calibrator_func_list_[] = {
  FUNC_REPORT_PROBE,
};

const uint16_t laser_20w40w_func_list_[] = {
  FUNC_SET_FAN,
  FUNC_SET_CAMERA_POWER,
  FUNC_SET_LASER_FOCUS,
  FUNC_REPORT_LASER_FOCUS,
  FUNC_SET_AUTOFOCUS_LIGHT,
  FUNC_REPORT_SECURITY_STATUS,
  FUNC_MODULE_ONLINE_SYNC,
  FUNC_MODULE_SET_TEMP,
  FUNC_MODULE_LASER_CTRL,
  FUNC_MODULE_GET_HW_VERSION,
  FUNC_REPORT_PIN_STATUS,
  FUNC_CONFIRM_PIN_STATUS,
  FUNC_SET_CROSSLIGHT,
  FUNC_GET_CROSSLIGHT_STATE,
  FUNC_SET_FIRE_SENSOR_SENSITIVITY,
  FUNC_GET_FIRE_SENSOR_SENSITIVITY,
  FUNC_SET_FIRE_SENSOR_REPORT_TIME,
  FUNC_REPORT_FIRE_SENSOR_RAW_DATA,
  FUNC_SET_CROSSLIGHT_OFFSET,
  FUNC_GET_CROSSLIGHT_OFFSET,
  FUNC_MODULE_LASER_BRANCH_CTRL,
};


Route routeInstance;
void Route::Init() {
  uint32_t moduleType = registryInstance.module();

  switch (moduleType) {
    case MODULE_PRINT:
    case MODULE_PRINT_V_SM1:
      module_ = new PrintHead;
      module_->Init();
      FUNC_LIST_INIT(print_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
    case MODULE_DUAL_EXTRUDER:
      SetBaseVersions(1, 13, 12);
      module_ = new DualExtruder;
      module_->Init();
      FUNC_LIST_INIT(dual_extruder_func_list_);
      break;
    case MODULE_LASER:
      module_ = new LaserHead;
      module_->Init();
      FUNC_LIST_INIT(laser_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
    case MODULE_LASER_10W:
      module_ = new LaserHead10W;
      module_->Init();
      FUNC_LIST_INIT(laser_10w_func_list_);
      SetBaseVersions(1, 11, 0);
      break;
    case MODULE_CNC:
      module_ = new CncHead;
      module_->Init();
      FUNC_LIST_INIT(cnc_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
    case MODULE_LINEAR:
      module_ = new LinearModule;
      module_->Init();
      FUNC_LIST_INIT(linear_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
    case MODULE_LINEAR_TMC:
      module_ = new LinearModule;
      module_->Init();
      FUNC_LIST_INIT(linear_func_list_);
      SetBaseVersions(1, 9, 1);
      break;
    case MODULE_LIGHT:
      module_ = new LightModule;
      module_->Init();
      FUNC_LIST_INIT(light_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
    case MODULE_CNC_TOOL_SETTING:
      module_ = new CncToolSetting;
      module_->Init();
      FUNC_LIST_INIT(tool_setting_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
    case MODULE_ENCLOSURE:
      module_ = new EnclosureModule;
      module_->Init();
      FUNC_LIST_INIT(enclosure_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
    case MODULE_FAN:
      module_ = new FanModule();
      module_->Init();
      FUNC_LIST_INIT(fan_func_list_);
      SetBaseVersions(1, 7, 0);
      break;
     case MODULE_PURIFIER:
      module_ = new PurifierModule();
      module_->Init();
      FUNC_LIST_INIT(purifier_func_list_);
      SetBaseVersions(1, 10, 3);
      break;
    case MODULE_EMERGENCY_STOP:
      module_ = new StopModule;
      module_->Init();
      FUNC_LIST_INIT(stop_func_list_);
      SetBaseVersions(1, 10, 4);
      break;
    case MODULE_ROTATE:
    case MODULE_ROTARY_2023:
      module_ = new RotateModule;
      module_->Init();
      if (moduleType == MODULE_ROTATE)
        SetBaseVersions(1, 9, 0);
      else
        SetBaseVersions(1, 13, 14);
      break;
    case MODULE_CNC_200W:
      module_ = new CncHead200W;
      module_->Init();
      FUNC_LIST_INIT(cnc_200w_func_list_);
      SetBaseVersions(1, 12, 0);
      break;

    case MODULE_ENCLOSURE_A400:
      SetBaseVersions(1, 12, 0);
      module_ = new EnclosureA400Module;
      module_->Init();
      FUNC_LIST_INIT(enclosure_a400_func_list_);
      break;
    case MODULE_DRYBOX:
      module_ = new DryBox;
      module_->Init();
      FUNC_LIST_INIT(drybox_func_list_);
      SetBaseVersions(1, 12, 2);
      break;
    case MODULE_CALIBRATOR:
      module_ = new Calibrator;
      module_->Init();
      FUNC_LIST_INIT(calibrator_func_list_);
      SetBaseVersions(1, 12, 2);
      break;

    case MODULE_LASER_20W:
    case MODULE_LASER_40W:
      module_ = new LaserHead20W40W;
      module_->Init();
      FUNC_LIST_INIT(laser_20w40w_func_list_);
      SetBaseVersions(1, 13, 13);
      break;

    default:
      module_ = new ModuleBase();
      module_->Init();
      SetBaseVersions(0, 0, 0);
  }

  hal_start_adc();
}

void Route::Invoke() {
  uint16_t func_id = contextInstance.funcid_;
  uint8_t * data = contextInstance.data_;
  uint8_t   data_len = contextInstance.len_;
  module_->HandModule(func_id, data, data_len);
}

void Route::ModuleLoop() {
  module_->Loop();
}

