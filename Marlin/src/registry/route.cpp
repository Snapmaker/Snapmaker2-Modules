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
#include "src/module/print_head.h"

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

const uint16_t laser_func_list_[] = {
  FUNC_SET_FAN,
  FUNC_SET_CAMERA_POWER,
  FUNC_SET_LASER_FOCUS,
  FUNC_REPORT_LASER_FOCUS,
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

Route routeInstance;
void Route::Init() {
  uint32_t moduleType = registryInstance.module();

  switch (moduleType) {
    case MODULE_PRINT:
    case MODULE_PRINT_V_SM1:
      module_ = new PrintHead;
      module_->Init();
      FUNC_LIST_INIT(print_func_list_);
      break;
    case MODULE_LASER:
      module_ = new LaserHead;
      module_->Init();
      FUNC_LIST_INIT(laser_func_list_);
      break;
    case MODULE_CNC:
      module_ = new CncHead;
      module_->Init();
      FUNC_LIST_INIT(cnc_func_list_);
      break;
    case MODULE_LINEAR:
    case MODULE_LINEAR_TMC:
      module_ = new LinearModule;
      module_->Init();
      FUNC_LIST_INIT(linear_func_list_);
      break;
    case MODULE_LIGHT:
      module_ = new LightModule;
      module_->Init();
      FUNC_LIST_INIT(light_func_list_);
      break;
    case MODULE_CNC_TOOL_SETTING:
      module_ = new CncToolSetting;
      module_->Init();
      FUNC_LIST_INIT(tool_setting_func_list_);
      break;
    case MODULE_ENCLOSURE:
      module_ = new EnclosureModule;
      module_->Init();
      FUNC_LIST_INIT(enclosure_func_list_);
      break;
    case MODULE_FAN:
      module_ = new FanModule();
      module_->Init();
      FUNC_LIST_INIT(fan_func_list_);
      break;
     case MODULE_PURIFIER:
      module_ = new PurifierModule();
      module_->Init();
      FUNC_LIST_INIT(purifier_func_list_);
      break;
    case MODULE_EMERGENCY_STOP:
      module_ = new StopModule;
      module_->Init();
      FUNC_LIST_INIT(stop_func_list_);
      break;
    case MODULE_ROTATE:
      module_ = new RotateModule;
      module_->Init();
      break;
  }
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

