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

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_DUAL_EXTRUDER_PRINT_HEAD_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_DUAL_EXTRUDER_PRINT_HEAD_H_

#include <src/device/switch.h>
#include <src/device/temperature.h>
#include <src/device/fan.h>
#include "module_base.h"
#include <src/configuration.h>

#define TOOLHEAD_3DP_EXTRUDER0    (0)
#define TOOLHEAD_3DP_EXTRUDER1    (1)

class DualExtruderPrintHead : public ModuleBase {
 public:
  void Init();
  void PeriphInit();
  void StartNozzleSwitching(uint8_t *data);
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();
  void ReportTemprature();
  void set_probe_sensor(uint8_t sensor_index);

  Fan fan_0_;
  Fan fan_1_;
  Fan fan_2_;
  Fan fan_3_;
  SwitchInput switch_probe_0_;
  SwitchInput switch_probe_1_;
  SwitchInput switch_probe_;
  SwitchInput switch_cut_0_;
  SwitchInput switch_cut_1_;
  SwitchOutput motor_cs_;
  Temperature  temperature_0_;
  Temperature  temperature_1_;

 private:
  uint32_t temp_report_time_ = 0;
  uint32_t cut_report_time_ = 0;
  bool is_report_cut_ = false;
  uint8_t active_nozzle = 0;
  uint8_t target_nozzle = 0;
  bool probe0_status = false;
  bool probe1_status = false;
  bool probe_switch_status = false;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_DUAL_EXTRUDER_PRINT_HEAD_H_
