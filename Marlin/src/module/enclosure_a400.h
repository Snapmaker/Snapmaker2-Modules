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

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_ENCLOSURE_A400_HEAD_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_ENCLOSURE_A400_HEAD_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/module/light_module.h"
#include "src/device/fan.h"
#include "module_base.h"

class EnclosureA400Module : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void EmergencyStop();
  void StartLightEffect();
  void ReportEnclosureStatus();
  void ReportConfigResult(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void EnclosureReportHWVersion();

 private:
  SwitchInput hall_switch_1_;
  SwitchInput hall_switch_2_;
  LightModule light_;
  Fan fan_;

  uint8_t is_report_ = 0;
  uint8_t door_1_sta_ = 0;
  uint8_t door_2_sta_ = 0;
  uint8_t start_flag_ = 1;
  uint8_t light_out_ = 0;
  uint8_t light_limit_sta_ = 1;
  uint8_t adc_index_;

  uint32_t door_1_check_time_ = 0;
  uint32_t door_2_check_time_ = 0;
  uint32_t light_limit_time_;
  uint32_t light_next_time_;
  uint32_t loop_next_time_;
};
#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_ENCLOSURE_A400_HEAD_H_
