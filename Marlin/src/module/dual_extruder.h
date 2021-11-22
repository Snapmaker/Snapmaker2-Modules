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

#ifndef __DUAL_EXTRUDER_H_
#define __DUAL_EXTRUDER_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "src/device/analog_io_ctrl.h"
#include "module_base.h"
#include "src/device/temperature.h"
#include "../device/nozzle_identify.h"

#define CAN_DATA_FRAME_LENGTH     (8)
#define TOOLHEAD_3DP_EXTRUDER0    (0)
#define TOOLHEAD_3DP_EXTRUDER1    (1)
#define INVALID_EXTRUDER          0xFF
#define EXTRUDER_SWITCH_TIME      5000

#define PROBE_PROXIMITY_SWITCH_PIN              PB6
#define PROBE_LEFT_EXTRUDER_OPTOCOUPLER_PIN     PA8
#define PROBE_RIGHT_EXTRUDER_OPTOCOUPLER_PIN    PA1
#define PROBE_LEFT_EXTRUDER_CONDUCTIVE_PIN      PA4
#define PROBE_RIGHT_EXTRUDER_CONDUCTIVE_PIN     PA4
#define OUT_OF_MATERIAL_DETECT_0_PIN            PA10
#define OUT_OF_MATERIAL_DETECT_1_PIN            PA2
#define EXTRUDER_0_CS_PIN                       PB11
#define EXTRUDER_1_CS_PIN                       PB7
#define TEMP_0_PIN                              PB1
#define TEMP_1_PIN                              PA3
#define HEATER_0_PIN                            PA9
#define HEATER_1_PIN                            PA0
#define NOZZLE_ID_0_PIN                         PA6
#define NOZZLE_ID_1_PIN                         PA5
#define LEFT_MODEL_FAN_PIN                      PA7
#define RIGHT_MODEL_FAN_PIN                     PB2
#define NOZZLE_FAN_PIN                          PB0

typedef enum {
    LEFT_MODEL_FAN,
    RIGHT_MODEL_FAN,
    NOZZLE_FAN
}fan_e;

typedef enum {
    EXTRUDER_STATUS_NORMAL,
    EXTRUDER_STATUS_SWITCHING,
    EXTRUDER_STATUS_REPORT_ERROR,
    EXTRUDER_STATUS_HALT
}extruder_status_e;

class DualExtruder : public ModuleBase {
    public:
        DualExtruder () {
            active_extruder_    = TOOLHEAD_3DP_EXTRUDER0;
            nozzle_check_time_  = 0;
            temp_report_time_   = 0;
            extruder_status_    = EXTRUDER_STATUS_NORMAL;
            extruder_switching_time_elapse_ = 0;
        }
        void Init();
        void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
        void ReportOutOfMaterial();
        void ReportProbe();
        void FanCtrl(fan_e fan, uint8_t duty_cycle, uint16_t delay_sec_kill);
        void SetTemperature(uint8_t *data);
        void ReportTemprature();
        void ActiveExtruder(uint8_t extruder);
        void ExtruderStatusCheck();
        void ExtruderSwitching(uint8_t *data);
        void ReportNozzleType();
        void ReportExtruderInfo();
        void EmergencyStop();
        void Loop();

        Fan left_model_fan_;
        Fan right_model_fan_;
        Fan nozzle_fan_;
        SwitchInput probe_proximity_switch_;               // proximity switch sensor
        SwitchInput probe_left_extruder_optocoupler_;      // left extruder optocoupler sensor
        SwitchInput probe_right_extruder_optocoupler_;     // right extruder optocoupler sensor
        SwitchInput probe_left_extruder_conductive_;       // left extruder conductive sensor
        SwitchInput probe_right_extruder_conductive_;      // right extruder conductive sensor
        SwitchInput out_of_material_detect_0_;
        SwitchInput out_of_material_detect_1_;
        SwitchOutput extruder_cs_0_;
        SwitchOutput extruder_cs_1_;
        Temperature temperature_0_;
        Temperature temperature_1_;
        NozzleIdentify nozzle_identify_0_;
        NozzleIdentify nozzle_identify_1_;

    private:
        uint32_t temp_report_time_;
        uint8_t active_extruder_;
        uint8_t target_extruder_;
        uint32_t nozzle_check_time_;
        extruder_status_e extruder_status_;
        uint32_t extruder_switching_time_elapse_;
};


#endif
