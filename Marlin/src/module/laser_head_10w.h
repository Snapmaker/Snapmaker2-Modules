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

#ifndef __LASER_HEAD_10W_H_
#define __LASER_HEAD_10W_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan.h"
#include "src/device/analog_io_ctrl.h"
#include "module_base.h"
#include "src/device/temperature.h"

#define LASER10W_FAN_PIN                  PA2
#define LASER10W_CAMERA_POWER_PIN         PA8
#define LASER10W_AUTOFOCUS_LIGHT_CTRL_PIN PA15
#define LASER10W_ENBLE_PIN                PA1
#define LASER10W_TEMP_PIN                 PB1

// security info
#define FAULT_IMU_CONNECTION        (1<<0)
#define FAULT_LASER_TEMP            (1<<1)
#define FAULT_LASER_GESTURE         (1<<2)

#define LASER_TEMP_LIMIT    55
#define LASER_TEMP_RECOVERY 45

class LaserHead10W : public ModuleBase {
    public:
        LaserHead10W () : ModuleBase () {
            roll_min_  = -10;
            roll_max_  = 10;
            pitch_min_ = -10;
            pitch_max_ = 10;
            yaw_   = 0;
            roll_  = 0;
            pitch_ = 0;
            security_status_ = 0;
            security_status_pre_ = 0xff;
            laser_celsius_ = 25;
            sync_id_ = 0xffffffff;
        }

        void Init();
        void Loop();
        void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
        void SecurityStatusCheck();
        void SetAutoFocusLight(uint8_t state);
        void ReportSecurityStatus();
        void LaserSaveFocus(uint8_t type, uint16_t foch);
        void LaserReportFocus(uint8_t type);
        void LaserOnlineStateSync(uint8_t *data);

        Fan fan_;
        SwitchOutput camera_power_;
        SwitchOutput autofocus_light_;
        SwitchOutput laser_power_ctrl_;
        SwitchOutput laser_fan_ctrl_;
        Temperature  temperature_;

    private:
        volatile float roll_min_;
        volatile float roll_max_;
        volatile float pitch_min_;
        volatile float pitch_max_;
        float yaw_;
        float roll_;
        float pitch_;
        uint8_t security_status_;
        uint8_t security_status_pre_;
        float laser_celsius_;
        uint32_t sync_id_;
};

#endif
