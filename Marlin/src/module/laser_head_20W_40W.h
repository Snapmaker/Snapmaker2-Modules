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

#ifndef __LASER_HEAD_20W_40W_H_
#define __LASER_HEAD_20W_40W_H_

#include "src/configuration.h"
#include "src/device/switch.h"
#include "src/device/fan_fb.h"
#include "src/device/analog_io_ctrl.h"
#include "module_base.h"
#include "src/device/temperature.h"
#include "src/core/moving_average_filter.h"
#include "laser_hw_version.h"

#define LASER_20W_40W_FAN_PIN                       PA2
#define LASER_20W_40W_ENBLE_PIN                     PA1
#define LASER_20W_40W_TEMP_PIN                      PB1
#define LASER_20W_40W_PWM_DETECT                    PA9
#define LASER_20W_40W_CROSS_LIGHT                   PB5
#define LASER_20W_40W_HW_VERSION_PIN                PB0
#define LASER_20W_40W_FIRE_SENSOR_PIN               PA0
#define LASER_40W_LASER2_OFF_CTRL_PIN               PA10
#define LASER_20W_40W_FIRE_SENSOR_ADC_TIMER         ADC_TIM_4
#define LASER_20W_40W_FIRE_SENSOR_ADC_PERIOD_US     (1000)
#define LASER_FIRE_SENSOR_MAF_SIZE                  (256)   // moving average filter size
#define LASER_FIRE_SENSOR_SAMPLE_FREQ               (100)   // fire sensor sample frequency
#define LASER_FIRE_SENSOR_PRE_CHECK_CNT             (LASER_FIRE_SENSOR_MAF_SIZE * 1.5)

// security info
#define FAULT_IMU_CONNECTION                        (1<<0)
#define FAULT_LASER_TEMP                            (1<<1)
#define FAULT_LASER_GESTURE                         (1<<2)
#define FAULT_LASER_PWM_PIN                         (1<<3)
#define FAULT_LASER_FAN_RUN                         (1<<4)
#define FAULT_FIRE_DECT                             (1<<5)

#define LASER_20W_40W_TEMP_LIMIT                    55
#define LASER_20W_40W_TEMP_RECOVERY                 45
#define FIRE_DETECT_SENSITIVITY_HIGHT               (3)
#define FIRE_DETECT_SENSITIVITY_MID                 (2)
#define FIRE_DETECT_SENSITIVITY_LOW                 (1)
#define FIRE_DETECT_SENSITIVITY_DIS                 (0)
#define FIRE_DETECT_SENSITIVITY_HIGHT_ADC_VALUE     (3000)
#define FIRE_DETECT_SENSITIVITY_MID_ADC_VALUE       (1500)
#define FIRE_DETECT_SENSITIVITY_LOW_ADC_VALUE       (500)
#define FIRE_DETECT_TRIGGER_ADC_VALUE               (500)
#define FIRE_DETECT_TRIGGER_LIMIT_ADC_VALUE         (4095)
#define FIRE_DETECT_TRIGGER_DISABLE_ADC_VALUE       (0xFFFF)

#define LSAER_FAN_FB_IC_TIM                         TIM_2
#define LSAER_FAN_FB_IT_CH                          TIM_IT_CH4
#define LSAER_FAN_FB_CH                             TIM_CH4
#define FAN_FEEDBACK_THRESHOLD                      100

#define LASER_20W_CL_OFFSET_X                       (0)
#define LASER_20W_CL_OFFSET_Y                       (15.1)
#define LASER_40W_CL_OFFSET_X                       (-21.6)
#define LASER_40W_CL_OFFSET_Y                       (0)


class LaserHead20W40W : public ModuleBase {
    public:
      LaserHead20W40W() : ModuleBase(), fire_sensor_maf_(LASER_FIRE_SENSOR_MAF_SIZE)
      {
        roll_min_ = -20;
        roll_max_ = 20;
        pitch_min_ = -20;
        pitch_max_ = 20;
        yaw_ = 0;
        roll_ = 0;
        pitch_ = 0;
        security_status_ = 0;
        security_status_pre_ = 0xff;
        laser_celsius_ = 25;
        sync_id_ = 0xffffffff;
        imu_celsius_ = 25;
        hw_version_.number = 0xAA;
        fire_sensor_maf_last_ms_ = 0;
      }

        void Init();
        void Loop();
        void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
        void EmergencyStop();
        void SecurityStatusCheck();
        void ReportSecurityStatus();
        void LaserSaveFocus(uint8_t type, uint16_t foch);
        void LaserReportFocus(uint8_t type);
        void LaserOnlineStateSync(uint8_t *data);
        void LaserSetProtectTemp(uint8_t *data);
        void LaserCtrl(uint8_t *data);
        void LaserBranchCtrl(bool onoff);
        void LaserReportHWVersion();
        void LaserReportPinState();
        void LaserConfirmPinState();
        void GetHwVersion();
        void LaserSetCrossLight(bool onoff);
        void LaserGetCrossLightState(void);
        void LaserSetFireSensorSensitivity(uint8_t fds, bool need_save=true);
        void LaserSetFireSensorSensitivity(uint16_t fdv, bool need_save=true);
        void LaserGetFireSensorSensitivity(void);
        void LaserSetFireSensorRawDataReportTime(uint16_t rp_itv_ms);
        void LaserReportFireSensorRawData(void);
        void LaserSetCrosslightOffset(float x, float y);
        void LaserGetCrosslightOffset(void);
        void LaserFireSensorReportLoop(void);
        void LaserFireSensorLoop(void);
        void LaserFireSensorDetectFilter(void);
        uint16_t LaserParmChecksumCal(AppParmInfo *param);

        FanFeedBack  fan_;
        SwitchOutput laser_power_ctrl_;
        Temperature  temperature_;
        SwitchInput  pwm_detect_;
        SwitchOutput cross_light_;
        SwitchOutput laser2_off_ctrl_;

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
        int8_t protect_temp_;
        int8_t recovery_temp_;
        int8_t imu_celsius_;
        float crosslight_offset_x_;
        float crosslight_offset_y_;
        uint8_t fire_sensor_adc_index_;
        uint16_t fire_sensor_raw_adc_;
        uint16_t fire_sensor_trigger_value_;
        uint8_t fire_sensor_trigger_;
        uint32_t fire_sensor_raw_data_report_tick_ms_;
        uint32_t fire_sensor_raw_data_report_interval_ms_;
        uint32_t fire_sensor_maf_last_ms_;
        uint32_t fire_sensor_trigger_reset_delay_;
        uint32_t pre_check_cnt_;
        MovingAverage fire_sensor_maf_;
        hw_version_t hw_version_;
};

#endif
