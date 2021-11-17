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
#include <wirish_time.h>
#include "../registry/route.h"
#include <src/HAL/hal_tim.h>
#include <math.h>
#include "dual_extruder.h"

void DualExtruder::Init() {
    probe_proximity_switch_.Init(PROBE_PROXIMITY_SWITCH_PIN);
    probe_left_extruder_optocoupler_.Init(PROBE_LEFT_EXTRUDER_OPTOCOUPLER_PIN, true, INPUT_PULLUP);
    probe_right_extruder_optocoupler_.Init(PROBE_RIGHT_EXTRUDER_OPTOCOUPLER_PIN, true, INPUT_PULLUP);
    probe_left_extruder_conductive_.Init(PROBE_LEFT_EXTRUDER_CONDUCTIVE_PIN, true, INPUT_PULLUP);
    probe_right_extruder_conductive_.Init(PROBE_RIGHT_EXTRUDER_CONDUCTIVE_PIN, true, INPUT_PULLUP);
    out_of_material_detect_0_.Init(out_of_material_detect_0_PIN);
    out_of_material_detect_1_.Init(out_of_material_detect_1_PIN);
    extruder_cs_0_.Init(EXTRUDER_0_CS_PIN, 0, OUTPUT);
    extruder_cs_1_.Init(EXTRUDER_1_CS_PIN, 1, OUTPUT);
    temperature_0_.InitCapture(TEMP_0_PIN, ADC_TIM_4);
    temperature_0_.InitOutCtrl(PWM_TIM1, PWM_CH2, HEATER_0_PIN);
    temperature_1_.InitCapture(TEMP_1_PIN, ADC_TIM_4);
    temperature_1_.InitOutCtrl(PWM_TIM2, PWM_CH1, HEATER_1_PIN);
    nozzle_identify_0_.Init(NOZZLE_ID_0_PIN, ADC_TIM_4);
    nozzle_identify_1_.Init(NOZZLE_ID_1_PIN, ADC_TIM_4);
    left_model_fan_.Init(LEFT_MODEL_FAN_PIN);
    right_model_fan_.Init(RIGHT_MODEL_FAN_PIN);
    nozzle_fan_.Init(NOZZLE_FAN_PIN);
}

void DualExtruder::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
    switch ((uint32_t)func_id) {
        case FUNC_REPORT_CUT:
            ReportOutOfMaterial();
            break;
        case FUNC_REPORT_PROBE:
            ReportProbe();
            break;
        case FUNC_SET_FAN:
            FanCtrl(LEFT_MODEL_FAN, data[1], data[0]);
            break;
        case FUNC_SET_FAN2:
            FanCtrl(RIGHT_MODEL_FAN, data[1], data[0]);
            break;
        case FUNC_SET_FAN_NOZZLE:
            FanCtrl(NOZZLE_FAN, data[1], data[0]);
            break;
        case FUNC_SET_TEMPEARTURE:
            SetTemperature(data);
            break;
        case FUNC_REPORT_TEMPEARTURE:
            ReportTemprature();
            break;
        case FUNC_REPORT_TEMP_PID:
            break;
        case FUNC_SET_PID:
            break;
        case FUNC_SWITCH_EXTRUDER:
            ExtruderSwitching(data);
            break;
        case FUNC_REPORT_NOZZLE_TYPE:
            ReportNozzleType();
            break;
        case FUNC_REPORT_EXTRUDER_INFO:
            ReportExtruderInfo();
            break;
        default:
            break;
    }
}

void DualExtruder::ReportOutOfMaterial() {
    uint8_t buf[CAN_DATA_FRAME_LENGTH];
    uint8_t index = 0;
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_CUT);
    if (msgid != INVALID_VALUE) {
        buf[index++] = out_of_material_detect_0_.Read();
        buf[index++] = out_of_material_detect_1_.Read();
        canbus_g.PushSendStandardData(msgid, buf, index);
    }
}

void DualExtruder::ReportProbe() {
    uint8_t buf[CAN_DATA_FRAME_LENGTH];
    uint8_t index = 0;
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PROBE);
    if (msgid != INVALID_VALUE) {
        buf[index++] = probe_proximity_switch_.Read();
        buf[index++] = probe_left_extruder_optocoupler_.Read();
        buf[index++] = probe_right_extruder_optocoupler_.Read();
        buf[index++] = probe_left_extruder_conductive_.Read();
        buf[index++] = probe_right_extruder_conductive_.Read();
        canbus_g.PushSendStandardData(msgid, buf, index);
    }
}

void DualExtruder::FanCtrl(fan_e fan, uint8_t duty_cycle, uint16_t delay_sec_kill) {
    switch (fan) {
        case LEFT_MODEL_FAN:
            left_model_fan_.ChangePwm(duty_cycle, delay_sec_kill);
            break;
        case RIGHT_MODEL_FAN:
            right_model_fan_.ChangePwm(duty_cycle, delay_sec_kill);
            break;
        case NOZZLE_FAN:
            nozzle_fan_.ChangePwm(duty_cycle, delay_sec_kill);
            break;
        default:
            break;
    }
}

void DualExtruder::SetTemperature(uint8_t *data) {
    temperature_0_.ChangeTarget(data[0] << 8 | data[1]);
    temperature_1_.ChangeTarget(data[2] << 8 | data[3]);
}

void DualExtruder::ReportTemprature() {
    int16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMPEARTURE);
    if (msgid != INVALID_VALUE) {
        uint16_t temp, target;
        uint8_t buf[CAN_DATA_FRAME_LENGTH];
        uint8_t index = 0;
        temp = temperature_0_.GetCurTemprature();
        target = temperature_0_.GetTargetTemprature();
        buf[index++] = temp >> 8;
        buf[index++] = temp;
        buf[index++] = target >> 8;
        buf[index++] = target;
        temp = temperature_1_.GetCurTemprature();
        target = temperature_1_.GetTargetTemprature();
        buf[index++] = temp >> 8;
        buf[index++] = temp;
        buf[index++] = target >> 8;
        buf[index++] = target;
        canbus_g.PushSendStandardData(msgid, buf, index);
    }
}

void DualExtruder::ActiveExtruder(uint8_t extruder) {
    if (extruder == TOOLHEAD_3DP_EXTRUDER0) {
        extruder_cs_0_.Out(0);
        extruder_cs_1_.Out(1);
    } else if (extruder == TOOLHEAD_3DP_EXTRUDER1) {
        extruder_cs_0_.Out(1);
        extruder_cs_1_.Out(0);
    }
}

void DualExtruder::ExtruderStatusCheck() {
    uint8_t left_extruder_status;
    uint8_t right_extruder_status;

    switch (extruder_status_) {
        case EXTRUDER_STATUS_NORMAL:
            left_extruder_status = probe_left_extruder_optocoupler_.Read();
            right_extruder_status = probe_right_extruder_optocoupler_.Read();
            if (left_extruder_status == 1 && right_extruder_status == 0) {
                active_extruder_ = TOOLHEAD_3DP_EXTRUDER0;
            } else if (left_extruder_status == 0 && right_extruder_status == 1) {
                active_extruder_ = TOOLHEAD_3DP_EXTRUDER1;
            } else {
                active_extruder_ = INVALID_EXTRUDER;
            }

            if (active_extruder_ != target_extruder_) {
                extruder_status_ = EXTRUDER_STATUS_REPORT_ERROR;
            }
            break;
        case EXTRUDER_STATUS_SWITCHING:
            if (extruder_switching_time_elapse_ + EXTRUDER_SWITCH_TIME < millis()) {
                extruder_status_ = EXTRUDER_STATUS_NORMAL;
            }
            break;
        case EXTRUDER_STATUS_REPORT_ERROR:
            extruder_status_ = EXTRUDER_STATUS_HALT;
            ReportExtruderInfo();
            break;
        case EXTRUDER_STATUS_HALT:
            // wait main ctrl to solve the problem
            break;
        default:
            break;
    }
}

void DualExtruder::ExtruderSwitching(uint8_t *data) {
    if (target_extruder_ != data[0]) {
        extruder_status_ = EXTRUDER_STATUS_SWITCHING;
        extruder_switching_time_elapse_ = millis();
        target_extruder_ = data[0];
    }

    uint8_t buf[CAN_DATA_FRAME_LENGTH];
    uint8_t index = 0;
    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_SWITCH_EXTRUDER);
    if (msgid != INVALID_VALUE) {
        buf[index++] = target_extruder_;
        canbus_g.PushSendStandardData(msgid, buf, index);
    }
}

void DualExtruder::ReportNozzleType() {
    uint8_t buf[CAN_DATA_FRAME_LENGTH];
    uint8_t index = 0;

    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_NOZZLE_TYPE);
    if (msgid != INVALID_VALUE) {
        buf[index++] = (uint8_t)nozzle_identify_0_.GetNozzleType();
        buf[index++] = (uint8_t)nozzle_identify_0_.GetNozzleType();
        canbus_g.PushSendStandardData(msgid, buf, index);
    }
}

void DualExtruder::ReportExtruderInfo() {
    uint8_t buf[CAN_DATA_FRAME_LENGTH];
    uint8_t index = 0;

    uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_EXTRUDER_INFO);
    if (msgid != INVALID_VALUE) {
        buf[index++] = active_extruder_ == target_extruder_ ? 0 : 1;
        buf[index++] = active_extruder_;
        canbus_g.PushSendStandardData(msgid, buf, index);
    }
}

void DualExtruder::EmergencyStop() {
    temperature_0_.ChangeTarget(0);
    temperature_1_.ChangeTarget(0);
    left_model_fan_.ChangePwm(0, 0);
    right_model_fan_.ChangePwm(0, 0);
    nozzle_fan_.ChangePwm(0, 0);
    extruder_cs_0_.Out(1);
    extruder_cs_1_.Out(1);
}

void DualExtruder::Loop() {
    if (hal_adc_status()) {
        temperature_0_.TemperatureOut(THERMISTOR_PT100);
        temperature_1_.TemperatureOut(THERMISTOR_PT100);
        if (nozzle_check_time_ + 500 < millis()) {
            nozzle_check_time_ = millis();
            nozzle_identify_0_.IdentifyProcess(0);
            nozzle_identify_1_.IdentifyProcess(1);
        }
    }

    if (temp_report_time_ + 500 < millis()) {
        temp_report_time_ = millis();
        ReportTemprature();
    }

    if (out_of_material_detect_0_.CheckStatusLoop() || out_of_material_detect_1_.CheckStatusLoop()) {
        ReportOutOfMaterial();
    }

    if (probe_proximity_switch_.Read() || probe_left_extruder_optocoupler_.Read() || probe_right_extruder_optocoupler_.Read() || probe_left_extruder_conductive_.Read() || probe_right_extruder_conductive_.Read()) {
        ReportProbe();
    }

    ExtruderStatusCheck();

    left_model_fan_.Loop();
    right_model_fan_.Loop();
    nozzle_fan_.Loop();
}
