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
#include "../device/soft_pwm.h"

#define NTC3950_ADC_MIN 168
#define NTC3950_ADC_MAX 417

#define Z_MAX_POS                      5.0
#define STEPPER_TIMER                  3
// #define Z_AXIS_STEPS_PER_UNIT          1600         // 2mm/r
#define Z_AXIS_STEPS_PER_UNIT          3200         // 1mm/r
#define ACCELERATION                   40


#define DEFAULT_PID_P (150)
#define DEFAULT_PID_I (1)
#define DEFAULT_PID_D (30000)

#define DEFAULT_HOTEND_OFFSET_X   (26)
#define DEFAULT_HOTEND_OFFSET_Y   (0)
#define DEFAULT_HOTEND_OFFSET_Z   (-1.5)

#define DEFAULT_SENSOR_COMPENSATION_L (0.8)
#define DEFAULT_SENSOR_COMPENSATION_R (0.8)

static DualExtruder * dual_extruder_p;

static void StepperTimerCallback() {
  dual_extruder_p->Stepper();
}

void DualExtruder::Init() {
  AppParmInfo *param = &registryInstance.cfg_;;

  dual_extruder_p = this;

  probe_proximity_switch_.Init(PROBE_PROXIMITY_SWITCH_PIN);
  probe_left_extruder_optocoupler_.Init(PROBE_LEFT_EXTRUDER_OPTOCOUPLER_PIN);
  probe_right_extruder_optocoupler_.Init(PROBE_RIGHT_EXTRUDER_OPTOCOUPLER_PIN);
  // probe_left_extruder_conductive_.Init(PROBE_LEFT_EXTRUDER_CONDUCTIVE_PIN);
  // probe_right_extruder_conductive_.Init(PROBE_RIGHT_EXTRUDER_CONDUCTIVE_PIN);
  out_of_material_detect_0_.Init(OUT_OF_MATERIAL_DETECT_0_PIN, true, INPUT_PULLUP);
  out_of_material_detect_1_.Init(OUT_OF_MATERIAL_DETECT_1_PIN, true, INPUT_PULLUP);
  extruder_cs_0_.Init(EXTRUDER_0_CS_PIN, 1, OUTPUT);
  extruder_cs_1_.Init(EXTRUDER_1_CS_PIN, 0, OUTPUT);
  left_model_fan_.Init(LEFT_MODEL_FAN_PIN, 100);
  right_model_fan_.Init(RIGHT_MODEL_FAN_PIN, 100);
  nozzle_fan_.Init(NOZZLE_FAN_PIN, 100);
  proximity_power_.Init(PROXIMITY_SWITCH_PIN, 0, OUTPUT);
  z_motor_cur_ctrl_.Init(LIFT_MOTOR_CUR_CTRL_PIN, 1, OUTPUT);

  z_motor_dir_.Init(LIFT_MOTOR_DIR_PIN, 0, OUTPUT);
  z_motor_step_.Init(LIFT_MOTOR_STEP_PIN, 0, OUTPUT);
  z_motor_en_.Init(LIFT_MOTOR_ENABLE_PIN, 0, OUTPUT);

  uint8_t adc_index0_temp, adc_index0_identify, adc_index1_temp, adc_index1_identify;

  if (param->parm_mark[0] != 0xaa || param->parm_mark[1] != 0x55) {
    param->temp_P = DEFAULT_PID_P;
    param->temp_I = DEFAULT_PID_I;
    param->temp_D = DEFAULT_PID_D;

    param->x_hotend_offset = DEFAULT_HOTEND_OFFSET_X;
    param->y_hotend_offset = DEFAULT_HOTEND_OFFSET_Y;
    param->z_hotend_offset = DEFAULT_HOTEND_OFFSET_Z;

    param->probe_sensor_compensation_0 = DEFAULT_SENSOR_COMPENSATION_L;
    param->probe_sensor_compensation_1 = DEFAULT_SENSOR_COMPENSATION_R;

    registryInstance.SaveCfg();
  }

  adc_index0_temp = temperature_0_.InitCapture(TEMP_0_PIN, ADC_TIM_4);
  temperature_0_.SetThermistorType(THERMISTOR_PT100);
  temperature_0_.InitOutCtrl(PWM_TIM1, PWM_CH2, HEATER_0_PIN);
  adc_index1_temp  = temperature_1_.InitCapture(TEMP_1_PIN, ADC_TIM_4);
  temperature_1_.SetThermistorType(THERMISTOR_PT100);
  temperature_1_.InitOutCtrl(PWM_TIM2, PWM_CH1, HEATER_1_PIN);

  adc_index0_identify = nozzle_identify_0_.Init(NOZZLE_ID_0_PIN, ADC_TIM_4);
  adc_index1_identify = nozzle_identify_1_.Init(NOZZLE_ID_1_PIN, ADC_TIM_4);

  hw_ver_.Init(HW_VERSION_ADC_PIN, ADC_TIM_4);

  hal_start_adc();

  temperature_0_.SetAdcIndex(adc_index0_temp);
  temperature_0_.SetThermistorType(THERMISTOR_PT100);
  nozzle_identify_0_.SetAdcIndex(adc_index0_identify);
  nozzle_identify_0_.SetNozzleTypeCheckArray(THERMISTOR_PT100);

  temperature_1_.SetAdcIndex(adc_index1_temp);
  temperature_1_.SetThermistorType(THERMISTOR_PT100);
  nozzle_identify_1_.SetAdcIndex(adc_index1_identify);
  nozzle_identify_1_.SetNozzleTypeCheckArray(THERMISTOR_PT100);
}

void DualExtruder::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  float val = 0.0;
  switch ((uint32_t)func_id) {
    case FUNC_REPORT_CUT:
      ReportOutOfMaterial();
      break;
    case FUNC_REPORT_PROBE:
      ReportProbe();
      break;
    case FUNC_SET_FAN:
      FanCtrl(LEFT_MODEL_FAN, data[1]*100 / 255, data[0]);
      FanCtrl(RIGHT_MODEL_FAN, data[1]*100 / 255, data[0]);
      break;
    case FUNC_SET_FAN2:
      FanCtrl(RIGHT_MODEL_FAN, data[1]*100 / 255, data[0]);
      FanCtrl(LEFT_MODEL_FAN, data[1]*100 / 255, data[0]);
      break;
    case FUNC_SET_FAN_NOZZLE:
      FanCtrl(NOZZLE_FAN, data[1]*100 / 255, data[0]);
      break;
    case FUNC_SET_TEMPEARTURE:
      SetTemperature(data);
      break;
    case FUNC_REPORT_TEMPEARTURE:
      ReportTemprature();
      break;
    case FUNC_REPORT_TEMP_PID:
      temperature_0_.ReportPid();
      break;
    case FUNC_SET_PID:
      val = (float)(((data[1]) << 24) | ((data[2]) << 16) | ((data[3]) << 8 | (data[4]))) / 1000;
      temperature_0_.SetPID(data[0], val);
      break;
    case FUNC_SWITCH_EXTRUDER:
      ExtruderSwitcingWithMotor(data);
      break;
    case FUNC_REPORT_NOZZLE_TYPE:
      ReportNozzleType();
      break;
    case FUNC_REPORT_EXTRUDER_INFO:
      ReportExtruderInfo();
      break;
    case FUNC_SET_EXTRUDER_CHECK:
      ExtruderStatusCheckCtrl((extruder_status_e)data[0]);
      break;
    case FUNC_SET_HOTEND_OFFSET:
      SetHotendOffset(data);
      break;
    case FUNC_REPORT_HOTEND_OFFSET:
      ReportHotendOffset();
      break;
    case FUNC_SET_PROBE_SENSOR_COMPENSATION:
      SetProbeSensorCompensation(data);
      break;
    case FUNC_REPORT_PROBE_SENSOR_COMPENSATION:
      ReportProbeSensorCompensation();
      break;
    case FUNC_MOVE_TO_DEST:
      MoveToDestination(data);
      break;
    case FUNC_SET_RIGHT_EXTRUDER_POS:
      SetRightExtruderPos(data);
      break;
    case FUNC_REPORT_RIGHT_EXTRUDER_POS:
      ReportRightExtruderPos();
      break;
    case FUNC_PROXIMITY_SWITCH_POWER_CTRL:
      ProximitySwitchPowerCtrl(data[0]);
      break;
    case FUNC_MODULE_GET_HW_VERSION:
      ReportHWVersion();
      break;
    default:
      break;
  }
}

void DualExtruder::Stepper() {
  if (end_stop_enable_ == true) {
    if (probe_right_extruder_optocoupler_.Read()) {
      hit_state_ = 1;
      stepps_count_ = 0;
      stepps_sum_   = 0;
      motor_state_  = 0;
      z_motor_step_.Out(0);
      z_motor_en_.Out(0);
      z_motor_cur_ctrl_.Out(1);
      StepperTimerStop();
      return;
    }
  }

  while (stepps_count_ == speed_ctrl_buffer_[speed_ctrl_index_].pulse_count) {
    if (speed_ctrl_index_ != 19) {
      speed_ctrl_index_++;
      StepperTimerStop();
      StepperTimerStart(speed_ctrl_buffer_[speed_ctrl_index_].timer_time);
    } else {
      stepps_count_ = 0;
      stepps_sum_   = 0;
      motor_state_  = 0;
      z_motor_step_.Out(0);
      z_motor_en_.Out(0);
      z_motor_cur_ctrl_.Out(1);
      StepperTimerStop();
      return;
    }
  }

  if (step_pin_state_ == 0) {
    step_pin_state_ = 1;
    z_motor_step_.Out(1);
    stepps_count_++;
  } else {
    step_pin_state_ = 0;
    z_motor_step_.Out(0);

    if (stepps_count_ == stepps_sum_) {
      stepps_count_ = 0;
      stepps_sum_   = 0;
      motor_state_  = 0;
      z_motor_en_.Out(0);
      z_motor_cur_ctrl_.Out(1);
      StepperTimerStop();
    }
  }
}

void DualExtruder::StepperTimerStart(uint16_t time) {
  HAL_timer_disable(STEPPER_TIMER);

  HAL_timer_init(STEPPER_TIMER, 72, time);
  HAL_timer_nvic_init(STEPPER_TIMER, 3, 3);
  HAL_timer_cb_init(STEPPER_TIMER, StepperTimerCallback);
  HAL_timer_enable(STEPPER_TIMER);
}

void DualExtruder::StepperTimerStop() {
  HAL_timer_disable(STEPPER_TIMER);
  soft_pwm_g.TimStart();
}

void DualExtruder::MoveSync() {
  while(motor_state_) {
    canbus_g.Handler();
    registryInstance.ConfigHandler();
    registryInstance.SystemHandler();
    routeInstance.ModuleLoop();
  }
}

move_state_e DualExtruder::GoHome() {
  extruder_check_status_ = EXTRUDER_STATUS_IDLE;
  move_state_e move_state = MOVE_STATE_SUCCESS;

  // if endstop triggered, leave current position
  uint32_t i = 0;
  for (i = 0; i < 4; i++) {
    if (digitalRead(PROBE_RIGHT_EXTRUDER_OPTOCOUPLER_PIN)) {
      DoBlockingMoveToZ(-2, 9);
      MoveSync();
    } else {
      DoBlockingMoveToZ(-0.2, 9);
      MoveSync();
      break;
    }
  }

  if (i >= 4) {
    move_state = MOVE_STATE_FAIL;
    goto EXIT;
  }

  end_stop_enable_ = true;
  DoBlockingMoveToZ(9, 9);
  MoveSync();
  if (hit_state_ == 1) {
    hit_state_ = 0;
  } else {
    move_state = MOVE_STATE_FAIL;
    goto EXIT;
  }
  end_stop_enable_ = false;

  // bump
  DoBlockingMoveToZ(-1, 9);
  MoveSync();

  end_stop_enable_ = true;
  DoBlockingMoveToZ(1.5, 1);
  MoveSync();
  if (hit_state_ == 1) {
    hit_state_ = 0;
  } else {
    move_state = MOVE_STATE_FAIL;
    goto EXIT;
  }
  end_stop_enable_ = false;

  // go to the home position
  DoBlockingMoveToZ(-raise_for_home_pos_, 6);
  MoveSync();

  homed_state_ = 1;
  current_position_ = 0;
  extruder_check_status_ = EXTRUDER_STATUS_CHECK;

  active_extruder_ = TOOLHEAD_3DP_EXTRUDER0;
  target_extruder_ = TOOLHEAD_3DP_EXTRUDER0;
  ActiveExtruder(TOOLHEAD_3DP_EXTRUDER0);

EXIT:
    return move_state;
}

void DualExtruder::MoveToDestination(uint8_t *data) {
  move_type_t move_type = (move_type_t)data[0];
  move_state_e move_state = MOVE_STATE_SUCCESS;

  switch (move_type) {
    case GO_HOME:
      move_state = GoHome();
      break;
    case MOVE_SYNC:
      break;
    case MOVE_ASYNC:
      break;
  }

  uint8_t buf[8], index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MOVE_TO_DEST);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (uint8_t)move_type;
    buf[index++] = (uint8_t)move_state;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DualExtruder::PrepareMoveToDestination(float position, float speed) {
  if (position > Z_MAX_POS) {
    position = Z_MAX_POS;
  } else if (position < 0) {
    position = 0;
  }

  DoBlockingMoveToZ(position - current_position_, speed);
  current_position_ = position;
}

// relative motion, the range of motion will not be checked here
void DualExtruder::DoBlockingMoveToZ(float length, float speed) {
  if (motor_state_) {
    return;
  }

  speed_ctrl_index_ = 0;

  float length_tmp = length;

  // set motor rotation direction
  if (length < 0) {
    z_motor_dir_.Out(0);
    length = -length;
  }
  else {
    z_motor_dir_.Out(1);
  }

  // convert motion distance to number of pulses
  stepps_sum_ = Z_AXIS_STEPS_PER_UNIT * length + 0.5;
  uint32_t half_stepps_sum = stepps_sum_ / 2;

  // calculate the number of pulses needed to accelerate and decelerate to the target speed
  uint32_t acc_dec_stepps = ((speed * speed) / (2 * ACCELERATION)) * Z_AXIS_STEPS_PER_UNIT;
  float acc_dec_time;
  float acc_dec_time_quantum;

  if (acc_dec_stepps <= half_stepps_sum) {
    // calculation of acceleration and deceleration time
    acc_dec_time = speed / ACCELERATION;
    acc_dec_time_quantum = acc_dec_time / 10;
    float acc_time = acc_dec_time_quantum;

    // calculate the number of pulses to be traveled for each portion of the acceleration process
    for (int32_t i = 0; i < 10; i++) {
      speed_ctrl_buffer_[i].pulse_count = (ACCELERATION * (acc_time * acc_time) / 2) * Z_AXIS_STEPS_PER_UNIT;
      speed_ctrl_buffer_[i].timer_time  = 1000000 / (ACCELERATION * acc_time * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // calculate the number of pulses that need to go for each share of time for uniform process
    speed_ctrl_buffer_[9].pulse_count = stepps_sum_ - acc_dec_stepps;

    // calculate the number of pulses to be traveled for each portion of the decleration process
    acc_time = acc_dec_time_quantum;
    for (int32_t i = 0; i < 10; i++) {
      speed_ctrl_buffer_[10+i].pulse_count = speed_ctrl_buffer_[9].pulse_count + (speed*acc_time - ACCELERATION*(acc_time * acc_time)/2) * Z_AXIS_STEPS_PER_UNIT;
      speed_ctrl_buffer_[10+i].timer_time  = 1000000 / ((speed - ACCELERATION * (acc_time-acc_dec_time_quantum)) * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // just in case
    speed_ctrl_buffer_[19].pulse_count = stepps_sum_;
  } else {
    // calculation of acceleration and deceleration time
    acc_dec_time = sqrt(length/ACCELERATION);
    acc_dec_time_quantum = acc_dec_time / 10;
    float acc_time = acc_dec_time_quantum;
    float velocity = ACCELERATION * acc_dec_time;

    // calculate the number of pulses to be traveled for each portion of the acceleration process
    for (int32_t i = 0; i < 10; i++) {
      speed_ctrl_buffer_[i].pulse_count = (ACCELERATION * (acc_time * acc_time) / 2) * Z_AXIS_STEPS_PER_UNIT;
      speed_ctrl_buffer_[i].timer_time  = 1000000 / (ACCELERATION * acc_time * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // calculate the number of pulses to be traveled for each portion of the decleration process
    acc_time = acc_dec_time_quantum;
    for (int32_t i = 0; i < 10; i++) {
      speed_ctrl_buffer_[10+i].pulse_count = speed_ctrl_buffer_[9].pulse_count + (velocity*acc_time - ACCELERATION*(acc_time * acc_time)/2) * Z_AXIS_STEPS_PER_UNIT;
      speed_ctrl_buffer_[10+i].timer_time  = 1000000 / ((velocity - ACCELERATION * (acc_time-acc_dec_time_quantum)) * Z_AXIS_STEPS_PER_UNIT);
      acc_time += acc_dec_time_quantum;
    }

    // just in case
    speed_ctrl_buffer_[19].pulse_count = stepps_sum_;
  }

  // wakeup
  motor_state_ = 1;
  z_motor_cur_ctrl_.Out(0);
  z_motor_en_.Out(0);
  current_position_ += length_tmp;
  StepperTimerStart(speed_ctrl_buffer_[0].timer_time);
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
    // buf[index++] = probe_left_extruder_conductive_.Read();
    // buf[index++] = probe_right_extruder_conductive_.Read();
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DualExtruder::FanCtrl(fan_e fan, uint8_t duty_cycle, uint16_t delay_sec_kill) {
  switch (fan) {
    case LEFT_MODEL_FAN:
      if ((duty_cycle > 0) && (duty_cycle < 153)) {
        duty_cycle = 153;
      }
      left_model_fan_.ChangePwm(duty_cycle, delay_sec_kill);
      break;
    case RIGHT_MODEL_FAN:
      if ((duty_cycle > 0) && (duty_cycle < 153)) {
        duty_cycle = 153;
      }
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

#define ERR_OVERTEMP_BIT_MASK         (0)
#define ERR_INVALID_NOZZLE_BIT_MASK   (1)

void DualExtruder::ReportTemprature() {
  int16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_TEMPEARTURE);
  if (msgid != INVALID_VALUE) {
    uint16_t temp;
    uint8_t temp_error = 0;
    uint8_t buf[CAN_DATA_FRAME_LENGTH];
    uint8_t index = 0;

    if (nozzle_identify_0_.GetNozzleType() == NOZZLE_TYPE_INVALID) {
      temp_error |= (1<<ERR_INVALID_NOZZLE_BIT_MASK);
    }

    temp = temperature_0_.GetCurTemprature();

    if (temp >= PROTECTION_TEMPERATURE*10) {
      temperature_0_.ChangeTarget(0);
      temp = 0;
      temp_error |= (1<<ERR_OVERTEMP_BIT_MASK);
    }
    buf[index++] = temp >> 8;
    buf[index++] = temp;
    buf[index++] = temp_error;
    buf[index++] = temp_error;

    temp_error = 0;
    if (nozzle_identify_1_.GetNozzleType() == NOZZLE_TYPE_INVALID) {
      temp_error |= (1<<ERR_INVALID_NOZZLE_BIT_MASK);
    }

    temp = temperature_1_.GetCurTemprature();

    if (temp >= PROTECTION_TEMPERATURE*10) {
      temperature_1_.ChangeTarget(0);
      temp = 0;
      temp_error |= (1<<ERR_OVERTEMP_BIT_MASK);
    }
    buf[index++] = temp >> 8;
    buf[index++] = temp;
    buf[index++] = temp_error;
    buf[index++] = temp_error;

    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DualExtruder::ActiveExtruder(uint8_t extruder) {
  if (extruder == TOOLHEAD_3DP_EXTRUDER0) {
    extruder_cs_0_.Out(1);
    extruder_cs_1_.Out(0);
  } else if (extruder == TOOLHEAD_3DP_EXTRUDER1) {
    extruder_cs_0_.Out(0);
    extruder_cs_1_.Out(1);
  }
}

void DualExtruder::ExtruderStatusCheckCtrl(extruder_status_e status) {
  if (status > EXTRUDER_STATUS_IDLE) {
    return;
  }

  extruder_check_status_ = status;
}

void DualExtruder::ExtruderStatusCheck() {
  uint8_t left_extruder_status;
  uint8_t right_extruder_status;

  switch (extruder_check_status_) {
    case EXTRUDER_STATUS_CHECK:
      left_extruder_status = probe_left_extruder_optocoupler_.Read();
      right_extruder_status = probe_right_extruder_optocoupler_.Read();
      if (left_extruder_status == 1 && right_extruder_status == 0) {
        active_extruder_ = TOOLHEAD_3DP_EXTRUDER0;
      } else if (left_extruder_status == 1 && right_extruder_status == 1) {
        active_extruder_ = TOOLHEAD_3DP_EXTRUDER1;
      } else {
        active_extruder_ = INVALID_EXTRUDER;
      }

      if ((active_extruder_ != target_extruder_) && (extruder_status_ == true)) {
        need_to_report_extruder_info_ = true;
        extruder_status_ = false;
      } else if ((active_extruder_ == target_extruder_) && (extruder_status_ == false)) {
        need_to_report_extruder_info_ = true;
        extruder_status_ = true;
      }

      if (need_to_report_extruder_info_ == true) {
        need_to_report_extruder_info_ = false;
        ReportExtruderInfo();
      }

      break;

    case EXTRUDER_STATUS_IDLE:
      need_to_report_extruder_info_ = false;
      extruder_status_ = true;
      break;

    default:
      break;
  }
}

void DualExtruder::ExtruderSwitching(uint8_t *data) {
  target_extruder_ = data[0];
  ActiveExtruder(target_extruder_);
}

void DualExtruder::ExtruderSwitcingWithMotor(uint8_t *data) {
  target_extruder_ = data[0];
  ActiveExtruder(target_extruder_);

  // won't move extruder if didn't home
  if (homed_state_) {
    extruder_check_status_ = EXTRUDER_STATUS_IDLE;
    if (target_extruder_ == 1) {
      PrepareMoveToDestination(z_max_position_, 9);
    } else if (target_extruder_ == 0) {
      PrepareMoveToDestination(0, 9);
    }
    MoveSync();
    extruder_check_status_ = EXTRUDER_STATUS_CHECK;
  }

  uint8_t buf[8], index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_SWITCH_EXTRUDER);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (uint8_t)target_extruder_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DualExtruder::ReportNozzleType() {
  uint8_t buf[CAN_DATA_FRAME_LENGTH];
  uint8_t index = 0;
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_NOZZLE_TYPE);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (uint8_t)nozzle_identify_0_.GetNozzleType();
    buf[index++] = (uint8_t)nozzle_identify_1_.GetNozzleType();
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DualExtruder::ReportExtruderInfo() {
  uint8_t buf[CAN_DATA_FRAME_LENGTH];
  uint8_t index = 0;

  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_EXTRUDER_INFO);
  if (msgid != INVALID_VALUE) {
    buf[index++] = (active_extruder_ == target_extruder_) ? 0 : 1;
    buf[index++] = active_extruder_;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void DualExtruder::SetHotendOffset(uint8_t *data) {
  uint8_t axis_index;
  float offset;

  axis_index = data[0];
  ((uint8_t *)&offset)[0] = data[1];
  ((uint8_t *)&offset)[1] = data[2];
  ((uint8_t *)&offset)[2] = data[3];
  ((uint8_t *)&offset)[3] = data[4];

  switch (axis_index) {
    case 0:
      registryInstance.cfg_.x_hotend_offset = offset;
      break;
    case 1:
      registryInstance.cfg_.y_hotend_offset = offset;
      break;
    case 2:
      registryInstance.cfg_.z_hotend_offset = offset;
      break;
    default:
      return;
      break;
  }

  registryInstance.SaveCfg();
}

void DualExtruder::ReportHotendOffset() {
  float offset[3];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_HOTEND_OFFSET);
  if (msgid == INVALID_VALUE) {
    return ;
  }

  uint8_t u8DataBuf[8], i;
  offset[0] = registryInstance.cfg_.x_hotend_offset;
  offset[1] = registryInstance.cfg_.y_hotend_offset;
  offset[2] = registryInstance.cfg_.z_hotend_offset;

  for (i = 0; i < 3; i++) {
    u8DataBuf[0] = i;
    u8DataBuf[1] = ((uint8_t *)&offset[i])[0];
    u8DataBuf[2] = ((uint8_t *)&offset[i])[1];
    u8DataBuf[3] = ((uint8_t *)&offset[i])[2];
    u8DataBuf[4] = ((uint8_t *)&offset[i])[3];

    canbus_g.PushSendStandardData(msgid, u8DataBuf, 5);
  }
}

void DualExtruder::SetProbeSensorCompensation(uint8_t *data) {
  uint8_t e;
  float compensation;

  e = data[0];
  ((uint8_t *)&compensation)[0] = data[1];
  ((uint8_t *)&compensation)[1] = data[2];
  ((uint8_t *)&compensation)[2] = data[3];
  ((uint8_t *)&compensation)[3] = data[4];

  switch (e) {
    case 0:
      registryInstance.cfg_.probe_sensor_compensation_0 = compensation;
      break;
    case 1:
      registryInstance.cfg_.probe_sensor_compensation_1 = compensation;
      break;
    default:
      return;
      break;
  }

  registryInstance.SaveCfg();
}

void DualExtruder::ReportProbeSensorCompensation() {
  float compensation[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_PROBE_SENSOR_COMPENSATION);
  if (msgid == INVALID_VALUE) {
    return ;
  }

  uint8_t u8DataBuf[8], i, j, u8Index = 0;
  compensation[0] = registryInstance.cfg_.probe_sensor_compensation_0;
  compensation[1] = registryInstance.cfg_.probe_sensor_compensation_1;

  for (i = 0; i < 2; i++) {
    u8DataBuf[0] = i;
    for (j = 0, u8Index = 1; j < 4; j ++) {
      u8DataBuf[u8Index++] = ((uint32_t)(compensation[i] * 1000)) >> (8 * (3 - j));
    }
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

void DualExtruder::SetRightExtruderPos(uint8_t *data) {
  raise_for_home_pos_ = (float)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]) / 1000;
  z_max_position_     = (float)((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7]) / 1000;
}

void DualExtruder::ReportRightExtruderPos() {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_RIGHT_EXTRUDER_POS);
  if (msgid != INVALID_VALUE) {
    uint32_t raise_for_home_pos_scaled = raise_for_home_pos_ * 1000;
    uint32_t z_max_position_scaled = z_max_position_ * 1000;

    uint8_t u8DataBuf[8], u8Index = 0;
    u8DataBuf[u8Index++] = (raise_for_home_pos_scaled >> 24) & 0xff;
    u8DataBuf[u8Index++] = (raise_for_home_pos_scaled >> 16) & 0xff;
    u8DataBuf[u8Index++] = (raise_for_home_pos_scaled >> 8) & 0xff;
    u8DataBuf[u8Index++] = raise_for_home_pos_scaled & 0xff;
    u8DataBuf[u8Index++] = (z_max_position_scaled >> 24) & 0xff;
    u8DataBuf[u8Index++] = (z_max_position_scaled >> 16) & 0xff;
    u8DataBuf[u8Index++] = (z_max_position_scaled >> 8) & 0xff;
    u8DataBuf[u8Index++] = z_max_position_scaled & 0xff;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

void DualExtruder::ProximitySwitchPowerCtrl(uint8_t state) {
  if (state == 0) {
    proximity_power_.Out(0);
  } else if (state == 1) {
    proximity_power_.Out(1);
  }

  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_PROXIMITY_SWITCH_POWER_CTRL);
  if (msgid != INVALID_VALUE) {
    uint8_t u8DataBuf[8], u8Index = 0;
    u8DataBuf[u8Index++] = state;
    canbus_g.PushSendStandardData(msgid, u8DataBuf, u8Index);
  }
}

void DualExtruder::ReportHWVersion() {
  ModuleMacInfo *mac_info = (ModuleMacInfo *)FLASH_MODULE_PARA;
  uint8_t buf[2];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_GET_HW_VERSION);

  if (msgid != INVALID_VALUE) {
    buf[0] = hw_ver_.GetVersion() + mac_info->hw_version;
    canbus_g.PushSendStandardData(msgid, buf, 1);
  }
}

void DualExtruder::EmergencyStop() {
  temperature_0_.ChangeTarget(0);
  temperature_1_.ChangeTarget(0);
  left_model_fan_.ChangePwm(0, 0);
  right_model_fan_.ChangePwm(0, 0);
  nozzle_fan_.ChangePwm(0, 0);
  extruder_cs_0_.Out(0);
  extruder_cs_1_.Out(0);
}

void DualExtruder::Loop() {
  if (hal_adc_status()) {
    temperature_0_.TemperatureOut();
    temperature_1_.TemperatureOut();

    bool nozzle_0_status = nozzle_identify_0_.CheckLoop();
    bool nozzle_1_status = nozzle_identify_1_.CheckLoop();
    if (nozzle_0_status || nozzle_1_status) {
      ReportNozzleType();
    }
  }

  if (temp_report_time_ + 500 < millis()) {
    temp_report_time_ = millis();
    ReportTemprature();
  }

  if (out_of_material_detect_0_.CheckStatusLoop() || out_of_material_detect_1_.CheckStatusLoop()) {
    ReportOutOfMaterial();
  }

  bool proximity_switch_status  = probe_proximity_switch_.CheckStatusLoop();
  bool left_optocoupler_status  = probe_left_extruder_optocoupler_.CheckStatusLoop();
  bool right_optocoupler_status = probe_right_extruder_optocoupler_.CheckStatusLoop();
  // bool left_conductive_status   = probe_left_extruder_conductive_.CheckStatusLoop();
  // bool right_conductive_status  = probe_right_extruder_conductive_.CheckStatusLoop();
  if (proximity_switch_status || left_optocoupler_status || right_optocoupler_status /*|| left_conductive_status || right_conductive_status*/) {
    ReportProbe();
  }

  ExtruderStatusCheck();
  left_model_fan_.Loop();
  right_model_fan_.Loop();
  nozzle_fan_.Loop();
}
