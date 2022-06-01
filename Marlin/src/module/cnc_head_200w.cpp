#include <wirish_time.h>
#include <wirish_math.h>
#include "cnc_head_200w.h"
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
#include "src/core/thermistor_table.h"

extern BLDC_SELF_TEST_STEP bldc_self_test_step;
extern BLDC_SELF_TEST_MOS_EXCEPTIONAL mos_test_index;
extern uint32_t bldc_self_test_err_sta;
extern uint32_t self_test_send_flag;
extern uint32_t g_hall_sta;
extern bool hardware_protect_trigger;
extern float bldc_mos_current;
extern uint8_t hall_check_index;
extern uint8_t hall_check_order[CHECK_HALL_CNT];

void CncHead200W::Init(void) {
  bldc_module_dev_.Init();
  // target_speed_duty_ = MAX_DUTY_CYCLE;
  // bldc_module_dev_.BldcControlMotorRunProcess(RUN);
}

// stop motor with EmergencyStop signal
void CncHead200W::EmergencyStop(void) {
  bldc_module_dev_.BldcControlMotorRunProcess(STOP);
}

void CncHead200W::ReportConfigResult(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  uint16_t msgid = INVALID_VALUE;
  if (!data)
    return;

  msgid = registryInstance.FuncId2MsgId(func_id); 
  if (msgid != INVALID_VALUE) {
    data_len = data_len > 8 ? 8 : data_len;
    canbus_g.PushSendStandardData(msgid, data, data_len);
  }
}

// report motor state msg
void CncHead200W::ReportMotorState(void) {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_MOTOR_STATUS_INFO);
  if (msgid != INVALID_VALUE) {
    uint16_t cur_speed = (uint16_t)bldc_module_dev_.BldcGetMotorRpm();
    uint8_t data[8];
    uint8_t index = 0;
    if (bldc_module_dev_.BldcGetMotorBlockState() != MOTOR_BLOCK_NORMAL) {
      if (bldc_module_dev_.BldcGetMotorBlockState() == MOTOR_BLOCK_S_SPIN) {
        curtten_cnc_state_ |= (1 << CNC_EXCEPTIONAL_STALL);
      }
      else if (bldc_module_dev_.BldcGetMotorBlockState() == MOTOR_BLOCK_H_PROTECT) {
        curtten_cnc_state_ |= (1 << CNC_EXCEPTIONAL_H_PROTECT);
      }
    }
    else {
      curtten_cnc_state_ &= (~((1 << CNC_EXCEPTIONAL_STALL) | (1 << CNC_EXCEPTIONAL_H_PROTECT)));
    }

    data[index++] = (cur_speed >> 8) & 0xff;
    data[index++] = (cur_speed >> 0) & 0xff;
    data[index++] = curtten_cnc_state_;
    data[index++] = (uint8_t)bldc_module_dev_.BldcGetMotorState();
    data[index++] = (uint8_t)bldc_module_dev_.BldcGetMotorDirection();
    data[index++] = (uint8_t)bldc_module_dev_.BldcGetMotorPidControl();
    data[index++] = bldc_self_test_step & 0xff;
    cur_speed = bldc_module_dev_.BldcGetMotorSpeedPower() * 100;
    data[index++] = cur_speed & 0xff;
    canbus_g.PushSendStandardData(msgid, data, index);
  }
}

void CncHead200W::ReportMotorTemperature(void) {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_MOTOR_SENSOR_INFO);
  int16_t tmp_adc = 0;
  uint8_t data[8];
  uint8_t index = 0;
  if (msgid != INVALID_VALUE) {
    // External NTC
    tmp_adc = temp_motor_ * 10;
    data[index++] = (tmp_adc >> 8) & 0xff;
    data[index++] = (tmp_adc >> 0) & 0xff;

    // On-board NTC
    tmp_adc = temp_pcb_ * 10;
    data[index++] = (tmp_adc >> 8) & 0xff;
    data[index++] = (tmp_adc >> 0) & 0xff;

    // Electric current 
    tmp_adc = motor_current_;
    data[index++] = (tmp_adc >> 8) & 0xff;
    data[index++] = (tmp_adc >> 0) & 0xff;

    // Input voltage
    tmp_adc = motor_voltage_ * 100;
    data[index++] = (tmp_adc >> 8) & 0xff;
    data[index++] = (tmp_adc >> 0) & 0xff;
    canbus_g.PushSendStandardData(msgid, data, index);
  }
}

void CncHead200W::ReportMotorPidValue(uint8_t index) {
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_SET_PID);
  uint8_t data[8];
  int i = 0;
  float pid_value;
  pid_value = bldc_module_dev_.BldcIncPIDGetPID(index);
  if (msgid != INVALID_VALUE) {
      data[i++] = index;
      data[i++] = ((int)(pid_value * 1000)) >> 24;
      data[i++] = ((int)(pid_value * 1000)) >> 16;
      data[i++] = ((int)(pid_value * 1000)) >> 8;
      data[i++] = ((int)(pid_value * 1000)) >> 0;
      canbus_g.PushSendStandardData(msgid, data, i);
  }
}

void CncHead200W::CncHeadReportHWVersion() {
  ModuleMacInfo *mac_info = (ModuleMacInfo *)FLASH_MODULE_PARA;
  uint8_t buf[1];
  uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_MODULE_GET_HW_VERSION);
  uint8_t index = 0;
  if (msgid != INVALID_VALUE) {
    buf[index++] = mac_info->hw_version;
    canbus_g.PushSendStandardData(msgid, buf, index);
  }
}

void CncHead200W::SetMotorSpeedPower(uint8_t power) {
  float rpm = 0.0;
  float val_f = 0.0;
  uint8_t ack_buff[1];
  // No control allowed in self-test mode
  if (bldc_self_test_step)
    return;
  power = power > 100 ? 100 : power;
  if (power) {
    if (bldc_module_dev_.BldcGetMotorState() != RUN){
      time_ = millis() + 500;
      bldc_module_dev_.BldcSetMotorBlockState(MOTOR_BLOCK_NORMAL);
      ReportMotorState();
    }
    if (bldc_module_dev_.BldcGetMotorPidControl()) {
      rpm = ((float)power * MOTOR_RATED_SPEED/100);
      bldc_module_dev_.BldcSetMotorTargetRpm((float)rpm);
    }
    else {
      val_f = (float)power / 100;
      if (val_f < MIN_DUTY_CYCLE)
        val_f = MIN_DUTY_CYCLE;
      else if (val_f >= MAX_DUTY_CYCLE)
        val_f = MAX_DUTY_CYCLE;
      target_speed_duty_ = val_f;
    }
    bldc_module_dev_.BldcControlMotorRunProcess(RUN);
  }
  else {
    bldc_module_dev_.BldcControlMotorRunProcess(STOP);
  }
  ack_buff[0] = (uint8_t)bldc_module_dev_.BldcGetMotorState();
  ReportConfigResult(FUNC_SET_MOTOR_SPEED, ack_buff, 1);
}

void CncHead200W::SetMotorSpeedRpm(uint16_t rpm) {
  float val_f = 0.0;
  uint8_t ack_buff[1];
  // No control allowed in self-test mode
  if (bldc_self_test_step)
    return;
  rpm = rpm > MOTOR_RATED_SPEED ? MOTOR_RATED_SPEED : rpm;
  if (rpm) {
    if (bldc_module_dev_.BldcGetMotorPidControl()) {
      bldc_module_dev_.BldcSetMotorTargetRpm((float)rpm);
    }
    else {
      val_f = (float)rpm / MOTOR_RATED_SPEED;
      if (val_f < MIN_DUTY_CYCLE)
        val_f = MIN_DUTY_CYCLE;
      else if (val_f >= MAX_DUTY_CYCLE)
        val_f = MAX_DUTY_CYCLE;
      target_speed_duty_ = val_f;
    }  
    bldc_module_dev_.BldcControlMotorRunProcess(RUN);
  }
  else {
    bldc_module_dev_.BldcControlMotorRunProcess(STOP);
  }
  ack_buff[0] = (uint8_t)bldc_module_dev_.BldcGetMotorState();
  ReportConfigResult(FUNC_SET_MOTOR_SPEED_RPM, ack_buff, 1);
}

void CncHead200W::SetMotorCtrMode(bool pid_mode) {
  bool ret = false;
  uint8_t ack_buff[2];
  ret = bldc_module_dev_.BldcSetMotorPidControl(!!pid_mode);
  ack_buff[0] = (uint8_t)ret;
  ack_buff[1] = (uint8_t)bldc_module_dev_.BldcGetMotorPidControl();
  ReportMotorState();
  ReportConfigResult(FUNC_SET_MOTOR_CTR_MODE, ack_buff, 2);
}

void CncHead200W::SetMotorDirState(uint8_t dir) {
  bool ret = false;
  int i = 0;
  uint8_t ack_buff[2];
  dir = dir > 0 ? 1 : 0; 
  ret = bldc_module_dev_.BldcSetMotorDirection((MOTOR_DIR)dir);
  ack_buff[i++] = (uint8_t)ret;
  ack_buff[i++] = bldc_module_dev_.BldcGetMotorDirection();
  ReportConfigResult(FUNC_SET_MOTOR_RUN_DIRECTION, ack_buff, i);
}

void CncHead200W::SetMotorFan(bool ctr) {
  uint8_t ack_buff[1];
  bldc_module_dev_.BldcSetMotorFanEnableState(!!ctr);
  ack_buff[0] = true;
  ReportConfigResult(FUNC_SET_FAN, ack_buff, 1);
}

void CncHead200W::ReportMotorSelfTestMosState(void) {
  uint8_t ack_buff[8];  
  int i = 0;
  ack_buff[i++] = 1;   // mos test info
  ack_buff[i++] = (uint8_t)mos_test_index;
  ack_buff[i++] = (uint8_t)hardware_protect_trigger;
  ack_buff[i++] = (((uint32_t)bldc_mos_current) >> 24) & 0xff;
  ack_buff[i++] = (((uint32_t)bldc_mos_current) >> 16) & 0xff;
  ack_buff[i++] = (((uint32_t)bldc_mos_current) >> 8) & 0xff;
  ack_buff[i++] = ((uint32_t)bldc_mos_current) & 0xff;
  ReportConfigResult(FUNC_REPORT_MOTOR_SELF_TEST_INFO, ack_buff, i);
}

void CncHead200W::ReportMotorSelfTestHallState(void) {
  uint8_t ack_buff[8];  
  int i = 0;
  uint8_t hall_index = hall_check_index;
  if (++hall_index >= CHECK_HALL_CNT)
  hall_index = 0;
  ack_buff[i++] = 2;   // mos test info
  ack_buff[i++] = hall_index;
  ack_buff[i++] = g_hall_sta & 0xff;
  ack_buff[i++] = hall_check_order[hall_index];
  ReportConfigResult(FUNC_REPORT_MOTOR_SELF_TEST_INFO, ack_buff, i);
}

void CncHead200W::ReportMotorSelfTestState(void) {
  uint8_t ack_buff[8];  
  int i = 0;
  ack_buff[i++] = 3;   // self test info
  ack_buff[i++] = (bldc_self_test_err_sta >> 24) & 0xff;
  ack_buff[i++] = (bldc_self_test_err_sta >> 16) & 0xff;
  ack_buff[i++] = (bldc_self_test_err_sta >> 8) & 0xff;
  ack_buff[i++] = (bldc_self_test_err_sta) & 0xff;
  ReportConfigResult(FUNC_REPORT_MOTOR_SELF_TEST_INFO, ack_buff, i);
}

void CncHead200W::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  float val_f = 0.0;
  switch (func_id) {
    case FUNC_MODULE_GET_HW_VERSION:
      CncHeadReportHWVersion();
    break;

    case FUNC_SET_MOTOR_SPEED:
      // set motor speed by power
      SetMotorSpeedPower(data[0]);
    break;

    case FUNC_SET_MOTOR_SPEED_RPM:
      // set motor seep by rpm
      SetMotorSpeedRpm(data[0] << 8 | data[1]);
    break;

    case FUNC_SET_MOTOR_CTR_MODE:
      SetMotorCtrMode(data[0]);
    break;

    case FUNC_SET_MOTOR_RUN_DIRECTION:
      SetMotorDirState(data[0]);
    break;

    case FUNC_REPORT_MOTOR_STATUS_INFO:
      ReportMotorState();
    break;

    case FUNC_REPORT_MOTOR_SENSOR_INFO:
      ReportMotorTemperature();
    break;

    case FUNC_SET_PID:
      if (data[0] == 0xff)
        ReportMotorPidValue(data[1]);
      else if (data[0] == 0xfe) {
        val_f = (float)(((data[2]) << 24) | ((data[3]) << 16) | ((data[4]) << 8 | (data[5]))) / 1000;
        bldc_module_dev_.BldcIncPIDSetPID(data[1], val_f);
      }
    break;

    case FUNC_REPORT_MOTOR_SELF_TEST_INFO:
      if (data[0] == 0)
        bldc_module_dev_.BldcStartSelfTest();
    break;

    case FUNC_SET_FAN:
      SetMotorFan(!!data[0]);
    break;

    default:
    break;
  }
}

void CncHead200W::MotorSpeedControlLoop(void) {
  if (ELAPSED(millis(),loop_ctr_time_)) {
    loop_ctr_time_ = millis() + MOTOR_CTR_INTERVAL_TIME;
    // Motor speed control
    if (bldc_module_dev_.BldcGetMotorState() == RUN) {
      float speed_power = bldc_module_dev_.BldcGetMotorSpeedPower();
      if (!bldc_module_dev_.BldcGetMotorPidControl()) {
        if (target_speed_duty_ != speed_power) {
          if (target_speed_duty_ > speed_power) {
            if (target_speed_duty_ > speed_power + PWM_MODE_CHANGE_RANGE)
              bldc_module_dev_.BldcSetMotorSpeedPower(speed_power + PWM_MODE_CHANGE_RANGE);
            else
              bldc_module_dev_.BldcSetMotorSpeedPower(target_speed_duty_);
          }
          else if (target_speed_duty_ < speed_power) {
            if (target_speed_duty_ + PWM_MODE_CHANGE_RANGE < speed_power)
              bldc_module_dev_.BldcSetMotorSpeedPower(speed_power - PWM_MODE_CHANGE_RANGE);
            else
              bldc_module_dev_.BldcSetMotorSpeedPower(target_speed_duty_);
          }
        }
      }
    }
    // Operating current detection
    uint32_t adc_value = 0;
    adc_value = bldc_module_dev_.BldcGetMultiChannelAdc(I_ADC_CHANNEL,0);
    // motor_current_ = ((float)adc_value) * 66000 / (4095 * 4.7);
    motor_current_ = ((float)adc_value) * 16500 / 4095;   // 0.01R * 20
    if (motor_current_ > MAX_WORK_CURRENT) {
      if (overcurrent_ < MAX_WORK_CURRENT_SAMP_CNT) 
        overcurrent_++;

      if (overcurrent_ >= MAX_WORK_CURRENT_SAMP_CNT) {
        if ((curtten_cnc_state_ & (1 << CNC_EXCEPTIONAL_OVERCURRENT)) == 0) {
          report_msg_ = true;
        }
        curtten_cnc_state_ |= (1 << CNC_EXCEPTIONAL_OVERCURRENT);
        bldc_module_dev_.BldcControlMotorRunProcess(STOP);
      }
    }
    else {
      curtten_cnc_state_ &= ~(1 << CNC_EXCEPTIONAL_OVERCURRENT);
      overcurrent_ = 0;
    }

    // PCB temperature detection
    temp_pcb_ = TempTableCalcCurTemp(bldc_module_dev_.BldcGetMultiChannelAdc(P_ADC_CHANNEL,1));
    if (temp_pcb_ > MAX_PCB_TEMP_LIMIT || temp_pcb_ < MIN_PCB_TEMP_LIMIT) {
      if (pcb_protect_cnt_ < MAX_PCB_TEMP_SAMP_CNT) {
        pcb_protect_cnt_++;
      }
      if (pcb_protect_cnt_ >= MAX_PCB_TEMP_SAMP_CNT) {
        if ((curtten_cnc_state_ & (1 << CNC_EXCEPTIONAL_P_TEMP)) == 0) {
          report_msg_ = true;
        }
        curtten_cnc_state_ |= (1 << CNC_EXCEPTIONAL_P_TEMP);
        bldc_module_dev_.BldcControlMotorRunProcess(STOP);
      }
    }
    else {
      curtten_cnc_state_ &= ~(1 << CNC_EXCEPTIONAL_P_TEMP);
      pcb_protect_cnt_ = 0;
    }

    // Motor temperature detection
    temp_motor_ = TempTableCalcCurTemp(bldc_module_dev_.BldcGetMultiChannelAdc(M_ADC_CHANNEL,1));
    if (temp_motor_ > MAX_MOTOR_TEMP_LIMIT || temp_motor_ < MIN_MOTOR_TEMP_LIMIT) {
      if (motor_protect_cnt_ < MAX_MOTOR_TEMP_SAMP_CNT) {
        motor_protect_cnt_++;
      }

      if (motor_protect_cnt_ >= MAX_MOTOR_TEMP_SAMP_CNT) {
        if ((curtten_cnc_state_ & (1 << CNC_EXCEPTIONAL_M_TEMP)) == 0) {
          report_msg_ = true;
        }
        curtten_cnc_state_ |= (1 << CNC_EXCEPTIONAL_M_TEMP);
        bldc_module_dev_.BldcControlMotorRunProcess(STOP);
      }
    } 
    else {
      curtten_cnc_state_ &= ~(1 << CNC_EXCEPTIONAL_M_TEMP);
      motor_protect_cnt_ = 0;
    }

    // Power voltage detection
    adc_value = bldc_module_dev_.BldcGetMultiChannelAdc(V_ADC_CHANNEL,0);
    motor_voltage_ = (float)adc_value * 3.3 * 11 / 4095;
    if (motor_voltage_ > MAX_MOTOR_VOLTAGE_LIMIT || motor_voltage_ < MIN_MOTOR_VOLTAGE_LIMIT) {
      if (voltage_protect_cnt_ < MAX_MOTOR_VOLTAGE_SAMP_CNT) {
        voltage_protect_cnt_++;
      }
      if (voltage_protect_cnt_ >= MAX_MOTOR_VOLTAGE_SAMP_CNT) {
        if ((curtten_cnc_state_ & (1 << CNC_EXCEPTIONAL_V_POWER)) == 0) {
          report_msg_ = true;
        }
        curtten_cnc_state_ |= (1 << CNC_EXCEPTIONAL_V_POWER);
        bldc_module_dev_.BldcControlMotorRunProcess(STOP);
      }
    }
    else {
      curtten_cnc_state_ &= ~(1 << CNC_EXCEPTIONAL_V_POWER);
      voltage_protect_cnt_ = 0;
    }
    MOTOR_BLOCK_STATE tmp_sta = bldc_module_dev_.BldcGetMotorBlockState();
    if (tmp_sta != motor_block_bak_)
      report_msg_ = true;
    motor_block_bak_ = tmp_sta;
  }
}

void CncHead200W::Loop(void) {
  MotorSpeedControlLoop();
  bldc_module_dev_.BldcSelfTestLoop(millis());
  if (ELAPSED(millis(), time_) || report_msg_) {
    time_ = millis() + 500;
    ReportMotorTemperature();
    ReportMotorState();
    report_msg_ = false;
  }

  if (self_test_send_flag & 0x3fffffff) {
    if (self_test_send_flag & (1 << SELF_TEST_MOS_INFO)) {
      ReportMotorSelfTestMosState();
      self_test_send_flag &= (~(1 << SELF_TEST_MOS_INFO));
    }

    if (self_test_send_flag & (1 << SELF_TEST_HALL_INFO)) {
      ReportMotorSelfTestHallState();
      self_test_send_flag &= (~(1 << SELF_TEST_HALL_INFO));
    }

    if (self_test_send_flag & (1 << SELF_TEST_END_INFO)) {
      ReportMotorSelfTestState();
      self_test_send_flag &= (~(1 << SELF_TEST_END_INFO));
    }
  }
}
