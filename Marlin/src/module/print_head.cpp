//
// Created by David Chen on 2019-07-19.
//

#include <src/device/fan.h>
#include <board/board.h>
#include "print_head.h"
#include "src/registry/context.h"
#include "src/core/can_bus.h"
#include <wirish_time.h>
#include "io.h"
//  Periph initialization according schema
void PrintHead::PeriphInit() {

}
void PrintHead::Init() {
  // Fan
  fan_1_.Init(FAN_1_PIN);
  fan_2_.Init(FAN_2_PIN);
  switch_probe_.Init(PA7);
  switch_cut_.Init(PB0);
  temperature_.InitCapture(6, PA6, 4);
  temperature_.InitOutCtrl(2, 2, PA1);
  uint32_t moduleType = registryInstance.module();
  if (MODULE_PRINT_V_SM1 == moduleType) {
    pinMode(PA2, OUTPUT);
    digitalWrite(PA2, HIGH);
  }
}
void PrintHead::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  float val = 0.0;
  switch ((uint32_t)func_id) {
    case FUNC_REPORT_CUT:
      this->switch_cut_.ReportStatus(FUNC_REPORT_CUT);
      break;
    case FUNC_REPORT_PROBE:
      this->switch_probe_.ReportStatus(FUNC_REPORT_PROBE);
      break;
    case FUNC_SET_FAN:
      fan_1_.ChangePwm(data[1], data[0]);
      break;
    case FUNC_SET_FAN2:
      fan_2_.ChangePwm(data[1], data[0]);
      break;
    case FUNC_SET_TEMPEARTURE:
      temperature_.ChangeTarget((contextInstance.data_[0] << 8) | contextInstance.data_[1]);
      break;
    case FUNC_REPORT_TEMPEARTURE:
      temperature_.ReportTemprature();
      break;
    case FUNC_REPORT_TEMP_PID:
      temperature_.ReportPid();
      break;
    case FUNC_SET_PID:
      val = (float)(((data[1]) << 24) | ((data[2]) << 16) | ((data[3]) << 8 | (data[4]))) / 1000;
      temperature_.SetPID(data[0], val);
      break;
  }
}

void PrintHead::EmergencyStop() {
  temperature_.ChangeTarget(0);
  fan_1_.ChangePwm(0, 0);
  fan_2_.ChangePwm(0, 0);
}

void PrintHead::Loop() {
  this->temperature_.Maintain();
  if ((temp_report_time_ + 500) < millis()) {
    temp_report_time_ = millis();
    temperature_.ReportTemprature();
  }

  if (switch_cut_.CheckStatusLoop()) {
    if (this->is_report_cut_) {
      this->is_report_cut_ = false;
    } else {
      this->is_report_cut_ = true;
    }
    this->cut_report_time_ = millis();
  }
  if (this->is_report_cut_ && ((this->cut_report_time_ + 500) > millis())) {
    this->is_report_cut_ = false;
    switch_cut_.ReportStatus(FUNC_REPORT_CUT);
  }

  if (switch_probe_.CheckStatusLoop()) {
    switch_probe_.ReportStatus(FUNC_REPORT_PROBE);
  }

  this->fan_1_.Loop();
  this->fan_2_.Loop();

}