#include "cnc_head.h"
#include <io.h>
#include <wirish_time.h>

void CncHead::Init() {
  this->speed_.InitCapture(PA6, SPEED_TIM_MUN);
  this->speed_.InitOut(PA1, 2, 2);
  this->speed_.InitDir(PA5, 0);
}

void CncHead::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_MOTOR_SPEED:
      this->speed_.ReportSpeed();
      break;
    case FUNC_SET_MOTOR_SPEED:
      this->speed_.SetSpeed(data[0]);
      break;
  }
}

void CncHead::EmergencyStop() {
  speed_.SetSpeed(0);
}

void CncHead::Loop() {
  this->speed_.SpeedOutCtrl();
  if ((this->time_ + 500) < millis()) {
    this->time_ = millis();
    this->speed_.ReportSpeed();
  }
}