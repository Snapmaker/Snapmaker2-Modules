#include "enclosure.h"
#include "io.h"
#include <wirish_time.h>
#include "src/registry/registry.h"
#include "src/core/can_bus.h"
void EnclosureModule::Init() {
  this->enclosure_.Init(PB0);
  report_statu_ = enclosure_.Read();
  this->light_.Init();
  this->fan_.Init(ENCLOSURE_FAN_PIN);
}

void EnclosureModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_ENCLOSURE:
      this->ReportStatus();
      break;
    case FUNC_SET_ENCLOSURE_LIGHT:
      this->light_.HandModule(func_id, data, data_len);
      break;
    case FUNC_SET_FAN_MODULE:
      this->fan_.ChangePwm(data[1], data[0]);
      break;
  }
}

void EnclosureModule::EmergencyStop() {
  fan_.ChangePwm(0, 0);
  light_.SetRGB(0, 0, 0);
}

void EnclosureModule::StartingUpLight() {
  static uint8_t start_flag_ = 1;
  static uint8_t out = 0;
  static uint32_t time = 0;
  if (!start_flag_) {
    return;
  }
  if (start_flag_ && (time < millis())) {
    time = millis() + 3;
    if (start_flag_ == 1) {
      out += 1;
      if (out == 200) {
        start_flag_ = 2;
      }
    } else if (start_flag_ == 2) {
      out -= 1;
      if (out == 0) {
        start_flag_ = 0;
      }
    }
    light_.SetRGB(out, out, out);
  }
}

void EnclosureModule::Loop() {
  uint8_t cur_statu = 0;
  // Take a breath when the light is startint up
  StartingUpLight();
  
  this->fan_.Loop();
  
  this->enclosure_.CheckStatusLoop();
  cur_statu = this->enclosure_.Read();
  if (cur_statu != this->last_statu_) {
    if (this->is_report_ == true){
      if ((cur_statu != ENCLOSURE_CLOSE_STATU) && (this->report_statu_ == ENCLOSURE_CLOSE_STATU)) {
        this->is_report_ = false;
      }
    } else {
      this->is_report_ = true;
    }
    this->report_statu_ = cur_statu;
    this->time_ = millis();
  }
  this->last_statu_ = cur_statu;
  if (this->is_report_) {
    if (this->report_statu_ == ENCLOSURE_CLOSE_STATU) {
      if ((this->time_ + 1000) < millis()) {
        this->is_report_ = false;
        this->ReportStatus();
      }
    } else {
      this->is_report_ = false;
      this->ReportStatus();
    }
  }
}

void EnclosureModule::ReportStatus() {
  uint8_t buf[8];
  uint8_t index = 0;
   uint16_t msgid = registryInstance.FuncId2MsgId(FUNC_REPORT_ENCLOSURE);
    if (msgid != INVALID_VALUE) {
      buf[index++] = this->report_statu_;
      canbus_g.PushSendStandardData(msgid, buf, index);
    }
}