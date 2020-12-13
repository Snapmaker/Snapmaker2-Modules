//
// Created by David Chen on 2019-07-16.
//

#include <board/board.h>
#include <src/core/can_bus.h>
#include <wirish_time.h>
#include "switch.h"
#include "src/configuration.h"
#include "src/registry/registry.h"

void SwitchInput::Init(uint8_t pin, WiringPinMode mode) {
  this->pin_ = pin;
  pinMode(pin, mode);
  this->time_ = millis();
  this->status_ = 0;
  digitalWrite(pin, 1);
  this->cur_statu = digitalRead(this->pin_);
}

bool SwitchInput::CheckStatuLoop() {
  bool ret = false;
  if ((this->time_ + 2) > millis()) {
    return ret;
  }
  // Disappears Shakes
  this->time_ = millis();
  uint8_t cur_statu = digitalRead(this->pin_);
  this->status_ = (this->status_ << 1) | (cur_statu != 0);
  if (((this->status_ & 0x0f) == 0x0f) || ((this->status_ & 0x0f) == 0x0)) {
    this->cur_statu = this->status_ & 0x1;
  }
  if (this->cur_statu != this->last_statu_) {
    ret = true;
    this->last_statu_ = this->cur_statu;
  }
  return ret;
}

uint8_t SwitchInput::Read() {
  return this->cur_statu;
}

void SwitchInput::ReportStatu(uint16_t funcid) {
  uint16_t msgid = registryInstance.FuncId2MsgId(funcid);
  if (msgid != INVALID_VALUE) {
    uint8_t statu = this->Read();
    canbus_g.PushSendStandardData(msgid, &statu, 1);
  }
}


void SwitchOutput::Init(uint8_t pin, uint8_t default_out, WiringPinMode mode) {
  this->pin_ = pin;
  pinMode(pin, mode);
  this->time_ = millis();
  this->out_val_ = 0;
  digitalWrite(pin, default_out);
}

void SwitchOutput::Out(uint8_t out) {
  digitalWrite(this->pin_, out);
}

void SwitchOutput::DelayOut(uint8_t out, uint32_t delay_time_ms) {
  this->delay_time_ = delay_time_ms;
  this->time_ = millis();
  this->out_val_ = 0 | 0x2 | (out != 0);
}

void SwitchOutput::ReastOut(uint32_t reset_time_ms) {
  if (!(this->out_val_ & 0x2)) {
    uint8_t cur_out = digitalRead(this->pin_);
    this->Out(!cur_out);
    this->DelayOut(cur_out, reset_time_ms);
  } else {
    this->DelayOut(this->out_val_ & 1, reset_time_ms);
  }
}

void SwitchOutput::OutCtrlLoop() {
  if (((this->time_ + this->delay_time_) > millis()) || !(this->out_val_ & 0x2)) {
      return ;
  }
  digitalWrite(this->pin_, this->out_val_ & 0x1);
  this->out_val_ = 0;
}