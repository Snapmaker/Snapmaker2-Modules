//
// Created by David Chen on 2019-07-16.
//

#include <src/HAL/hal_can.h>
#include "can_bus.h"
#include "src/HAL/hal_can.h"
#include <wirish_time.h>

CanBus::CanBus() {

//  rb_init(&standardSendBuffer, 1024, rawStandardSendBuffer);
//  rb_init(&standard_recv_buffer_, 512, rawStandardRecvBuffer);
}
CanBus::~CanBus() {

}

void CanBus::SetRemoteCtrlCmd() {
  Can_AddRemoteExtIdFilter(REMOTE_EXT_REPORT_MAC);
  Can_AddRemoteStdIdFilter(REMOTE_STD_HEARTBEAT);
  Can_AddRemoteStdIdFilter(REMOTE_STD_EM_STOP);
}

void CanBus::SetRecvSysCfgCmd(uint32_t module_id) {
  Can_AddDataExtIdFilter(module_id & (~0x01));
}

void CanBus::SetRecvMsgID(uint16_t msg_id) {
  Can_AddDataStdIdFilter(msg_id);
}

void CanBus::Init(uint32_t module_id) {
  CAN_ConfigInit();
  extend_send_id_ = module_id;
  new_extended_id_ = extend_send_id_;
  this->SetRemoteCtrlCmd();
  this->SetRecvSysCfgCmd(extend_send_id_);
}

void CanBus::RenewExternedID() {
  if (this->extend_send_id_ != this->new_extended_id_) {
    if (this->extended_send_buffer_.isEmpty()) {
      this->extend_send_id_ = this->new_extended_id_;
      this->SetRecvSysCfgCmd(extend_send_id_);
    }
  }
}

void  CanBus::SetNewExternedID(uint32_t id) {
  this->new_extended_id_ = id;
}
// handle incoming data
// send outgoing data
void CanBus::Handler() {
//  IncomingHandler();
  OutgoingHandler();
}

// put data into specific buffer
void CanBus::IncomingHandler() {

}
void CanBus::OutgoingHandler() {
  HAL_CAN_try_send();
}
void CanBus::PushRecvRemoteData(uint32_t id, uint8_t ide) {
  if (ide) {
    remote_extended_recv_buffer_.insert(id);
  } else {
    remote_standard_recv_buffer_.insert((uint16_t)id);
  }
}
void CanBus::PushRecvExtendedData(uint8_t *data, uint8_t len) {
  for (int i = 0; i < len; ++i) {
    extended_recv_buffer_.insert(data[i]);
  }
}
void CanBus::PushRecvStandardData(uint32_t stdId, uint8_t *data, uint8_t len) {
  CanRxStruct rx_struct;
  rx_struct.std_id = stdId;
  for (int i = 0; i < len; ++i) {
    rx_struct.data[i] = data[i];
  }
  rx_struct.len = len;

  standard_recv_buffer_.insert(rx_struct);
}

void CanBus::PushSendRemoteData(uint32_t std_id) {
  remote_send_buffer_.insert(std_id);
}

void CanBus::PushSendStandardData(uint32_t std_id, uint8_t *data, uint8_t len) {
  CanTxStruct tx_struct;
  tx_struct.std_id = std_id;
  tx_struct.len = len;
  for (int i = 0; i < len; ++i) {
    tx_struct.data[i] = data[i];
  }
  standard_send_buffer_.insert(tx_struct);
}

void CanBus::PushSendExtendedData(uint8_t *data, uint8_t len) {
  for (int i = 0; i < len; ++i) {
    extended_send_buffer_.insert(data[i]);
  }
}

uint32_t CanBus::GetSendTime() {
  return millis();
}
CanBus canbus_g;
