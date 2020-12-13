//
// Created by David Chen on 2019-07-16.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_CANBUS_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_CANBUS_H_

#include <src/utils/RingBuffer.h>

#define STANDARD_SEND_BUFFER_SIZE 16
#define STANDARD_RECV_BUFFER_SIZE 16
#define EXTENDED_SEND_BUFFER_SIZE 128
#define EXTENDED_RECV_BUFFER_SIZE 256
#define REMOTE_SEND_BUFFER_SIZE 3
#define REMOTE_RECV_BUFFER_SIZE 3

#define REMOTE_EXT_REPORT_MAC 0x01
#define REMOTE_STD_HEARTBEAT  0x01
#define REMOTE_STD_EM_STOP    0x02

struct CanTxStruct {
  uint32_t std_id;
  uint8_t data[8];
  uint8_t len;
};

struct CanRxStruct {
  uint32_t std_id;
  uint8_t data[8];
  uint32_t len;
};

class CanBus {
 public:
  CanBus();
  ~CanBus();

  void Init(uint32_t module_id);
  void Handler();
  void PushSendRemoteData(uint32_t std_id);
  void PushSendStandardData(uint32_t std, uint8_t *data, uint8_t len);
  void PushSendExtendedData(uint8_t *data, uint8_t len);
  void PushRecvRemoteData(uint32_t id, uint8_t ide);
  void PushRecvExtendedData(uint8_t *data, uint8_t len);
  void PushRecvStandardData(uint32_t std_id, uint8_t *data, uint8_t len);
  void RenewExternedID();
  void SetNewExternedID(uint32_t id);
  void SetRemoteCtrlCmd();
  void SetRecvSysCfgCmd(uint32_t module_id);
  void SetRecvMsgID(uint16_t msg_id);
  uint32_t GetSendTime();
  uint32_t extend_send_id_ = 0;
  RingBuffer<uint32_t> remote_send_buffer_{REMOTE_SEND_BUFFER_SIZE};
  RingBuffer<uint32_t> remote_extended_recv_buffer_{REMOTE_RECV_BUFFER_SIZE};
  RingBuffer<uint16_t> remote_standard_recv_buffer_{REMOTE_RECV_BUFFER_SIZE};
  RingBuffer<uint8_t> extended_send_buffer_{EXTENDED_SEND_BUFFER_SIZE};
  RingBuffer<uint8_t> extended_recv_buffer_{EXTENDED_RECV_BUFFER_SIZE};
  RingBuffer<CanTxStruct> standard_send_buffer_{STANDARD_SEND_BUFFER_SIZE};
  RingBuffer<CanRxStruct> standard_recv_buffer_{STANDARD_RECV_BUFFER_SIZE};

 private:
  void IncomingHandler();
  void OutgoingHandler();
  uint32_t new_extended_id_;
};

extern CanBus canbus_g;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CORE_CANBUS_H_
