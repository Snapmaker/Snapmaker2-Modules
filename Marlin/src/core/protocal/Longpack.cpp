//
// Created by David Chen on 2019-07-24.
//

#include <src/core/can_bus.h>
#include <src/core/utils.h>

#include "Longpack.h"

#define MAGIC_PART_1 0xAA
#define MAGIC_PART_2 0x55

ERR_E Longpack::parseCmd() {
  uint8_t data;
  uint16_t dataFieldLen;
  while (!canbus_g.extended_recv_buffer_.isEmpty()) {
    data = canbus_g.extended_recv_buffer_.remove();

    if (recv_index_ == 0 && data == MAGIC_PART_1) {
      // parse started
      packData_[recv_index_++] = data;
      continue;
    } else if (recv_index_ == 1 && data != MAGIC_PART_2) {
      // wrong data, skip
      recv_index_ = 0;
      continue;
    } else if (recv_index_ > 0) {
      packData_[recv_index_++] = data;
      if (recv_index_ == 6) {
        // len_high(bit 2) concat len_low(bit 3)  = len_check(bit 5)
        if ((packData_[2] ^ packData_[3]) != packData_[5]) {
          // wrong data, skip
          recv_index_ = 0;
          continue;
        }
      } else if (recv_index_ > 6) {
        dataFieldLen = packData_[2] << 8 | packData_[3];
        if (dataFieldLen + sizeof(PackHead) == recv_index_) {
          len_ = recv_index_ - sizeof(PackHead);
          packData_[recv_index_] = 0;
          recv_index_ = 0;
          // reach the end of the pack
          uint8_t * dataFiled = packData_ + sizeof(PackHead); // skip the packhead
          uint16_t checksum = CalcChecksum(dataFiled, dataFieldLen);

          // len_check_high(bit 6) concat len_check_low(bit 7) were calculated by caller.
          // This check will avoid most data corruption.
          if (checksum == ((packData_[6] << 8) | packData_[7])) {
            return E_TRUE;
          } else {
            return E_FALSE;
          }
        }
      }
    }
    // if all above three criteria are not matched, then that is wrong data, skip.

  }

  return E_DOING;
}
void Longpack::sendLongpack(uint8_t *data, uint16_t len) {
  uint16_t dataLen = 0;
  dataLen = (data == NULL) ? 0 : len;

  PackHead headInfo;

  headInfo.magic1 = MAGIC_PART_1;
  headInfo.magic2 = MAGIC_PART_2;

  headInfo.lenHigh = dataLen >> 8 & 0xff;
  headInfo.lenLow = dataLen & 0xff;

  headInfo.version = 0x00;

  headInfo.lenCheck = headInfo.lenHigh ^ headInfo.lenLow;

  uint16_t checksum = CalcChecksum(data, len);
  headInfo.dataCheckHigh = checksum >> 8 & 0xff;
  headInfo.dataCheckLow = checksum & 0xff;

  // send head info
  uint8_t * iter = (uint8_t *) &headInfo;
  for (int i = 0; i < 8; ++i) {
    canbus_g.extended_send_buffer_.insert(*iter);
    ++iter;
  }

  // send data field
  for (int i = 0; i < len; ++i) {
    canbus_g.extended_send_buffer_.insert(data[i]);
  }

}
void Longpack::sendLongpack(uint16_t *data, uint16_t len) {
  sendLongpack((uint8_t*) data, len * 2);
}

void Longpack::cmd_clean() {
  memset(packData_, 0, sizeof(packData_));
}
Longpack longpackInstance;
