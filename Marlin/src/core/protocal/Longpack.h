//
// Created by David Chen on 2019-07-24.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PROTOCAL_LONGPACK_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PROTOCAL_LONGPACK_H_

#include <stdint.h>
#include <src/core/common_type.h>

#define MAX_SYS_CMD_LEN 1024

struct PackHead {
  uint8_t magic1;
  uint8_t magic2;
  uint8_t lenHigh;
  uint8_t lenLow;
  uint8_t version;
  uint8_t lenCheck;
  uint8_t dataCheckHigh;
  uint8_t dataCheckLow;
};



class Longpack {
 public:
  ERR_E parseCmd();
  void sendLongpack(uint8_t* data, uint16_t len);
  void sendLongpack(uint16_t* data, uint16_t len);
  void cmd_clean();
  public:
    uint8_t packData_[MAX_SYS_CMD_LEN];
    uint8_t * cmd = packData_ + sizeof(PackHead);
    uint16_t len_ = 0;

  private:
    uint16_t recv_index_ = 0;
};

extern Longpack longpackInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PROTOCAL_LONGPACK_H_
