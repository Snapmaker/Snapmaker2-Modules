//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_UTILS_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_UTILS_H_

#include <stdint.h>

int Number36To10(uint8_t *data, uint8_t len);
int Number10To36str(uint32_t num, uint8_t *OutBuf, uint8_t BufLen);
uint16_t CalcChecksum(uint8_t *data, uint16_t len);

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CORE_UTILS_H_
