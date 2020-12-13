//
// Created by David Chen on 2019-07-25.
//

#include "utils.h"
#include "string.h"
int Number10To36str(uint32_t num, uint8_t *OutBuf, uint8_t BufLen) {
	int i = 0, mod;
	memset(OutBuf, '0', BufLen);
	for (i = 0; (i < BufLen) && (num > 0); i++) {
		mod = num % 36;
		if (mod < 10) {
			OutBuf[BufLen - i - 1] = mod + '0';
		}
		else {
			OutBuf[BufLen - i - 1] = (mod - 10) + 'A';
		}
		num /= 36;
	}
	return (num == 0);
}
int Number36To10(uint8_t *data, uint8_t len) {
  int ret = 0;

  for (int i = 0; i < len; ++i) {
    if (data[i] >= '0' && data[i] <= '9') {
      ret = ret * 36 + (data[i] - '0');
    } else if (data[i] >= 'A' && data[i] <= 'Z') {
      ret = ret * 36 + (data[i] - 'A' + 10);
    } else {
      // wrong parameter
    }
  }
  return ret;
}

uint16_t CalcChecksum(uint8_t *data, uint16_t len) {
  uint32_t checksum = 0;
  for (int i = 0; i < len -1; i = i + 2) {
    checksum += ((data[i] << 8) | data[i + 1]);
  }

  if (len % 2) {
    checksum += data[len - 1];
  }

  while (checksum > 0xffff) {
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
  }
  checksum = ~checksum;

  return checksum;
}