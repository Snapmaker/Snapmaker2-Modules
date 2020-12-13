//
// Created by David Chen on 2019-07-26.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_HAL_HAL_FLASH_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_HAL_HAL_FLASH_H_

#include <stdint.h>

void HAL_flash_write(uint32_t updateAddr, uint8_t * data, uint8_t len);
void HAL_flash_read(uint32_t addr, uint8_t * out, uint16_t read_len);
void HAL_flash_erase_page(uint32_t start_addr, uint8_t page_count);


#endif //MODULES_WHIMSYCWD_MARLIN_SRC_HAL_HAL_FLASH_H_
