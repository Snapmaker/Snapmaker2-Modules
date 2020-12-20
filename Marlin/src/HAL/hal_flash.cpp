/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <string.h>
#include <src/configuration.h>
#include <flash_stm32.h>

void HAL_flash_write(uint32_t updateAddr, uint8_t *data, uint8_t len) {
  uint16_t i = 0, u16Data;
  uint8_t  u8SingleFlag = len % 2;
  uint32_t u32FlashBase = updateAddr;

  FLASH_Unlock();    
  len = len / 2 * 2; 
  while(i < len) {
    u16Data = * ((uint16_t *)(data + i));
    FLASH_ProgramHalfWord(u32FlashBase + i, u16Data);
    i += 2;
  }
  if(u8SingleFlag) {
    FLASH_ProgramHalfWord(u32FlashBase + i, data[i]);
  }
  FLASH_Lock();
}

void HAL_flash_read(uint32_t addr, uint8_t * out, uint16_t read_len) {
  memcpy((char *)out, (char *)addr, read_len);
}
void HAL_flash_erase_page(uint32_t start_addr, uint8_t page_count) {
  FLASH_Unlock();
  for (int i = 0; i < page_count; ++i) {
    FLASH_ErasePage(start_addr + i * FLASH_PAGE_SIZE);
  }
  FLASH_Lock();
}
