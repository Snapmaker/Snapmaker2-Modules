//
// Created by David Chen on 2019-07-26.
//
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
