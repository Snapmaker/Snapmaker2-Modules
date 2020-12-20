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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_HAL_HAL_FLASH_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_HAL_HAL_FLASH_H_

#include <stdint.h>

void HAL_flash_write(uint32_t updateAddr, uint8_t * data, uint8_t len);
void HAL_flash_read(uint32_t addr, uint8_t * out, uint16_t read_len);
void HAL_flash_erase_page(uint32_t start_addr, uint8_t page_count);


#endif //MODULES_WHIMSYCWD_MARLIN_SRC_HAL_HAL_FLASH_H_
