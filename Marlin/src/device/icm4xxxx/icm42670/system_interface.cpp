/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/* board driver */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "system_interface.h"
// #include "../icm4xxxx_driver.h"

// #include "Invn/EmbUtils/Message.h"

extern unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len, const unsigned char *buf);
extern unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf);

int inv_io_hal_read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	switch (serif->serif_type) {
		case UI_SPI4:
			//return inv_spi_master_read_register(INV_SPI_AP, reg, rlen, rbuffer);
			// 这里是最底层的SPI读函数
			return MPU_Read_Len(0, reg, rlen, rbuffer);
		case UI_I2C:

			return 0;
		default:
			return -1;
	}
}

int inv_io_hal_write_reg(struct inv_imu_serif *serif, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	switch (serif->serif_type) {
		case UI_SPI4:
			// for(uint32_t i=0; i<wlen; i++) {
			// 	rc = inv_spi_master_write_register(INV_SPI_AP, reg+i, 1, &wbuffer[i]);
			// 	if (rc)
			// 		return rc;
			// }
			// return 0;
			// 这里是最底层的SPI写函数
			return MPU_Write_Len(0, reg, wlen, wbuffer);
		case UI_I2C:

			return 0;
		default:
			return -1;
	}
}

