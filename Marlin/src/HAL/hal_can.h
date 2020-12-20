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


#ifndef _CAN_S_H_
#define _CAN_S_H_
//#include "std_library/inc/stm32f10x.h"
#include "src/core/common_type.h"

// CLK=36MHz
#define CAN_BAUD_RATE_1M    CAN_SJW_1tq, CAN_BS1_10tq, CAN_BS2_1tq, 3
#define CAN_BAUD_RATE_900K  CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_1tq, 4
#define CAN_BAUD_RATE_800K  CAN_SJW_2tq, CAN_BS1_13tq, CAN_BS2_1tq, 13
#define CAN_BAUD_RATE_666K  CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 3
#define CAN_BAUD_RATE_600K  CAN_SJW_1tq, CAN_BS1_13tq, CAN_BS2_1tq, 4
#define CAN_BAUD_RATE_500K  CAN_SJW_1tq, CAN_BS1_14tq, CAN_BS2_3tq, 4
#define CAN_BAUD_RATE_400K  CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 5
#define CAN_BAUD_RATE_300K  CAN_SJW_1tq, CAN_BS1_15tq, CAN_BS2_1tq, 7
#define CAN_BAUD_RATE_250K  CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 8
#define CAN_BAUD_RATE_225K  CAN_SJW_1tq, CAN_BS1_14tq, CAN_BS2_1tq, 10
#define CAN_BAUD_RATE_200K  CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 10
#define CAN_BAUD_RATE_160K  CAN_SJW_1tq, CAN_BS1_13tq, CAN_BS2_1tq, 15
#define CAN_BAUD_RATE_150K  CAN_SJW_1tq, CAN_BS1_14tq, CAN_BS2_1tq, 15
#define CAN_BAUD_RATE_144K  CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_1tq, 25
#define CAN_BAUD_RATE_125K  CAN_SJW_1tq, CAN_BS1_6tq, CAN_BS2_1tq, 36
#define CAN_BAUD_RATE_120K  CAN_SJW_1tq, CAN_BS1_13tq, CAN_BS2_1tq, 20
#define CAN_BAUD_RATE_100K  CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 20
#define CAN_BAUD_RATE_90K   CAN_SJW_1tq, CAN_BS1_14tq, CAN_BS2_1tq, 25
#define CAN_BAUD_RATE_80K   CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 25
#define CAN_BAUD_RATE_75K   CAN_SJW_1tq, CAN_BS1_14tq, CAN_BS2_1tq, 30
#define CAN_BAUD_RATE_60K   CAN_SJW_1tq, CAN_BS1_13tq, CAN_BS2_1tq, 40
#define CAN_BAUD_RATE_50K   CAN_SJW_1tq, CAN_BS1_14tq, CAN_BS2_1tq, 45
#define CAN_BAUD_RATE_40K   CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 50
#define CAN_BAUD_RATE_30K   CAN_SJW_1tq, CAN_BS1_14tq, CAN_BS2_1tq, 75
#define CAN_BAUD_RATE_20K   CAN_SJW_1tq, CAN_BS1_16tq, CAN_BS2_1tq, 100


#define CAN_MESSAGE_BUF_SIZE 32

#define EXT_FILTER_GROUP_NUM_START 0
#define MAX_EXT_FILTER_GROUP_NUM 1
#define FILTER_GROUP_MAX_EXT_COUNT 2  // 每个滤波器组可以有2个32位id

#define STD_FILTER_GROUP_NUM_START MAX_EXT_FILTER_GROUP_NUM
#define MAX_STD_FILTER_GROUP_NUM 13
#define FILTER_GROUP_MAX_STD_COUNT 4  // 每个滤波器组可以有4个16位id

//typedef struct CAN_MESSAGE_BUF_S {
//    uint32_t        u32Id;
//    uint8_t         u8Buf[8];
//    uint8_t         u8BufLen;
//} CAN_MESSAGE_BUF_S;

void CAN_GPIO_Config(void);
void CAN_ConfigInit();
void HAL_CAN_try_send();
ERR_E CAN1_Send_Msg(uint8_t * pu8Data, uint8_t u8DataLen, uint32_t u32Id, uint32_t u32IdType, uint32_t u32RtrType);
//ERR_E CAN1_ReadData(CanRxMsg * suMessage);
// 发送标准数据帧
ERR_E CAN1_SendStandardData(uint8_t * pu8Data, uint8_t u8DataLen, uint32_t u32Id);
// 发送标准扩展帧
ERR_E CAN1_SendStandardExtended(uint8_t * pu8Data, uint8_t u8DataLen, uint32_t u32Id);

// 发送远程扩展帧
ERR_E CAN1_SendRemoteExtended(uint32_t u32Id);

// 滤波器配置
bool Can_AddRemoteExtIdFilter(uint32_t id);
bool Can_AddDataExtIdFilter(uint32_t id);
bool Can_AddRemoteStdIdFilter(uint16_t id);
bool Can_AddDataStdIdFilter(uint16_t id);

#endif

