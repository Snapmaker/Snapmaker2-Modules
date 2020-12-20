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

#include <string.h>
#include <src/core/can_bus.h>
#include <include/libmaple/libmaple_types.h>
#include "std_library/inc/stm32f10x.h"
#include "hal_can.h"

#define CAN_DATA_LEN_LIMIT 8


/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置,PB8上拉输入，PB9推挽输出
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
void CAN_GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 外设时钟设置 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* IO设置 */
    /* Configure CAN pin: RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure CAN pin: TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}


/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  // CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
}


/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(CAN_TypeDef * stCAN, uint8_t u8SJW, uint8_t u8BS1,
                                  uint8_t u8BS2, uint16_t u16PreScale) {
    CAN_InitTypeDef CAN_InitStructure;

    /************************CAN通信参数设置**********************************/
    /* CAN寄存器初始化 */
    CAN_DeInit(stCAN);
    CAN_StructInit(&CAN_InitStructure);

    /*CAN单元初始化*/
    CAN_InitStructure.CAN_TTCM = DISABLE;           // MCR-TTCM  关闭时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM = DISABLE;           // ENABLE;  //MCR-ABOM 自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;           // ENABLE;  //MCR-AWUM 使用自动唤醒模式
    CAN_InitStructure.CAN_NART = ENABLE;            // DISABLE; //MCR-NART 禁止报文自动重传
    CAN_InitStructure.CAN_RFLM = DISABLE;           // MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文
    CAN_InitStructure.CAN_TXFP = ENABLE;            // MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;   // CAN_Mode_Normal 正常工作模式,CAN_Mode_LoopBack 回环模式

    CAN_InitStructure.CAN_SJW = u8SJW;              // 重新同步跳跃宽度
    CAN_InitStructure.CAN_BS1 = u8BS1;              // BTR-TS1 时间段1
    CAN_InitStructure.CAN_BS2 = u8BS2;              // BTR-TS1 时间段2
    CAN_InitStructure.CAN_Prescaler = u16PreScale;  // BTR-BRP 波特率分频器  定义了时间单元的时间长度

    CAN_Init(stCAN, &CAN_InitStructure);
}

// 使用一个过滤器组，分成一个远程帧指令和一个系统配置用的扩展帧指令的过滤器
// 过滤器编号 0 装远程帧id，过滤器编号 1 用来做系统配置id
static bool CAN_AddExtIdlistFilter(uint8_t RTR, uint32_t filter_Id) {
  static uint32_t FilterValue[FILTER_GROUP_MAX_EXT_COUNT];
  CAN_FilterInitTypeDef CAN_FilterInitStruct;
  uint8_t filter_group =  EXT_FILTER_GROUP_NUM_START;

  // bit[32:22]-STD_ID    bit[21:3]-EXT_ID   bit[2]-IDE   bit[1]-RTR   bit[0]-0
  uint8_t ext_rtr_bit = (RTR > 0) << 1;
  uint8_t ext_ide_bit = 0x1 << 2;
  filter_Id = ext_ide_bit | ext_rtr_bit | (filter_Id << 3);
  if (RTR == CAN_RTR_Remote) {
    FilterValue[0] = filter_Id;
  } else {
    FilterValue[1] = filter_Id;
  }

  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInitStruct.CAN_FilterIdHigh = FilterValue[0] >> 16;
  CAN_FilterInitStruct.CAN_FilterIdLow =  FilterValue[0];
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = FilterValue[1] >> 16;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = FilterValue[1];
  CAN_FilterInitStruct.CAN_FilterNumber = filter_group;
  CAN_FilterInit(&CAN_FilterInitStruct);
  return true;
}

static bool CAN_AddStdIdlistFilter(uint8_t RTR, uint16_t filter_Id) {
  static uint16_t FilterValue[FILTER_GROUP_MAX_STD_COUNT];
  static uint8_t std_filter_index = 0;
  CAN_FilterInitTypeDef CAN_FilterInitStruct;
  uint8_t filter_group = (std_filter_index / FILTER_GROUP_MAX_STD_COUNT) + STD_FILTER_GROUP_NUM_START;
 
  if (filter_group > MAX_STD_FILTER_GROUP_NUM) {
    return false;
  }

  // bit[15:5]-ID   bit[4]-RTR   bit[3]-IDE   bit[2:0]-EXT_ID[17:15]
  uint8_t std_ide_bit = 0x0 << 3;
  uint8_t std_rtr_bit = (RTR > 0) << 4;
  filter_Id = std_ide_bit | std_rtr_bit | (filter_Id << 5);

  uint8_t cur_filter_index = std_filter_index % FILTER_GROUP_MAX_STD_COUNT;
  for (int i = 0; i < FILTER_GROUP_MAX_STD_COUNT; i++) {
    // FilterValue 的每个值代表一个id，将后边没用到的填充相同的数，防止误过滤其他的id
    if (i >= cur_filter_index) {
      FilterValue[i] = filter_Id;
    }
  }
  std_filter_index++;

  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;
  CAN_FilterInitStruct.CAN_FilterIdHigh = FilterValue[0];
  CAN_FilterInitStruct.CAN_FilterIdLow =  FilterValue[1];
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = FilterValue[2];
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = FilterValue[3];
  CAN_FilterInitStruct.CAN_FilterNumber = filter_group;
  CAN_FilterInit(&CAN_FilterInitStruct);
  return true;
}

bool Can_AddRemoteExtIdFilter(uint32_t id) {
  return CAN_AddExtIdlistFilter(CAN_RTR_Remote, id);
}
bool Can_AddDataExtIdFilter(uint32_t id) {
  return CAN_AddExtIdlistFilter(CAN_RTR_Data, id);
}
bool Can_AddRemoteStdIdFilter(uint16_t id) {
  return CAN_AddStdIdlistFilter(CAN_RTR_Remote, id);
}
bool Can_AddDataStdIdFilter(uint16_t id) {
  return CAN_AddStdIdlistFilter(CAN_RTR_Data, id);
}

/*
 * 函数名：CAN_Config_Init
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_ConfigInit() {

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    CAN_GPIO_Config();
    CAN_Mode_Config(CAN1, CAN_BAUD_RATE_500K);
    CAN_NVIC_Config();
}

uint8_t mbox_g = 0;
uint32_t send_out_time_g = 0;
bool can_send_flag_g = false;
CanTxMsg txMessage_g;
void CAN_SendData(CanTxMsg *TxMessage) {
    mbox_g = CAN_Transmit(CAN1, TxMessage);
    send_out_time_g = canbus_g.GetSendTime();
    can_send_flag_g = true;
}

bool CAN_GetSendStat() {
   if(!IS_CAN_TRANSMITMAILBOX(mbox_g)) {
      mbox_g = 0;
      return true;
    }
    uint8_t u8TransmitStatus = CAN_TransmitStatus(CAN1, mbox_g);
    if (can_send_flag_g == false) {
        return true;
    } else if (u8TransmitStatus == CAN_TxStatus_Ok) {
        can_send_flag_g = false;
        return true;
    } else if (u8TransmitStatus == CAN_TxStatus_Failed) {
        CAN_SendData(&txMessage_g);
    }else if(canbus_g.GetSendTime() > (send_out_time_g + 20)) {
        CAN_CancelTransmit(CAN1, mbox_g);
        CAN_SendData(&txMessage_g);
    }
    return false;
}

void HAL_CAN_try_send_remote() {
  if (!CAN_GetSendStat()) {
    return ;
  }
  if (!canbus_g.remote_send_buffer_.isEmpty()) {
    uint32  remoteId = canbus_g.remote_send_buffer_.peek();

    txMessage_g.RTR = CAN_RTR_REMOTE;
    txMessage_g.DLC = 0;
    txMessage_g.IDE = CAN_ID_EXT;
    txMessage_g.ExtId = remoteId;
    CAN_SendData(&txMessage_g);
    canbus_g.remote_send_buffer_.remove();
  }
}

void HAL_CAN_try_send_standard() {
  if (!CAN_GetSendStat()) {
    return ;
  }
  if (!canbus_g.standard_send_buffer_.isEmpty()) {
    CanTxStruct txItem = canbus_g.standard_send_buffer_.peek();
    txMessage_g.StdId = txItem.std_id | 0x600; // TODO verify interrupt base driven
    txMessage_g.IDE = CAN_ID_STD;
    txMessage_g.RTR = CAN_RTR_DATA;
    for (int i = 0; i < txItem.len; ++i) {
      txMessage_g.Data[i] = txItem.data[i];
    }
    txMessage_g.DLC = txItem.len;
    CAN_SendData(&txMessage_g);
    canbus_g.standard_send_buffer_.remove();
  }
}

void HAL_CAN_try_send_extended() {
  uint8_t  len = 0;
  if (!CAN_GetSendStat()) {
    return ;
  }
  if (!canbus_g.extended_send_buffer_.isEmpty() || len > 0) {

    while (len < CAN_DATA_LEN_LIMIT && !canbus_g.extended_send_buffer_.isEmpty()) {
      txMessage_g.Data[len++] = canbus_g.extended_send_buffer_.peek();
      canbus_g.extended_send_buffer_.remove();
    }

    if (len > 0) {
      txMessage_g.IDE = CAN_ID_EXT;
      txMessage_g.DLC = len;
      txMessage_g.RTR = CAN_RTR_DATA;
      txMessage_g.ExtId = canbus_g.extend_send_id_;
      CAN_SendData(&txMessage_g);
    }
    canbus_g.RenewExternedID();
  }
}
void HAL_CAN_try_send() {
  HAL_CAN_try_send_remote();
  HAL_CAN_try_send_standard();
  HAL_CAN_try_send_extended();
}

extern "C" {
  void __irq_usb_lp_can_rx0(void) {
    CanRxMsg rxMessage;
    CAN_Receive(CAN1, CAN_FIFO0, &rxMessage);

    if (rxMessage.RTR == CAN_RTR_REMOTE) {
      canbus_g.PushRecvRemoteData(rxMessage.ExtId, rxMessage.IDE);
    } else if (rxMessage.IDE == CAN_ID_EXT) {
      canbus_g.PushRecvExtendedData(rxMessage.Data, rxMessage.DLC);
    }
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
  }

  void __irq_can_rx1(void) {
    CanRxMsg rxMessage;
    CAN_Receive(CAN1, CAN_FIFO1, &rxMessage);
    if (rxMessage.RTR == CAN_RTR_REMOTE) {
      canbus_g.PushRecvRemoteData(rxMessage.StdId, rxMessage.IDE);
    } else if (rxMessage.IDE == CAN_ID_STD) {
      canbus_g.PushRecvStandardData(rxMessage.StdId, rxMessage.Data, rxMessage.DLC);
    }
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
  }

   void __irq_usb_hp_can_tx(void) {
      // @TODO? Clear interrupt bit? clear before send another message
      CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
//      HAL_CAN_try_send();
   }
}






