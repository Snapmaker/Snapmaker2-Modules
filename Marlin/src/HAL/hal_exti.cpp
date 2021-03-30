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
#include "hal_exti.h"
#include "hal_gpio.h"
#include "stdio.h"

EXTI_CB_F exti_cb[16] = {NULL};

static uint8_t PinToExtiIRQ(uint8_t pin_source) {
  uint8_t exti_irqn = EXTI0_IRQn;
  if (pin_source <= 4) {
    exti_irqn += pin_source;
  } else if (pin_source <= 9) {
    exti_irqn = EXTI9_5_IRQn;
  } else {
    exti_irqn = EXTI15_10_IRQn;
  }
  return exti_irqn;
}

void ExtiNvicInit(uint8_t pin_source)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
  uint8_t exti_reqn = PinToExtiIRQ(pin_source);
  NVIC_InitStructure.NVIC_IRQChannel = exti_reqn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

uint8_t ExtiInit(uint8_t pin, EXTI_MODE_E exti_mode, EXTI_CB_F cb) {
  uint8_t port_source = pin / 16;
  uint8_t pin_source = pin % 16;
  EXTI_InitTypeDef   EXTI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GpioInit(pin, GPIO_Mode_IN_FLOATING);
  GPIO_EXTILineConfig(port_source, pin_source);

  EXTI_InitStructure.EXTI_Line = 1 << pin_source;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = (EXTITrigger_TypeDef)exti_mode;

  EXTI_Init(&EXTI_InitStructure);
  ExtiNvicInit(pin_source);
  exti_cb[pin_source] = cb;
  return pin_source;
}

uint8_t SoftExtiInit(SOFT_EXTI_LINE_E exti_line, EXTI_CB_F cb, uint8_t pro, uint8_t sub_pro) {
  EXTI_InitTypeDef   EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint8_t irq = PinToExtiIRQ(exti_line);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  EXTI_InitStructure.EXTI_Line = 1 << exti_line;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;

	EXTI_Init(&EXTI_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = irq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = pro;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub_pro;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  exti_cb[exti_line] = cb;
  return exti_line;
}

void SoftExtiTrigger(SOFT_EXTI_LINE_E exti_line) {
  uint8_t irq_line = 1 << exti_line;
  EXTI_GenerateSWInterrupt(irq_line);
}

extern "C" {

void __irq_exti0(void) {
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
    if (exti_cb[0])
      exti_cb[0](0);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void __irq_exti1(void) {
	if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
    if (exti_cb[1])
      exti_cb[1](1);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void __irq_exti2(void) {
	if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
    if (exti_cb[2])
      exti_cb[2](2);
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void __irq_exti3(void) {
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) {
    if (exti_cb[3])
      exti_cb[3](3);
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

void __irq_exti4(void) {
	if(EXTI_GetITStatus(EXTI_Line4) != RESET) {
    if (exti_cb[4])
      exti_cb[4](4);
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

void __irq_exti9_5(void) {
    if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
      if (exti_cb[5])
        exti_cb[5](5);
      EXTI_ClearITPendingBit(EXTI_Line5);
    }
    if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
      if (exti_cb[6])
        exti_cb[6](6);
      EXTI_ClearITPendingBit(EXTI_Line6);
    }
    if(EXTI_GetITStatus(EXTI_Line7) != RESET) {
      if (exti_cb[7])
        exti_cb[7](7);
      EXTI_ClearITPendingBit(EXTI_Line7);
    }
    if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
      if (exti_cb[8])
        exti_cb[8](8);
      EXTI_ClearITPendingBit(EXTI_Line8);
    }
    if(EXTI_GetITStatus(EXTI_Line9) != RESET) {
      if (exti_cb[9])
        exti_cb[9](9);
      EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

void __irq_exti15_10(void) {
    if(EXTI_GetITStatus(EXTI_Line10) != RESET) {
      if (exti_cb[10])
        exti_cb[10](10);
      EXTI_ClearITPendingBit(EXTI_Line10);
    }
    if(EXTI_GetITStatus(EXTI_Line11) != RESET) {
      if (exti_cb[11])
        exti_cb[11](11);
      EXTI_ClearITPendingBit(EXTI_Line11);
    }
    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
      if (exti_cb[12])
        exti_cb[12](12);
      EXTI_ClearITPendingBit(EXTI_Line12);
    }
    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
      if (exti_cb[13])
        exti_cb[13](13);
      EXTI_ClearITPendingBit(EXTI_Line13);
    }
    if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
      if (exti_cb[14])
        exti_cb[14](14);
      EXTI_ClearITPendingBit(EXTI_Line14);
    }
    if(EXTI_GetITStatus(EXTI_Line15) != RESET) {
      if (exti_cb[15])
        exti_cb[15](15);
      EXTI_ClearITPendingBit(EXTI_Line15);
    }
}

}
