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
#include "hal_RGB.h"
#include "hal_gpio.h"
#include "hal_exti.h"

static GPIO_TypeDef * rgb_port = NULL;
static uint16_t rgb_pin;
static SOFT_EXTI_LINE_E soft_exti_line;
static uint8_t is_busy = false;
void HAL_ShowRGB(uint8_t exti);
// pin: eg. PA1
void HAL_RGBInit(uint8_t pin, SOFT_EXTI_LINE_E exti) {
  GpioInit(pin, GPIO_Mode_Out_PP);
  SoftExtiInit(exti, HAL_ShowRGB);
  soft_exti_line = exti;
}

static void rgb_delay(__IO uint32_t time) {
  while(time--);
}

static void RGBHigh() {
  // T1H 580ns ~ 1µs
  // T1L 580ns ~ 1µs
  rgb_port->BSRR = rgb_pin;
  rgb_delay(4);
  rgb_port->BRR = rgb_pin;
  rgb_delay(1);
}

static void RGBLow() {
  // T0H 220ns ~ 380ns
  // T0L 580ns ~ 1µs
  rgb_port->BSRR = rgb_pin;
  rgb_delay(0);
  rgb_port->BRR = rgb_pin;
  rgb_delay(0);
}

static RGB_T *cur_rgb;
static uint8_t cur_light_count = 0;

void HAL_SetAllRGB(uint8_t pin, RGB_T *rgb, uint8_t light_count) {
  rgb_port = GpioGetPort(pin);
  rgb_pin = (1 << (pin % 16));
  cur_rgb = rgb;
  cur_light_count = light_count;
  while (is_busy);
  SoftExtiTrigger(soft_exti_line);
  while (is_busy);
  rgb_delay(1000);
}

void HAL_ShowRGB(uint8_t exti)
{
  uint32_t i = 0, k = 0;
  uint8_t r, g, b;
  is_busy = true;
  for (k = 0; k < cur_light_count; k ++) {
    r = cur_rgb[k].r;
    g = cur_rgb[k].g;
    b = cur_rgb[k].b;

    for (i = 0; i < 8; i++) {
      if (g & (1 << (7 - i))) {
        RGBHigh();
      } else {
        RGBLow();
      }
    }

    for (i = 0; i < 8; i++) {
      if (r & (1 << (7 - i))) {
        RGBHigh();
      } else {
        RGBLow();
      }
    }

    for (i = 0; i < 8; i++) {
      if (b & (1 << (7 - i))) {
        RGBHigh();
      } else {
        RGBLow();
      }
    }
  }
  is_busy = false;
}
