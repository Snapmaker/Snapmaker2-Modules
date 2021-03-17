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
#include "src/module/light_module.h"
#include <io.h>

void LightModule::Init() {
  this->r_chn_ = PWM_CH1;
  HAL_PwmInit(LED_PWM_TIM, this->r_chn_ , LED_R_PIN, 100000, 256);
  this->g_chn_ = PWM_CH2;
  HAL_PwmInit(LED_PWM_TIM, this->g_chn_ , LED_G_PIN, 100000, 256);
  this->b_chn_ = PWM_CH3;
  HAL_PwmInit(LED_PWM_TIM, this->b_chn_ , LED_B_PIN, 100000, 256);
}

void LightModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  if (data[0] == 0) {
    switch (data[1]) {
      case LED_COLOR_OFF :
        this->SetRGB(LED_COLOR_OFF_RGB);
        break;
      case LED_COLOR_ALL :
        this->SetRGB(LED_COLOR_ALL_RGB);
        break;
      case LED_COLOR_RED :
        this->SetRGB(LED_COLOR_RED_RGB);
        break;
      case LED_COLOR_GREEN :
        this->SetRGB(LED_COLOR_GREEN_RGB);
        break;
      case LED_COLOR_BLUE :
        this->SetRGB(LED_COLOR_BLUE_RGB);
        break;
    }
  } else {
    this->SetRGB(data[1], data[2], data[3]);
  }
}

void LightModule::Loop() {

}

void LightModule::SetColor(uint8_t chn, uint8_t val) {
  HAL_PwmSetPulse(LED_PWM_TIM, chn, val);
}

void LightModule::SetRGB(uint8_t r, uint8_t g, uint8_t b) {
  this->SetColor(this->r_chn_, r);
  this->SetColor(this->g_chn_, g);
  this->SetColor(this->b_chn_, b);
}