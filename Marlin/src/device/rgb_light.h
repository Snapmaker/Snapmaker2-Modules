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
#ifndef MODULES_SRC_DEVICE_RGB_LIGHT_H_
#define MODULES_SRC_DEVICE_RGB_LIGHT_H_

#include <stdint.h>
#include "src/HAL/hal_RGB.h"
#define TIM_FREQUENCY 100000

typedef enum {
  MODE_BREATH,
  MODE_WATERFALL,
  MODE_STATIC,
  MODE_FLICKER,
}LIGHT_MODE_t;

class RGBLight {
 public:
  void Init(uint8_t pin, uint8_t light_count, SOFT_EXTI_LINE_E exti);
  void BreathLight(RGB_T min_color, RGB_T max_color, uint32_t breathe_period_ms);
  void WaterfallLight(RGB_T *rgb_list, uint32_t delay_ms);
  void StaticLight(RGB_T *rgb_list);
  void StaticLight(RGB_T rgb);
  void FlickeringLight(RGB_T rgb, uint32_t delay_ms);
  void Loop();

 private:
  void FullColor(RGB_T rgb);
  void BreathProcess();
  void WaterfallProcess();
  void StaticProcess();
  void FlickeringProcess();
  void CopyRGB(RGB_T *dst, RGB_T *src, uint8_t count);

 private:
  uint8_t light_count_ = 0;
  uint8_t light_pin_;
  RGB_T * light_list_;
  uint32_t execute_time_ = 0;
  float one_step_[3];
  uint32_t total_step_= 0;
  uint32_t cur_step_ = 0;
  bool dir_ = true;
  uint32_t breathe_min_pwm[3];
  uint32_t breathe_max_pwm[3];

  uint32_t wait_ms_ = 0;

  LIGHT_MODE_t cur_mode_ = MODE_STATIC;
};

#endif
