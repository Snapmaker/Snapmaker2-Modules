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
#include <io.h>
#include "src/device/soft_pwm.h"
#include "src/device/rgb_light.h"
#include <wirish_time.h>

#define BREATH_ONCE_TIME_MS 5

void RGBLight::FullColor(RGB_T rgb) {
  for (uint8_t i = 0; i < light_count_; i++) {
      light_list_[i] = rgb;
  }
  HAL_SetAllRGB(light_pin_, light_list_, light_count_);
}

void RGBLight::Loop() {
  switch (cur_mode_) {
    case MODE_BREATH:
      BreathProcess();
      break;
    case MODE_STATIC:
      StaticProcess();
      break;
    case MODE_WATERFALL:
      WaterfallProcess();
      break;
    case MODE_FLICKER:
      FlickeringProcess();
    default:
      break;
  }
}

void RGBLight::CopyRGB(RGB_T *dst, RGB_T *src, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    dst[i] = src[i];
  }
}

void RGBLight::Init(uint8_t pin, uint8_t light_count, SOFT_EXTI_LINE_E exti) {
  RGB_T default_color = {0, 0, 0};
  light_count_ = light_count;
  light_pin_ = pin;
  light_list_ = new RGB_T[light_count_];
  HAL_RGBInit(light_pin_, exti);
  FullColor(default_color);
  cur_mode_ = MODE_STATIC;
}

void RGBLight::BreathLight(RGB_T min_color, RGB_T max_color, uint32_t breathe_period_ms) {
  uint8_t i = 0;
  uint32_t pwm_period = SOFT_PWM_MS(BREATH_ONCE_TIME_MS);
  total_step_ = breathe_period_ms / BREATH_ONCE_TIME_MS / 2;
  uint8_t * p_min = (uint8_t *)&min_color;
  uint8_t * p_max = (uint8_t *)&max_color;
  for (i = 0; i < 3; i++) {
    if (p_min[i] > 255)
      p_min[i] = 0;
    if (p_max[i] < p_min[i] || p_max[i] > 255)
      p_max[i] = 255;
    breathe_min_pwm[i] = pwm_period * p_min[i] / 255;
    breathe_max_pwm[i] = pwm_period * p_max[i] / 255;
    one_step_[i] = (float)(breathe_max_pwm[i] - breathe_min_pwm[i]) / total_step_;
  }
  cur_mode_ = MODE_BREATH;
  cur_step_ = 0;
  dir_ = true;
}


void RGBLight::WaterfallLight(RGB_T *color, uint32_t delay_ms) {
  CopyRGB(light_list_, color, light_count_);
  wait_ms_ = delay_ms;
  cur_mode_ = MODE_WATERFALL;
}

void RGBLight::StaticLight(RGB_T *color) {
  CopyRGB(light_list_, color, light_count_);
  HAL_SetAllRGB(light_pin_, light_list_, light_count_);
  cur_mode_ = MODE_STATIC;
}

void RGBLight::StaticLight(RGB_T rgb) {
  FullColor(rgb);
  cur_mode_ = MODE_STATIC;
}

void RGBLight::FlickeringLight(RGB_T rgb, uint32_t delay_ms) {
  FullColor(rgb);
  wait_ms_ = delay_ms;
  cur_mode_ = MODE_FLICKER;
}

void RGBLight::BreathProcess() {
  if ((execute_time_ + BREATH_ONCE_TIME_MS) < millis()) {
    execute_time_ = millis();
    uint8_t out[3];
    for (int i = 0; i < 3; i++) {
      uint32_t temp = (uint32_t)(cur_step_ * one_step_[i]) + breathe_min_pwm[i];
      out[i] = temp > 255 ? 255 : temp;
    }
    FullColor(*((RGB_T *)out));
    if (dir_) {
      cur_step_++;
      if (cur_step_ >= (total_step_)) {
        dir_ = false;
      }
    } else {
      cur_step_--;
      if (cur_step_ == 0) {
        dir_ = true;
      }
    }
  }
}

void RGBLight::WaterfallProcess() {
  if ((execute_time_ + wait_ms_) < millis()) {
    execute_time_ = millis();
    HAL_SetAllRGB(light_pin_, light_list_, light_count_);
    RGB_T temp_color = light_list_[light_count_ - 1];
    for (uint8_t i = 1; i < light_count_; i++) {
      light_list_[light_count_ - i] = light_list_[light_count_ - i - 1];
    }
    light_list_[0] = temp_color;
  }
}

void RGBLight::StaticProcess() {
  if (execute_time_ < millis()) {
    execute_time_ = millis() + 300;
    HAL_SetAllRGB(light_pin_, light_list_, light_count_);
  }
}

void RGBLight::FlickeringProcess() {
  static uint8_t flag = 0;
  if (execute_time_ < millis()) {
    execute_time_ = millis() + wait_ms_;
    if (flag) {
      HAL_SetAllRGB(light_pin_, light_list_, light_count_);
    } else {
      RGB_T rgb[] = {{0,0,0}, {0,0,0}, {0,0,0},  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
      HAL_SetAllRGB(light_pin_, rgb, light_count_);
    }
    flag = !flag;
  }
}

