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

#include <wirish_time.h>
#include "fan_fb.h"

feed_back_t FanFeedBack::fb = {0, 0, 0, 0};


void FanFeedBack::Init(uint8_t fan_pin, uint8_t ic_tim, uint8_t ic_ch, uint8_t it_type, uint16_t threshold) {
    fab_base_->Init(fan_pin);

    ic_timer = ic_tim;
    HAL_timer_init(ic_tim, 7200, 10000);
    HAL_timer_nvic_ic_init(ic_tim, 2, 2, it_type | TIM_IT_UPDATE);
    HAL_timer_ic_init(ic_tim, ic_ch, &FanFeedBack::ic_isr_cb, &FanFeedBack::update_isr_cb);
    set_fb_threshold(threshold);
}


void FanFeedBack::ChangePwm(uint8_t threshold, uint16_t delay_close_time_s) {
    fab_base_->ChangePwm(threshold, delay_close_time_s);

    if (threshold) {
        fb_check_ = true;
        fb_result = true;
        if (ic_timer < TIM_MAX) {
            fb.record_pulse_ = 0;
            fb.current_pulse_ = 0xffff;
            fb.start_time_ = millis();
            fb.last_time_ = fb.start_time_;
            HAL_timer_enable(ic_timer);
        }
    } else {
        fb_check_ = false;
        fb_result = true;
        if (ic_timer < TIM_MAX)
            HAL_timer_disable(ic_timer);
    }
}


void FanFeedBack::ic_isr_cb(void) {
    ++fb.record_pulse_;
}


void FanFeedBack::update_isr_cb(void) {

    if ((uint32_t)(millis() - fb.start_time_) > 3000)
        fb.current_pulse_ = fb.record_pulse_;

    fb.last_time_ = millis();
    fb.record_pulse_ = 0;
}


void FanFeedBack::Loop(void) {
    fab_base_->Loop();

    if (fb_check_) {
        if ((uint32_t)(millis() - fb.last_time_) > 3000) {
            fb_result = false;
            return;
        }

        if (fb.current_pulse_ != 0xffff && fb.current_pulse_ < threshold_) {
            fb_result = false;
        }
    }
}