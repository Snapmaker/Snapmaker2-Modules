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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_fab_base_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_DEVICE_fab_base_H_

#include <stdint.h>
#include <src/HAL/hal_tim_ic.h>
#include "fan.h"

typedef struct {
    uint32_t start_time_;
    uint32_t last_time_;
    volatile uint32_t record_pulse_;
    volatile uint32_t current_pulse_;
} feed_back_t;


class FanFeedBack : public Fan {
    public:
        FanFeedBack() { fab_base_ = new Fan(); }
        virtual ~FanFeedBack() { delete fab_base_; }

        void Init(uint8_t fan_pin, uint8_t ic_tim, uint8_t ic_ch, uint8_t it_type, uint16_t threshold);
        void ChangePwm(uint8_t threshold, uint16_t delay_close_time_s);
        void Loop();

        void set_fb_threshold(uint16_t threshold) { threshold_ = threshold; }
        bool get_feed_back_state() { return fb_result; }

    private:
        Fan* fab_base_   = nullptr;
        bool fb_check_   = false;
        bool fb_result   = true;
        uint8_t ic_timer = 0xFF;
        uint16_t threshold_ = 0;

        static feed_back_t fb;
        static void ic_isr_cb();
        static void update_isr_cb();
};

#endif