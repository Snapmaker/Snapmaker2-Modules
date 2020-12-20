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

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PID_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PID_H_

#include <cstdint>

#define MAX_TARGET_TEMPERATURE 275
#define MIN_TARGET_TEMPERATURE 0

#define MAX_TEMPERATURE 300
#define MIN_TEMPERATURE 0


class Pid {
 public:
  void Init(float p, float i, float d);
  void target(int32_t target);
  void k_p(float kP);
  void k_i(float kI);
  void k_d(float kD);

  uint32_t output(float actual);

  uint32_t getTarget();
  float k_p_;
  float k_i_;
  float k_d_;
 private:
  float k1_;
  float k2_;

  float  target_ = 0;


  int32_t bang_threshold_ = 0;
  int32_t bang_max_ = 0;
  int32_t pid_max_ = 0;

  // above need initialization

  float err_ = 0;
  float pre_err_ = 0;
  float i_sum_ = 0;

  float d_term_ = 0;
  float i_sum_min_ = 0;
  float i_sum_max_ = 0;

  int32_t output_value_ = 0;

  void Refresh();
};

extern Pid pidInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CORE_PID_H_
