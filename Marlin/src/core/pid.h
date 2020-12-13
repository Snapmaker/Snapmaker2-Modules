//
// Created by David Chen on 2019-07-18.
//

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
