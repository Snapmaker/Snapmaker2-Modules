//
// Created by David Chen on 2019-07-18.
//

#include "pid.h"


void Pid::Init(float p, float i, float d) {
  k1_ = 0.95;
  k2_ = 1 - k1_;

  bang_threshold_ = 20;
  bang_max_ = 255;
  pid_max_ = 255;

  i_sum_min_ = 0;
  i_sum_max_ = 0;
  d_term_ = 0;

  target_ = 0;
  this->k_p(p);
  this->k_i(i);
  this->k_d(d);
}

void Pid::Refresh() {
  if (k_i_ != 0) {
    i_sum_max_ = pid_max_ / k_i_;
  } else {
    i_sum_max_ = 0;
  }

  i_sum_ = 0;
}

uint32_t Pid::output(float actual) {
  float ret_val = 0;
  float  err = target_ - actual;
  float p_term = 0;
  float i_term = 0;

  d_term_ = k2_ * k_d_ * (actual - pre_err_) + k1_ * d_term_;
  pre_err_ = actual;

  if ((actual > MAX_TEMPERATURE) || (actual < MIN_TEMPERATURE)) {
    ret_val = 0;
    i_sum_ = 0;
  } else if (err > bang_threshold_) {
    ret_val = bang_max_;
    i_sum_ = 0;
  } else if ((err < (-bang_threshold_)) || (target_ == 0)) {
    ret_val = 0;
    i_sum_ = 0;
  } else {
    p_term = k_p_ * err;
    i_sum_ += err;

    if (i_sum_ < i_sum_min_) {
      i_sum_ = i_sum_min_;
    } else if (i_sum_ > i_sum_max_) {
      i_sum_ = i_sum_max_;
    }

    i_term = k_i_ * i_sum_;
    ret_val = p_term + i_term - d_term_;

    // if exceed limit, then undo integral calculation
    if (ret_val > pid_max_) {
      if (err > 0) {
        i_sum_ -= err;
      }
      ret_val = pid_max_;
    } else if (ret_val < 0) {
      if (err < 0) {
        i_sum_ -= err;
      }
      ret_val = 0;
    }
  }

  return ((uint32_t ) ret_val);
}
void Pid::target(int32_t target) {
  if (target > MAX_TARGET_TEMPERATURE) {
    target = MAX_TARGET_TEMPERATURE;
  } else if (target < MIN_TARGET_TEMPERATURE) {
    target = MIN_TARGET_TEMPERATURE;
  }
  this->target_ = target;
}
void Pid::k_p(float k_p) {
  this->k_p_ = k_p;
}

void Pid::k_i(float k_i) {
  this->k_i_ = k_i;
  Refresh();
}

void Pid::k_d(float k_d) {
  this->k_d_ = k_d;
}
uint32_t Pid::getTarget() {
  return target_;
}




