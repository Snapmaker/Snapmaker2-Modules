//
// Created by David Chen on 2019-07-16.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FEATURE_BASE_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FEATURE_BASE_H_

class DeviceBase {
 public:
  DeviceBase() {

  }
  ~DeviceBase() {

  }

  virtual void Init() = 0;
  virtual void Test() = 0;
};

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_FEATURE_BASE_H_
