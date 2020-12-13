//
// Created by David Chen on 2019-07-19.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_BOOTSTRAP_STARTUP_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_BOOTSTRAP_STARTUP_H_

class Startup {
 public:
  uint32_t SelfDetct();
  void PeriphInit();
  void BasePeriphInit();
  void FuncIdListInit();
};

extern Startup startupInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_BOOTSTRAP_STARTUP_H_
