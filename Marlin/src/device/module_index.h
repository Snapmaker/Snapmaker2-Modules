#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_MODULE_INDEX_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_MODULE_INDEX_H_

#include <stdint.h>

#define  MODULE_INDEX_NONE 0xff
#define  PIN_MODULE_INDEX PB7
class ModuleIndex {
 public:
  void Init();
  uint8_t SetModuleIndex(uint8_t axis_index);
 private:
  uint8_t module_index_ = MODULE_INDEX_NONE;
};

extern ModuleIndex moduleIndex;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_SWITCH_H_
