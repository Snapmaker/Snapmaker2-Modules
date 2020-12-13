  #include <io.h>
  #include "src/device/module_index.h"
  void ModuleIndex::Init() {
    pinMode(PIN_MODULE_INDEX, INPUT_FLOATING);
  }
  uint8_t ModuleIndex::SetModuleIndex(uint8_t axis_index) {
    module_index_ = MODULE_INDEX_NONE;
    if (digitalRead(PIN_MODULE_INDEX) != 0) {
      module_index_ = axis_index;
    }
    return module_index_ != MODULE_INDEX_NONE;
  }
  ModuleIndex moduleIndex;