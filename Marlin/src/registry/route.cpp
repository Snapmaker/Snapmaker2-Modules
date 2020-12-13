//
// Created by David Chen on 2019-07-23.
//

#include <src/core/can_bus.h>
#include "route.h"

#define FUNC_LIST_INIT(list)  func_list_ = list;\
                              func_count_ = sizeof(list) / sizeof(list[0]);

Route routeInstance;
void Route::Init() {
  uint32_t moduleType = registryInstance.module();

  switch (moduleType) {
  }
}

void Route::Invoke() {
  uint16_t func_id = contextInstance.funcid_;
  uint8_t * data = contextInstance.data_;
  uint8_t   data_len = contextInstance.len_;
  module_->HandModule(func_id, data, data_len);
}

void Route::ModuleLoop() {
  module_->Loop();
}

