//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_CONTEXT_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_CONTEXT_H_

#include <include/libmaple/libmaple_types.h>
#include <src/configuration.h>
#include "registry.h"

// Data for invocation

class Context {
 public:
  uint16_t funcid_;
  uint16_t msgid_;
  uint8_t * data_;
  uint32 len_;

  MODULE_TYPE module() {
    return registryInstance.module();
  }
};

extern Context contextInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_CONTEXT_H_
