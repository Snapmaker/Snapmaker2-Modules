//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_

#include <src/configuration.h>
#include "registry.h"
#include "context.h"
#include "src/module/module_base.h"
class Route {
 public:
  void Invoke();
  void Init();
  void ModuleLoop();

 public:
  ModuleBase * module_;
  uint16_t const * func_list_ = NULL;
  uint8_t func_count_ = 0;
 private:
};

extern Route routeInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_
