//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_ROUTE_H_

#include <src/configuration.h>
#include <src/device/temperature.h>
#include <src/module/print_head.h>
#include <src/device/fan.h>
#include <src/module/linear_module.h>
#include "registry.h"
#include "context.h"
#include "src/module/laser_head.h"
#include "src/module/cnc_head.h"
#include "src/module/light_module.h"
#include "src/module/cnc_tool_setting.h"
#include "src/module/enclosure.h"
#include "src/module/fan_module.h"
#include "src/module/purifier_module.h"
#include "src/module/stop_module.h"
#include "src/module/rotate_module.h"
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
