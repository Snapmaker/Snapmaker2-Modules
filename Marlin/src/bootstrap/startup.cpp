//
// Created by David Chen on 2019-07-19.
//

#include <src/configuration.h>
#include <src/core/utils.h>
#include <src/registry/registry.h>
#include <src/core/can_bus.h>
#include <src/registry/route.h>
#include "startup.h"

uint32_t Startup::SelfDetct() {
  registryInstance.Init();
  return 0;
}

void Startup::BasePeriphInit() {
  canbus_g.Init(registryInstance.ModuleCanId());
}
void Startup::PeriphInit() {

  routeInstance.Init();
}
void Startup::FuncIdListInit() {
  registryInstance.InitlizeFuncIds();
}

Startup startupInstance;
