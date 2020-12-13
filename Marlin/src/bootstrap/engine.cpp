//
// Created by David Chen on 2019-07-19.
//

#include <src/registry/registry.h>
#include <src/core/can_bus.h>
#include <src/registry/route.h>
#include "engine.h"

void Engine::Run() {
  bool loop_flag = true;

  while (loop_flag) {
    canbus_g.Handler();
    registryInstance.ConfigHandler();
    registryInstance.ServerHandler();
    registryInstance.SystemHandler();
    routeInstance.ModuleLoop();
  }
}

Engine engineInstance;