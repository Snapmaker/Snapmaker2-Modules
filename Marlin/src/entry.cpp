
#include <Arduino.h>
#include <src/bootstrap/engine.h>
#include <src/bootstrap/startup.h>
#include "src/HAL/hal_reset.h"
/*
 * Below line mush define to override default weak definition in `boards_setup.cpp`, or CAN can't work as expected.
 * Because by default, maple enable USB port. PA12 should be saved for CAN usage.
 */
namespace wirish {
  namespace priv {
    void board_setup_usb(void) {
    }
  }
}

void setup() {
  /*Initialization process
    1 init module id
    2 init Class Route module variable
    3 init module
    4 init func id list
  */
  
  startupInstance.SelfDetct();
  startupInstance.BasePeriphInit();
  startupInstance.PeriphInit();
  startupInstance.FuncIdListInit();
}

void loop() {
  // won't return
  //HAL_reset();
  engineInstance.Run();
}