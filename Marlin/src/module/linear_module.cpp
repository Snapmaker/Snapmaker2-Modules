//
// Created by David Chen on 2019-08-03.
//

#include <board/board.h>
#include "linear_module.h"
#include "src/configuration.h"
#include <io.h>

void LinearModule::Init() {
  this->limit_.Init(PB1);

  // set the tmc2209 to low power mode
  pinMode(PA9, OUTPUT);
  pinMode(PA10, OUTPUT);
  digitalWrite(PA9, 0);
  digitalWrite(PA10, 0);
}

void LinearModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_LIMIT:
      this->limit_.ReportStatus(FUNC_REPORT_LIMIT);
      break;
  }
}

void LinearModule::Loop() {
  if (this->limit_.CheckStatusLoop()) {
    this->limit_.ReportStatus(FUNC_REPORT_LIMIT);
  }
}