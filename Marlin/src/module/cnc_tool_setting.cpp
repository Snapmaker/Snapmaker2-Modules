//
// Created by David Chen on 2019-08-03.
//

#include <board/board.h>
#include "src/module/cnc_tool_setting.h"
#include "src/configuration.h"


void CncToolSetting::Init() {
  this->switch_.Init(PA7);
}

void CncToolSetting::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  switch (func_id) {
    case FUNC_REPORT_TOOL_SETTING:
      this->switch_.ReportStatu(FUNC_REPORT_TOOL_SETTING);
      break;
  }
}

void CncToolSetting::Loop() {
  if (this->switch_.CheckStatuLoop()) {
    this->switch_.ReportStatu(FUNC_REPORT_TOOL_SETTING);
  }
}