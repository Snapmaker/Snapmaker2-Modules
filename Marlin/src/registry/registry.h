//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_REGISTRY_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_REGISTRY_H_

#include <stdint.h>
#include <src/configuration.h>

#define FUNC_MAX_LEN 30

class Registry {

 public:
  void Init();
  void InitlizeFuncIds();
  void ServerHandler();
  void ConfigHandler();
  void set_module_id(uint16_t moduleId);
  void set_random_id(uint32_t randomId);
  MODULE_TYPE module();
  void ModuleInfoInit();
  uint32_t ModuleCanId();
  void SystemHandler();
  uint16_t FuncId2MsgId(uint16_t funcid);

  // funcIds and len are return values
  void ReportFunctionIds();
  void ReportModuleIndex(uint8_t * data);
  void RegisterMsgId(uint8_t * data);
  void ReportVersions(uint8_t * data);
  void IsUpdate(uint8_t * data);
  void ReportFuncidAndMsgid();
  bool IsConnect();
  void SetConnectTimeout(uint32_t timeout);
 private:
  void ReadMacInfo(ModuleMacInfo * mac);
  void WriteMacInfo(ModuleMacInfo  *mac);
  void ReportMacInfo(uint8_t type, uint8_t cmd);
  void SetMacRandom(uint8_t *data);
  void SetLinearModuleLen(uint8_t *data);
  void SetLinearModuleLead(uint8_t *data);
  void SetLinearModuleLimit(uint8_t *data);
  void RenewMoudleID();
  void SendUpdateRequest();
  void SysUpdate(uint8_t *versions);
  void EnableAPP();
  void Heartbeat();
  uint32_t MsgId2FuncId(uint32_t msgId);
 private:
  uint32_t module_can_id_;  // moudleId + random number
  uint32_t module_id_;
  uint32_t randome_id_;
  uint32_t last_recv_time_ = 0;
  bool     is_configured = false;
  uint16_t func_ids_[FUNC_MAX_LEN];
  uint16_t len_ = 0;
  uint16_t msg_ids_[FUNC_MAX_LEN];
  uint32_t timeout_ms_ = 2000;
};

extern Registry registryInstance;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_REGISTRY_REGISTRY_H_
