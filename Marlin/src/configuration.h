//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CONFIGURATION_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CONFIGURATION_H_

#include <stdint.h>
#define APP_VERSIONS "v1.9.1"
#define FLASH_BASE 0x8000000

#define FLASH_PAGE_COUNT (128)
#define FLASH_PAGE_SIZE  (1024)
#define FLASH_TOTAL_SIZE (FLASH_PAGE_COUNT * FLASH_PAGE_SIZE)

#define BOOT_CODE_SIZE  (20 * 1024)
#define MODULE_PARA_SIZE  (1  * 1024)
#define PUBLIC_PARA_SIZE (1  * 1024)
#define APP_PARA_SIZE (1  * 1024)

#define FLASH_BOOT_CODE   (FLASH_BASE)
#define FLASH_MODULE_PARA   (FLASH_BOOT_CODE + BOOT_CODE_SIZE)  // readonly
#define FLASH_PUBLIC_PARA (FLASH_MODULE_PARA + MODULE_PARA_SIZE)  // read & write
#define FLASH_APP_PARA    (FLASH_PUBLIC_PARA + PUBLIC_PARA_SIZE)  // read & write
#define FLASH_APP       (FLASH_APP_PARA + APP_PARA_SIZE)

#define INVALID_VALUE 9999

#define MODULE_MAC_INFO_ADDR (ModuleMacInfo *)(FLASH_MODULE_PARA)

#define  APP_VARSIONS_SIZE 32


// same is a physical serial number put in the package, so it have to compact.
typedef struct {
  uint8_t moduleId[2];
  uint8_t year[1];
  uint8_t mon[1];
  uint8_t day[1];
  uint8_t hour[2];
  uint8_t min[2];
  uint8_t sec[2];
  uint8_t random[4]; // random number, range [0 - 36^4=1679616];
  uint32_t u32random; // random number, range [0 - 36^4=1679616]; 10进制
  int32_t other_parm[4];
} ModuleMacInfo;

typedef struct {
    uint8_t versions[APP_VARSIONS_SIZE];  // 32位版本号,位置和大小不能做更改
    uint8_t parm_mark[2];  // aa 55
} AppParmInfo;

typedef enum {

} MODULE_TYPE;


typedef enum {
  CMD_M_CONFIG = 0,            // 0
  CMD_S_CONFIG_REACK,          // 1
  CMD_M_REQUEST_FUNCID,        // 2
  CMD_S_REPORT_FUNCID,         // 3
  CMD_M_CONFIG_FUNCID,         // 4
  CMD_S_CONFIG_FUNCID_REACK,   // 5
  CMD_M_UPDATE_REQUEST,        // 6
  CMD_S_UPDATE_REQUEST_REACK,  // 7
  CMD_M_UPDATE_PACKDATA,       // 8
  CMD_S_UPDATE_PACK_REQUEST,   // 9
  CMD_M_UPDATE_END,            // a
  CMD_M_VERSIONS_REQUEST,      // b
  CMD_S_VERSIONS_REACK,        // c
  CMD_M_SET_RANDOM,            // d
  CMD_S_SET_RANDOM_REACK,       // e
  CMD_M_SET_LINEAR_LEN,         // f
  CMD_S_SET_LINEAR_LEN_REACK,   // 10
  CMD_M_SET_LINEAR_LEAD,        // 11
  CMD_S_SET_LINEAR_LEAD_REACK,  // 12
  CMD_M_SET_LINEAR_LIMIT,       // 13
  CMD_S_SET_LINEAR_LIMIT_REACK, // 14
  CMD_M_UPDATE_STATUS_REQUEST,  // 15
  CMD_S_UPDATE_STATUS_REACK,    // 16
  CMD_M_UPDATE_START,           // 17
  CMD_M_DEBUG_INFO = 0xFE,
  CMD_S_DEBUG_INFO = 0xFF,
} SYSTEM_CMD;


typedef enum {

} FUNC_ID;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CONFIGURATION_H_
