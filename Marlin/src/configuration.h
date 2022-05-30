/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_CONFIGURATION_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_CONFIGURATION_H_

#include <stdint.h>
#define APP_VERSIONS "v1.12.1"
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

#define LASER_DEFAULT_HIGH 15000
#define TEMP_DEFAULT_KP 13
#define TEMP_DEFAULT_KI 0.016
#define TEMP_DEFAULT_KD 106.25

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
  uint8_t hw_version;
} ModuleMacInfo;

typedef struct {
    uint8_t versions[APP_VARSIONS_SIZE];  // 32位版本号,位置和大小不能做更改
    uint8_t parm_mark[2];  // aa 55
    float temp_P;
    float temp_I;
    float temp_D;
    uint16_t laser_high;
    uint16_t laser_high_4_axis;
    uint8_t purifier_lifetime;
    uint8_t purifier_forced_run;
    uint8_t purifier_fan_gears;
    int8_t laser_protect_temp;
    uint32_t module_sync_id;
    int8_t laser_recovery_temp;
} AppParmInfo;

typedef enum {
  MODULE_PRINT             = 0,  // 0
  MODULE_CNC               = 1,  // 1
  MODULE_LASER             = 2,  // 2
  MODULE_LINEAR            = 3,  // 3
  MODULE_LIGHT             = 4,  // 4
  MODULE_ENCLOSURE         = 5,  // 5
  MODULE_ROTATE            = 6,  // 6
  MODULE_PURIFIER          = 7,  // 7
  MODULE_EMERGENCY_STOP    = 8,  // 8
  MODULE_CNC_TOOL_SETTING  = 9,  // 9
  MODULE_PRINT_V_SM1       = 10, // 10
  MODULE_FAN               = 11, // 11
  MODULE_LINEAR_TMC        = 12, // 12
  MODULE_3DP_DUAL_EXTRUDER = 13, // 13
  MODULE_LASER_10W         = 14, // 14
  MODULE_CNC_200W          = 15, // 15
  MODULE_ENCLOSURE_A400    = 16, // 16 
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
    FUNC_REPORT_LIMIT          ,  // 0
    FUNC_REPORT_PROBE          ,  // 1
    FUNC_REPORT_CUT            ,  // 2
    FUNC_SET_STEP_CTRL         ,  // 3
    FUNC_SET_MOTOR_SPEED       ,  // 4
    FUNC_REPORT_MOTOR_SPEED    ,  // 5
    FUNC_REPORT_TEMPEARTURE    ,  // 6
    FUNC_SET_TEMPEARTURE       ,  // 7
    FUNC_SET_FAN               ,  // 8
    FUNC_SET_FAN2              ,  // 9
    FUNC_SET_PID               ,  // 10
    FUNC_SET_CAMERA_POWER      ,  // 11
    FUNC_SET_LASER_FOCUS       ,  // 12
    FUNC_REPORT_LASER_FOCUS    ,  // 13
    FUNC_SET_LIGHT_COLOR       ,  // 14
    FUNC_REPORT_ENCLOSURE      ,  // 15
    FUNC_REPORT_TEMP_PID       ,  // 16
    FUNC_REPORT_TOOL_SETTING   ,  // 17
    FUNC_SET_ENCLOSURE_LIGHT   ,  // 18
    FUNC_SET_FAN_MODULE        ,  // 19
    FUNC_REPORT_STOP_SWITCH    ,  // 20
    FUNC_TMC_IOCTRL            ,  // 21
    FUNC_TMC_PUBLISH           ,  // 22
    FUNC_SET_PURIFIER          ,  // 23
    FUNC_REPORT_PURIFIER       ,  // 24
    FUNC_SET_AUTOFOCUS_LIGHT   ,  // 25
    FUNC_REPORT_SECURITY_STATUS,  // 26
    FUNC_MODULE_ONLINE_SYNC    ,  // 27
    FUNC_MODULE_SET_TEMP       ,  // 28
    FUNC_MODULE_LASER_CTRL     ,  // 29
    FUNC_MODULE_GET_HW_VERSION ,  // 30
    FUNC_REPORT_PIN_STATUS     ,  // 31
    FUNC_CONFIRM_PIN_STATUS    ,  // 32
    FUNC_SET_MOTOR_SPEED_RPM  =   38,  // 38 
    FUNC_SET_MOTOR_CTR_MODE         ,  // 39 
    FUNC_SET_MOTOR_RUN_DIRECTION    ,  // 40 
    FUNC_REPORT_MOTOR_STATUS_INFO   ,  // 41
    FUNC_REPORT_MOTOR_SENSOR_INFO   ,  // 42
    FUNC_REPORT_MOTOR_SELF_TEST_INFO = 53,  // 53
} FUNC_ID;

typedef enum {
    SET_P_INDEX,
    SET_I_INDEX,
    SET_D_INDEX,
}PID_SETINDEX_E;

#endif //MODULES_WHIMSYCWD_MARLIN_SRC_CONFIGURATION_H_
