
#ifndef FIRMWARE_USER_COMMON_TYPE_H_
#define FIRMWARE_USER_COMMON_TYPE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "std_library/inc/stm32f10x.h"

#define SOFT_VARSIONS "V1.0"
#define SOFT_VARSIONS_SIZE sizeof(SOFT_VARSIONS)

#define MALLOC(type,count) (type *)malloc(sizeof(type) * (count))
#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))
#define ARRAY2_SIZE(array) (sizeof(array) / (sizeof(array[0][0]) * 2))
#define ARRAY_INDEX_ASSERT(array, index) (ARRAY_SIZE(array) < (index))
#define ARRAY2_INDEX_ASSERT(array, index) (ARRAY2_SIZE(array) < (index))


/*
下边为不共用工程的模块类宏定义
CUR_PROJECT 赋值需要的工程
*/
/* 宏定义已经放到工程配置中了 */
// #define SNAP_OLD_BASE  0 //老设备支持,调试温度用的
// #define EXTENDED_MODULE_PROJECT 0 //扩展模块
// #define LINEAR_MODULE_PROJECT 1  //直线模组
// #define CUR_PROJECT EXTENDED_MODULE_PROJECT
// #define CUR_PROJECT EXTENDED_MODULE_PROJECT
// // 有符号整型
// typedef    char       int8_t     ;
// typedef    short      int16_t ;
// typedef    int        int32_t   ;
// typedef    long long  int64_t   ;
// //无符号整型
// typedef char          uint8_t  ;
// typedef short         uint16_t;
// typedef int           uint32_t ;
// typedef long long     uint64_t;
// 浮点型
typedef float float32;
typedef double float64;

#define EXTERN  extern

// Err 型
typedef enum ERR_E {
    E_TRUE = 0,
    E_FALSE,
    E_DOING,
    E_INIT_FAIL,
    E_SEND_LEN,
    E_SEND_NONE,
    E_SEND_FAIL,
    E_RECV_NONE,
    E_BUF_OVERFLOW,
    E_BUF_FULL,
    E_PARM_ERR,
    E_MALLOC_FAIL,
    E_FIND_NONE,
} ERR_E;


#endif  // FIRMWARE_USER_COMMON_TYPE_H_


