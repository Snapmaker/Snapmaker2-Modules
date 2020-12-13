

/*热敏电子阻值与温度转换*/
#ifndef _THERMISTOR_TABLE_H_
#define _THERMISTOR_TABLE_H_
#include "common_type.h"

#define OVERSAMPLENR 16  // 采集数据的累计个数

// 参数 u32Raw: 累计 OVERSAMPLENR 次的采样值
extern float32 TempTableCalcCurTemp(uint32_t u32Raw);

#endif

