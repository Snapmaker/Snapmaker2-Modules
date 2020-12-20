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
#include "thermistor_table.h"
/*
根据阻值计算ad的值
公式： 阻值/（阻值+分压电阻） * 分辨率
 	    阻值 / （阻值 + 4.7）* 4096
ntc3590温度-阻值对照表：http://www.thermistors.cn/news/287.html
*/
#define TEMP_TABLE_LEN  (sizeof(temptable)/(sizeof(temptable[0])))
#define OV(N) ((N) * (OVERSAMPLENR))
const int32_t temptable[][2] = {
  { OV(   4), 938 },
  { OV(  91), 300 },
  { OV( 118), 290 },
  { OV( 136), 272 },
  { OV( 169), 258 },
  { OV( 201), 247 },
  { OV( 236), 237 },
  { OV( 270), 229 },
  { OV( 309), 221 },
  { OV( 343), 215 },
  { OV( 380), 209 },
  { OV( 415), 204 },
  { OV( 454), 199 },
  { OV( 487), 195 },
  { OV( 533), 190 },
  { OV( 563), 187 },
  { OV( 605), 183 },
  { OV( 651), 179 },
  { OV( 687), 176 },
  { OV( 766), 170 },
  { OV( 839), 165 },
  { OV( 918), 160 },
  { OV(1005), 155 },
  { OV(1098), 150 },
  { OV(1220), 144 },
  { OV(1330), 139 },
  { OV(1472), 133 },
  { OV(1811), 128 },
  { OV(1729), 123 },
  { OV(1895), 117 },
  { OV(2068), 111 },
  { OV(2245), 105 },
  { OV(2394), 100 },
  { OV(2542),  95 },
  { OV(2689),  90 },
  { OV(2832),  85 },
  { OV(2996),  79 },
  { OV(3175),  72 },
  { OV(3246),  69 },
  { OV(3337),  65 },
  { OV(3500),  57 },
  { OV(3537),  55 },
  { OV(3605),  51 },
  { OV(3697),  45 },
  { OV(3775),  39 },
  { OV(3888),  28 },
  { OV(3927),  23 },
  { OV(3966),  17 },
  { OV(4005),   9 },
  { OV(4038),   0 },
  { OV(4062), -10 },
  { OV(4083), -27 },
  { OV(4085), -30 }
};



// 参数 u32Raw: 累计 OVERSAMPLENR 次的采样值
float32 TempTableCalcCurTemp(uint32_t u32Raw) {
    float32 celsius = 0;
    uint16_t i;
    uint32_t tableAnalogValue;
    uint32_t u32TableLen = TEMP_TABLE_LEN;

    for (i = 1; i < u32TableLen; i++) {
        tableAnalogValue = temptable[i][0];
        if (tableAnalogValue > u32Raw) {
            celsius =
                 temptable[i][1] + ((temptable[i][0] -u32Raw) *
                 (float) (temptable[i - 1][1] -temptable[i][1]) /
                 (float) (temptable[i][0] -temptable[i - 1][0]));
            break;
        }
    }

    // Overflow: Set to last value in the table
    if (i == u32TableLen)
        celsius = temptable[i - 1][1];

    return celsius;
}




