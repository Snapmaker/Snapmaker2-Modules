#ifndef ICM4XXXX_DELAY_H_
#define ICM4XXXX_DELAY_H_

#include <stdint.h>



void icm4xxxx_delay_ms(uint32_t ms);
void icm4xxxx_delay_us(uint32_t us);
void inv_imu_sleep_us(uint32_t us);
uint64_t inv_imu_get_time_us(void);



#endif