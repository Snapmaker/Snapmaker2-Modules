
#include "icm4xxxx_delay.h"
#include "../../../../snapmaker/lib/STM32F1/cores/maple/wirish_time.h"

void icm4xxxx_delay_ms(uint32_t ms) {
  delay(ms);
}

void icm4xxxx_delay_us(uint32_t us) {
  delayMicroseconds(us);
}

void inv_imu_sleep_us(uint32_t us) {
  delayMicroseconds(us);
}

uint64_t inv_imu_get_time_us(void)
{
	return micros();
}
