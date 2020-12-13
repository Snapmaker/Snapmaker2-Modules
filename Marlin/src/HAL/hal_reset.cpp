#include "std_library/inc/stm32f10x.h"
#include "std_library/inc/stm32f10x_iwdg.h"
void HAL_reset() {
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_64);
  IWDG_SetReload(30);
  IWDG_ReloadCounter();
  IWDG_Enable();
}

void HAL_JTAGDisable() {
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);	
}