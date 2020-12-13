#include "src/module/light_module.h"
#include <io.h>
#include "src/HAL/hal_pwm.h"
void LightModule::Init() {
  this->r_chn_ = 1;
  HAL_pwn_config(LED_PWM_TIM, this->r_chn_ , 100000, 256, 0, 0);
  this->g_chn_ = 2;
  HAL_pwn_config(LED_PWM_TIM, this->g_chn_ , 100000, 256, 0, 0);
  this->b_chn_ = 3;
  HAL_pwn_config(LED_PWM_TIM, this->b_chn_ , 100000, 256, 0, 0);

  pinMode(LED_R_PIN, PWM);
  pinMode(LED_G_PIN, PWM);
  pinMode(LED_B_PIN, PWM);
}

void LightModule::HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len) {
  if (data[0] == 0) {
    switch (data[1]) {
      case LED_COLOR_OFF :
        this->SetRGB(LED_COLOR_OFF_RGB);
        break;
      case LED_COLOR_ALL :
        this->SetRGB(LED_COLOR_ALL_RGB);
        break;
      case LED_COLOR_RED :
        this->SetRGB(LED_COLOR_RED_RGB);
        break;
      case LED_COLOR_GREEN :
        this->SetRGB(LED_COLOR_GREEN_RGB);
        break;
      case LED_COLOR_BLUE :
        this->SetRGB(LED_COLOR_BLUE_RGB);
        break;
    }
  } else {
    this->SetRGB(data[1], data[2], data[3]);
  }
}

void LightModule::Loop() {

}

void LightModule::SetColor(uint8_t chn, uint8_t val) {
  HAL_pwm_set_pulse(LED_PWM_TIM, chn, val);
}

void LightModule::SetRGB(uint8_t r, uint8_t g, uint8_t b) {
  this->SetColor(this->r_chn_, r);
  this->SetColor(this->g_chn_, g);
  this->SetColor(this->b_chn_, b);
}