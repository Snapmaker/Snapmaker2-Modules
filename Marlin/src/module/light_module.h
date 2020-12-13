//
// Created by David Chen on 2019-08-03.
//

#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_LIGHT_MODULE_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_LIGHT_MODULE_H_

#include "src/configuration.h"
#include "module_base.h"

#define LED_PWM_TIM 1

#define LED_COLOR_OFF_RGB    0, 0, 0
#define LED_COLOR_ALL_RGB    255, 255, 255
#define LED_COLOR_RED_RGB    255, 0, 0
#define LED_COLOR_GREEN_RGB  0, 255, 0
#define LED_COLOR_BLUE_RGB   0, 0, 255

#define LED_R_PIN PA8
#define LED_G_PIN PA9
#define LED_B_PIN PA10

typedef enum {
    LED_COLOR_OFF,
    LED_COLOR_ALL,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
}LED_COLOR_E;

class LightModule : public ModuleBase {
 public:
  void Init();
  void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
  void Loop();
  void SetRGB(uint8_t r, uint8_t g, uint8_t b);
  void SetColor(uint8_t chn, uint8_t val);

 private:
  uint8_t r_chn_;
  uint8_t g_chn_;
  uint8_t b_chn_;
};

#endif //SNAPMAKERMODULES_MARLIN_SRC_MODULE_LINEAR_MODULE_H_
