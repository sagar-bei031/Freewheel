#ifndef ROBOTLIB_ENCODER_HPP_
#define ROBOTLIB_ENCODER_HPP_

#include "stm32f1xx.h"

class Encoder {
public:
  TIM_HandleTypeDef *henc;
  float omega;

  uint32_t last_reset_time = 0;

  Encoder(TIM_HandleTypeDef *_henc) : henc(_henc) {}
  Encoder() {}
  float get_omega(uint16_t cpr);
  void init(void);
  void reset_encoder_count(void);
  int16_t get_count(void);
};

#endif // ROBOTLIB_ENCODER_HPP_
