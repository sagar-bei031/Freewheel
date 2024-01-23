#ifndef ROBOTLIB_ENCODER_HPP_
#define ROBOTLIB_ENCODER_HPP_

#include "tim.h"
#include "arm_math.h"

class Encoder {
public:
  TIM_HandleTypeDef *henc;
  uint16_t cpr;
  uint32_t last_reset_time = 0;

  Encoder(TIM_HandleTypeDef *_henc, uint16_t _cpr) : henc(_henc), cpr(_cpr) {}
  void init();
  int32_t get_count();
  float32_t get_omega();
  void reset_encoder_count();
};

#endif // ROBOTLIB_ENCODER_HPP_
