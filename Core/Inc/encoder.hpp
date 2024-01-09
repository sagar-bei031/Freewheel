#ifndef ROBOTLIB_ENCODER_HPP_
#define ROBOTLIB_ENCODER_HPP_

#include "stm32f1xx.h"

class Encoder {
public:
  TIM_HandleTypeDef *henc;
  uint16_t cpr;
  double omega;

  uint32_t last_reset_time = 0;

  Encoder(TIM_HandleTypeDef *_henc, uint16_t _cpr) : henc(_henc), cpr(_cpr) {}
  Encoder() {}
  double get_omega(void);
  void init(void);
  void reset_encoder_count(void);
  int16_t get_count(void);
};

#endif // ROBOTLIB_ENCODER_HPP_
