#include "encoder.hpp"
#include "stm32f1xx.h"
#include <math.h>
#include "arm_math.h"

int16_t Encoder::get_count(void) {
  int32_t count = henc->Instance->CNT;
  if (count > 32768)
    count = count - 65535;
  return count;
}

float Encoder::get_omega(uint16_t cpr) {
  int16_t count = get_count();
  int32_t sample_time = HAL_GetTick() - last_reset_time;
  omega = 2.0f * (float)M_PI * count / (cpr * (sample_time / 1000.0f));
  return omega;
}

void Encoder::init(void) { HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL); }

void Encoder::reset_encoder_count(void) {
  henc->Instance->CNT = 0;
  last_reset_time = HAL_GetTick();
}
