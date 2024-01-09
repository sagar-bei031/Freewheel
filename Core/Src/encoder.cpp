#include "encoder.hpp"
#include "stm32f1xx.h"
#include <math.h>
#include "arm_math.h"

int16_t Encoder::get_count(void)
{
  int32_t count = henc->Instance->CNT;
  if (count > 32768)
    count = count - 65536;
  return count;
}

double Encoder::get_omega()
{
  int16_t count = get_count();
  int32_t sample_time = HAL_GetTick() - last_reset_time;
  // omega = 2.0f * (float)M_PI * (float)count / (float)(cpr * sample_time) * 1000.0f;
  omega = (2.0 * M_PI * count * 1000.0) / (cpr * sample_time);
  return omega;
}

void Encoder::init(void)
{
  HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);
}

void Encoder::reset_encoder_count(void)
{
  henc->Instance->CNT = 0;
  last_reset_time = HAL_GetTick();
}
