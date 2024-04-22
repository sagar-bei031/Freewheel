#include "arm_math.h"
#include "encoder.hpp"

int32_t Encoder::get_count()
{
  uint32_t count = henc->Instance->CNT;
  if (count > 32768U)
    count = count - 65536U;
  return (int32_t)count;
}

float32_t Encoder::get_omega()
{
  int32_t count = get_count();
  int32_t sample_time = HAL_GetTick() - last_reset_time;
  float32_t omega = 2.0f * (float32_t)PI * (float32_t)count / (float32_t)(cpr * sample_time) * 1000.0f;
  return omega;
}

void Encoder::init(void)
{
  HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);
}

void Encoder::reset_encoder_count(void)
{
  henc->Instance->CNT = 0U;
  last_reset_time = HAL_GetTick();
}
