#include "free_wheel.h"
#include "gpio.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
// #include <cmath>
// #include <cstdint>
// #include <cstdio>
// #include <string.h>
// #include <qfplib-m3.h>
#include <arm_math.h>

// #define __COUNT__
// #define __EACH__

#define xR 0.265
#define yrR 0.235
#define ylR 0.221
#define Wheel_Diameter 0.0574

#define cm_per_ticks (M_PI * Wheel_Diameter / 4000.0)

#define START_BYTE (0XA5)

Free_Wheel free_wheel;

// float imu_omega = 0, imu_yaw = 0, prev_imu_yaw;
uint8_t start_byte = START_BYTE, hash;

// float imu_theta_degree, fw_theta_degree, imu_omega_degree, filtered_theta_degree;
// float base_yaw = 0;

// uint32_t omega_input_tick = 0;

#ifdef __COUNT__
int32_t temp[3];
#endif

#ifdef __EACH__
float each[3];
#endif

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     __HAL_UART_FLUSH_DRREGISTER(huart);
//     if (huart->Instance == huart1.Instance)
//     {
//         if (free_wheel.receive_uart1[0] == START_BYTE)
//         {
//             HAL_UART_Receive_DMA(&huart1, &free_wheel.receive_uart1[1], 13);
//             if (free_wheel.crc.get_Hash(&free_wheel.receive_uart1[1], 12) == free_wheel.receive_uart1[13])
//             {
//                 void *inm = (void *)&free_wheel.receive_uart1[9];
//                 imu_yaw = (*(float *)inm);

//                 omega_input_tick = HAL_GetTick();
//             }
//             free_wheel.receive_uart1[0] = 0x00;
//         }
//         else
//         {
//             HAL_UART_Receive_DMA(&huart1, &free_wheel.receive_uart1[0], 1);
//         }
//     }
// }

uint32_t last_led_tick = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((HAL_GetTick() - last_led_tick) > 100)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_led_tick = HAL_GetTick();
    }
}

Free_Wheel::Free_Wheel()
{
    odom.x = 0;
    odom.y = 0;
    odom.theta = 0;

    // last_xCnt = 0;
    // last_ylCnt = 0;
    // last_yrCnt = 0;
}
void Free_Wheel::init()
{
    enc[0] = Encoder(&htim3, 4000); // back
    enc[1] = Encoder(&htim1, 4000); // right
    enc[2] = Encoder(&htim2, 4000); // left

    enc[0].init();
    enc[1].init();
    enc[2].init();

    // ftheta = 0.0;

    // HAL_UART_Receive_DMA(&huart1, &free_wheel.receive_uart1[0], 1);
}

// float xcm, ycm;
void Free_Wheel::read_data()
{
    xCnt = enc[0].get_count();
    yrCnt = -enc[1].get_count();
    ylCnt = enc[2].get_count();

    enc[0].reset_encoder_count();
    enc[1].reset_encoder_count();
    enc[2].reset_encoder_count();
}

void Free_Wheel::process_data()
{
    float backX_dist = M_PI * Wheel_Diameter * (xCnt) / enc[0].ppr;
    float rightY_dist = M_PI * Wheel_Diameter * (yrCnt) / enc[1].ppr;
    float leftY_dist = M_PI * Wheel_Diameter * (ylCnt) / enc[2].ppr;

#ifdef __COUNT__
    temp[0] += xCnt;
    temp[1] += yrCnt;
    temp[2] += ylCnt;
#endif

    float d_theta = (rightY_dist - leftY_dist) / (ylR + yrR);

    // float d_yaw = imu_yaw - prev_imu_yaw;

    // float d_yaw_filtered = 0.1 * (d_theta) + (1 - 0.1) * (d_yaw);

    // ftheta += d_theta;

    // float dy = leftY_dist + ylR * d_theta;

    float dy = (rightY_dist * ylR + leftY_dist * yrR) / (ylR + yrR);
    float dx = backX_dist - xR * d_theta;
    odom.x += dx * arm_cos_f32(odom.theta + d_theta / 2.0) - dy * arm_sin_f32(odom.theta + d_theta / 2.0);
    odom.y += dx * arm_sin_f32(odom.theta + d_theta / 2.0) + dy * arm_cos_f32(odom.theta + d_theta / 2.0);

#ifdef __EACH__
    each[0] += dx;
    each[1] += dy;
    each[2] += d_theta;
#endif

    // odom.x = round3(odom.x);
    // odom.y = round3(odom.y);

    // d_yaw_filtered = 0.9 * d_yaw + 0.1 * d_theta;

    // if ((HAL_GetTick() - omega_input_tick) > 500)
    // {
    odom.theta += d_theta;

    if (odom.theta > M_PI)
    {
        odom.theta -= 2 * M_PI;
    }
    else if (odom.theta < (-M_PI))
    {
        odom.theta += 2 * M_PI;
    }

    // odom.theta = round3(odom.theta);
    // }
    // else
    // {
    // odom.theta += d_yaw_filtered;
    // }

    // filtered_theta_degree = odom.theta * 180 / M_PI;
    // imu_theta_degree = imu_yaw * 180 / M_PI;
    // fw_theta_degree = ftheta * 180 / M_PI;

    // xcm = odom.x * 100;
    // ycm = odom.y * 100;

    // prev_imu_yaw = imu_yaw;
}

// int k = 0;
void send_data()
{
    free_wheel.init();
    // prev_imu_yaw = imu_yaw;
    uint32_t last_tick = HAL_GetTick();
    uint32_t last_uart_tick = last_tick;

    while (1)
    {
        if ((HAL_GetTick() - last_tick) < 10)
            continue;

        free_wheel.read_data();
        free_wheel.process_data();

        if ((HAL_GetTick() - last_uart_tick) > 50)
        {
            free_wheel.sending_bytes[0] = START_BYTE;

#if defined __COUNT__
            memcpy(&free_wheel.sending_bytes[1], (uint8_t *)(temp), 12);
#elif defined __EACH__
            memcpy(&free_wheel.sending_bytes[1], (uint8_t *)(each), 12);
#else
            memcpy(&free_wheel.sending_bytes[1], (uint8_t *)(&free_wheel.odom), 12);
#endif
            free_wheel.sending_bytes[13] = free_wheel.crc.get_Hash((uint8_t *)(&free_wheel.sending_bytes[1]), 12);
            HAL_UART_Transmit_DMA(&huart2, free_wheel.sending_bytes, 14);
            last_uart_tick = HAL_GetTick();
        }

        last_tick = HAL_GetTick();
    }
}

inline float round3(float val)
{
    return (float)((int32_t)(val * 1000.0f)) / 1000.0f;
}