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

#define CPR_CW 4025
#define CPR_ACW 3868

// #define cm_per_ticks (M_PI * Wheel_Diameter / 4000.0)

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

bool is_transmitting = false;

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
    if ((HAL_GetTick() - last_led_tick) > 50)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_led_tick = HAL_GetTick();
    }

    is_transmitting = false;
}

Free_Wheel::Free_Wheel()
{
    robostate.odometry.x = 0;
    robostate.odometry.y = 0;
    robostate.odometry.theta = 0;

    // last_xCnt = 0;
    // last_ylCnt = 0;
    // last_yrCnt = 0;
}

void Free_Wheel::init()
{
    enc[0] = Encoder(&htim3); // back
    enc[1] = Encoder(&htim1); // right
    enc[2] = Encoder(&htim2); // left

    enc[0].init();
    enc[1].init();
    enc[2].init();

    // ftheta = 0.0;

    // HAL_UART_Receive_DMA(&huart1, &free_wheel.receive_uart1[0], 1);
}

// float xcm, ycm;
void Free_Wheel::read_data()
{
    xCnt = enc[0].get_count();   // (+) -> CW
    yrCnt = -enc[1].get_count(); // (+) -> ACW
    ylCnt = enc[2].get_count();  // (+) -> CW

    if (xCnt >= 0)
    {
        xRev = (float)xCnt / CPR_CW; // (+) -> CW
        wx = enc[0].get_omega(CPR_CW);
    }
    else
    {
        xRev = (float)xCnt / CPR_ACW; // (-) -> ACW
        wx = enc[0].get_omega(CPR_ACW);
    }

    if (yrCnt >= 0)
    {
        yrRev = (float)yrCnt / CPR_ACW; // (+) -> ACW
        wyr = -enc[1].get_omega(CPR_ACW);
    }
    else
    {
        yrRev = (float)yrCnt / CPR_CW; // (-) -> CW
        wyr = -enc[1].get_omega(CPR_CW);
    }

    if (ylCnt >= 0)
    {
        ylRev = (float)ylCnt / CPR_CW; // (+) -> CW
        wyl = enc[2].get_omega(CPR_CW);
    }
    else
    {
        ylRev = (float)ylCnt / CPR_ACW; // (-) -> ACW
        wyl = enc[2].get_omega(CPR_ACW);
    }

    enc[0].reset_encoder_count();
    enc[1].reset_encoder_count();
    enc[2].reset_encoder_count();
}

void Free_Wheel::process_data()
{

    float backX_dist = M_PI * Wheel_Diameter * xRev;
    float rightY_dist = M_PI * Wheel_Diameter * yrRev;
    float leftY_dist = M_PI * Wheel_Diameter * ylRev;

    float x_vel = wx * Wheel_Diameter / 2.0f;
    float yr_vel = wyr * Wheel_Diameter / 2.0f;
    float yl_vel = wyl * Wheel_Diameter / 2.0f;

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
    robostate.odometry.x += dx * arm_cos_f32(robostate.odometry.theta + d_theta / 2.0) - dy * arm_sin_f32(robostate.odometry.theta + d_theta / 2.0);
    robostate.odometry.y += dx * arm_sin_f32(robostate.odometry.theta + d_theta / 2.0) + dy * arm_cos_f32(robostate.odometry.theta + d_theta / 2.0);

#ifdef __EACH__
    each[0] += dx;
    each[1] += dy;
    each[2] += d_theta;
#endif

    // robostate.robostate.odometryetry.x = round3(robostate.odometry.x);
    // robostate.odometry.y = round3(robostate.odometry.y);

    // d_yaw_filtered = 0.9 * d_yaw + 0.1 * d_theta;

    // if ((HAL_GetTick() - omega_input_tick) > 500)
    // {
    robostate.odometry.theta += d_theta;

    if (robostate.odometry.theta > M_PI)
    {
        robostate.odometry.theta -= 2 * M_PI;
    }
    else if (robostate.odometry.theta < (-M_PI))
    {
        robostate.odometry.theta += 2 * M_PI;
    }

    // robostate.odometry.theta = round3(robostate.odometry.theta);
    // }
    // else
    // {
    // robostate.odometry.theta += d_yaw_filtered;
    // }

    // filtered_theta_degree = robostate.odometry.theta * 180 / M_PI;
    // imu_theta_degree = imu_yaw * 180 / M_PI;
    // fw_theta_degree = ftheta * 180 / M_PI;

    // xcm = robostate.odometry.x * 100;
    // ycm = robostate.odometry.y * 100;

    // prev_imu_yaw = imu_yaw;

    robostate.twist.w = (yr_vel - yl_vel) / (yrR + ylR);
    robostate.twist.vy = (yr_vel + yl_vel) / (yrR + ylR);
    robostate.twist.vx = x_vel - robostate.twist.w * xR;
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

        uint32_t d_time = HAL_GetTick() - last_uart_tick;

        if (((last_uart_tick >= 50) && (!is_transmitting)) | (d_time >= 500))
        {
            free_wheel.sending_bytes[0] = START_BYTE;

#if defined __COUNT__
            memcpy(&free_wheel.sending_bytes[1], (uint8_t *)(temp), 12);
#elif defined __EACH__
            memcpy(&free_wheel.sending_bytes[1], (uint8_t *)(each), 12);
#else
            memcpy(&free_wheel.sending_bytes + 1, (uint8_t *)(&free_wheel.robostate.odometry), 24);
#endif
            free_wheel.sending_bytes[25] = free_wheel.crc.get_Hash((uint8_t *)(free_wheel.sending_bytes + 1), 24);
            HAL_UART_Transmit_DMA(&huart2, free_wheel.sending_bytes, 26);
            last_uart_tick = HAL_GetTick();
            is_transmitting = true;
        }

        last_tick = HAL_GetTick();
    }
}

inline float round3(float val)
{
    return (float)((int32_t)(val * 1000.0f)) / 1000.0f;
}