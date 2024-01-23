/**
 ******************************************************************************
 * @file           free_wheel.cpp
 * @brief          Application file for localizing a robot with free wheels.
 * @author         Robotics Team, IOE Pulchowk Campus
 * @date           2023
 ******************************************************************************
 */

#include "free_wheel.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "arm_math.h"
#include "crc.hpp"
#include <memory.h>

// Define __count for testing purposes
// #define __count

Free_Wheel free_wheel;

#ifdef __count
int32_t bc = 0;
int32_t rc = 0;
int32_t lc = 0;
#endif

float32_t imu_yaw = 0.0f;
float32_t prev_imu_yaw = 0.0f;
uint32_t imu_input_tick = 0;

uint8_t receiving_bytes[6];
bool is_waiting_for_start_byte = true;

CRC_Hash crc{7};

/**
 * @brief Callback function for handling the reception of data via UART.
 * @param huart Pointer to the UART handle structure.
 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     __HAL_UART_FLUSH_DRREGISTER(huart);

//     if (is_waiting_for_start_byte)
//     {
//         if (receiving_bytes[0] == START_BYTE)
//         {
//             is_waiting_for_start_byte = false;
//             receiving_bytes[0] = 0x00;
//             HAL_UART_Receive_DMA(&huart1, receiving_bytes + 1, 5);
//         }
//         else
//         {
//             HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);
//         }
//     }
//     else
//     {
//         is_waiting_for_start_byte = true;
//         HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);

//         if (crc.get_Hash(receiving_bytes + 1, 4) == receiving_bytes[5])
//         {
//             memcpy((uint8_t *) &imu_yaw, receiving_bytes + 1, 4);
//             imu_input_tick = HAL_GetTick();
//         }
//     }
// }

// /**
//  * @brief Callback function for handling UART errors.
//  * @param huart Pointer to the UART handle structure.
//  */
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     __HAL_UART_FLUSH_DRREGISTER(huart);

//     if (huart->Instance == huart1.Instance)
//     {
//         is_waiting_for_start_byte = true;
//         HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);
//     }
// }

/**
 * @brief Callback function for handling the completion of UART transmission.
 * @param huart Pointer to the UART handle structure.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(&huart2);
    free_wheel.is_transmitting = false;

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/**
 * @brief Initializes the Free_Wheel object.
 */
void Free_Wheel::init()
{
    back_enc.init();
    right_enc.init();
    left_enc.init();

    // HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);
}

/**
 * @brief Reads data from encoders and updates the robot state.
 */
void Free_Wheel::read_data()
{
    back_count = -back_enc.get_count();
    right_count = -right_enc.get_count();
    left_count = left_enc.get_count();

#ifdef __count
    bc += back_count;
    rc += right_count;
    lc += left_count;
#endif

    back_omega = -back_enc.get_omega();
    right_omega = -right_enc.get_omega();
    left_omega = left_enc.get_omega();

    back_enc.reset_encoder_count();
    right_enc.reset_encoder_count();
    left_enc.reset_encoder_count();
}

/**
 * @brief Processes the collected data to update the robot's position and orientation.
 */

void Free_Wheel::process_data()
{
    float32_t back_dist = M_PI * Wheel_Diameter * back_count / CPR;
    float32_t right_dist = M_PI * Wheel_Diameter * right_count / CPR;
    float32_t left_dist = M_PI * Wheel_Diameter * left_count / CPR;

    float32_t back_vel = back_omega * Wheel_Diameter / 2.0f;
    float32_t right_vel = right_omega * Wheel_Diameter / 2.0f;
    float32_t left_vel = left_omega * Wheel_Diameter / 2.0f;

    float32_t d_theta = (right_dist - left_dist) / (LEFT_RADIUS + RIGHT_RADIUS);

    // if (HAL_GetTick() - imu_input_tick < 500U)
    // {
    //     float32_t d_imu_yaw;

    //     if ((prev_imu_yaw > M_PI) && (prev_imu_yaw < M_PI) && (imu_yaw < -M_PI_2) && (imu_yaw > -M_PI))
    //     {
    //         d_imu_yaw = (M_PI - prev_imu_yaw) + (M_PI + imu_yaw);
    //     }
    //     else if ((prev_imu_yaw < -M_PI) && (prev_imu_yaw > -M_PI) && (imu_yaw > M_PI_2) && (imu_yaw < M_PI))
    //     {
    //         d_imu_yaw = -(M_PI + prev_imu_yaw) - (M_PI - imu_yaw);
    //     }
    //     else
    //     {
    //         d_imu_yaw = imu_yaw - prev_imu_yaw;
    //     }

    //     d_theta = 0.1f * d_theta + 0.9f * d_imu_yaw;
    // }

    float32_t dx = (right_dist * LEFT_RADIUS + left_dist * RIGHT_RADIUS) / (LEFT_RADIUS + RIGHT_RADIUS);
    float32_t dy = back_dist + BACK_RADIUS * d_theta;

    float32_t cos_value = arm_cos_f32(theta + d_theta / 2.0f);
    float32_t sin_value = arm_sin_f32(theta + d_theta / 2.0f);

    x += dx * cos_value - dy * sin_value;
    y += dx * sin_value + dy * cos_value;

    theta += d_theta;

    if (theta > M_PI)
    {
        theta -= 2.0f * M_PI;
    }
    else if (theta < (-M_PI))
    {
        theta += 2.0f * M_PI;
    }

    float32_t omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
    float32_t vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
    float32_t vy = back_vel + omega * BACK_RADIUS;

#ifdef __count
    static uint32_t last_process_tick;
    uint32_t now = HAL_GetTick();
    uint16_t dt = now - last_process_tick;

    robostate.pose.x = bc;
    robostate.pose.y = rc;
    robostate.pose.theta = lc;

    robostate.twist.vx = back_omega;
    robostate.twist.vy = right_omega;
    robostate.twist.w = left_omega;

    last_process_tick = now;
#else
    robostate.pose.x = x;
    robostate.pose.y = y;
    robostate.pose.theta = theta;

    robostate.twist.vx = vx;
    robostate.twist.vy = vy;
    robostate.twist.w = omega;
#endif
}

/**
 * @brief Continuously sends robot data via UART.
 */
void send_data()
{
    free_wheel.init();
    uint32_t loop_tick = 0;

    while (1)
    {
        if (HAL_GetTick() - loop_tick < 10)
            continue;

        free_wheel.read_data();
        free_wheel.process_data();

        static uint32_t transmit_tick;
        if ((HAL_GetTick() - transmit_tick >= 50) && (!free_wheel.is_transmitting))
        {
            free_wheel.sending_bytes[0] = START_BYTE;
            memcpy(free_wheel.sending_bytes + 1, (uint8_t *)(&free_wheel.robostate), 24);
            free_wheel.sending_bytes[25] = crc.get_Hash((uint8_t *)(free_wheel.sending_bytes + 1), 24);

            HAL_UART_Transmit_DMA(&huart2, free_wheel.sending_bytes, 26);
            free_wheel.is_transmitting = true;
            transmit_tick = HAL_GetTick();
        }

        loop_tick = HAL_GetTick();
    }
}