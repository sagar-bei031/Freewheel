/**
 ******************************************************************************
 * @file           free_wheel.cpp
 * @brief          Application file for localizing a robot with free wheels.
 * @author         Robotics Team, IOE Pulchowk Campus
 * @date           2023
 ******************************************************************************
 */

#include "free_wheel.h"
#include "encoder.hpp"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "arm_math.h"
#include "crc.hpp"
#include <memory.h>

// Define __count for testing purposes
#define __count

#define F32_2_PI  6.28318530717958f
#define F32_PI    3.14159265358979f
#define F32_PI_2  1.57079632679489f

Free_Wheel free_wheel;


int32_t bc = 0;
int32_t rc = 0;
int32_t lc = 0;


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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    static uint32_t led1_tick;
    if (HAL_GetTick() - led1_tick > 20)
    {
        HAL_GPIO_TogglePin(YELLOW_LED1_GPIO_Port, YELLOW_LED1_Pin);
        led1_tick = HAL_GetTick();
    }

    if (is_waiting_for_start_byte)
    {
        if (receiving_bytes[0] == START_BYTE)
        {
            is_waiting_for_start_byte = false;
            receiving_bytes[0] = 0x00;
            HAL_UART_Receive_DMA(&huart1, receiving_bytes + 1, 5);
        }
        else
        {
            HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);
        }
    }
    else
    {
        is_waiting_for_start_byte = true;
        HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);

        if (crc.get_Hash(receiving_bytes + 1, 4) == receiving_bytes[5])
        {
            memcpy((uint8_t *)&imu_yaw, receiving_bytes + 1, 4);
            imu_input_tick = HAL_GetTick();

            static uint32_t board_led_tick;
            if (HAL_GetTick() - board_led_tick > 20)
            {
                HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
                board_led_tick = HAL_GetTick();
            }
        }
    }
}

// /**
//  * @brief Callback function for handling UART errors.
//  * @param huart Pointer to the UART handle structure.
//  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    static uint32_t red_led_tick;
    if (HAL_GetTick() - red_led_tick > 20)
    {
        HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
        red_led_tick = HAL_GetTick();
    }

    if (huart->Instance == huart1.Instance)
    {
        is_waiting_for_start_byte = true;
        HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);
    }
}

/**
 * @brief Callback function for handling the completion of UART transmission.
 * @param huart Pointer to the UART handle structure.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(&huart2);
    free_wheel.is_transmitting = false;

    HAL_GPIO_TogglePin(YELLOW_LED2_GPIO_Port, YELLOW_LED2_Pin);
}

/**
 * @brief Initializes the Free_Wheel object.
 */
void Free_Wheel::init()
{
    back_enc.init();
    right_enc.init();
    left_enc.init();

    HAL_UART_Receive_DMA(&huart1, receiving_bytes, 1);
}

/**
 * @brief Reads data from encoders and updates the robot state.
 */
void Free_Wheel::read_data()
{
    back_count = -back_enc.get_count();
    right_count = -right_enc.get_count();
    left_count = left_enc.get_count();

    bc += back_count;
    rc += right_count;
    lc += left_count;

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
    float32_t back_dist = F32_PI * Wheel_Diameter * (float32_t)back_count / (float)CPR;
    float32_t right_dist = F32_PI * Wheel_Diameter * (float32_t)right_count / (float32_t)CPR;
    float32_t left_dist = F32_PI * Wheel_Diameter * (float32_t)left_count / (float32_t)CPR;

    float32_t back_vel = back_omega * Wheel_Diameter / 2.0f;
    float32_t right_vel = right_omega * Wheel_Diameter / 2.0f;
    float32_t left_vel = left_omega * Wheel_Diameter / 2.0f;

    float32_t d_theta = (right_dist - left_dist) / (LEFT_RADIUS + RIGHT_RADIUS);

    if (HAL_GetTick() - imu_input_tick < 100U)
    {
        float32_t d_imu_yaw;

        if ((prev_imu_yaw > F32_PI_2) && (imu_yaw < -F32_PI_2))
        {
            d_imu_yaw = (F32_PI - prev_imu_yaw) + (F32_PI + imu_yaw);
        }
        else if ((prev_imu_yaw < -F32_PI_2) && (imu_yaw > F32_PI_2))
        {
            d_imu_yaw = -(F32_PI + prev_imu_yaw) - (F32_PI - imu_yaw);
        }
        else
        {
            d_imu_yaw = imu_yaw - prev_imu_yaw;
        }

        d_theta = 0.1f * d_theta + 0.9f * d_imu_yaw;
        prev_imu_yaw = imu_yaw;
    }

    float32_t dx = (right_dist * LEFT_RADIUS + left_dist * RIGHT_RADIUS) / (LEFT_RADIUS + RIGHT_RADIUS);
    float32_t dy = back_dist + BACK_RADIUS * d_theta;

    float32_t cos_value = arm_cos_f32(theta + d_theta / 2.0f);
    float32_t sin_value = arm_sin_f32(theta + d_theta / 2.0f);

    x += dx * cos_value - dy * sin_value;
    y += dx * sin_value + dy * cos_value;
    theta += d_theta;

    if (theta > F32_PI)
    {
        theta -= 2.0f * F32_2_PI;
    }
    else if (theta < (-F32_PI))
    {
        theta += 2.0f * F32_2_PI;
    }

    float32_t omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
    float32_t vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
    float32_t vy = back_vel + omega * BACK_RADIUS;

        robostate.pose.x = x;
    robostate.pose.y = y;
    robostate.pose.theta = theta;

#ifdef __count
    robostate.twist.vx = bc;
    robostate.twist.vy = rc;
    robostate.twist.w = lc;
#else
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

        loop_tick = HAL_GetTick();

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
    }
}