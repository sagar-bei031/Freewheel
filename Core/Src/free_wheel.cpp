/**
 ******************************************************************************
 * @file           free_wheel.cpp
 * @brief          Application file for localizing a robot with free wheels.
 * @author         Robotics Team, IOE Pulchowk Campus
 * @date           2023
 ******************************************************************************
 */

#include <memory.h>
#include <stdio.h>
#include "arm_math.h"
#include "gpio.h"
#include "usart.h"
#include "tim.h"
#include "crc.hpp"
#include "encoder.hpp"
#include "free_wheel.h"

#define PRINT_COUNT

#define F32_2_PI 6.28318530717958f
#define F32_PI 3.14159265358979f
#define F32_PI_2 1.57079632679489f

Free_Wheel free_wheel;

int32_t total_back_count = 0;
int32_t total_right_count = 0;
int32_t total_left_count = 0;

float32_t imu_yaw = 0.0f;
float32_t prev_imu_yaw = 0.0f;
uint32_t imu_input_tick = 0;

uint8_t receiving_bytes[6];
bool is_waiting_for_start_byte = true;

CRC_Hash crc{7};

inline float32_t degreeChange(float32_t curr, float32_t prev);
inline float32_t radianChange(float32_t curr, float32_t prev);
inline float32_t angleClamp(float32_t angle);

/**
 * @brief Callback function for handling the reception of data via UART.
 * @param huart Pointer to the UART handle structure.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);
    static uint32_t board_led_tick = 0;

    uint32_t now = HAL_GetTick();
    if (now - board_led_tick > 50)
    {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
        board_led_tick = now;
    }

    if (huart->Instance == free_wheel.bno.huart->Instance)
    {
        if (free_wheel.bno.receive() == Bno08::BnoRecvStatus::CHECKSUM_MATCHED)
        {
            imu_yaw = free_wheel.bno.data.yaw;
            // printf("imu: %f %f %f %f %f %f\n", free_wheel.bno.data.yaw,
            //                           free_wheel.bno.data.pitch,
            //                           free_wheel.bno.data.roll,
            //                           free_wheel.bno.data.accel_x,
            //                           free_wheel.bno.data.accel_y,
            //                           free_wheel.bno.data.accel_z);
        }
    }
}

/**
 * @brief Callback function for handling UART errors.
 * @param huart Pointer to the UART handle structure.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    static uint32_t red_led_tick = 0;
    uint32_t now = HAL_GetTick();

    if (now - red_led_tick > 20)
    {
        HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
        red_led_tick = now;
    }

    if (huart->Instance == free_wheel.bno.huart->Instance)
    {
        free_wheel.bno.init();
    }
}

/**
 * @brief Callback function for handling the completion of UART transmission.
 * @param huart Pointer to the UART handle structure.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_GPIO_TogglePin(YELLOW_LED2_GPIO_Port, YELLOW_LED2_Pin);
}

/**
 * @brief Initializes the Free_Wheel object.
 */
void Free_Wheel::init()
{
    bno.init();
    back_enc.init();
    right_enc.init();
    left_enc.init();
}

/**
 * @brief Reads data from encoders and updates the robot state.
 */
void Free_Wheel::read_data()
{
    back_count = -back_enc.get_count();
    right_count = -right_enc.get_count();
    left_count = left_enc.get_count();

    total_back_count += back_count;
    total_right_count += right_count;
    total_left_count += left_count;

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

    if (bno.isConnected())
    {
        float32_t d_imu_yaw = radianChange(imu_yaw, prev_imu_yaw);
        d_theta = d_imu_yaw;
        prev_imu_yaw = imu_yaw;
    }

    float32_t dx = (right_dist * LEFT_RADIUS + left_dist * RIGHT_RADIUS) / (LEFT_RADIUS + RIGHT_RADIUS);
    float32_t dy = back_dist + BACK_RADIUS * d_theta;

    float32_t theta_t = angleClamp(theta + d_theta / 2.0f);
    float32_t cos_value = arm_cos_f32(theta_t);
    float32_t sin_value = arm_sin_f32(theta_t);

    x += dx * cos_value - dy * sin_value;
    y += dx * sin_value + dy * cos_value;
    theta += d_theta;
    theta = angleClamp(theta);

    float32_t omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
    float32_t vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
    float32_t vy = back_vel + omega * BACK_RADIUS;

    robostate.pose.x = x;
    robostate.pose.y = y;
    robostate.pose.theta = theta;

    robostate.twist.vx = vx;
    robostate.twist.vy = vy;
    robostate.twist.w = omega;
}

/**
 * @brief Continuously sends robot data via UART.
 */
void send_data()
{
    free_wheel.init();
    uint32_t loop_tick = 0;
    uint32_t now;

    while (1)
    {
        now = HAL_GetTick();

        if (now - loop_tick < 5)
            continue;

        loop_tick = HAL_GetTick();

        free_wheel.read_data();
        free_wheel.process_data();

        static uint32_t transmit_tick;
        if (now - transmit_tick >= 50)
        {
            free_wheel.sending_bytes[0] = START_BYTE;
            memcpy(free_wheel.sending_bytes + 1, (uint8_t *)(&free_wheel.robostate), sizeof(Robostate));
            free_wheel.sending_bytes[sizeof(Robostate) + 1] = crc.get_Hash((uint8_t *)(free_wheel.sending_bytes + 1), sizeof(Robostate));

            HAL_UART_Transmit_DMA(&huart2, free_wheel.sending_bytes, sizeof(Robostate) + 2);
            transmit_tick = now;

#ifdef PRINT_COUNT
            // printf("count: %ld %ld %ld\n", total_back_count, total_right_count, total_left_count);
#endif
        }
    }
}

float32_t degreeChange(float32_t curr, float32_t prev)
{
    float32_t change;

    if ((prev > 90) && (curr < -90))
    {
        change = (180 - prev) + (180 + curr);
    }
    else if ((prev < -90) && (curr > 90))
    {
        change = -(180 + prev) - (180 - curr);
    }
    else
    {
        change = curr - prev;
    }

    return change;
}

float32_t radianChange(float32_t curr, float prev)
{
    float32_t change;

    if ((prev > F32_PI_2) && (curr < -F32_PI_2))
    {
        change = (F32_PI - prev) + (F32_PI + curr);
    }
    else if ((prev < -F32_PI_2) && (curr > F32_PI_2))
    {
        change = -(F32_PI + prev) - (F32_PI - curr);
    }
    else
    {
        change = curr - prev;
    }

    return change;
}

float32_t angleClamp(float32_t angle)
{
    if (angle > F32_PI)
    {
        angle -= F32_2_PI;
    }
    else if (angle < (-F32_PI))
    {
        angle += F32_2_PI;
    }
    return angle;
}
