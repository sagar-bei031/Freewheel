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
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "usart.h"
#include "tim.h"
#include "crc.hpp"
#include "encoder.hpp"
#include "free_wheel.h"

#define F32_2_PI 6.28318530717958f
#define F32_PI 3.14159265358979f
#define F32_PI_2 1.57079632679489f

Free_Wheel free_wheel;

int32_t total_mid_count = 0;
int32_t total_right_count = 0;
int32_t total_left_count = 0;

float32_t &x = free_wheel.free_wheel_data.pose.x;         /**< X-coordinate (meter) of the robot. */
float32_t &y = free_wheel.free_wheel_data.pose.y;         /**< Y-coordinate (meter) of the robot. */
float32_t &theta = free_wheel.free_wheel_data.pose.theta; /**< Orientation (yaw angle in radians) of the robot. */

float32_t &vx = free_wheel.free_wheel_data.twist.vx;   /**< Linear velocity in x-direction (meter/second). */
float32_t &vy = free_wheel.free_wheel_data.twist.vy;   /**< Linear velocity in y-direction (meter/second). */
float32_t &omega = free_wheel.free_wheel_data.twist.w; /**< Angular velocity (radian/second) of the robot. */

bool is_imu_ready = false;
float32_t &imu_yaw = free_wheel.free_wheel_data.imu.yaw;     /**< Yaw angle in radians. */
float32_t &imu_pitch = free_wheel.free_wheel_data.imu.pitch; /**< Pitch angle in radians. */
float32_t &imu_roll = free_wheel.free_wheel_data.imu.roll;   /**< Roll angle in radians. */

float32_t &ax = free_wheel.free_wheel_data.imu.accel_x; /**< Acceleration in x-direction (m/s^2). */
float32_t &ay = free_wheel.free_wheel_data.imu.accel_y; /**< Acceleration in y-direction (m/s^2). */
float32_t &az = free_wheel.free_wheel_data.imu.accel_z; /**< Acceleration in z-direction (m/s^2). */

float32_t cur_imu_yaw;
float32_t cur_imu_pitch;
float32_t cur_imu_roll;

float32_t prev_imu_yaw = 0.0f;
float32_t prev_imu_pitch = 0.0f;
float32_t prev_imu_roll = 0.0f;

uint32_t board_led_tick = 0;
uint32_t red_led_tick = 0;

uint8_t receiving_bytes[6];
bool is_waiting_for_start_byte = true;

CRC_Hash crc(7);

float32_t degreeChange(float32_t curr, float32_t prev);
float32_t radianChange(float32_t curr, float32_t prev);
float32_t angleClamp(float32_t angle);

/**
 * @brief Callback function for handling the reception of data via UART.
 * @param huart Pointer to the UART handle structure.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    // uint32_t now = HAL_GetTick();

    if (huart->Instance == free_wheel.bno.huart->Instance)
    {
        // uint32_t last_receive = free_wheel.bno.lastReceive;
        // if (
        free_wheel.bno.receive();
        //  == Bno08::BnoRecvStatus::CHECKSUM_MATCHED)
        // {

        // printf("imu: %lu %f %f %f %f %f %f\n",
        //        now - last_receive,
        //        free_wheel.bno.data.yaw,
        //        free_wheel.bno.data.pitch,
        //        free_wheel.bno.data.roll,
        //        free_wheel.bno.data.accel_x,
        //        free_wheel.bno.data.accel_y,
        //        free_wheel.bno.data.accel_z);
        // }
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
    static uint32_t yellow_led_tick = 0;
    uint32_t now = HAL_GetTick();

    if (now - yellow_led_tick > 20)
    {
        HAL_GPIO_TogglePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);
        yellow_led_tick = now;
    }
}

/**
 * @brief Initializes the Free_Wheel object.
 */
void Free_Wheel::init()
{
    bno.init();
    mid_enc.init();
    right_enc.init();
    left_enc.init();
}

/**
 * @brief Reads data from encoders and updates the robot state.
 */
void Free_Wheel::read_data()
{
    mid_count = -mid_enc.get_count();
    right_count = right_enc.get_count();
    left_count = -left_enc.get_count();

    total_mid_count += mid_count;
    total_right_count += right_count;
    total_left_count += left_count;

    mid_omega = -mid_enc.get_omega();
    right_omega = -right_enc.get_omega();
    left_omega = left_enc.get_omega();

    mid_enc.reset_encoder_count();
    right_enc.reset_encoder_count();
    left_enc.reset_encoder_count();

    if (bno.isConnected())
    {
        cur_imu_yaw = free_wheel.bno.data.yaw * M_PI / 180;
        cur_imu_pitch = free_wheel.bno.data.pitch * M_PI / 180;
        cur_imu_roll = free_wheel.bno.data.roll * M_PI / 180;

        ax = free_wheel.bno.data.accel_x;
        ay = free_wheel.bno.data.accel_y;
        az = free_wheel.bno.data.accel_z;
    }

    // printf("cnt: %ld %ld %ld\n", total_mid_count, total_right_count, total_left_count);
}

/**
 * @brief Processes the collected data to update the robot's position and orientation.
 */

void Free_Wheel::process_data()
{
    uint32_t now = HAL_GetTick();

    float32_t mid_dist = F32_PI * Wheel_Diameter * (float32_t)mid_count / (float)CPR;
    float32_t right_dist = F32_PI * Wheel_Diameter * (float32_t)right_count / (float32_t)CPR;
    float32_t left_dist = F32_PI * Wheel_Diameter * (float32_t)left_count / (float32_t)CPR;

    float32_t mid_vel = mid_omega * Wheel_Diameter / 2.0f;
    float32_t right_vel = right_omega * Wheel_Diameter / 2.0f;
    float32_t left_vel = left_omega * Wheel_Diameter / 2.0f;

    // float32_t d_theta = (right_dist - left_dist) / (LEFT_RADIUS + RIGHT_RADIUS);
    float32_t d_theta = 0; 
    if (bno.isConnected())
    {
        if (is_imu_ready)
        {
            float32_t d_imu_yaw = radianChange(cur_imu_yaw, prev_imu_yaw);
            float32_t d_imu_pitch = radianChange(cur_imu_pitch, prev_imu_pitch);
            float32_t d_imu_roll = radianChange(cur_imu_roll, prev_imu_roll);

            d_theta = d_imu_yaw;
            imu_yaw = angleClamp(imu_yaw + d_imu_yaw);
            imu_pitch = angleClamp(imu_pitch + d_imu_pitch);
            imu_roll = angleClamp(imu_roll + d_imu_roll);

            if (now - board_led_tick > 20)
            {
                HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
                board_led_tick = now;
            }
        }
        else
        {
            if (now - red_led_tick > 20)
            {
                HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
                red_led_tick = now;
            }
        }

        prev_imu_yaw = cur_imu_yaw;
        prev_imu_pitch = cur_imu_pitch;
        prev_imu_roll = cur_imu_roll;
        is_imu_ready = true;
    }
    else
    {
        is_imu_ready = false;
    }

    float32_t dx = (right_dist * LEFT_RADIUS + left_dist * RIGHT_RADIUS) / (LEFT_RADIUS + RIGHT_RADIUS);
    float32_t dy = mid_dist - MID_RADIUS * d_theta;

    float32_t theta_t = angleClamp(theta + d_theta / 2.0f);
    float32_t cos_value = arm_cos_f32(theta_t);
    float32_t sin_value = arm_sin_f32(theta_t);

    x += dx * cos_value - dy * sin_value;
    y += dx * sin_value + dy * cos_value;
    theta = angleClamp(theta + d_theta);

    omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
    vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
    vy = mid_vel - omega * MID_RADIUS;
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
        if (now - transmit_tick >= 10)
        {
            free_wheel.sending_bytes[0] = START_BYTE;
            memcpy(free_wheel.sending_bytes + 1, (uint8_t *)(&free_wheel.free_wheel_data), sizeof(free_wheel.free_wheel_data));
            free_wheel.sending_bytes[sizeof(free_wheel.free_wheel_data) + 1] = crc.get_Hash((uint8_t *)(free_wheel.sending_bytes + 1), sizeof(free_wheel.free_wheel_data));

#ifdef USE_USB
            CDC_Transmit_FS(free_wheel.sending_bytes, sizeof(free_wheel.free_wheel_data) + 2);
            HAL_GPIO_TogglePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);
#else
            if (HAL_UART_Transmit_DMA(&huart2, free_wheel.sending_bytes, sizeof(free_wheel.free_wheel_data) + 2) != HAL_OK)
            {
                static uint32_t red_led_tick = 0;
                if (now - red_led_tick > 20)
                {
                    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
                    red_led_tick = now;
                }
            }
#endif
            // printf("x:%f y:%f theta:%f vx:%f vy:%f w:%f yaw:%f pitch:%f roll:%f ax:%f ay:%f az:%f\n",
            // free_wheel.free_wheel_data.pose.x, free_wheel.free_wheel_data.pose.y, free_wheel.free_wheel_data.pose.theta,
            // free_wheel.free_wheel_data.twist.vx, free_wheel.free_wheel_data.twist.vy, free_wheel.free_wheel_data.twist.w,
            // free_wheel.free_wheel_data.imu.yaw, free_wheel.free_wheel_data.imu.pitch, free_wheel.free_wheel_data.imu.roll,
            // free_wheel.free_wheel_data.imu.accel_x, free_wheel.free_wheel_data.imu.accel_y, free_wheel.free_wheel_data.imu.accel_z);

            transmit_tick = now;
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
