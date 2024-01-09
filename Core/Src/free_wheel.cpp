#include "free_wheel.h"
#include "encoder.hpp"

#include "gpio.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include <arm_math.h>

#define xr_Radius 0.255
#define xl_Radius 0.223
#define y_Radius 0.260
#define Wheel_Diameter 0.0574
#define CPR 4000
#define START_BYTE 0XA5

Free_Wheel free_wheel;
bool is_transmitting = false;

Free_Wheel::Free_Wheel()
{
    odometry.pose.x = 0;
    odometry.pose.y = 0;
    odometry.pose.theta = 0;
}

void Free_Wheel::init()
{
    enc_[0] = Encoder(&htim1, CPR); // right
    enc_[1] = Encoder(&htim2, CPR); // left
    enc_[2] = Encoder(&htim3, CPR); // back

    encoder.xr = &enc_[0];
    encoder.xl = &enc_[1];
    encoder.y = &enc_[2];
}

void Free_Wheel::read_data()
{
    xr_encoder_count = -encoder.xr->get_count();
    xl_encoder_count = encoder.xl->get_count();
    y_encoder_count = -encoder.y->get_count();

    xr_encoder_velocity = -encoder.xr->get_omega() * Wheel_Diameter / 2.0;
    xl_encoder_velocity = encoder.xl->get_omega() * Wheel_Diameter / 2.0;
    y_encoder_velocity = -encoder.y->get_omega() * Wheel_Diameter / 2.0;

    encoder.xr->reset_encoder_count();
    encoder.xl->reset_encoder_count();
    encoder.y->reset_encoder_count();
}

void Free_Wheel::process_data()
{
    double xr_encoder_distance = M_PI * Wheel_Diameter * xr_encoder_count / CPR;
    double xl_encoder_distance = M_PI * Wheel_Diameter * xl_encoder_count / CPR;  
    double y_encoder_distance = M_PI * Wheel_Diameter * y_encoder_count / CPR;  

    double d_theta = (xr_encoder_distance * xl_Radius - xl_encoder_distance * xr_Radius) / (xl_Radius + xr_Radius);

    odometry.pose.theta += d_theta;
    if (odometry.pose.theta > M_PI)
    {
        odometry.pose.theta -= 2 * M_PI;
    }
    else if (odometry.pose.theta < (-M_PI))
    {
        odometry.pose.theta += 2 * M_PI;
    }

    double dx = (xr_encoder_distance * xl_Radius + xl_encoder_distance * xr_Radius) / (xl_Radius + xr_Radius);
    double dy = y_encoder_distance - y_Radius * d_theta;

    float cos_value = arm_cos_f32(odometry.pose.theta + d_theta / 2.0);
    float sin_value = arm_sin_f32(odometry.pose.theta + d_theta / 2.0);

    odometry.pose.x += dx * cos_value - dy * sin_value;
    odometry.pose.y += dx * sin_value + dy * cos_value;

    odometry.twist.vx = (xr_encoder_velocity * xl_Radius + xl_encoder_velocity * xr_Radius) / (xl_Radius + xr_Radius);
    odometry.twist.vy = y_encoder_velocity;
    odometry.twist.omega = (xr_encoder_velocity - xl_encoder_velocity) / (xr_Radius + xl_Radius) * 2.0 / Wheel_Diameter;

    sending_odometry.x = odometry.pose.x;
    sending_odometry.y = odometry.pose.y;
    sending_odometry.theta = odometry.pose.theta;

    sending_odometry.vx = odometry.twist.vx;
    sending_odometry.vy = odometry.twist.vy;
    sending_odometry.omega = odometry.twist.omega;
}

void send_data()
{
    free_wheel.init();
    uint32_t last_tick = HAL_GetTick();
    uint32_t last_uart_tick = last_tick;

    while (1)
    {
        if ((HAL_GetTick() - last_tick) < 10)
            continue;

        free_wheel.read_data();
        free_wheel.process_data();

        uint32_t d_time = HAL_GetTick() - last_uart_tick;

        if (((d_time > 33) && (!is_transmitting)) || (d_time > 100))
        {
            free_wheel.sending_bytes[0] = START_BYTE;

            memcpy(free_wheel.sending_bytes + 1, (uint8_t *)(&free_wheel.sending_odometry), 24);

            free_wheel.sending_bytes[25] = free_wheel.crc.get_Hash((uint8_t *)(free_wheel.sending_bytes + 1), 24);

            HAL_UART_Transmit_DMA(&huart2, free_wheel.sending_bytes, 26);
            last_uart_tick = HAL_GetTick();
            is_transmitting = true;
        }

        last_tick = HAL_GetTick();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
 
    is_transmitting = false;
}
