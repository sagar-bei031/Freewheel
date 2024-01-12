#include "free_wheel.h"
#include "gpio.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include <arm_math.h>

#define xR 0.260
#define yrR 0.255
#define ylR 0.223
#define Wheel_Diameter 0.0574

#define CPR_CW 4000
#define CPR_ACW 4000
#define CPR 4000

#define START_BYTE (0XA5)

Free_Wheel free_wheel;

uint8_t start_byte = START_BYTE, hash;

uint32_t last_led_tick = 0;

bool is_transmitting = false;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((HAL_GetTick() - last_led_tick) > 20)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_led_tick = HAL_GetTick();
    }
    is_transmitting = false;
}

Free_Wheel::Free_Wheel()
{
    x = y = theta = 0;
}

void Free_Wheel::init()
{
    enc[0] = Encoder(&htim3); // back
    enc[1] = Encoder(&htim1); // right
    enc[2] = Encoder(&htim2); // left

    enc[0].init();
    enc[1].init();
    enc[2].init();
}

void Free_Wheel::read_data()
{
    xCnt = enc[0].get_count();   // (+) -> CW
    yrCnt = -enc[1].get_count(); // (+) -> ACW
    ylCnt = enc[2].get_count();  // (+) -> CW

    xRev = (float)xCnt / CPR; // (+) -> CW
    wx = enc[0].get_omega(CPR);

    yrRev = (float)yrCnt / CPR; // (+) -> ACW
    wyr = -enc[1].get_omega(CPR);

    ylRev = (float)ylCnt / CPR; // (-) -> ACW
    wyl = enc[2].get_omega(CPR);


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

    float d_theta = (rightY_dist - leftY_dist) / (ylR + yrR);

    theta += d_theta;

    if (theta > M_PI)
    {
        theta -= 2.0f * M_PI;
    }
    else if (theta < (-M_PI))
    {
        theta += 2.0f * M_PI;
    }

    // float dy = leftY_dist + ylR * d_theta;

    float dy = (rightY_dist * ylR + leftY_dist * yrR) / (ylR + yrR);
    float dx = backX_dist - xR * d_theta;

    x += dx * arm_cos_f32(theta + d_theta / 2.0f) - dy * arm_sin_f32(theta + d_theta / 2.0f);
    y += dx * arm_sin_f32(theta + d_theta / 2.0f) + dy * arm_cos_f32(theta + d_theta / 2.0f);

    omega = (yr_vel - yl_vel) / (yrR + ylR);
    vy = (yr_vel + yl_vel) / (yrR + ylR);
    vx = x_vel - omega * xR;

    robostate.odometry.x = y;
    robostate.odometry.y = -x;
    robostate.odometry.theta = theta;

    robostate.twist.vx = vy;
    robostate.twist.vy = -vx;
    robostate.twist.w = omega;

    x = round3(x);
    y = round3(y);
    theta = round3(theta);
}

void send_data()
{
    free_wheel.init();
    uint32_t last_tick = HAL_GetTick();
    uint32_t last_uart_tick = last_tick;

    HAL_UART_Transmit_DMA(&huart2, free_wheel.sending_bytes, 26);
    while (1)
    {
        if ((HAL_GetTick() - last_tick) < 10)
            continue;

        free_wheel.read_data();
        free_wheel.process_data();

        uint32_t d_time = HAL_GetTick() - last_uart_tick;

        if (((d_time >= 50) && (!is_transmitting)) | (d_time > 100))
        {
            free_wheel.sending_bytes[0] = START_BYTE;

            memcpy(free_wheel.sending_bytes + 1, (uint8_t *)(&free_wheel.robostate), 24);
            free_wheel.sending_bytes[25] = free_wheel.crc.get_Hash((uint8_t *)(free_wheel.sending_bytes + 1), 24);
            last_uart_tick = HAL_GetTick();
        }

        last_tick = HAL_GetTick();
    }
}

float round3(float val)
{
    return (float)((int32_t)(val * 1000.0f)) / 1000.0f;
}
