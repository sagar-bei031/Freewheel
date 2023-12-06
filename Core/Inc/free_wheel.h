#ifndef FREE_WHEEL_H
#define FREE_WHEEL_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal_uart.h"

#ifdef __cplusplus
#include "crc.hpp"
#include "encoder.hpp"

struct Odometry
{
    float x = 0;
    float y = 0;
    float theta = 0;
};

class Free_Wheel
{
public:
    Free_Wheel();
    void init();
    void read_data();
    void process_data();

    Encoder enc[3];
    Odometry odom;
    CRC_Hash crc{7};
    uint8_t sending_bytes[14];
    // uint8_t receive_uart1[3];

    int32_t xCnt, ylCnt, yrCnt;
    // int32_t last_xCnt, last_ylCnt, last_yrCnt;

    // float ftheta;
};

#endif

#ifdef __cplusplus
extern "C"
{
#endif
    void send_data();
#ifdef __cplusplus
}
#endif

#endif // FREE_WHEEL_H