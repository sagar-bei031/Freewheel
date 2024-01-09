#ifndef FREE_WHEEL_H
#define FREE_WHEEL_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal_uart.h"

#ifdef __cplusplus
#include "crc.hpp"
#include "encoder.hpp"

struct Pose
{
    double x = 0;
    double y = 0;
    double theta = 0;
};

struct Twist
{
    double vx = 0;
    double vy = 0;
    double omega = 0;
};

struct Odometry
{
    Pose pose;
    Twist twist;
};

struct Sending_Odometry
{
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
};

struct Encoder_HandleType
{
    Encoder* xr;
    Encoder* xl;
    Encoder* y;
};

class Free_Wheel
{
private:
    Encoder enc_[3];

public:
    Free_Wheel();
    void init();
    void read_data();
    void process_data();

    Encoder_HandleType encoder;
    Odometry odometry;
    Sending_Odometry sending_odometry;
    uint8_t sending_bytes[26];
    CRC_Hash crc{7};

    int32_t xr_encoder_count, xl_encoder_count, y_encoder_count;
    double xr_encoder_velocity, xl_encoder_velocity, y_encoder_velocity;
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