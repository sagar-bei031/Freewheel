#ifndef BNO08_HPP_
#define BNO08_HPP_

#include "usart.h"

#define BNO_PACKET_SIZE 19
#define BNO_HEADER 0xAAAA
#define BNO_TIME_PERIOD 10 // milliseconds for 100Hz
#define _g 9.80665f

#pragma pack(push, 1)
struct BnoData
{
    uint16_t header;
    uint8_t index;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    uint8_t mi;
    uint8_t mr;
    uint8_t rsvd;
    uint8_t csum;
};

struct ImuData
{
    float yaw;
    float pitch;
    float roll;
    float accel_x;
    float accel_y;
    float accel_z;
};
#pragma pack(pop)

class Bno08
{
public:
    enum BnoRecvStatus
    {
        HEADER_MATCHED,
        HEADER_ERROR,
        CHECKSUM_MATCHED,
        CHECKSUM_ERROR
    };

    UART_HandleTypeDef *huart;
    BnoData buffer;
    ImuData data;
    bool isWaitingForHeader;
    uint32_t lastReceive;

    Bno08() : huart(nullptr) {}
    Bno08(UART_HandleTypeDef *_huart): huart(_huart) {}
    ~Bno08() = default;

    void init();
    uint8_t calcChecksum();
    BnoRecvStatus receive();
    void parseImuData();
    bool isConnected();
    void printBuffer();
};

#endif // BNO08_HPP_