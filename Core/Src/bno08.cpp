#include "bno08.hpp"
#include <stdio.h>

void Bno08::init()
{
    if (huart != nullptr)
    {
        isWaitingForHeader = true;
        lastReceive = 0;
        HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer, 2);
    }
}

uint8_t Bno08::calcChecksum()
{
    uint8_t csum = 0;
    uint8_t *pbyte = (uint8_t *)&buffer.index;
    for (int i = 0; i < 16; i++)
    {
        csum += *(pbyte++);
    }
    return csum;
}

Bno08::BnoRecvStatus Bno08::receive()
{
    BnoRecvStatus status;
    if (isWaitingForHeader)
    {
        if (buffer.header == BNO_HEADER)
        {
            isWaitingForHeader = false;
            HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer.index, 17);
            status = HEADER_MATCHED;
        }
        else
        {
            HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer, 2);
            status = HEADER_ERROR;
            printf("header error\n");
        }
    }
    else
    {
        if (buffer.csum == calcChecksum())
        {
            parseImuData();
            lastReceive = HAL_GetTick();
            status = CHECKSUM_MATCHED;
        }
        else
        {
            status = CHECKSUM_ERROR;
            printf("checksum error\n");
        }
        isWaitingForHeader = true;
        HAL_UART_Receive_DMA(huart, (uint8_t *)&buffer, 2);
    }
    return status;
}

void Bno08::parseImuData()
{
    data.yaw = -static_cast<float>(buffer.yaw / 100) + 0.01f * static_cast<float>(buffer.yaw % 100);
    data.pitch = static_cast<float>(buffer.pitch / 100) + 0.01f * static_cast<float>(buffer.pitch % 100);
    data.roll = static_cast<float>(buffer.roll / 100) + 0.01f * static_cast<float>(buffer.roll % 100);
    data.accel_x = buffer.accel_x * _g / 1000;
    data.accel_y = buffer.accel_y * _g / 1000;
    data.accel_z = buffer.accel_z * _g / 1000;
}

bool Bno08::isConnected()
{
    if ((lastReceive != 0) && (HAL_GetTick() - lastReceive < 100))
    {
        return true;
    }
    return false;
}
void Bno08::printBuffer()
{
    printf("bno_buf: %02x %hu %hd %hd %hd %hd %hd %hd %hu %hu %hu %02x\n",
           buffer.header,
           buffer.index,
           buffer.yaw,
           buffer.pitch,
           buffer.roll,
           buffer.accel_x,
           buffer.accel_y,
           buffer.accel_z,
           buffer.mi,
           buffer.mr,
           buffer.rsvd,
           buffer.csum);
}