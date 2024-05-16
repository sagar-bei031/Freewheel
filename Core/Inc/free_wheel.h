/**
 ******************************************************************************
 * @file           freewheel.h
 * @brief          Header file containing the Free_Wheel class definition.
 * @author         Robotics Team, IOE Pulchowk Campus
 * @date           2023
 ******************************************************************************
 */

#ifndef FREE_WHEEL_H
#define FREE_WHEEL_H

#ifdef __cplusplus

#include "encoder.hpp"
#include "tim.h"
#include "usart.h"
#include "bno08.hpp"

/**
 * @brief Radius of the back wheel in meter.
 */
#define BACK_RADIUS 0.132f

/**
 * @brief Radius of the right wheel in meter.
 */
#define RIGHT_RADIUS 0.245f

/**
 * @brief Radius of the left wheel in meter.
 */
#define LEFT_RADIUS 0.225f

/**
 * @brief Diameter of the wheels in meter.
 */
#define Wheel_Diameter 0.0574f

/**
 * @brief Start byte for data transmission.
 */
#define START_BYTE 0xA5

/**
 * @brief Counts per revolution for encoders.
 */
#define CPR 4000U

/**
 * @brief Structure representing the pose (position and orientation) of the robot.
 */

#pragma pack(push, 1)
struct Pose
{
    float32_t x;     /**< X-coordinate. */
    float32_t y;     /**< Y-coordinate. */
    float32_t theta; /**< Orientation (angle in radians). */
};

struct YawPitchRoll
{
    float32_t yaw;
    float32_t pitch;
    float32_t roll;
};

/**
 * @brief Structure representing the twist (linear and angular velocities) of the robot.
 */
struct Twist
{
    float32_t vx; /**< Linear velocity in the x-direction. */
    float32_t vy; /**< Linear velocity in the y-direction. */
    float32_t w;  /**< Angular velocity. */
};

struct EncoderCount
{
    int32_t back;
    int32_t right;
    int32_t left;
};

struct FreeWheelData
{
    Pose pose;
    Twist twist;
    YawPitchRoll imu;
};
#pragma pack(pop)

/**
 * @brief Class for controlling a robot with free wheels.
 */
class Free_Wheel
{
public:
    /**
     * @brief Default constructor.
     */
    Free_Wheel() = default;

    /**
     * @brief Default destructor.
     */
    ~Free_Wheel() = default;

    /**
     * @brief Initialize the Free_Wheel object.
     */
    void init();

    /**
     * @brief Read data from encoders and update the robot state.
     */
    void read_data();

    /**
     * @brief Process the collected data.
     */
    void process_data();

    Bno08 bno{&huart1};
    FreeWheelData free_wheel_data{0, 0, 0, 0, 0, 0, 0, 0, 0};

    Encoder back_enc{&htim3, CPR};  /**< Encoder for the back wheel. */
    Encoder right_enc{&htim1, CPR}; /**< Encoder for the right wheel. */
    Encoder left_enc{&htim2, CPR};  /**< Encoder for the left wheel. */

    int32_t back_count;  /**< Count from the back wheel encoder. */
    int32_t right_count; /**< Count from the right wheel encoder. */
    int32_t left_count;  /**< Count from the left wheel encoder. */

    // with respect to robot itself
    float32_t back_omega;  /**< Angular velocity of the back wheel. */
    float32_t right_omega; /**< Angular velocity of the right wheel. */
    float32_t left_omega;  /**< Angular velocity of the left wheel. */

    uint8_t sending_bytes[sizeof(free_wheel_data) + 2]; /**< Buffer for storing data to be transmitted. */
};
#endif

#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Send data from the Free_Wheel object.
     *
     * It uses free_wheel object to read, process and send data.
     * It joins c program with cpp.
     * It is called in main.
     */
    void send_data();
#ifdef __cplusplus
}
#endif

#endif // FREE_WHEEL_H
