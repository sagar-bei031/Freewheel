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

/**
 * @brief Radius of the back wheel in meter.
 */
#define BACK_RADIUS 0.258f

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
struct Pose
{
    float32_t x;     /**< X-coordinate. */
    float32_t y;     /**< Y-coordinate. */
    float32_t theta; /**< Orientation (angle in radians). */
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

/**
 * @brief Structure representing the state of the robot.
 */
struct Robostate
{
    Pose pose;   /**< Robot pose. */
    Twist twist; /**< Robot twist. */
};

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

    Robostate robostate; /**< Robot position and twist. */

    Encoder back_enc{&htim3, CPR};  /**< Encoder for the back wheel. */
    Encoder right_enc{&htim1, CPR}; /**< Encoder for the right wheel. */
    Encoder left_enc{&htim2, CPR};  /**< Encoder for the left wheel. */

    int32_t back_count; /**< Count from the back wheel encoder. */
    int32_t right_count; /**< Count from the right wheel encoder. */
    int32_t left_count; /**< Count from the left wheel encoder. */

    // with respect to robot itself
    float32_t back_omega; /**< Angular velocity of the back wheel. */
    float32_t right_omega; /**< Angular velocity of the right wheel. */
    float32_t left_omega; /**< Angular velocity of the left wheel. */

    // with respect to gamefield
    float32_t x = 0.0f;     /**< X-coordinate (meter) of the robot. */
    float32_t y = 0.0f;     /**< Y-coordinate (meter) of the robot. */
    float32_t theta = 0.0f; /**< Orientation (yaw angle in radians) of the robot. */

    // start_byte:1, x:4, y:4, theta:4, vx:4, vy:4, omega:4, back_count:4, right_count:4, left_count:4, crc:1
    uint8_t sending_bytes[sizeof(Robostate)+2]; /**< Buffer for storing data to be transmitted. */
    bool is_transmitting = false; /**< Flag indicating if data transmission is in progress. */
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
