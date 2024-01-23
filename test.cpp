#include <iostream>
using namespace std;

#include <stdint.h>
#include <cmath>

#define PI 3.141592654f

/**
 * @brief Radius of the back wheel in meter.
 */
#define BACK_RADIUS 0.255f

/**
 * @brief Radius of the right wheel in meter.
 */
#define RIGHT_RADIUS 0.255f

/**
 * @brief Radius of the left wheel in meter.
 */
#define LEFT_RADIUS 0.225f

/**
 * @brief Diameter of the wheels in meter.
 */
#define Wheel_Diameter 0.0574

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
    float x;     /**< X-coordinate. */
    float y;     /**< Y-coordinate. */
    float theta; /**< Orientation (angle in radians). */
};

/**
 * @brief Structure representing the twist (linear and angular velocities) of the robot.
 */
struct Twist
{
    float vx; /**< Linear velocity in the x-direction. */
    float vy; /**< Linear velocity in the y-direction. */
    float w;  /**< Angular velocity. */
};

/**
 * @brief Structure representing the state of the robot.
 */
struct Robostate
{
    Pose pose;   /**< Robot pose. */
    Twist twist; /**< Robot twist. */
};

float x = 0.0f;
float y = 0.0;
float theta = 0.0f;

Robostate robostate;

int main()
{

    int32_t back_count = 400;
    int32_t right_count = -400;
    int32_t left_count = 4000;

    float back_omega = 1.0f;
    float right_omega = 1.0f;
    float left_omega = 1.0f;

    float back_dist = PI * Wheel_Diameter * back_count / CPR;
    float right_dist = PI * Wheel_Diameter * right_count / CPR;
    float left_dist = PI * Wheel_Diameter * left_count / CPR;

    float back_vel = back_omega * Wheel_Diameter / 2.0f;
    float right_vel = right_omega * Wheel_Diameter / 2.0f;
    float left_vel = left_omega * Wheel_Diameter / 2.0f;

    float d_theta = (right_dist - left_dist) / (LEFT_RADIUS + RIGHT_RADIUS);

    // if (HAL_GetTick() - imu_input_tick < 500U)
    // {
    //     float d_imu_yaw;

    //     if ((prev_imu_yaw > PI) && (prev_imu_yaw < PI) && (imu_yaw < -PI_2) && (imu_yaw > -PI))
    //     {
    //         d_imu_yaw = (PI - prev_imu_yaw) + (PI + imu_yaw);
    //     }
    //     else if ((prev_imu_yaw < -PI) && (prev_imu_yaw > -PI) && (imu_yaw > PI_2) && (imu_yaw < PI))
    //     {
    //         d_imu_yaw = -(PI + prev_imu_yaw) - (PI - imu_yaw);
    //     }
    //     else
    //     {
    //         d_imu_yaw = imu_yaw - prev_imu_yaw;
    //     }

    //     d_theta = 0.1f * d_theta + 0.9f * d_imu_yaw;
    // }

    float dx = (right_dist * LEFT_RADIUS + left_dist * RIGHT_RADIUS) / (LEFT_RADIUS + RIGHT_RADIUS);
    float dy = back_dist + BACK_RADIUS * d_theta;

    float cos_value = cos(theta + d_theta / 2.0f);
    float sin_value = sin(theta + d_theta / 2.0f);

    x += dx * cos_value - dy * sin_value;
    y += dx * sin_value + dy * cos_value;

    theta += d_theta;

    if (theta > PI)
    {
        theta -= 2.0f * PI;
    }
    else if (theta < (-PI))
    {
        theta += 2.0f * PI;
    }

    float omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
    float vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
    float vy = back_vel + omega * BACK_RADIUS;

#ifdef __count
    static uint32_t last_process_tick;
    uint32_t now = HAL_GetTick();
    uint16_t dt = now - last_process_tick;

    robostate.pose.x = bc;
    robostate.pose.y = rc;
    robostate.pose.theta = lc;

    robostate.twist.vx = back_omega;
    robostate.twist.vy = right_omega;
    robostate.twist.w = arm_sin_f32(PI / 6);

    last_process_tick = now;
#else
    robostate.pose.x = x;
    robostate.pose.y = y;
    robostate.pose.theta = theta;

    robostate.twist.vx = vx;
    robostate.twist.vy = vy;
    robostate.twist.w = omega;
#endif

    cout << x << endl;
    cout << y << endl;
    cout << theta << endl;
}