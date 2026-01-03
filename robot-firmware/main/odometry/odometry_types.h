#pragma once

#include <cstdint>

#include "drive/drive_base.h"

struct Pose2D
{
    float x_m{0.0f};
    float y_m{0.0f};
    float yaw_rad{0.0f};
};

struct Twist2D
{
    float linear_mps{0.0f};
    float angular_radps{0.0f};
};

struct OdometryState
{
    Pose2D pose{};
    Twist2D twist{};
    WheelStates wheels{};
    int64_t timestamp_us{0};
};

float normalizeAngle(float angle_rad);
