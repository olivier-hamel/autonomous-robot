#include "odometry/odometry_types.h"

#include <cmath>

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;
}

float normalizeAngle(float angle_rad)
{
    while (angle_rad > kPi)
    {
        angle_rad -= kTwoPi;
    }
    while (angle_rad < -kPi)
    {
        angle_rad += kTwoPi;
    }
    return angle_rad;
}
