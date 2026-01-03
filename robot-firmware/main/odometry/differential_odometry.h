#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "odometry/odometry_types.h"

class DifferentialDriveOdometry
{
public:
    DifferentialDriveOdometry(float track_width_m, float wheel_radius_m);

    void reset(const Pose2D &initial_pose = {});
    bool integrate(const WheelStates &wheels, int64_t timestamp_us, OdometryState &out_state);
    void setYawHint(float yaw_rad) { yaw_hint_rad_ = yaw_rad; }

private:
    float track_width_m_;
    float wheel_radius_m_;

    Pose2D pose_{};
    Twist2D twist_{};

    std::array<float, 4> last_wheel_pos_rad_{};
    bool has_last_{false};
    int64_t last_timestamp_us_{0};
    std::optional<float> yaw_hint_rad_{};
};
