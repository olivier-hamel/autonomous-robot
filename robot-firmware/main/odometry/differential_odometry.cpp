#include "odometry/differential_odometry.h"

#include <cmath>

namespace
{
    constexpr float kEpsilon = 1e-6f;
}

DifferentialDriveOdometry::DifferentialDriveOdometry(float track_width_m, float wheel_radius_m)
    : track_width_m_(track_width_m), wheel_radius_m_(wheel_radius_m) {}

void DifferentialDriveOdometry::reset(const Pose2D &initial_pose)
{
    pose_ = initial_pose;
    twist_ = {};
    last_wheel_pos_rad_.fill(0.0f);
    has_last_ = false;
    last_timestamp_us_ = 0;
    yaw_hint_rad_.reset();
}

bool DifferentialDriveOdometry::integrate(const WheelStates &wheels, int64_t timestamp_us, OdometryState &out_state)
{
    if (!has_last_)
    {
        last_wheel_pos_rad_ = wheels.position_rad;
        last_timestamp_us_ = timestamp_us;
        has_last_ = true;

        if (yaw_hint_rad_.has_value())
        {
            pose_.yaw_rad = normalizeAngle(yaw_hint_rad_.value());
            yaw_hint_rad_.reset();
        }

        out_state.pose = pose_;
        out_state.twist = twist_;
        out_state.wheels = wheels;
        out_state.timestamp_us = timestamp_us;
        return false;
    }

    const float dt = static_cast<float>(timestamp_us - last_timestamp_us_) / 1e6f;
    if (dt < kEpsilon)
    {
        return false;
    }

    const float left_now = 0.5f * (wheels.position_rad[static_cast<size_t>(Wheel::FrontLeft)] +
                                   wheels.position_rad[static_cast<size_t>(Wheel::RearLeft)]);
    const float right_now = 0.5f * (wheels.position_rad[static_cast<size_t>(Wheel::FrontRight)] +
                                    wheels.position_rad[static_cast<size_t>(Wheel::RearRight)]);

    const float left_prev = 0.5f * (last_wheel_pos_rad_[static_cast<size_t>(Wheel::FrontLeft)] +
                                    last_wheel_pos_rad_[static_cast<size_t>(Wheel::RearLeft)]);
    const float right_prev = 0.5f * (last_wheel_pos_rad_[static_cast<size_t>(Wheel::FrontRight)] +
                                     last_wheel_pos_rad_[static_cast<size_t>(Wheel::RearRight)]);

    const float delta_left_rad = left_now - left_prev;
    const float delta_right_rad = right_now - right_prev;

    last_wheel_pos_rad_ = wheels.position_rad;
    last_timestamp_us_ = timestamp_us;

    const float delta_left_m = delta_left_rad * wheel_radius_m_;
    const float delta_right_m = delta_right_rad * wheel_radius_m_;

    const float delta_s = 0.5f * (delta_left_m + delta_right_m);
    const float delta_theta = (track_width_m_ > kEpsilon) ? ((delta_right_m - delta_left_m) / track_width_m_) : 0.0f;

    const float heading_mid = pose_.yaw_rad + 0.5f * delta_theta;
    pose_.x_m += delta_s * std::cos(heading_mid);
    pose_.y_m += delta_s * std::sin(heading_mid);
    pose_.yaw_rad = normalizeAngle(pose_.yaw_rad + delta_theta);

    if (yaw_hint_rad_.has_value())
    {
        pose_.yaw_rad = normalizeAngle(yaw_hint_rad_.value());
        yaw_hint_rad_.reset();
    }

    twist_.linear_mps = delta_s / dt;
    twist_.angular_radps = delta_theta / dt;

    out_state.pose = pose_;
    out_state.twist = twist_;
    out_state.wheels = wheels;
    out_state.timestamp_us = timestamp_us;
    return true;
}
