#pragma once

#include "esp_err.h"


class DriveCommandSink
{
public:
    virtual ~DriveCommandSink() = default;

    virtual esp_err_t applyTwist(float linear_x_mps, float angular_z_radps) = 0;
};

esp_err_t micro_ros_cmd_vel_start(DriveCommandSink &sink);
