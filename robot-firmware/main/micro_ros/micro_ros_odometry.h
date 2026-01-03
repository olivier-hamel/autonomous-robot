#pragma once

#include "esp_err.h"

#include "odometry/odometry_task.h"

esp_err_t micro_ros_odometry_start(OdometrySource &source);
