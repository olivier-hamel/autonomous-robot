#include "drive/drive_base.h"

#include <algorithm>
#include <cmath>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace
{
    static const char *TAG = "DriveBase";
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDeltaSampleSeconds = 0.01f; // fenetre de 10 ms
}

DriveBase::DriveBase(YahboomMotorDriver &driver, MotorConfig config, DriveGeometry geometry) : driver_(driver), config_(config), geometry_(geometry) {}

esp_err_t DriveBase::setTwist(float linear_x_mps, float angular_z_radps)
{
    // mapping différentiel: left = v - w * b/2, right = v + w * b/2.
    const float half_track = geometry_.track_width_m * 0.5f;
    const float left_mps = linear_x_mps - angular_z_radps * half_track;
    const float right_mps = linear_x_mps + angular_z_radps * half_track;

    // Convertion en mm/s pour chaque roue
    std::array<float, 4> wheel_mmps{
        left_mps * 1000.0f,  // Avant gauche
        left_mps * 1000.0f,  // Arrière gauche
        right_mps * 1000.0f, // avant droit
        right_mps * 1000.0f  // Arrière droit
    };

    return setWheelLinearSpeedsMmps(wheel_mmps);
}

esp_err_t DriveBase::setWheelLinearSpeedsMmps(const std::array<float, 4> &wheel_mmps)
{
    std::array<int16_t, 4> cmd{};
    for (size_t i = 0; i < wheel_mmps.size(); ++i)
    {
        cmd[i] = mmpsToRegister(wheel_mmps[i]);
    }
    return driver_.setSpeed(cmd);
}

esp_err_t DriveBase::setWheelPwm(const std::array<int16_t, 4> &pwm)
{
    return driver_.setPwm(pwm);
}

esp_err_t DriveBase::sampleWheelStates(WheelStates &out)
{
    std::array<int32_t, 4> totals{};
    std::array<int16_t, 4> deltas{};
    float battery = 0.0f;

    ESP_RETURN_ON_ERROR(driver_.readEncoderTotals(totals), TAG, "Read totals failed");
    ESP_RETURN_ON_ERROR(driver_.readEncoderDelta(deltas), TAG, "Read deltas failed");
    ESP_RETURN_ON_ERROR(driver_.readBatteryVoltage(battery), TAG, "Read battery failed");

    for (size_t i = 0; i < totals.size(); ++i)
    {
        out.position_rad[i] = wheelCountsToRadians(totals[i]);
        out.velocity_rad_s[i] = deltaCountsToRadPerSec(deltas[i], kDeltaSampleSeconds);
    }

    out.battery_v = battery;
    out.timestamp_us = esp_timer_get_time();
    return ESP_OK;
}

float DriveBase::wheelCountsToRadians(int32_t counts) const
{
    const float counts_per_rev = YahboomMotorDriver::countsPerWheelRevolution(config_);
    const float revs = static_cast<float>(counts) / counts_per_rev;
    return revs * 2.0f * kPi;
}

float DriveBase::deltaCountsToRadPerSec(int16_t delta_counts, float dt_seconds) const
{
    const float counts_per_rev = YahboomMotorDriver::countsPerWheelRevolution(config_);
    const float revs = static_cast<float>(delta_counts) / counts_per_rev;
    return (revs * 2.0f * kPi) / dt_seconds;
}

int16_t DriveBase::mmpsToRegister(float mmps) const
{
    const float clamped = std::clamp(mmps, -geometry_.max_linear_mmps, geometry_.max_linear_mmps);
    // Le registre de vitesse est big-endian [-1000, 1000]
    const float limited = std::clamp(clamped, -1000.0f, 1000.0f);
    return static_cast<int16_t>(std::lround(limited));
}