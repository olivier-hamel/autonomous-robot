#pragma once

#include <array>
#include <cstdint>

#include "esp_err.h"
#include "drive/yahboom_motor_driver.h"

/** @brief Index des roues */
enum class Wheel : uint8_t
{
    FrontLeft = 0,  /** @brief Roue avant gauche (M1). */
    RearLeft = 1,   /** @brief Roue arrière gauche (M2). */
    FrontRight = 2, /** @brief Roue avant droite (M3). */
    RearRight = 3   /** @brief Roue arrière droite (M4). */
};

/** @brief Paramètre géométrique de la base */
struct DriveGeometry
{
    float track_width_m;   /** @brief Distance entre les centres des roues gauche et droite (mètres). */
    float max_linear_mmps; /** @brief Saturation de la consigne de vitesse linéaire roue (mm/s). */
};

/**
 * @brief États estimés des roues + batterie
 */
struct WheelStates
{
    std::array<float, 4> position_rad{};   /** @brief Angles des roues (rad). */
    std::array<float, 4> velocity_rad_s{}; /** @brief Vitesses angulaires des roues (rad/s). */
    float battery_v{0.0f};                 /** @brief Tension mesurée (V). */
    int64_t timestamp_us{0};               /** @brief Timestamp de l’échantillon (microsecondes, esp_timer). */
};

class DriveBase
{
public:
    DriveBase(YahboomMotorDriver &driver, MotorConfig config, DriveGeometry geometry);

    /**
     * @brief Applique une commande de twist (vitesse linéaire(v) + vitesse angulaire(w))
     * vitesse gauche = v - w * (b/2)
     * vitesse droit = v + w * (b/2)
     * Où b = track_with_m
     *
     * @param linear_x_mps vitesse lineraire
     * @param angular_z_radps vitesse angulaire autour de l'axe verticale
     */
    esp_err_t setTwist(float linear_x_mps, float angular_z_radps);

    esp_err_t setWheelLinearSpeedsMmps(const std::array<float, 4> &wheel_mmps);
    esp_err_t setWheelPwm(const std::array<int16_t, 4> &pwm);
    esp_err_t sampleWheelStates(WheelStates &out);

    const MotorConfig &config() const { return config_; }
    const DriveGeometry &geometry() const { return geometry_; };

private:
    YahboomMotorDriver &driver_;
    MotorConfig config_;
    DriveGeometry geometry_;

    float wheelCountsToRadians(int32_t counts) const;
    float deltaCountsToRadPerSec(int16_t delta_counts, float dt_seconds) const;
    int16_t mmpsToRegister(float mmps) const;
};