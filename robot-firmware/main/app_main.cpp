#include <cstdio>

#include <array>
#include <cmath>
#include <memory>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bus/i2c_bus.h"
#include "orientation/mpu6050.h"
#include "orientation/orientation_task.h"
#include "drive/yahboom_motor_driver.h"
#include "drive/drive_base.h"

#include "micro_ros/micro_ros_cmd_vel.h"
#include "micro_ros/micro_ros_odometry.h"

#include "odometry/differential_odometry.h"
#include "odometry/odometry_task.h"

static const char *TAG = "app";

class DriveBaseCommandSink : public DriveCommandSink
{
public:
    explicit DriveBaseCommandSink(DriveBase &drive) : drive_(drive) {}

    esp_err_t applyTwist(float linear_x_mps, float angular_z_radps) override
    {
        return drive_.setTwist(linear_x_mps, angular_z_radps);
    }

private:
    DriveBase &drive_;
};

static void scan_bus(I2CBus &bus)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 0x7F; ++addr)
    {
        esp_err_t r = bus.probeAddress(addr);
        if (r == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Scan complete");
}

static void motor_test_task(void *ctx)
{

    // ============= MOTOR TEST TASK ============= //

    auto *drive_base = static_cast<DriveBase *>(ctx);
    if (!drive_base)
    {
        ESP_LOGW(TAG, "Motor test aborted : drive_base wrong");
        vTaskDelete(nullptr);
        return;
    }

    auto command = [&](float lin_mps, float ang_radps, int duration_ms)
    {
        if (drive_base->setTwist(lin_mps, ang_radps) != ESP_OK)
        {
            ESP_LOGW(TAG, "Drive command failled (lin=%.2f m/s, ang=%.2f rad/s)", lin_mps, ang_radps);
            vTaskDelay(pdMS_TO_TICKS(duration_ms));
        }
    };

    ESP_LOGI(TAG, "Starting motor test");

    command(0.25f, 0.0f, 2000);  // avance 2 sec
    command(0.0f, 0.0f, 500);    // pause 0.5 sec
    command(0.0f, 0.6f, 1500);   // tourne gauche 1.5 sec
    command(0.0f, 0.0f, 500);    // pause 0.5 sec
    command(-0.25f, 0.0f, 2000); // recule 2 sec
    command(0.0f, 0.0f, 500);    // pause 0.5 sec
    command(0.0f, -0.6f, 1500);  // tourne droit 1.5 sec
    command(0.0f, 0.0f, 800);    // stop

    WheelStates states{};
    if (drive_base->sampleWheelStates(states) == ESP_OK)
    {
        ESP_LOGI(TAG, "Test complete | wheel rad : %.3f   %.3f   %.3f   %.3f | rad/s : %.3f   %.3f   %.3f   %.3f | Batt : %.2f V",
                 states.position_rad[0], states.position_rad[1], states.position_rad[2], states.position_rad[3],
                 states.velocity_rad_s[0], states.velocity_rad_s[1], states.velocity_rad_s[2], states.velocity_rad_s[3],
                 states.battery_v);
    }

    (void)drive_base->setWheelPwm({0, 0, 0, 0});
    ESP_LOGI(TAG, "Motor test ended");
    vTaskDelete(nullptr);
}

extern "C" void app_main()
{
    I2CBus bus;
    ESP_ERROR_CHECK(bus.begin());

    scan_bus(bus);

    // ============= MOTOR SETUP ============= //

    YahboomMotorDriver motor(bus);
    bool motor_ready = false;
    std::unique_ptr<DriveBase> drive_base;
    std::unique_ptr<DriveBaseCommandSink> cmd_vel_sink;
    std::unique_ptr<DriveBaseWheelSource> wheel_source;
    std::unique_ptr<DifferentialDriveOdometry> odom_estimator;
    std::unique_ptr<QueueOdometrySink> odom_queue;
    std::unique_ptr<OdometryTask> odom_task;
    MotorConfig motor_cfg = MotorConfig::Default520();
    const DriveGeometry drive_geom{0.23f, 800.f};
    const float wheel_radius_m = motor_cfg.wheel_diameter_mm * 0.0005f;

    esp_err_t probe_26 = bus.probeAddress(0x26);
    if (probe_26 != ESP_OK)
    {
        ESP_LOGW(TAG, "Motor board not found at 0x26");
    }

    if (motor.init() == ESP_OK)
    {
        esp_err_t cfg_err = motor.applyConfig(motor_cfg);
        if (cfg_err != ESP_OK)
        {
            ESP_LOGW(TAG, "Motor config failed : %s", esp_err_to_name(cfg_err));
        }
        else
        {
            esp_err_t stop_err = motor.setPwm({0, 0, 0, 0});
            if (stop_err != ESP_OK)
            {
                ESP_LOGW(TAG, "Motor stop sommand failed : %s", esp_err_to_name(stop_err));
            }
            else
            {
                motor_ready = true;
                ESP_LOGI(TAG, "Motor driver ready");
                drive_base = std::make_unique<DriveBase>(motor, motor_cfg, drive_geom);
                cmd_vel_sink = std::make_unique<DriveBaseCommandSink>(*drive_base);
                wheel_source = std::make_unique<DriveBaseWheelSource>(*drive_base);
                odom_estimator = std::make_unique<DifferentialDriveOdometry>(drive_geom.track_width_m, wheel_radius_m);
                odom_queue = std::make_unique<QueueOdometrySink>(16);
                if (odom_queue && odom_queue->valid() && wheel_source && odom_estimator)
                {
                    odom_task = std::make_unique<OdometryTask>(*wheel_source, *odom_estimator, *odom_queue, pdMS_TO_TICKS(50));
                    esp_err_t odom_task_err = odom_task->start("odom_task", tskIDLE_PRIORITY + 2, 4096, tskNO_AFFINITY);
                    if (odom_task_err != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Odometry task not started: %s", esp_err_to_name(odom_task_err));
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Odometry components not initialized");
                }
                xTaskCreate(motor_test_task, "motor_test", 4096, drive_base.get(), tskIDLE_PRIORITY + 3, nullptr);
            }
        }
    }
    else
    {
        ESP_LOGW(TAG, "Motor init failed");
    }

    if (!motor_ready)
    {
        ESP_LOGW(TAG, "Motor driver not ready. Skipping motor control");
    }

    // ============= MPU6050 SETUP ============= //

    Mpu6050 imu(bus, 0x68, GPIO_NUM_2);
    MadgwickFilter filter(0.1f);

    while (true)
    {
        esp_err_t init_err = imu.init();
        if (init_err == ESP_OK)
        {
            ESP_LOGI(TAG, "MPU6050 initialized");
            break;
        }
        ESP_LOGE(TAG, "MPU init failed: %s", esp_err_to_name(init_err));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    OrientationTask orientation_task(imu, filter, pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(orientation_task.start("imu_task", tskIDLE_PRIORITY + 2, 4096, tskNO_AFFINITY));

    if (cmd_vel_sink)
    {
        esp_err_t uros_err = micro_ros_cmd_vel_start(*cmd_vel_sink);
        if (uros_err != ESP_OK)
        {
            ESP_LOGW(TAG, "micro-ROS cmd_vel subscriber not started: %s", esp_err_to_name(uros_err));
        }
    }
    else
    {
        ESP_LOGW(TAG, "micro-ROS cmd_vel subscriber skipped (drive not ready)");
    }

    if (odom_queue && odom_queue->valid())
    {
        esp_err_t odom_err = micro_ros_odometry_start(*odom_queue);
        if (odom_err != ESP_OK)
        {
            ESP_LOGW(TAG, "micro-ROS odometry publisher not started: %s", esp_err_to_name(odom_err));
        }
    }
    else
    {
        ESP_LOGW(TAG, "micro-ROS odometry publisher skipped (odometry not ready)");
    }

    OrientationSample sample{};
    while (true)
    {
        if (xQueueReceive(orientation_task.queue(), &sample, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            if (odom_estimator)
            {
                const float yaw_rad = sample.orientation.yaw * static_cast<float>(M_PI) / 180.0f;
                odom_estimator->setYawHint(yaw_rad);
            }

            ESP_LOGI(TAG, "Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg (ts=%lld)",
                     sample.orientation.roll, sample.orientation.pitch, sample.orientation.yaw,
                     static_cast<long long>(sample.timestamp_us));

            /**
            AccelReading accel{};
            GyroReading gyro{};
            if (imu.readAcceleration(accel) == ESP_OK && imu.readGyroscope(gyro) == ESP_OK) {
                 ESP_LOGI(TAG, "MPU6050 accel[g] x=%.3f y=%.3f z=%.3f | gyro[dps] x=%.3f y=%.3f z=%.3f",
                          accel.x_g, accel.y_g, accel.z_g,
                          gyro.x_dps, gyro.y_dps, gyro.z_dps);
            }
            */
        }
        else
        {
            ESP_LOGW(TAG, "No orientation update received");
        }
    }
}