#include <cstdio>

#include <array>
#include <memory>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bus/i2c_bus.h"
#include "orientation/mpu6050.h"
#include "orientation/orientation_task.h"

static const char *TAG = "app";

static void scan_bus(I2CBus &bus) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
        esp_err_t r = bus.probeAddress(addr);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Scan complete");
}

extern "C" void app_main() { 
    I2CBus bus;
    ESP_ERROR_CHECK(bus.begin());

    scan_bus(bus);

    Mpu6050 imu(bus, 0x68, GPIO_NUM_2);
    MadgwickFilter filter(0.1f);

    while (true)
    {
        esp_err_t init_err = imu.init();
        if (init_err == ESP_OK) { 
            ESP_LOGI(TAG, "MPU6050 initialized");
            break;
        }
        ESP_LOGE(TAG, "MPU init failed: %s", esp_err_to_name(init_err));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    OrientationTask orientation_task(imu, filter, pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(orientation_task.start("imu_task", tskIDLE_PRIORITY + 2, 4096, tskNO_AFFINITY));

    OrientationSample sample{};
    while(true) {
        if (xQueueReceive(orientation_task.queue(), &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {
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

        } else {
            ESP_LOGW(TAG, "No orientation update received");
        }
    }
    
 
}