#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "odometry/differential_odometry.h"

class WheelStateSource
{
public:
    virtual ~WheelStateSource() = default;
    virtual esp_err_t sample(WheelStates &out) = 0;
};

class OdometrySink
{
public:
    virtual ~OdometrySink() = default;
    virtual void publish(const OdometryState &state) = 0;
};

class OdometrySource
{
public:
    virtual ~OdometrySource() = default;
    virtual bool take(OdometryState &out, TickType_t wait_ticks) = 0;
};

class QueueOdometrySink : public OdometrySink, public OdometrySource
{
public:
    explicit QueueOdometrySink(size_t depth);
    ~QueueOdometrySink();

    void publish(const OdometryState &state) override;
    bool take(OdometryState &out, TickType_t wait_ticks) override;
    QueueHandle_t handle() const { return queue_; }
    bool valid() const { return queue_ != nullptr; }

private:
    QueueHandle_t queue_{};
};

class OdometryTask
{
public:
    OdometryTask(WheelStateSource &wheel_source,
                 DifferentialDriveOdometry &estimator,
                 OdometrySink &sink,
                 TickType_t period_ticks = pdMS_TO_TICKS(50));

    esp_err_t start(const char *task_name = "odom_task",
                    UBaseType_t priority = tskIDLE_PRIORITY + 2,
                    uint32_t stack_words = 4096,
                    BaseType_t core_id = tskNO_AFFINITY);

private:
    static void taskEntry(void *arg);
    void run();

    WheelStateSource &wheel_source_;
    DifferentialDriveOdometry &estimator_;
    OdometrySink &sink_;

    TickType_t period_;
    TaskHandle_t task_{nullptr};
};

class DriveBaseWheelSource : public WheelStateSource
{
public:
    explicit DriveBaseWheelSource(DriveBase &drive) : drive_(drive) {}
    esp_err_t sample(WheelStates &out) override { return drive_.sampleWheelStates(out); }

private:
    DriveBase &drive_;
};
