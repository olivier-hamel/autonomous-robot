#include "odometry/odometry_task.h"

#include "esp_log.h"

namespace
{
    static const char *TAG = "OdometryTask";
}

QueueOdometrySink::QueueOdometrySink(size_t depth)
{
    queue_ = xQueueCreate(depth, sizeof(OdometryState));
}

QueueOdometrySink::~QueueOdometrySink()
{
    if (queue_ != nullptr)
    {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }
}

void QueueOdometrySink::publish(const OdometryState &state)
{
    if (queue_ == nullptr)
    {
        return;
    }

    if (xQueueSend(queue_, &state, 0) != pdTRUE)
    {
        OdometryState dropped{};
        (void)xQueueReceive(queue_, &dropped, 0);
        (void)xQueueSend(queue_, &state, 0);
    }
}

bool QueueOdometrySink::take(OdometryState &out, TickType_t wait_ticks)
{
    if (queue_ == nullptr)
    {
        return false;
    }
    return xQueueReceive(queue_, &out, wait_ticks) == pdTRUE;
}

OdometryTask::OdometryTask(WheelStateSource &wheel_source,
                           DifferentialDriveOdometry &estimator,
                           OdometrySink &sink,
                           TickType_t period_ticks)
    : wheel_source_(wheel_source), estimator_(estimator), sink_(sink), period_(period_ticks)
{
}

esp_err_t OdometryTask::start(const char *task_name, UBaseType_t priority, uint32_t stack_words, BaseType_t core_id)
{
    if (task_ != nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t created = xTaskCreatePinnedToCore(taskEntry, task_name, stack_words, this, priority, &task_, core_id);
    return (created == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void OdometryTask::taskEntry(void *arg)
{
    auto *self = static_cast<OdometryTask *>(arg);
    if (self != nullptr)
    {
        self->run();
    }
    vTaskDelete(nullptr);
}

void OdometryTask::run()
{
    while (true)
    {
        WheelStates wheels{};
        esp_err_t rc = wheel_source_.sample(wheels);
        if (rc != ESP_OK)
        {
            ESP_LOGW(TAG, "Wheel sample failed: %s", esp_err_to_name(rc));
            vTaskDelay(period_);
            continue;
        }

        OdometryState state{};
        (void)estimator_.integrate(wheels, wheels.timestamp_us, state);
        sink_.publish(state);

        vTaskDelay(period_);
    }
}
