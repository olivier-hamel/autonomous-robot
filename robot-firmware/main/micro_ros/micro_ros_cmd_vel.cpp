#include "micro_ros_cmd_vel.h"

#include "micro_ros_serial_transport.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>

namespace
{
    static const char *TAG = "micro_ros_cmd_vel";

    constexpr char kNodeName[] = "robot_cmd_vel_bridge";
    constexpr char kCmdVelTopic[] = "/cmd_vel";
    constexpr unsigned kExecutorSpinMs = 50;
    constexpr unsigned kTaskSleepMs = 10;

#define RCCHECK(fn)                                                                                 \
    do                                                                                              \
    {                                                                                               \
        rcl_ret_t rc_ = (fn);                                                                       \
        if (rc_ != RCL_RET_OK)                                                                      \
        {                                                                                           \
            ESP_LOGE(TAG, "RCL error %d at %s:%d", (int)rc_, __FILE__, __LINE__);                   \
            vTaskDelete(NULL);                                                                      \
        }                                                                                           \
    } while (0)

#define RCSOFTCHECK(fn)                                                                              \
    do                                                                                               \
    {                                                                                                \
        rcl_ret_t rc_ = (fn);                                                                        \
        if (rc_ != RCL_RET_OK)                                                                       \
        {                                                                                            \
            ESP_LOGW(TAG, "RCL soft error %d at %s:%d", (int)rc_, __FILE__, __LINE__);              \
        }                                                                                            \
    } while (0)

    class CmdVelSubscriber
    {
    public:
        explicit CmdVelSubscriber(DriveCommandSink &sink) : sink_(sink) {}

        esp_err_t start()
        {
            if (task_handle_ != nullptr)
            {
                return ESP_OK;
            }

            BaseType_t ok = xTaskCreate(
                &CmdVelSubscriber::taskEntry,
                "uros_cmd_vel",
                CONFIG_ROBOT_MICRO_ROS_APP_STACK,
                this,
                CONFIG_ROBOT_MICRO_ROS_APP_TASK_PRIO,
                &task_handle_);

            ESP_LOGI(TAG, "micro-ROS cmd_vel task create result: %s", (ok == pdPASS) ? "ok" : "failed");
            return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
        }

    private:
        static void taskEntry(void *arg)
        {
            auto *self = static_cast<CmdVelSubscriber *>(arg);
            self->run();
            vTaskDelete(NULL);
        }

        static void cmdVelCallback(const void *msgin, void *context)
        {
            auto *self = static_cast<CmdVelSubscriber *>(context);
            const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);
            if (self != nullptr)
            {
                self->handleTwist(msg);
            }
        }

        void handleTwist(const geometry_msgs__msg__Twist *msg)
        {
            if (msg == nullptr)
            {
                return;
            }

            const float linear_x = static_cast<float>(msg->linear.x);
            const float angular_z = static_cast<float>(msg->angular.z);

            esp_err_t rc = sink_.applyTwist(linear_x, angular_z);
            if (rc != ESP_OK)
            {
                ESP_LOGW(TAG, "Drive command rejected (lin=%.3f, ang=%.3f): %s", linear_x, angular_z, esp_err_to_name(rc));
            }
        }

        void run()
        {
            ESP_LOGI(TAG, "micro-ROS cmd_vel task starting");

            rcl_allocator_t allocator = rcl_get_default_allocator();
            rclc_support_t support;
            RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

            rcl_node_t node;
            RCCHECK(rclc_node_init_default(&node, kNodeName, "", &support));

            rcl_subscription_t subscription;
            RCCHECK(rclc_subscription_init_default(
                &subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                kCmdVelTopic));

            geometry_msgs__msg__Twist twist_msg{};

            rclc_executor_t executor;
            RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
            RCCHECK(rclc_executor_add_subscription_with_context(
                &executor,
                &subscription,
                &twist_msg,
                &CmdVelSubscriber::cmdVelCallback,
                this,
                ON_NEW_DATA));

            ESP_LOGI(TAG, "Subscribed to '%s' on node '%s'", kCmdVelTopic, kNodeName);

            while (true)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(kExecutorSpinMs));
                vTaskDelay(pdMS_TO_TICKS(kTaskSleepMs));
            }
        }

        DriveCommandSink &sink_;
        TaskHandle_t task_handle_{nullptr};
    };
}

esp_err_t micro_ros_cmd_vel_start(DriveCommandSink &sink)
{
#if !CONFIG_ROBOT_MICRO_ROS_CMD_VEL_SUBSCRIBER
    return ESP_OK;
#else
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    static uart_port_t uart_port = (uart_port_t)CONFIG_ROBOT_MICRO_ROS_UART_PORT;

    ESP_LOGI(TAG,
             "Configuring micro-ROS custom UART transport: uart_port=%d baud=%d TX=%d RX=%d",
             (int)uart_port,
             (int)CONFIG_ROBOT_MICRO_ROS_UART_BAUDRATE,
             (int)CONFIG_MICROROS_UART_TXD,
             (int)CONFIG_MICROROS_UART_RXD);

    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        micro_ros_serial_open,
        micro_ros_serial_close,
        micro_ros_serial_write,
        micro_ros_serial_read);
#else
    return ESP_ERR_INVALID_STATE;
#endif

    static CmdVelSubscriber subscriber(sink);
    return subscriber.start();
#endif
}
