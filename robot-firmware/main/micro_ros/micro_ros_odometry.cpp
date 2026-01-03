#include "micro_ros_odometry.h"

#include "micro_ros_serial_transport.h"

#include <cmath>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

namespace
{
    static const char *TAG = "micro_ros_odom";

    constexpr char kNodeName[] = "robot_odometry_bridge";
    constexpr char kOdomTopic[] = "/odom";
    constexpr TickType_t kTakeTimeout = pdMS_TO_TICKS(200);
    constexpr TickType_t kLoopDelay = pdMS_TO_TICKS(10);

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

    geometry_msgs__msg__Quaternion yawToQuaternion(float yaw_rad)
    {
        geometry_msgs__msg__Quaternion q{};
        const float half = 0.5f * yaw_rad;
        q.z = std::sin(half);
        q.w = std::cos(half);
        return q;
    }

    builtin_interfaces__msg__Time toRosTime(int64_t timestamp_us)
    {
        builtin_interfaces__msg__Time t{};
        const int64_t sec = timestamp_us / 1000000LL;
        const int64_t nsec = (timestamp_us - sec * 1000000LL) * 1000LL;
        t.sec = static_cast<int32_t>(sec);
        t.nanosec = static_cast<uint32_t>(nsec);
        return t;
    }

    class OdometryPublisher
    {
    public:
        explicit OdometryPublisher(OdometrySource &source) : source_(source) {}

        esp_err_t start()
        {
            if (task_ != nullptr)
            {
                return ESP_OK;
            }

            BaseType_t ok = xTaskCreate(
                &OdometryPublisher::taskEntry,
                "uros_odom",
                CONFIG_ROBOT_MICRO_ROS_ODOM_STACK,
                this,
                CONFIG_ROBOT_MICRO_ROS_ODOM_TASK_PRIO,
                &task_);

            ESP_LOGI(TAG, "micro-ROS odom task create result: %s", (ok == pdPASS) ? "ok" : "failed");
            return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
        }

    private:
        static void taskEntry(void *arg)
        {
            auto *self = static_cast<OdometryPublisher *>(arg);
            if (self != nullptr)
            {
                self->run();
            }
            vTaskDelete(NULL);
        }

        void run()
        {
            ESP_LOGI(TAG, "micro-ROS odom task starting");

            rcl_allocator_t allocator = rcl_get_default_allocator();
            rclc_support_t support;
            RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

            rcl_node_t node;
            RCCHECK(rclc_node_init_default(&node, kNodeName, "", &support));

            rcl_publisher_t publisher;
            RCCHECK(rclc_publisher_init_default(
                &publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                kOdomTopic));

            nav_msgs__msg__Odometry odom_msg{};
            nav_msgs__msg__Odometry__init(&odom_msg);

            ESP_LOGI(TAG, "Publishing odometry to '%s' from node '%s'", kOdomTopic, kNodeName);

            while (true)
            {
                OdometryState state{};
                if (source_.take(state, kTakeTimeout))
                {
                    odom_msg.header.stamp = toRosTime(state.timestamp_us);
                    (void)rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, CONFIG_ROBOT_MICRO_ROS_ODOM_FRAME_ID);
                    (void)rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, CONFIG_ROBOT_MICRO_ROS_BASE_FRAME_ID);

                    odom_msg.pose.pose.position.x = state.pose.x_m;
                    odom_msg.pose.pose.position.y = state.pose.y_m;
                    odom_msg.pose.pose.position.z = 0.0;
                    odom_msg.pose.pose.orientation = yawToQuaternion(state.pose.yaw_rad);

                    odom_msg.twist.twist.linear.x = state.twist.linear_mps;
                    odom_msg.twist.twist.linear.y = 0.0;
                    odom_msg.twist.twist.linear.z = 0.0;
                    odom_msg.twist.twist.angular.x = 0.0;
                    odom_msg.twist.twist.angular.y = 0.0;
                    odom_msg.twist.twist.angular.z = state.twist.angular_radps;

                    RCSOFTCHECK(rcl_publish(&publisher, &odom_msg, nullptr));
                }
                vTaskDelay(kLoopDelay);
            }
        }

        OdometrySource &source_;
        TaskHandle_t task_{nullptr};
    };
}

esp_err_t micro_ros_odometry_start(OdometrySource &source)
{
#if !CONFIG_ROBOT_MICRO_ROS_ODOM_PUBLISHER
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

    static OdometryPublisher publisher(source);
    return publisher.start();
#endif
}
