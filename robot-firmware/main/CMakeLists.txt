idf_component_register(
    SRCS
        "app_main.cpp"
        "micro_ros/micro_ros_cmd_vel.cpp"
        "micro_ros/micro_ros_odometry.cpp"
        "micro_ros/micro_ros_serial_transport.c"
        "bus/i2c_bus.cpp"
        "drive/yahboom_motor_driver.cpp"
        "drive/drive_base.cpp"
        "orientation/mpu6050.cpp"
        "orientation/orientation_filter.cpp"
        "orientation/orientation_task.cpp"
        "odometry/odometry_types.cpp"
        "odometry/differential_odometry.cpp"
        "odometry/odometry_task.cpp"
    INCLUDE_DIRS "." "drive" "odometry"
    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_uart esp_timer micro_ros_espidf_component
)