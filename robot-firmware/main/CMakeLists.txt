idf_component_register(
    SRCS
        "app_main.cpp"
        "bus/i2c_bus.cpp"
        "drive/yahboom_motor_driver.cpp"
        "drive/drive_base.cpp"
        "orientation/mpu6050.cpp"
        "orientation/orientation_filter.cpp"
        "orientation/orientation_task.cpp"
    INCLUDE_DIRS "." "drive"
    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_timer
)