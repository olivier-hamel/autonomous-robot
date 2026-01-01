idf_component_register(
    SRCS
        "app_main.cpp"
        "bus/i2c_bus.cpp"
        "orientation/mpu6050.cpp"
        "orientation/orientation_filter.cpp"
        "orientation/orientation_task.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_timer
)