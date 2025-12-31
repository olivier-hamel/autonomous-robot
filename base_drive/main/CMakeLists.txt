idf_component_register(
    SRCS
        "app_main.cpp"
        "bus/i2c_bus.cpp"
        "sensors/mpu6050.cpp"
        "orientation/orientation_filter.cpp"
        "orientation/orientation_task.cpp"
    INCLUDE_DIRS "."
)