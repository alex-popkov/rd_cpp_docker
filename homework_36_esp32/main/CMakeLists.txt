idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_timer esp_driver_i2c esp_driver_uart device_core
)