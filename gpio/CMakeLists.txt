idf_component_register(
    SRCS "gpio_functions.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio lsm6dsox
)