idf_component_register(
    SRCS "lsm6dsox.cpp" "esp_i2c.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)
