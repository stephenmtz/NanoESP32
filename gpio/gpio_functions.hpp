#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "lsm6dsox.hpp"

namespace gpio_functions {

    static constexpr gpio_num_t kAccelInt1Pin = GPIO_NUM_2;
    static constexpr uint64_t kAccelInt1PinMask = (1ULL << kAccelInt1Pin);

    void setupAccelInterrupt(drivers::LSM6DSOX& imu);

}  // namespace gpio_functions
