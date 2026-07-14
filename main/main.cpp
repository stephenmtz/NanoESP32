#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_i2c.hpp"
#include "lsm6dsox.hpp"
#include "gpio_functions.hpp"

static const char* TAG = "main";

static constexpr gpio_num_t kI2cSdaPin = GPIO_NUM_11;
static constexpr gpio_num_t kI2cSclPin = GPIO_NUM_12;

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Nano ESP32 booting");

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = kI2cSdaPin,
        .scl_io_num = kI2cSclPin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    auto i2c = std::make_shared<hal::EspI2C>(bus);

    static drivers::LSM6DSOX imu(i2c, drivers::LSM6DSOX::kDeviceAddLow);

    if (!imu.init()) {
        ESP_LOGE(TAG, "IMU init failed, check wiring/address");
        return;
    }

    if (!imu.setAccelFullScale(drivers::LSM6DSOX::kXlFs2g) ||
        !imu.setAccelOdr(drivers::LSM6DSOX::kXlOdr104Hz)) {
        ESP_LOGE(TAG, "Accel config failed");
        return;
    }

    if (!imu.setInt1Ctrl(drivers::LSM6DSOX::INT1_DRDY_XL)) {
        ESP_LOGE(TAG, "Failed to route accel data-ready to INT1");
        return;
    }

    for (uint8_t reg = 0x10; reg <= 0x19; ++reg) {
        uint8_t val;
        if (i2c->read(drivers::LSM6DSOX::kDeviceAddLow, reg, val)) {
            ESP_LOGI(TAG, "0x%02X = 0x%02X", reg, val);
        }
    }

    gpio_functions::setupAccelInterrupt(imu);

    ESP_LOGI(TAG, "Setup complete, accel_task running");
}
