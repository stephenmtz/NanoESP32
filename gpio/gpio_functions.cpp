#include "gpio_functions.hpp"

#include "esp_log.h"

namespace gpio_functions {
    
    namespace {  

        static const char* TAG = "gpio_functions";

        constexpr int kIntrFlagDefault = 0;

        QueueHandle_t gpio_evt_queue = nullptr;

        void IRAM_ATTR gpio_isr_handler(void* arg) {

            uint32_t gpio_num = reinterpret_cast<uint32_t>(arg);
            BaseType_t woken = pdFALSE;
            xQueueSendFromISR(gpio_evt_queue, &gpio_num, &woken);
            portYIELD_FROM_ISR(woken);
        }

        void accel_task(void* arg) {

            auto* imu = static_cast<drivers::LSM6DSOX*>(arg);
            uint32_t io_num;

            for (;;) {
                if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
                    int16_t x, y, z;
                    if (imu->readAccel(x, y, z)) {
                        ESP_LOGI(TAG, "raw[%6d %6d %6d]  mg[%8.1f %8.1f %8.1f]",
                                x, y, z,
                                imu->lsm6soxzFromFs2ToMg(x),
                                imu->lsm6soxzFromFs2ToMg(y),
                                imu->lsm6soxzFromFs2ToMg(z));
                    } else {
                        ESP_LOGW(TAG, "readAccel failed");
                    }
                }
            }
        }

}  // namespace

    void setupAccelInterrupt(drivers::LSM6DSOX& imu) {

        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        io_conf.pin_bit_mask = kAccelInt1PinMask;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        gpio_config(&io_conf);

        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

        xTaskCreate(accel_task, "accel_task", 4096, &imu, 10, nullptr);

        gpio_install_isr_service(kIntrFlagDefault);
        gpio_isr_handler_add(kAccelInt1Pin, gpio_isr_handler,
                            reinterpret_cast<void*>(kAccelInt1Pin));
    }

}  // namespace gpio_functions