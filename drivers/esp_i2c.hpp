#pragma once

#include "i2c.hpp"

#include "driver/i2c_master.h"

namespace hal {

class EspI2C : public I2C {
    public:
        EspI2C(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz = 400000);
        ~EspI2C() override;

        EspI2C(const EspI2C&);
        EspI2C& operator=(const EspI2C&);

        bool read(uint8_t addr, uint8_t reg, void *data, size_t length) override;
        bool write(uint8_t addr, uint8_t reg, const void *data, size_t length) override;

    private:
        i2c_master_dev_handle_t getDevice(uint8_t addr);

        i2c_master_bus_handle_t bus_;
        uint32_t scl_speed_hz_;

        static constexpr int kXferTimeoutMs = 100;
        static constexpr size_t kMaxDevices = 8;

        struct DeviceEntry {
            uint8_t addr = 0;
            i2c_master_dev_handle_t dev = nullptr;
        };
        DeviceEntry devices_[kMaxDevices];
        size_t deviceCount_ = 0;
};

}
