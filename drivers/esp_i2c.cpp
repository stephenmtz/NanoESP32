#include "esp_i2c.hpp"

#include <cstring>

namespace hal {

EspI2C::EspI2C(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz)
    : bus_(bus), scl_speed_hz_(scl_speed_hz) {}

EspI2C::~EspI2C() {
    for (size_t i = 0; i < deviceCount_; ++i) {
        i2c_master_bus_rm_device(devices_[i].dev);
    }
}

i2c_master_dev_handle_t EspI2C::getDevice(uint8_t addr) {
    for (size_t i = 0; i < deviceCount_; ++i) {
        if (devices_[i].addr == addr) {
            return devices_[i].dev;
        }
    }

    if (deviceCount_ >= kMaxDevices) {
        return nullptr;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = scl_speed_hz_,
        .scl_wait_us = 0,
        .flags =
            {
                .disable_ack_check = false,
            },
    };

    i2c_master_dev_handle_t dev = nullptr;
    if (i2c_master_bus_add_device(bus_, &dev_cfg, &dev) != ESP_OK) {
        return nullptr;
    }

    devices_[deviceCount_++] = DeviceEntry{addr, dev};
    return dev;
}

bool EspI2C::read(uint8_t addr, uint8_t reg, void* data, size_t length) {
    i2c_master_dev_handle_t dev = getDevice(addr);
    if (dev == nullptr) {  // empty
        return false;
    }
    return i2c_master_transmit_receive(dev, &reg, 1, static_cast<uint8_t*>(data), length,
                                       kXferTimeoutMs) == ESP_OK;
}

bool EspI2C::write(uint8_t addr, uint8_t reg, const void* data, size_t length) {
    i2c_master_dev_handle_t dev = getDevice(addr);
    if (dev == nullptr) {
        return false;
    }

    static constexpr size_t kMaxPayload = 32;
    if (length > kMaxPayload) {
        return false;
    }

    uint8_t buf[1 + kMaxPayload];
    buf[0] = reg;
    std::memcpy(buf + 1, data, length);

    return i2c_master_transmit(dev, buf, 1 + length, kXferTimeoutMs) == ESP_OK;
}

}  // namespace hal
