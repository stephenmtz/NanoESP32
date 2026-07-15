#pragma once

#include <memory>
#include <cstdint>
#include <cstddef>

namespace hal {
class I2C {
public:
    using SharedPtr = std::shared_ptr<I2C>;
    virtual ~I2C() {}

    virtual bool read(uint8_t addr, uint8_t reg, void *data, size_t length) = 0;
    virtual bool write(uint8_t addr, uint8_t reg, const void *data, size_t length) = 0;

    bool read(uint8_t addr, uint8_t reg, uint8_t &val) {
        return read(addr, reg, &val, sizeof(val));
    }
    bool read(uint8_t addr, uint8_t reg, uint16_t &val) {
        return read(addr, reg, &val, sizeof(val));
    }
    bool read(uint8_t addr, uint8_t reg, uint32_t &val) {
        return read(addr, reg, &val, sizeof(val));
    }

    bool write(uint8_t addr, uint8_t reg, uint8_t val) {
        return write(addr, reg, &val, sizeof(val));
    }
    bool write(uint8_t addr, uint8_t reg, uint16_t val) {
        return write(addr, reg, &val, sizeof(val));
    }
    bool write(uint8_t addr, uint8_t reg, uint32_t val) {
        return write(addr, reg, &val, sizeof(val));
    }
};
}  // namespace hal