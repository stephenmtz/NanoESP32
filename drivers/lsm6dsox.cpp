#include "lsm6dsox.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace drivers {

    static const char* TAG = "LSM6DSOX";

    LSM6DSOX::LSM6DSOX(hal::I2C::SharedPtr i2c, I2CAddr addr)
        : i2c_(i2c), i2c_addr_(addr) {
        configASSERT(i2c_ != nullptr);
    }

    std::optional<LSM6DSOX::RegVal> LSM6DSOX::readReg(RegAddr reg_addr){
        RegVal val;
        if (!i2c_->read(i2c_addr_, reg_addr, val)) {
            return std::nullopt;
        }
        return val;
    }

    bool LSM6DSOX::writeReg(RegAddr reg_addr, RegVal reg_val){
        return i2c_->write(i2c_addr_, reg_addr, reg_val);
    }

    bool LSM6DSOX::writeBitField(RegAddr addr, RegMask mask, RegBitPos bit_pos, RegVal val){
        auto cur = readReg(addr);
        if (!cur) {
            return false;
        }
        RegVal updated = static_cast<RegVal>((*cur & ~mask) | ((val << bit_pos) & mask));
        return writeReg(addr, updated);
    }

    std::optional<int16_t> LSM6DSOX::readAxis(RegAddr low_reg){
        auto lo = readReg(low_reg);
        if (!lo) {
            return std::nullopt;
        }
        auto hi = readReg(static_cast<RegAddr>(low_reg + 1));
        if (!hi) {
            return std::nullopt;
        }
        return static_cast<int16_t>(static_cast<uint16_t>(*lo) |
                                    (static_cast<uint16_t>(*hi) << 8));
    }

    bool LSM6DSOX::readAccelBurst(int16_t& x, int16_t& y, int16_t& z){
        uint8_t buff[6];
        if (!i2c_->read(i2c_addr_, kOutXLXl, buff, sizeof(buff))) {
            return false;
        }
        x = static_cast<int16_t>(buff[0] | (static_cast<uint16_t>(buff[1]) << 8));
        y = static_cast<int16_t>(buff[2] | (static_cast<uint16_t>(buff[3]) << 8));
        z = static_cast<int16_t>(buff[4] | (static_cast<uint16_t>(buff[5]) << 8));
        return true;
    }

    bool LSM6DSOX::whoAmI(uint8_t& id){
        auto val = readReg(kWhoAmI);
        if (!val) {
            return false;
        }
        id = *val;
        return true;
    }

    bool LSM6DSOX::reset(){
        if (!writeBitField(kCtrl3C, kSwResetMask, 0, 1)) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
        for (int i = 0; i < 20; ++i) {
            auto val = readReg(kCtrl3C);
            if (val && (*val & kSwResetMask) == 0) {
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        ESP_LOGE(TAG, "Software reset timed out");
        return false;
    }

    bool LSM6DSOX::init(){
        uint8_t id;
        if (!whoAmI(id) || id != kWhoAmIExpected) {
            ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", id, kWhoAmIExpected);
            return false;
        }
        ESP_LOGI(TAG, "WHO_AM_I matches");

        if (!reset()) {
            ESP_LOGE(TAG, "Reset failed");
            return false;
        }

        if (!writeBitField(kCtrl3C, kIfIncMask, 2, 1)){
            ESP_LOGE(TAG, "Failed to set IF_INC");
            return false;
        }

        if (!setBdu(true)) {
            ESP_LOGE(TAG, "Failed to set BDU");
            return false;
        }

        return true;
    }

    bool LSM6DSOX::setBdu(bool enable){
        return writeBitField(kCtrl3C, kBduMask, 6, enable ? 1 : 0);
    }

    float LSM6DSOX::lsm6soxzFromFs2ToMg(int16_t lsb){ 
        return static_cast<float>(lsb) * 0.061f; 
    }

    float LSM6DSOX::lsm6soxzFromFs4ToMg(int16_t lsb){ 
        return static_cast<float>(lsb) * 0.122f; 
    }
    
    float LSM6DSOX::lsm6soxzFromFs8ToMg(int16_t lsb){ 
        return static_cast<float>(lsb) * 0.244f; 
    }

    float LSM6DSOX::lsm6soxzFromFs16ToMg(int16_t lsb){ 
        return static_cast<float>(lsb) * 0.488f; 
    }

    bool LSM6DSOX::setAccelOdr(AccelOdr odr){
        return writeBitField(kCtrl1Xl, kOdrMask, 4, odr);
    }

    bool LSM6DSOX::setAccelFullScale(AccelFullScale fs){
        return writeBitField(kCtrl1Xl, kFsMask, 2, fs);
    }

    std::optional<int16_t> LSM6DSOX::readAccelX(){ 
        return readAxis(kOutXLXl); 
    }
    std::optional<int16_t> LSM6DSOX::readAccelY(){ 
        return readAxis(kOutYLXl); 
    }
    std::optional<int16_t> LSM6DSOX::readAccelZ(){ 
        return readAxis(kOutZLXl); 
    }

    bool LSM6DSOX::readAccel(int16_t &x, int16_t &y, int16_t &z){
        return readAccelBurst(x, y, z);
    }

    bool LSM6DSOX::setAccelUserOffset(int8_t x, int8_t y, int8_t z){
        return writeReg(kXOfsUsr, static_cast<RegVal>(x)) &&
            writeReg(kYOfsUsr, static_cast<RegVal>(y)) &&
            writeReg(kZOfsUsr, static_cast<RegVal>(z));
    }

    bool LSM6DSOX::enableAccelLowPowerMode(bool enable){
        return writeBitField(kCtrl5C, kXlUlpMask, 7, enable ? 1 : 0);
    }

    std::optional<LSM6DSOX::RegVal> LSM6DSOX::readStatus(){
        return readReg(kStatusReg);
    }

    bool LSM6DSOX::accelDataReady(){
        auto s = readStatus();
        return s.has_value() && (*s &XL_DA);
    }

    bool LSM6DSOX::setInt1Ctrl(RegVal bits){
        return writeReg(kInt1Ctrl, bits);
    }

    bool LSM6DSOX::setInt2Ctrl(RegVal bits){
        return writeReg(kInt2Ctrl, bits);
    }

    bool LSM6DSOX::routeInt1(RegVal md1Cfg){
        return writeReg(kMd1Cfg, md1Cfg);
    }

    bool LSM6DSOX::routeInt2(RegVal md2Cfg){
        return writeReg(kMd2Cfg, md2Cfg);
    }

}  // namespace drivers
