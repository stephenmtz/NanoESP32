#include "lsm6dsox.hpp"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

namespace drivers {

    static const char* TAG = "LSM6DSOX";

    LSM6DSOX::LSM6DSOX(hal::I2C::SharedPtr i2c, I2CAddr addr)
        : i2c_(std::move(i2c)), i2c_addr_(addr){
            configASSERT(i2c_ != nullptr);
        }

    std::optional<LSM6DSOX::RegVal> LSM6DSOX::readReg(RegAddr reg_addr){
        RegVal val;
        if(!i2c_->read(i2c_addr_, reg_addr, val)){
            return std::nullopt;
        }
        return val;
    }

    std::optional<LSM6DSOX::RegVal> LSM6DSOX::writeBitField(RegAddr addr, RegMask mask, RegBitPos bit_pos, RegVal val){
        
    }

    bool LSM6DSOX::writeReg(RegAddr reg_addr, RegVal reg_val){
        return i2c_->write(i2c_addr_, reg_addr, reg_val);
    }

    bool LSM6DSOX::whoAmI(uint8_t &id){
        auto val = readReg(kWhoAmI);
        if(!val){
            return false;
        }
        id = *val;
        return true;
    }

    bool LSM6DSOX::init(){
        uint8_t id;
        if(!whoAmI(id) || id != kWhoAmIExpected){
            ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", id, kWhoAmIExpected);
            return false;
        }
        ESP_LOGI(TAG, "WHO_AM_I matches");
        return true;
    }

    bool LSM6DSOX::setAccelOdr(AccelOdr odr) {
        return writeBitField(kCtrl1Xl, kOdrMask, 4, odr);
    }

    bool LSM6DSOX::setGyroOdr(GyroOdr odr){ 
        return writeBitField(KCtrl2G, kOdrMask, 4, odr)
    }

    float LSM6DSOX::lsm6soxzFromFs2ToMg(uint16_t lsb){ 
        return static_cast<float>(lsb) * 0.061f; 
    }

    float LSM6DSOX::lsm6soxzFromFs4ToMg(int16_t lsb){
        return static_cast<float>(lsb) * 0.122f; 
    }
    
    float LSM6DSOX::lsm6soxzFromFs8ToMg(uint16_t lsb){ 
        return static_cast<float>(lsb) * 0.244f; 
    }

    float LSM6DSOX::lsm6soxzFromFs16ToMg(uint16_t lsb){ 
        return static_cast<float>(lsb) * 0.488f; 
    } 

    float LSM6DSOX::lsm6soxzFromFs125ToMdps(uint16_t lsb){ 
        return static_cast<float>(lsb) * 4.375f; 
    }

    float 
}