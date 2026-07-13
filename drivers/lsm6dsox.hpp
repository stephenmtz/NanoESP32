#pragma once

#include <cstdint>
#include <optional>

#include "i2c.hpp"

namespace drivers {

class LSM6DSOX {
    public:
        using RegVal = uint8_t;
        using RegAddr = uint8_t;
        using RegBitPos = uint8_t;
        using RegMask = uint8_t;

        enum I2CAddr : uint8_t {
            kDeviceAddLow = 0x6A,
        };

        LSM6DSOX(hal::I2C::SharedPtr i2c, I2CAddr i2c_address);
        LSM6DSOX(const LSM6DSOX&) = delete;
        LSM6DSOX& operator=(const LSM6DSOX&) = delete;

        bool init();
        bool reset();
        bool whoAmI(uint8_t &id);
        float lsm6soxzFromFs2ToMg(int16_t lsb);
        float lsm6soxzFromFs4ToMg(int16_t lsb);
        float lsm6soxzFromFs8ToMg(int16_t lsb);
        float lsm6soxzFromFs16ToMg(int16_t lsb);
        float lsm6soxzFromFs125ToMdps(int16_t lsb);
        float lsm6soxzFromFs250ToMdps(int16_t lsb);
        float lsm6soxzFromFs500ToMdps(int16_t lsb); 
        float lsm6soxzFromFs1000ToMdps(int16_t lsb);
        float lsm6soxzFromFs2000ToMdps(int16_t lsb);
        float lsm6soxzFromLsbToCelsius(int16_t lsb);
        float lsm6soxzFromLsbToNsec(int16_t lsb);

        enum AccelOdr : RegVal {
            kXlOdrOff = 0x00,
            kXlOdr12_5Hz = 0x01,
            kXlOdr26Hz = 0x02,
            kXlOdr52Hz = 0x03,
            kXlOdr104Hz = 0x04,
            kXlOdr208Hz = 0x05,
            kXlOdr416Hz = 0x06,
            kXlOdr833Hz = 0x07,
            kXlOdr1666Hz = 0x08,
            kXlOdr3332Hz = 0x09,
            kXlOdr6664Hz = 0x0A,
            kXlOdr1_6Hz = 0x0B,
        };

        enum AccelFullScale : RegVal {
            kXlFs2g = 0x00,
            kXlFs16g = 0x01,
            kXlFs4g = 0x02,
            kXlFs8g = 0x03,
        };

        enum AccelLpfBw : RegVal {
            kXlLpfDiv2 = 0x00,
            kXlLpfDiv4 = 0x01,
            kXlLpfDiv10 = 0x02,
            kXlLpfDiv20 = 0x03,
            kXlLpfDiv45 = 0x04,
            kXlLpfDiv100 = 0x05,
            kXlLpfDiv200 = 0x06,
            kXlLpfDiv400 = 0x07,
        };

        bool setAccelOdr(AccelOdr odr);
         bool readAccelBurst(int16_t& x, int16_t& y, int16_t& z); 
        bool setAccelFullScale(AccelFullScale fs);
        bool setAccelLpf(AccelLpfBw bw);
        bool readAccel(int16_t& x, int16_t& y, int16_t& z);
        std::optional<int16_t> readAccelX();
        std::optional<int16_t> readAccelY();
        std::optional<int16_t> readAccelZ();

        enum GyroOdr : RegVal {
            kGOdrOff = 0x00,
            kGOdr12_5Hz = 0x01,
            kGOdr26Hz = 0x02,
            kGOdr52Hz = 0x03,
            kGOdr104Hz = 0x04,
            kGOdr208Hz = 0x05,
            kGOdr416Hz = 0x06,
            kGOdr833Hz = 0x07,
            kGOdr1666Hz = 0x08,
            kGOdr3332Hz = 0x09,
            kGOdr6664Hz = 0x0A,
        };

        enum GyroFullScale : RegVal {
            kGFs250dps = 0x00,
            kGFs500dps = 0x01,
            kGFs1000dps = 0x02,
            kGFs2000dps = 0x03,
        };

        bool setGyroOdr(GyroOdr odr);
        bool setGyroFullScale(GyroFullScale fs);
        bool readGyro(int16_t& x, int16_t& y, int16_t& z);
        std::optional<int16_t> readGyroX();
        std::optional<int16_t> readGyroY();
        std::optional<int16_t> readGyroZ();

        std::optional<int16_t> readTempRaw();
        std::optional<float> readTempC();

        enum StatusBits : RegVal {
            XL_DA = 1 << 0,
            GDA = 1 << 1,
            TDA = 1 << 2,
        };

        std::optional<RegVal> readStatus();
        bool accelDataReady();
        bool gyroDataReady();
        bool tempDataReady();

        bool setBdu(bool enable);

        enum Int1CtrlBits : RegVal {
            INT1_DRDY_XL = 1 << 0,
            INT1_DRDY_G = 1 << 1,
            INT1_BOOT = 1 << 2,
            INT1_FIFO_TH = 1 << 3,
            INT1_FIFO_OVR = 1 << 4,
            INT1_FIFO_FULL = 1 << 5,
            INT1_CNT_BDR = 1 << 6,
            INT1_DEN_DRDY = 1 << 7,
        };

        enum Int2CtrlBits : RegVal {
            INT2_DRDY_XL = 1 << 0,
            INT2_DRDY_G = 1 << 1,
            INT2_DRDY_TEMP = 1 << 2,
            INT2_FIFO_TH = 1 << 3,
            INT2_FIFO_OVR = 1 << 4,
            INT2_FIFO_FULL = 1 << 5,
            INT2_CNT_BDR = 1 << 6,
        };

        bool setInt1Ctrl(RegVal bits);
        bool setInt2Ctrl(RegVal bits);

        bool enableEmbeddedFuncAccess();
        bool disableEmbeddedFuncAccess();

        std::optional<uint32_t> readTimeStamp();
        bool enableTimestamp(bool enable);

        enum FsmOutBits : RegVal {
            NV = 1 << 0,
            PV = 1 << 1,
            NZ = 1 << 2,
            PZ = 1 << 3,
            NY = 1 << 4,
            PY = 1 << 5,
            NX = 1 << 6,
            PX = 1 << 7,
        };

        bool enableFsm(uint16_t fsmMask);
        std::optional<RegVal> readFsmStatusA();
        std::optional<RegVal> readFsmStatusB();

        enum MlcStatusBits : RegVal {
            IS_MLC1 = 1 << 0,
            IS_MLC2 = 1 << 1,
            IS_MLC3 = 1 << 2,
            IS_MLC4 = 1 << 3,
            IS_MLC5 = 1 << 4,
            IS_MLC6 = 1 << 5,
            IS_MLC7 = 1 << 6,
            IS_MLC8 = 1 << 7,
        };

        std::optional<RegVal> readMlcStatus();

        enum StatusMasterBits : RegVal {
            SENS_HUB_ENDOP = 1 << 0,
            SLAVE0_NACK = 1 << 3,
            SLAVE1_NACK = 1 << 4,
            SLAVE2_NACK = 1 << 5,
            SLAVE3_NACK = 1 << 6,
            WR_ONCE_DONE = 1 << 7,
        };

        std::optional<RegVal> readMasterStatus();
        bool enableSensorHub(bool enable);
        bool configureSlave(uint8_t slaveIdx, uint8_t addr, uint8_t reg, uint8_t len);

        enum FifoMode : RegVal {
            kFifoBypass = 0x00,
            kFifoFifo = 0x01,
            kFifoContinuous = 0x06,
            kFifoBypassCont = 0x04,
            kFifoContinuousToFifo = 0x03,
        };

        enum FifoBdr : RegVal {
            kFifoBdrNotStored = 0x00,
            kFifoBdr12_5Hz = 0x01,
            kFifoBdr26Hz = 0x02,
            kFifoBdr52Hz = 0x03,
            kFifoBdr104Hz = 0x04,
            kFifoBdr208Hz = 0x05,
            kFifoBdr416Hz = 0x06,
            kFifoBdr833Hz = 0x07,
            kFifoBdr1666Hz = 0x08,
            kFifoBdr3332Hz = 0x09,
            kFifoBdr6664Hz = 0x0A,
        };

        enum FifoTag : RegVal {
            kTagGyroNc = 0x01,
            kTagXlNc = 0x02,
            kTagTemp = 0x03,
            kTagTimestamp = 0x04,
            kTagCfgChange = 0x05,
            kTagXlNcT2 = 0x06,
            kTagXlNcT1 = 0x07,
            kTagGyroNcT2 = 0x08,
            kTagGyroNcT1 = 0x09,
            kTagSensorHub0 = 0x0E,
            kTagSensorHub1 = 0x0F,
            kTagSensorHub2 = 0x10,
            kTagSensorHub3 = 0x11,
            kTagStepCounter = 0x12,
            kTagMlc = 0x1A,
        };

        bool setFifoModeGyro(FifoMode mode);
        bool setFifoModeAccel(FifoMode mode);
        bool setFifoWatermark(uint16_t wtm);
        bool setFifoBatchXl(FifoBdr bdr);
        bool setFifoBatchGyro(FifoBdr bdr);
        std::optional<uint16_t> readFifoLevel();
        bool isFifoFull();
        bool isFifoOverrun();
        bool readFifoWord(FifoTag& tag, uint8_t data[6]);

        enum XlPowerMode : RegVal {
            kXlHighPerf = 0x00,
            kXlLowPower = 0x01,
            kXlNormal = 0x02,
        };

        bool setAccelPowerMode(XlPowerMode mode);
        bool setGyroHighPerf(bool enable);
        bool sleepGyro(bool enable);

        enum SelfTestMode : RegVal {
            kStDisabled = 0x00,
            kStPositive = 0x01,
            kStNegative = 0x02,
        };

        bool setAccelSelfTest(SelfTestMode mode);
        bool setGyroSelfTest(SelfTestMode mode);

        bool configureTap(bool enableX, bool enableY, bool enableZ, RegVal threshold);
        bool setTapDuration(RegVal quiet, RegVal shock, RegVal dur);
        bool enableDoubleTap(bool enable);
        std::optional<RegVal> readTapSrc();

        bool setWakeUpThreshold(RegVal threshold);
        bool setWakeUpDuration(RegVal duration);
        std::optional<RegVal> readWakeUpSrc();

        enum FreeFallThs : RegVal {
            kFfThs156mg = 0x00,
            kFfThs219mg = 0x01,
            kFfThs250mg = 0x02,
            kFfThs312mg = 0x03,
            kFfThs344mg = 0x04,
            kFfThs406mg = 0x05,
            kFfThs469mg = 0x06,
            kFfThs500mg = 0x07,
        };

        bool setFreeFallThreshold(FreeFallThs threshold);
        bool setFreeFallDuration(RegVal duration);

        bool set6dThreshold(RegVal threshold);
        std::optional<RegVal> read6dSrc();

        bool routeInt1(RegVal md1Cfg);
        bool routeInt2(RegVal md2Cfg);

        bool setAccelUserOffset(int8_t x, int8_t y, int8_t z);
        bool enableAccelUserOffset(bool enable);
        bool enableAccelLowPowerMode(bool enable);

    private:
        hal::I2C::SharedPtr i2c_;
        const I2CAddr i2c_addr_;
        static constexpr RegMask kSleepGMask = 0x40;   
        static constexpr RegMask kXlUlpMask = 0x80;   
        static constexpr RegAddr kFuncCfgAccess = 0x01;
        static constexpr RegAddr kPinCtrl = 0x02;
        static constexpr RegAddr kFifoCtrl1 = 0x07;
        static constexpr RegAddr kFifoCtrl2 = 0x08;
        static constexpr RegAddr kFifoCtrl3 = 0x09;
        static constexpr RegAddr kFifoCtrl4 = 0x0A;
        static constexpr RegAddr kCounterBdrReg1 = 0x0B;
        static constexpr RegAddr kCounterBdrReg2 = 0x0C;
        static constexpr RegAddr kInt1Ctrl = 0x0D;
        static constexpr RegAddr kInt2Ctrl = 0x0E;
        static constexpr RegAddr kWhoAmI = 0x0F;
        static constexpr RegAddr kCtrl1Xl = 0x10;
        static constexpr RegAddr kCtrl2G = 0x11;
        static constexpr RegAddr kCtrl3C = 0x12;
        static constexpr RegAddr kCtrl4C = 0x13;
        static constexpr RegAddr kCtrl5C = 0x14;
        static constexpr RegAddr kCtrl6C = 0x15;
        static constexpr RegAddr kCtrl7G = 0x16;
        static constexpr RegAddr kCtrl8Xl = 0x17;
        static constexpr RegAddr kCtrl9Xl = 0x18;
        static constexpr RegAddr kCtrl10C = 0x19;
        static constexpr RegAddr kAllIntSrc = 0x1A;
        static constexpr RegAddr kWakeUpSrc = 0x1B;
        static constexpr RegAddr kTapSrc = 0x1C;
        static constexpr RegAddr kD6dSrc = 0x1D;
        static constexpr RegAddr kStatusReg = 0x1E;
        static constexpr RegAddr kOutTempL = 0x20;
        static constexpr RegAddr kOutTempH = 0x21;
        static constexpr RegAddr kOutXLG = 0x22;
        static constexpr RegAddr kOutXHG = 0x23;
        static constexpr RegAddr kOutYLG = 0x24;
        static constexpr RegAddr kOutYHG = 0x25;
        static constexpr RegAddr kOutZLG = 0x26;
        static constexpr RegAddr kOutZHG = 0x27;
        static constexpr RegAddr kOutXLXl = 0x28;
        static constexpr RegAddr kOutXHXl = 0x29;
        static constexpr RegAddr kOutYLXl = 0x2A;
        static constexpr RegAddr kOutYHXl = 0x2B;
        static constexpr RegAddr kOutZLXl = 0x2C;
        static constexpr RegAddr kOutZHXl = 0x2D;
        static constexpr RegAddr kEmbFuncStatusMainpage = 0x35;
        static constexpr RegAddr kFsmStatusAMainpage = 0x36;
        static constexpr RegAddr kFsmStatusBMainpage = 0x37;
        static constexpr RegAddr kMlcStatusMainpage = 0x38;
        static constexpr RegAddr kStatusMasterMainpage = 0x39;
        static constexpr RegAddr kFifoStatus1 = 0x3A;
        static constexpr RegAddr kFifoStatus2 = 0x3B;
        static constexpr RegAddr kTimestamp0 = 0x40;
        static constexpr RegAddr kTimestamp1 = 0x41;
        static constexpr RegAddr kTimestamp2 = 0x42;
        static constexpr RegAddr kTimestamp3 = 0x43;
        static constexpr RegAddr kTapCfg0 = 0x56;
        static constexpr RegAddr kTapCfg1 = 0x57;
        static constexpr RegAddr kTapCfg2 = 0x58;
        static constexpr RegAddr kTapThs6d = 0x59;
        static constexpr RegAddr kIntDur2 = 0x5A;
        static constexpr RegAddr kWakeUpThs = 0x5B;
        static constexpr RegAddr kWakeUpDur = 0x5C;
        static constexpr RegAddr kFreeFall = 0x5D;
        static constexpr RegAddr kMd1Cfg = 0x5E;
        static constexpr RegAddr kMd2Cfg = 0x5F;
        static constexpr RegAddr kI3cBusAvb = 0x62;
        static constexpr RegAddr kInternalFreqFine = 0x63;
        static constexpr RegAddr kXOfsUsr = 0x73;
        static constexpr RegAddr kYOfsUsr = 0x74;
        static constexpr RegAddr kZOfsUsr = 0x75;
        static constexpr RegAddr kFifoDataOutTag = 0x78;
        static constexpr RegAddr kFifoDataOutXL = 0x79;
        static constexpr RegAddr kFifoDataOutXH = 0x7A;
        static constexpr RegAddr kFifoDataOutYL = 0x7B;
        static constexpr RegAddr kFifoDataOutYH = 0x7C;
        static constexpr RegAddr kFifoDataOutZL = 0x7D;
        static constexpr RegAddr kFifoDataOutZH = 0x7E;

        static constexpr RegAddr kPageSel = 0x02;
        static constexpr RegAddr kEmbFuncEnA = 0x04;
        static constexpr RegAddr kEmbFuncEnB = 0x05;
        static constexpr RegAddr kPageAddr = 0x08;
        static constexpr RegAddr kPageValue = 0x09;
        static constexpr RegAddr kEmbFuncInt1 = 0x0A;
        static constexpr RegAddr kFsmInt1A = 0x0B;
        static constexpr RegAddr kFsmInt1B = 0x0C;
        static constexpr RegAddr kMlcInt1 = 0x0D;
        static constexpr RegAddr kEmbFuncInt2 = 0x0E;
        static constexpr RegAddr kFsmInt2A = 0x0F;
        static constexpr RegAddr kFsmInt2B = 0x10;
        static constexpr RegAddr kMlcInt2 = 0x11;
        static constexpr RegAddr kEmbFuncStatus = 0x12;
        static constexpr RegAddr kFsmStatusA = 0x13;
        static constexpr RegAddr kFsmStatusB = 0x14;
        static constexpr RegAddr kMlcStatus = 0x15;
        static constexpr RegAddr kPageRw = 0x17;
        static constexpr RegAddr kEmbFuncFifoCfg = 0x44;
        static constexpr RegAddr kFsmEnableA = 0x46;
        static constexpr RegAddr kFsmEnableB = 0x47;
        static constexpr RegAddr kFsmLongerCounterL = 0x48;
        static constexpr RegAddr kFsmLongerCounterH = 0x49;
        static constexpr RegAddr kFsmLongerCounterClear = 0x4A;
        static constexpr RegAddr kFsmOut1 = 0x4C;
        static constexpr RegAddr kEmbFuncOdrCfgB = 0x5F;
        static constexpr RegAddr kEmbFuncOdrCfgC = 0x60;
        static constexpr RegAddr kStepCounterL = 0x62;
        static constexpr RegAddr kStepCounterH = 0x63;
        static constexpr RegAddr kEmbFuncSrc = 0x64;
        static constexpr RegAddr kEmbFuncInitA = 0x66;
        static constexpr RegAddr kEmbFuncInitB = 0x67;

        static constexpr RegMask kOdrMask = 0xF0;
        static constexpr RegMask kFsMask = 0x0C;
        static constexpr RegMask kBduMask = 0x40;
        static constexpr RegMask kSwResetMask = 0x01;
        static constexpr RegMask kIfIncMask = 0x04;

        static constexpr RegAddr kFsmOuts1 = 0x4C;
        static constexpr RegAddr kFsmOuts2 = 0x4D;
        static constexpr RegAddr kFsmOuts3 = 0x4E;
        static constexpr RegAddr kFsmOuts4 = 0x4F;
        static constexpr RegAddr kFsmOuts5 = 0x50;
        static constexpr RegAddr kFsmOuts6 = 0x51;
        static constexpr RegAddr kFsmOuts7 = 0x52;
        static constexpr RegAddr kFsmOuts8 = 0x53;
        static constexpr RegAddr kFsmOuts9 = 0x54;
        static constexpr RegAddr kFsmOuts10 = 0x55;
        static constexpr RegAddr kFsmOuts11 = 0x56;
        static constexpr RegAddr kFsmOuts12 = 0x57;
        static constexpr RegAddr kFsmOuts13 = 0x58;
        static constexpr RegAddr kFsmOuts14 = 0x59;
        static constexpr RegAddr kFsmOuts15 = 0x5A;
        static constexpr RegAddr kFsmOuts16 = 0x5B;

        static constexpr RegAddr kMlc0Src = 0x70;
        static constexpr RegAddr kMlc1Src = 0x71;
        static constexpr RegAddr kMlc2Src = 0x72;
        static constexpr RegAddr kMlc3Src = 0x73;
        static constexpr RegAddr kMlc4Src = 0x74;
        static constexpr RegAddr kMlc5Src = 0x75;
        static constexpr RegAddr kMlc6Src = 0x76;
        static constexpr RegAddr kMlc7Src = 0x77;

        static constexpr RegVal kWhoAmIExpected = 0x6C;

        std::optional<RegVal> readReg(RegAddr reg_addr);
        std::optional<int16_t> readAxis(RegAddr low_reg);

        bool writeReg(RegAddr reg_addr, RegVal reg_val);
        bool writeBitField(RegAddr addr, RegMask mask, RegBitPos bit_pos, RegVal val);

        template <typename T> 
        std::optional<T> readBitField(RegAddr addr, RegMask mask, RegBitPos bit_pos); 

        template <typename T> 
        std::optional<T> readMultReg(RegAddr start_reg_addr, T bitMask); 

    };

}
