enum {
    kWarpI2C_INA219_CONFIG_REG	= 0x00,
    kWarpI2C_INA219_SHUNT_V_REG	= 0x01,
    kWarpI2C_INA219_BUS_V_REG   = 0x02,
    kWarpI2C_INA219_POWER_REG   = 0x03,
    kWarpI2C_INA219_CURRENT_REG = 0x04,
    kWarpI2C_INA219_CALIBRATION_REG = 0x05
};


void initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister);
//WarpStatus  writeSensorRegisterINA219(uint8_t command, uint8_t deviceRegister, uint8_t writeValue);