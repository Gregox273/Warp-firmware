void initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer);
WarpStatus readSensorRegisterINA219(uint8_t deviceRegister);
//WarpStatus  writeSensorRegisterINA219(uint8_t command, uint8_t deviceRegister, uint8_t writeValue);
