#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "devINA219.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#define					kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;

void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
    deviceStatePointer->i2cAddress	= i2cAddress;
    deviceStatePointer->signalType	= (
            kWarpTypeMaskVoltage
    );
    return;
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister)
{
    uint8_t cmdBuf[1] = {deviceRegister};
    i2c_status_t returnValue;

    switch (deviceRegister)
    {
        case kWarpI2C_INA219_CONFIG_REG:
        case kWarpI2C_INA219_SHUNT_V_REG:
        case kWarpI2C_INA219_BUS_V_REG:
        case kWarpI2C_INA219_POWER_REG:
        case kWarpI2C_INA219_CURRENT_REG:
        case kWarpI2C_INA219_CALIBRATION_REG:
        {
            /* OK */
            break;
        }

        default:
        {
            SEGGER_RTT_printf(0, "\rreadSensorRegisterINA219() received bad deviceRegister\n");

            return kWarpStatusBadDeviceCommand;
        }
    }

    enableI2Cpins(65535 /* pullupValue*/);
    i2c_device_t slave =
            {
                    .address = deviceINA219State.i2cAddress,
                    .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    returnValue = I2C_DRV_MasterReceiveDataBlocking(
            0 /* I2C peripheral instance */,
            &slave,
            cmdBuf,
            sizeof(cmdBuf),
            (uint8_t *)deviceINA219State.i2cBuffer,
            sizeof(deviceINA219State.i2cBuffer),
            500 /* timeout in milliseconds */);

    disableI2Cpins();

//    SEGGER_RTT_printf(0, "\nI2C_DRV_MasterReceiveData returned [%d] (read register)\n", returnValue);

    if (returnValue == kStatus_I2C_Success)
    {
//        SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x 0x%02x\n", cmdBuf[0], deviceINA219State.i2cBuffer[0], deviceINA219State.i2cBuffer[1]);
    }
    else
    {
        SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, const uint8_t *data) {
    uint8_t cmdBuf[3]	= {deviceRegister, data[0], data[1]};
    i2c_status_t returnValue;

    switch (deviceRegister)
    {
        case kWarpI2C_INA219_CONFIG_REG:
        case kWarpI2C_INA219_CALIBRATION_REG:
        {
            /* OK */
            break;
        }

        default:
        {
            SEGGER_RTT_printf(0, "\rwriteSensorRegisterINA219() received bad deviceRegister\n");

            return kWarpStatusBadDeviceCommand;
        }
    }

    enableI2Cpins(65535 /* pullupValue*/);
    i2c_device_t slave =
            {
                    .address = deviceINA219State.i2cAddress,
                    .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    returnValue = I2C_DRV_MasterSendDataBlocking(
            0 /* I2C peripheral instance */,
            &slave,
            cmdBuf,
            sizeof(cmdBuf),
            NULL,
            0,
            500 /* timeout in milliseconds */);

    disableI2Cpins();

    SEGGER_RTT_printf(0, "\nI2C_DRV_MasterSendData returned [%d] (write register)\n", returnValue);

    if (returnValue == kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x 0x%02x\n", cmdBuf[0], data[0], data[1]);
    }
    else
    {
        SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

int16_t concat_2x8bit(uint8_t b1, uint8_t b2)
{
	return b1 << 8 | b2;
}

void split_2x8bit(uint16_t input, uint8_t *output)
{
	output[0] = input >> 8 & 0xFF;
	output[1] = input & 0xFF;
}
