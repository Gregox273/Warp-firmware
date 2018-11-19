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

//#include "devINA219.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;

enum {
    kWarpI2C_INA219_CONFIG_REG  = 0x00,
    kWarpI2C_INA219_SHUNT_V_REG = 0x01,
    kWarpI2C_INA219_BUS_V_REG   = 0x02,
    kWarpI2C_INA219_POWER_REG   = 0x03,
    kWarpI2C_INA219_CURRENT_REG = 0x04,
    kWarpI2C_INA219_CALIBRATION_REG = 0x05
};

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
    uint8_t cmdBuf[1]	= {deviceRegister};
    i2c_status_t	returnValue;

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
            //SEGGER_RTT_printf(0, "\rreadSensorRegisterINA219() received bad deviceRegister\n");

            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
            {
                    .address = deviceINA219State.i2cAddress,
                    .baudRate_kbps = gWarpI2cBaudRateKbps
            };


    cmdBuf[0] = deviceRegister;

    returnValue = I2C_DRV_MasterReceiveDataBlocking(
            0 /* I2C peripheral instance */,
            &slave,
            cmdBuf,
            1,
            (uint8_t *)deviceINA219State.i2cBuffer,
            2,
            500 /* timeout in milliseconds */);

    //SEGGER_RTT_printf(0, "\nI2C_DRV_MasterReceiveData returned [%d] (read register)\n", returnValue);

    if (returnValue == kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "\r[0x%02x]	0x%02x 0x%02x\n", cmdBuf[0], deviceINA219State.i2cBuffer[0]);
    }
    else
    {
        //SEGGER_RTT_printf(0, kWarpConstantStringI2cFailure, cmdBuf[0], returnValue);

        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

//WarpStatus
//writeSensorRegisterADXL362(uint8_t command, uint8_t deviceRegister, uint8_t writeValue)
//{
//    /*
//     *	Populate the shift-out register with the read-register command,
//     *	followed by the register to be read, followed by a zero byte.
//     */
//    deviceADXL362State.spiSourceBuffer[0] = command;
//    deviceADXL362State.spiSourceBuffer[1] = deviceRegister;
//    deviceADXL362State.spiSourceBuffer[2] = writeValue;
//
//    deviceADXL362State.spiSinkBuffer[0] = 0x00;
//    deviceADXL362State.spiSinkBuffer[1] = 0x00;
//    deviceADXL362State.spiSinkBuffer[2] = 0x00;
//
//    /*
//     *	First, create a falling edge on chip-select.
//     *
//     */
//    GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);
//    OSA_TimeDelay(50);
//    GPIO_DRV_ClearPinOutput(kWarpPinADXL362_CS);
//
//    /*
//     *	The result of the SPI transaction will be stored in deviceADXL362State.spiSinkBuffer.
//     *
//     *	Providing a device structure here is optional since it
//     *	is already provided when we did SPI_DRV_MasterConfigureBus(),
//     *	so we pass in NULL.
//     *
//     *	TODO: the "master instance" is always 0 for the KL03 since
//     *	there is only one SPI peripheral. We however should remove
//     *	the '0' magic number and place this in a Warp-HWREV0 header
//     *	file.
//     */
//    enableSPIpins();
//    deviceADXL362State.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
//                                                                        NULL /* spi_master_user_config_t */,
//                                                                        (const uint8_t * restrict)deviceADXL362State.spiSourceBuffer,
//                                                                        (uint8_t * restrict)deviceADXL362State.spiSinkBuffer,
//                                                                        3 /* transfer size */,
//                                                                        1000 /* timeout in microseconds (unlike I2C which is ms) */);
//    disableSPIpins();
//
//    /*
//     *	Disengage the ADXL362
//     */
//    GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);
//
//    return kWarpStatusOK;
//}
