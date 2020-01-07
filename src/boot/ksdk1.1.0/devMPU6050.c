#include <stdlib.h>
#include <stdint.h>
#include "devMPU6050.h"
#include "fsl_i2c_master_driver.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState	deviceMPU6050State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void initMPU6050(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);
	return;
}

void configMPU6050(uint16_t menuI2cPullup)
{
    uint8_t config;
    WarpStatus status;
    config = 36 + MPU6050_CLOCK_PLL_XGYRO; //also disables sleep
    status = writeSensorRegisterMPU6050(MPU6050_RA_PWR_MGMT_1, config, menuI2cPullup);
    if(status != kWarpStatusOK)
        SEGGER_RTT_printf(0,"I2C failed with %d \n", status);
    // setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    config = 8*MPU6050_GYRO_FS_250;
    status = writeSensorRegisterMPU6050(MPU6050_RA_GYRO_CONFIG, config, menuI2cPullup);
    if(status != kWarpStatusOK)
        SEGGER_RTT_printf(0,"I2C failed with %d \n", status);
    // setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    config = 8*MPU6050_ACCEL_FS_2;
    status = writeSensorRegisterMPU6050(MPU6050_RA_ACCEL_CONFIG, config, menuI2cPullup);
    if(status != kWarpStatusOK)
        SEGGER_RTT_printf(0,"I2C failed with %d \n", status);
    // setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    // setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
}

WarpStatus writeSensorRegisterMPU6050(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	// switch (deviceRegister)
	// {
	// 	case 0x09: case 0x0a: case 0x0e: case 0x0f:
	// 	case 0x11: case 0x12: case 0x13: case 0x14:
	// 	case 0x15: case 0x17: case 0x18: case 0x1d:
	// 	case 0x1f: case 0x20: case 0x21: case 0x23:
	// 	case 0x24: case 0x25: case 0x26: case 0x27:
	// 	case 0x28: case 0x29: case 0x2a: case 0x2b:
	// 	case 0x2c: case 0x2d: case 0x2e: case 0x2f:
	// 	case 0x30: case 0x31:
	// 	{
	// 		/* OK */
	// 		break;
	// 	}
    //
	// 	default:
	// 	{
	// 		return kWarpStatusBadDeviceCommand;
	// 	}
	// }

	i2c_device_t slave =
	{
		.address = deviceMPU6050State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus readSensorRegisterMPU6050(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	// switch (deviceRegister)
	// {
	// 	case 0x00: case 0x01: case 0x02: case 0x03:
	// 	case 0x04: case 0x05:
	// 	{
	// 		/* OK */
	// 		break;
	// 	}
    //
	// 	default:
	// 	{
	// 		return kWarpStatusBadDeviceCommand;
	// 	}
	// }


	i2c_device_t slave =
	{
		.address = deviceMPU6050State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMPU6050State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void devMPU6050getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
    readSensorRegisterMPU6050(MPU6050_RA_ACCEL_XOUT_H, 6);
    *x = (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
    *y = (((int16_t)deviceMPU6050State.i2cBuffer[2]) << 8) | deviceMPU6050State.i2cBuffer[3];
    *z = (((int16_t)deviceMPU6050State.i2cBuffer[4]) << 8) | deviceMPU6050State.i2cBuffer[5];
}
/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
int16_t devMPU6050getAccelerationX()
{
    WarpStatus status = readSensorRegisterMPU6050(MPU6050_RA_ACCEL_XOUT_H, 2);
    if(status != kWarpStatusOK)
        SEGGER_RTT_printf(0, "I2c failed with: %d \n", status);
    return (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
}
/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_YOUT_H
 */
int16_t devMPU6050getAccelerationY()
{
    readSensorRegisterMPU6050(MPU6050_RA_ACCEL_YOUT_H, 2);
    return (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
}
/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_ZOUT_H
 */
int16_t devMPU6050getAccelerationZ()
{
    readSensorRegisterMPU6050(MPU6050_RA_ACCEL_ZOUT_H, 2);
    return (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
}

void devMPU6050getRotation(int16_t* x, int16_t* y, int16_t* z)
{
    readSensorRegisterMPU6050(MPU6050_RA_GYRO_XOUT_H, 6);
    *x = (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
    *y = (((int16_t)deviceMPU6050State.i2cBuffer[2]) << 8) | deviceMPU6050State.i2cBuffer[3];
    *z = (((int16_t)deviceMPU6050State.i2cBuffer[4]) << 8) | deviceMPU6050State.i2cBuffer[5];
}
/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
int16_t devMPU6050getRotationX()
{
    readSensorRegisterMPU6050(MPU6050_RA_GYRO_XOUT_H, 2);
    return (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
}
/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_YOUT_H
 */
int16_t devMPU6050getRotationY()
{
    readSensorRegisterMPU6050(MPU6050_RA_GYRO_YOUT_H, 2);
    return (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
}
/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_ZOUT_H
 */
int16_t devMPU6050getRotationZ()
{
    readSensorRegisterMPU6050(MPU6050_RA_GYRO_ZOUT_H, 2);
    return (((int16_t)deviceMPU6050State.i2cBuffer[0]) << 8) | deviceMPU6050State.i2cBuffer[1];
}