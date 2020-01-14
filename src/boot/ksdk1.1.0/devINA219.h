/*
 * Author O. G Jones 2019
 * Driver for KL03z to use INA219 current monitor within the Warp Firmware.
 */
volatile WarpI2CDeviceState			deviceINA219State;

#ifndef WARP_BUILD_ENABLE_DEVINA219
#define WARP_BUILD_ENABLE_DEVINA219
#endif

/*
 * Initialises the INA219 WarpI2CDeviceState within the warp firmware
 * Based on MMA8451Q driver
 */
void		initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

/*
 * Reads <numberOfBytes> bytes from register <deviceRegister> on the INA219
 * over I2C.
 * Based on MMA8451Q driver
 */
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);

/*
 * Writes 2byte <payload> to register <deviceRegister> on the IN219 over I2C -
 * note the config registers of the INA219 are 2bytes in size.
 * Based on MMA8451Q driver
 */
WarpStatus  writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload, uint16_t menuI2cPullupValue);

/*
 * Writes to the <payloadF_SETUP> and <payloadCTRL_REG1> to the Status and
 * Control registers of the INA219 over I2C.
 */
WarpStatus	configureSensorINA219(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1, uint16_t menuI2cPullupValue);

/*
 * Prints all sensor registers on tha INA219
 * Based on MMA8451Q driver
 */
void		printSensorDataINA219(bool hexModeFlag);

/* Gets the current from the INA219, returns current value in int_16_t*0.01mA */
int16_t 	getCurrentINA219();
