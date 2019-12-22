#include "PID.h"
#include "warp.h"

#include <math.h>
#include "fsl_lptmr_driver.h"
#include "devMMA8451Q.h"
#include "SEGGER_RTT.h"

extern volatile WarpI2CDeviceState	deviceMMA8451QState;

int8_t pid_op(controller_t * settings)
{
    static uint32_t old_time = 0;
    uint32_t     new_time = LPTMR_DRV_GetCurrentTimeUs(0);
    uint32_t     dt; //units =1/clkfrq

    static float error_int = 0.0;
    static float old_error = 0;
    float        error;
    float        output    = 0.0;

    // get the time change since the last loop
    if (old_time == new_time) { //first initialisation
        dt = 1;
    }
    else if (old_time > new_time) { //timer has overflowed
        dt = new_time + 0x2F00000 - old_time;
    }
    else {
        dt = new_time-old_time;
    }

    // calculate the error
    error = settings->target - settings->real;
    error_int += (error+old_error)*dt/2;

    output += settings->Kp*error;
    output += settings->Ki*error_int;

    if(old_time != 0 || old_error != 0){
        output += settings->Kd*(error-old_error)/dt;
    }

    // cap the output since the PWM can only take 0-100%
    if (output > 100){
        output = 100;
    }
    if (output < -100){
        output = -100;
    }

    old_time = new_time;
    old_error = error;

    return (int8_t)output;
}


void collect_readings(controller_t * settings)
{
    uint16_t	readSensorRegisterValueLSB;
    uint16_t	readSensorRegisterValueMSB;
    WarpStatus	i2cReadStatus;
    WarpSensorOutputRegister MSBaddress[] = {kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB};

    // see MMA8451q.c printSensorDataMMA8451Q
    for(int8_t i=0; i<2; i++)
    {
        // SEGGER_RTT_WriteString(0,"Hanging on read\n");
        i2cReadStatus = readSensorRegisterMMA8451Q(MSBaddress[i], 2 /* numberOfBytes */);
        // SEGGER_RTT_WriteString(0,"Read Complete\n");
        if (i2cReadStatus != kWarpStatusOK)
        {
            SEGGER_RTT_printf(0, "Unsuccessful read from MMA8451Q OX%X\n", MSBaddress[i]);
        }

        readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
        settings->accel[i] = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

        // Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
        settings->accel[i] = (settings->accel[i] ^ (1 << 13)) - (1 << 13);
    }
    // SEGGER_RTT_WriteString(0,"calculating angle\n");
    settings->real = (float)atan((float)settings->accel[1]/(float)settings->accel[0]);
    return;
}
