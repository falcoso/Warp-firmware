#include "PID.h"
#include "warp.h"

#include <math.h>
#include "fsl_lptmr_driver.h"
#include "fsl_tpm_driver.h"
#include "devMMA8451Q.h"
#include "devMPU6050.h"
#include "SEGGER_RTT.h"

extern volatile WarpI2CDeviceState	deviceMMA8451QState;

int8_t pid_op(controller_t * settings)
{
    uint32_t     dt = LPTMR_DRV_GetCurrentTimeUs(0);

    static float error_int = 0.0;
    static float old_error = 0;
    float        error, output;

    // calculate the error
    error = settings->target - settings->real;
    error_int += (error+old_error)*dt*1E-6/2;

    output = settings->Kp*error + settings->Ki*error_int + settings->Kd*(error-old_error)/(dt*1E-6);

    // cap the output since the PWM can only take 0-100%
    if (output > 100)
        output = 100;
    if (output < -100)
        output = -100;

    old_error = error;
    // SEGGER_RTT_printf(0, "ERROR %d\n", (int)output);
    return (int8_t)output;
}


void collect_readings(controller_t * settings)
{
    uint16_t	readSensorRegisterValueLSB;
    uint16_t	readSensorRegisterValueMSB;
    int16_t     accel[2];
    WarpStatus	i2cReadStatus;
    WarpSensorOutputRegister MSBaddress[] = {kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB};

    // see MMA8451q.c printSensorDataMMA8451Q
    for(int8_t i=0; i<2; i++)
    {
        i2cReadStatus = readSensorRegisterMMA8451Q(MSBaddress[i], 2 /* numberOfBytes */);
        if (i2cReadStatus != kWarpStatusOK)
        {
            SEGGER_RTT_printf(0, "Unsuccessful read from MMA8451Q Ox%X\n", MSBaddress[i]);
        }

        readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
        accel[i] = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

        // Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
        accel[i] = (accel[i] ^ (1 << 13)) - (1 << 13);
    }
    settings->real = (float)atan2((float)accel[1],(float)accel[0]);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculate_angle(int16_t accx, int16_t accz, int16_t gyroy, controller_t *pid_settings)
{
    uint32_t sampleTime = LPTMR_DRV_GetCurrentTimeUs(0);
    int16_t gyroRate;
    float gyroAngle, accAngle;
#ifdef PRINT_F
    static float rotAngle = 0;
    static bool init = true;
    if(init == true)
    {
        init = false;
        rotAngle = pid_settings->real;
    }
#endif

    accAngle = atan2(accz, accx)*180.0/M_PI;
    gyroRate = map(gyroy, -32768, 32767, -250, 250);
    gyroAngle = (float)gyroRate*3*sampleTime*1E-6;
    pid_settings->real = 0.99*(pid_settings->real + gyroAngle) + 0.01*(accAngle);
#ifdef PRINT_F
    rotAngle += gyroAngle;
    SEGGER_RTT_printf(0, "%d,", (int)(accAngle));
    SEGGER_RTT_printf(0, "%d,", (int)(rotAngle));
    SEGGER_RTT_printf(0, "%d\n", (int)(pid_settings->real));
#endif
}
