#include "PID.h"

int8_t pid_op(controller_t * settings)
{
    static uint32_t old_time = 0;
    uint32_t new_time = LPTMR_DRV_GetCurrentTimeUs(0);
    uint32_t dt; //units =1/clkfrq

    static float error_int = 0.0;
    static float old_error = 0;
    float error;
    float output = 0.0;

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
