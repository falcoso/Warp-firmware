/*
 * Author O.G. Jones 2019
 * PID controller, deals with collecting and processing of readings from the
 * MPU6050 and MMA8451Q to generate a controller output.
 * LPTMR is used to get interval times.
 */

#include <stdint.h>
#include "math.h"

typedef struct Controller
{
    float Kp;     //PID controller parameters
    float Ki;     //PID controller parameters
    float Kd;     //PID controller parameters
    float target; //reference value
    float real;   // actual value
} controller_t;

// Set PID parameters here as well as the reference angle where the robot balances
volatile controller_t pidSettings = {
    .Kp = 20,
    .Ki = 30,
    .Kd = 0.3,
    .target = -3,
    .real = 0.0,
};

/*
 * Calculates the PID controller output based on the current readings in
 * <settings>. Output value is number from -100 to 100 representing the
 * percentage duty cycle of the PWM used.
 */
int8_t pid_op(controller_t * settings);

/*
 * Collects readings from MMA8451Q and calculates the angle of inclination of
 * the robot. Result is stored in settings.real.
 */
void collect_readings(controller_t * settings);

/*
 * Calculates the angle of the robot by combing X & Z axis accelerometer
 * readings (<accx>, <accz> respectively) and then using a complimentary filter
 * with the Y axis rotation speed <gyroy>. Result is stored in settings.real
 */
void calculate_angle(int16_t accx, int16_t accz, int16_t gyroy, controller_t *pid_settings);