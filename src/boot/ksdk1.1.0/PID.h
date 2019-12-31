#include <stdint.h>
#include "math.h"

typedef struct Controller
{
    float Kp;
    float Ki;
    float Kd;
    float target;
    float real;
    int16_t accel[2];
} controller_t;

volatile controller_t pidSettings = {
    .Kp = 1000,
    .Ki = 700,
    .Kd = 1,
    .target = -(5.0/180)*M_PI,
    .real = 0.0,
    .accel = {0,0}
};

int8_t pid_op(controller_t * settings);

void collect_readings(controller_t * settings);