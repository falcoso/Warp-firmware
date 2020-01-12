#include <stdint.h>
#include "math.h"

#define PRINT_F

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
    .Kp = 20,
    .Ki = 30,//5,
    .Kd = 0.3,
    .target = -3,
    .real = 0.0,
    .accel = {0,0}
};

int8_t pid_op(controller_t * settings);

void collect_readings(controller_t * settings);

void calculate_angle(int16_t accx, int16_t accz, int16_t gyroy, controller_t *pid_settings);