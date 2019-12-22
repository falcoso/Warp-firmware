#include <stdint.h>

typedef struct Controller
{
    float Kp;
    float Ki;
    float Kd;
    float target;
    float real;
    int16_t accel[2];
} controller_t;

controller_t pidSettings = {
    .Kp = 10,
    .Ki = 0,
    .Kd = 0,
    .target = 0.0,
    .real = 0.0,
    .accel = {0,0}
};

int8_t pid_op(controller_t * settings);

void collect_readings(controller_t * settings);