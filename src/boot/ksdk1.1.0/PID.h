#include <stdio.h>
#include "warp.h"
#include "fsl_lptmr_driver.h"

typedef struct Controller
{
    float Kp;
    float Ki;
    float Kd;
    float target;
    float real;
} controller_t;

int8_t pid_op(controller_t * settings);