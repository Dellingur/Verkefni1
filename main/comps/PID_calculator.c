#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#define INTEGRAL_MAX 1000

//Calculates motor RPM
int calc_motor_RPM (uint32_t degree, uint32_t* prev_degree, int dt) {
    uint32_t RPM = (degree - *prev_degree) * (1000/dt) *(60/100);
    *prev_degree = degree;
    
    return RPM;
}

// Calculates PWM values by comparing target RPM and current RPM using PID
float PID_calculator (int target_RPM, int RPM, float* prev_error, float* integral, float Kp, float Ki, float Kd, int dt) {
    float error = target_RPM - RPM;
    *integral += error * dt;
    *integral = fmin(fmax(*integral, -INTEGRAL_MAX), INTEGRAL_MAX);
    float derivative = ( error - *prev_error)/dt;
    *prev_error = error;
    return ((Kp * error) + (Ki * *integral) + (Kd * derivative));
}

