#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

static volatile uint32_t encoder_A1_counter = 0; // Degree counter
static volatile uint32_t encoder_B1_counter = 0;
static volatile uint32_t encoder_A2_counter = 0;
static volatile uint32_t encoder_B2_counter = 0;

static volatile uint32_t prev_encoder_A1_counter = 0; // Prev Degree counter
static volatile uint32_t prev_encoder_B1_counter = 0;
static volatile uint32_t prev_encoder_A2_counter = 0;
static volatile uint32_t prev_encoder_B2_counter = 0;

uint32_t distance_travelled = 0;

//PID values
float Kp = 0.1;
float Ki = 0.05;
float Kd = 0.1;



//PID state
float prev_error_A1 = 0;
float integral_A1 = 0;
float prev_error_B1 = 0;
float integral_B1 = 0;
float prev_error_A2 = 0;
float integral_A2 = 0;
float prev_error_B2 = 0;
float integral_B2 = 0;

float target_RPM_A1 = 0;
float target_RPM_B1 = 0;
float target_RPM_A2 = 0;
float target_RPM_B2 = 0;

float RPM_A1 = 0;
float RPM_B1 = 0;
float RPM_A2 = 0;
float RPM_B2 = 0;

int dt = 100;
int instruction_pointer = 0;



typedef struct 
{
    int turn_ratio;
    int distance;
    int RPM;
}instruction;

//Instructions
instruction instructions[3] = {
        {1, 1000, 10},  // First instruction
        {1, 1, 1},  // Second instruction
        {1, 1, 1}   // Third instruction
    };


