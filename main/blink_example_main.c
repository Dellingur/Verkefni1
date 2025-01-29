/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>
#include "./comps/pins.c"
#include "./comps/encoder_interrupt.c"
#include "./comps/gpio_pwm_config.c"
#include "./comps/variables.c"
#include "./comps/PID_calculator.c"
#include "./comps/instructions.c"

// Task to process encoder events
void encoder_task(void *pvParameter) {
    uint32_t gpio_num;

    while (1) {
        // Wait for an event from the queue
        if (xQueueReceive(encoder_event_queue, &gpio_num, portMAX_DELAY)) {
            // Increment degree counter for  rising edge
            switch (gpio_num)
            {
            case GPIO_ENCODER_PIN_A1:
                encoder_A1_counter++;
                distance_travelled++; //Only measures distance from motor A1
                if (encoder_A1_counter % 1000 == 0) { 
                printf("Encoder event on GPIO:%" PRIu32 "\n", gpio_num);
                printf("Total Degrees:%" PRIu32 "\n", encoder_A1_counter);
                }
                break;

            case GPIO_ENCODER_PIN_B1:
                encoder_B1_counter++;
                if (encoder_B1_counter % 1000 == 0) { 
                printf("Encoder event on GPIO:%" PRIu32 "\n", gpio_num);
                printf("Total Degrees:%" PRIu32 "\n", encoder_B1_counter);
                }
                break;
            
            case GPIO_ENCODER_PIN_A2:
                encoder_A2_counter++;
                if (encoder_A2_counter % 1000 == 0) { 
                printf("Encoder event on GPIO:%" PRIu32 "\n", gpio_num);
                printf("Total Degrees:%" PRIu32 "\n", encoder_A2_counter);
                }
                break;
            
            case GPIO_ENCODER_PIN_B2:
                encoder_B2_counter++;
                if (encoder_B2_counter % 1000 == 0) { 
                printf("Encoder event on GPIO:%" PRIu32 "\n", gpio_num);
                printf("Total Degrees:%" PRIu32 "\n", encoder_B2_counter);
                }
                break;
            default:
                break;
            }
            
            
        }
    }
}

//Testing task. Not used
void gpio_pwm_task(void *pvParameter) {
    while (1) {
        gpio_set_level(GPIO_OUTPUT_IO_INI11, 1);
        gpio_set_level(GPIO_OUTPUT_IO_INI21, 0);
        gpio_set_level(GPIO_OUTPUT_IO_INI31, 1);
        gpio_set_level(GPIO_OUTPUT_IO_INI41, 0);

        gpio_set_level(GPIO_OUTPUT_IO_INI12, 1);
        gpio_set_level(GPIO_OUTPUT_IO_INI22, 0);
        gpio_set_level(GPIO_OUTPUT_IO_INI32, 1);
        gpio_set_level(GPIO_OUTPUT_IO_INI42, 0);


        set_pwm_duty_cycle(LEDC_CHANNEL_0, 25);
        set_pwm_duty_cycle(LEDC_CHANNEL_1, 0);
        set_pwm_duty_cycle(LEDC_CHANNEL_2, 0);
        set_pwm_duty_cycle(LEDC_CHANNEL_3, 0);
        //printf("Test2.\n");
        vTaskDelay(pdMS_TO_TICKS(1000));



    }
}

void motor_control_task(void *pvParameter) {
    while (1) {
    //Calculate RPM for each motor.
    RPM_A1 = calc_motor_RPM(encoder_A1_counter, &prev_encoder_A1_counter, dt);
    RPM_B1 = calc_motor_RPM(encoder_B1_counter, &prev_encoder_B1_counter, dt);
    RPM_A2 = calc_motor_RPM(encoder_A2_counter, &prev_encoder_A2_counter, dt);
    RPM_B2 = calc_motor_RPM(encoder_B2_counter, &prev_encoder_B2_counter, dt);

    //Calculate PWM output using PID
    float PID_A1 = PID_calculator(target_RPM_A1, RPM_A1, &prev_error_A1, &integral_A1, Kp, Ki, Kd, dt);
    float PID_B1 = PID_calculator(target_RPM_B1, RPM_B1, &prev_error_B1, &integral_B1, Kp, Ki, Kd, dt);
    float PID_A2 = PID_calculator(target_RPM_A2, RPM_A2, &prev_error_A2, &integral_A2, Kp, Ki, Kd, dt);
    float PID_B2 = PID_calculator(target_RPM_B2, RPM_B2, &prev_error_B2, &integral_B2, Kp, Ki, Kd, dt);

    /* printf("PID_A1:%f" "\n", PID_A1);
    printf("PID_B1:%f" "\n", PID_B1);
    printf("PID_A2:%f" "\n", PID_A2);
    printf("PID_B2:%f" "\n", PID_B2); */

    //Set new PWM signal
    set_pwm_duty_cycle(LEDC_CHANNEL_0, fmin(fmax(PID_A1, 0), 100));
    set_pwm_duty_cycle(LEDC_CHANNEL_1, fmin(fmax(PID_B1, 0), 100));
    set_pwm_duty_cycle(LEDC_CHANNEL_2, fmin(fmax(PID_A2, 0), 100));
    set_pwm_duty_cycle(LEDC_CHANNEL_3, fmin(fmax(PID_B2, 0), 100));

    vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void instruction_executer(void *pvParameter) {
    while (1) {
        //Testing. Does not include turning. 
    target_RPM_A1 = instructions[instruction_pointer].RPM;
    target_RPM_B1 = instructions[instruction_pointer].RPM;
    target_RPM_A2 = instructions[instruction_pointer].RPM;
    target_RPM_B2 = instructions[instruction_pointer].RPM;

    if (distance_travelled > instructions[instruction_pointer].distance)
    {
        instruction_pointer++;
        distance_travelled = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{

    encoder_event_queue = xQueueCreate(ENCODER_QUEUE_LENGTH, sizeof(uint32_t));
    if (encoder_event_queue == NULL) {
        printf("Failed to create encoder event queue\n");
        return;
    }


    configure_gpio_outputs();
    configure_pwm_output();
    configure_encoder_gpio();

    xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 11, NULL);
    //xTaskCreate(gpio_pwm_task, "gpio_pwm_task", 2048, NULL, 10, NULL);
    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 10, NULL);
    xTaskCreate(instruction_executer, "instruction_executer", 4096, NULL, 10, NULL);
}
