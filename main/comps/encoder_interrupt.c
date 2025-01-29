#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "./pins.c"
#define ENCODER_QUEUE_LENGTH 10

static QueueHandle_t encoder_event_queue; // Queue to store encoder events


// Interrupt Service Routine (ISR) for the encoder
static void IRAM_ATTR encoder_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Push the GPIO number into the queue
    xQueueSendFromISR(encoder_event_queue, &gpio_num, &xHigherPriorityTaskWoken);

    // Request a context switch if necessary
    /* if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    } */
}

// GPIO configuration for encoder pins
void configure_encoder_gpio(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Interrupt on rising edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_ENCODER_PIN_A1) | (1ULL << GPIO_ENCODER_PIN_B1),
        .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // Install ISR handlers for both pins
    gpio_install_isr_service(0); // Default ISR service with no flags
    gpio_isr_handler_add(GPIO_ENCODER_PIN_A1, encoder_isr_handler, (void *)GPIO_ENCODER_PIN_A1);
    gpio_isr_handler_add(GPIO_ENCODER_PIN_B1, encoder_isr_handler, (void *)GPIO_ENCODER_PIN_B1);
    gpio_isr_handler_add(GPIO_ENCODER_PIN_A2, encoder_isr_handler, (void *)GPIO_ENCODER_PIN_A2);
    gpio_isr_handler_add(GPIO_ENCODER_PIN_B2, encoder_isr_handler, (void *)GPIO_ENCODER_PIN_B2);
}
