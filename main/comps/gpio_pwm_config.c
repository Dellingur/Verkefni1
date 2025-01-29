#include "./pins.c"
#include "driver/gpio.h"
#include "driver/ledc.h"

// LEDC (PWM) configuration
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ 1000 // 1 kHz
#define PWM_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution

void configure_gpio_outputs(void) {
    // Configure GPIO 18 and 19 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_IO_INI11) | (1ULL << GPIO_OUTPUT_IO_INI21) | (1ULL << GPIO_OUTPUT_IO_INI31) | (1ULL << GPIO_OUTPUT_IO_INI41) | (1ULL << GPIO_OUTPUT_IO_INI12) | (1ULL << GPIO_OUTPUT_IO_INI22) | (1ULL << GPIO_OUTPUT_IO_INI32) | (1ULL << GPIO_OUTPUT_IO_INI42),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void configure_pwm_output(void) {
    // Configure LEDC PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure LEDC PWM channels
    ledc_channel_config_t ledc_channels[4] = {
        {
            .gpio_num = GPIO_PWM_IO_ENA1,
            .speed_mode = PWM_MODE,
            .channel = 0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = PWM_TIMER,
            .duty = 0, // Initial duty
            .hpoint = 0
        },
        {
            .gpio_num = GPIO_PWM_IO_ENB1,
            .speed_mode = PWM_MODE,
            .channel = 1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = PWM_TIMER,
            .duty = 0, // Initial duty
            .hpoint = 0
        },
        {
            .gpio_num = GPIO_PWM_IO_ENA2,
            .speed_mode = PWM_MODE,
            .channel = 2,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = PWM_TIMER,
            .duty = 0, // Initial duty
            .hpoint = 0
        },
        {
            .gpio_num = GPIO_PWM_IO_ENB2,
            .speed_mode = PWM_MODE,
            .channel = 3,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = PWM_TIMER,
            .duty = 0, // Initial duty
            .hpoint = 0
        },
    };

   ledc_channel_config(&ledc_channels[0]);
   printf("PWM config 0.\n");
   ledc_channel_config(&ledc_channels[1]);
   printf("PWM config 1.\n");
   ledc_channel_config(&ledc_channels[2]);
   printf("PWM config 2.\n");
   ledc_channel_config(&ledc_channels[3]);
   printf("PWM config 3.\n");
}

void set_pwm_duty_cycle(ledc_channel_t channel, uint32_t duty_percentage) {
    // Convert percentage to duty cycle value
    uint32_t duty = (duty_percentage * ((1 << PWM_RESOLUTION) - 1)) / 100;
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}