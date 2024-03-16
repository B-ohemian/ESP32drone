#ifndef LEDC_MOTOR_H
#define LEDC_MOTOR_H 

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

// #define LEDC_OUTPUT_IO          (40) // Define the output GPIO
// #define LEDC_OUTPUT_IO2          (4) // Define the output GPIO
// #define LEDC_OUTPUT_IO3          (5) // Define the output GPIO
// #define LEDC_OUTPUT_IO4          (41) // Define the output GPIO

// #define LEDC_OUTPUT_IO          (39) // Define the output GPIO
// #define LEDC_OUTPUT_IO2          (42) // Define the output GPIO
// #define LEDC_OUTPUT_IO3          (5) // Define the output GPIO
// #define LEDC_OUTPUT_IO4          (6) // Define the output GPIO

#define LEDC_OUTPUT_IO          (37) // Define the output GPIO
#define LEDC_OUTPUT_IO2          (42) // Define the output GPIO
#define LEDC_OUTPUT_IO3          (5) // Define the output GPIO
#define LEDC_OUTPUT_IO4          (6) // Define the output GPIO

// #define LEDC_OUTPUT_IO          (4) // Define the output GPIO
// #define LEDC_OUTPUT_IO2          (5) // Define the output GPIO
// #define LEDC_OUTPUT_IO3          (6) // Define the output GPIO
// #define LEDC_OUTPUT_IO4          (7) // Define the output GPIO

#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_CHANNEL2            LEDC_CHANNEL_1
#define LEDC_CHANNEL3            LEDC_CHANNEL_2
#define LEDC_CHANNEL4            LEDC_CHANNEL_3

#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 10 bits

// #define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
// #define LEDC_DUTY2               (1000) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
// #define LEDC_DUTY3               (500) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
// #define LEDC_DUTY4               (100) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095

#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

void ledc_motor_init(void);
void ledc_motor_run(uint16_t motor1_duty, uint16_t motor2_duty,uint16_t motor3_duty,uint16_t motor4_duty);

#endif