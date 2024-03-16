
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/adc.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "user_udp.h"
#include "init.h"

void battery_task(void *arg);
void adc_Init(void);
void BatTimerCallback(TimerHandle_t xTimer);
float get_battery_adc(void);
