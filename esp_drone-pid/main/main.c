#include <stdio.h>
#include <stdlib.h>
#include "init.h"

#include <stdio.h>
#include <lwip/sockets.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "user_wifi.h"
#include "user_udp.h"
#include "driver/ledc.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "freertos/queue.h"

#include "led.h"
#include "init.h"
// #include "motor.h"
#include "user_udp.h"
#include "battery.h"
#include "mpu6050.h"
#include "pid.h"
#include "imu.h"
#include "ws2812.h"
#include "ledc_motor.h"
// #include "bmx280_bits.h"
// #include "bmx280.h"

#include "spl06.h"
#include "supersconic.h"



extern xQueueHandle gpio_evt_queue = NULL;
extern TaskHandle_t xTask_motor = NULL;
extern QueueHandle_t QHandle_recv;
extern TaskHandle_t xTask_LED;
extern TaskHandle_t allTask_task = NULL; // 所有任务的等待句柄
extern TaskHandle_t xTask_pidrec = NULL;
extern TaskHandle_t xTask_pid_change;
extern char rx_buffer[1024];

void app_main()
{
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // xTaskCreate(init, "init", 2048, NULL, 1, NULL);
  wifi_init_sta(); // 启动WIFI的STA
  // wifi_init_ap();

  PID_controllerInit();//初始化PID参数
  ledc_motor_init();//初始化电机
  adc_Init();//初始化电池电压读取

  // xTaskCreate(mpu6050_scan_task, "mpu6050_scan_task", 1024 * 3, NULL, 15, NULL,1);
  xTaskCreatePinnedToCore(mpu6050_scan_task, "mpu6050_scan_task", 1024 * 3, NULL, 15, NULL,1);//姿态控制任务
  xTaskCreatePinnedToCore(pid_chage_task, "pid_chage_task", 1024*3, NULL, 10, NULL,1);//调参任务
  // vTaskDelay(4000 / portTICK_RATE_MS);
  xTaskCreate(led_task, "task2", 2048, NULL, 1, &xTask_LED);//LED任务

  // xTaskCreate(argument_receive_task, "argument_receive_task", 2048 * 2, (void *)QHandle_recv, 10, &xTask_pidrec);
  // xTaskCreate(battery_task, "battery_task", 2048, NULL, 1, NULL);
  // xTaskCreate(creat_motor_task, "creat_motor_task", 4096, NULL, 2, &xTask_motor);
}



