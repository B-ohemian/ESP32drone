#include <string.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "string.h" 
#include "user_udp.h"


QueueHandle_t QHandle_recv;  //接收队列
QueueHandle_t QHandle_tran;  //发送队列
TaskHandle_t xTask_motor;    //电机任务句柄
TaskHandle_t xTask_LED;      //LED任务句柄
TaskHandle_t allTask_task;   //所有任务的等待句柄
TaskHandle_t xTask_pid;
TaskHandle_t xTask_pidrec;
TaskHandle_t xTask_pid_change;
void init(void *arg);
char *my_strcat(char *str1, char *str2);