#include "battery.h"

#define ADC1_TEST_CHANNEL ADC1_CHANNEL_7
/*ADC初始化
ADC_ATTEN_DB_0:表示参考电压为1.1V
ADC_ATTEN_DB_2_5:表示参考电压为1.5V
ADC_ATTEN_DB_6:表示参考电压为2.2V
ADC_ATTEN_DB_11:表示参考电压为3.9V*/

// extern xStruct_tran trandata={0.0,0.0,0.0,0.0};
extern QueueHandle_t QHandle_tran = NULL;
// ADC所接的通道
void adc_Init()
{
  // 12位分辨率
  adc1_config_width(ADC_WIDTH_12Bit);
  // 设置通道6和1.1V参考电压
  adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_0db);
}

float get_battery_adc(void)
{
  int read_raw;
  float voltage;
  read_raw = adc1_get_raw(ADC1_TEST_CHANNEL);
  voltage = (float)read_raw / 4095;
  voltage = voltage * 5.02 * 0.96;

  return  voltage;

  // printf("adc=%d,%.2fV\n", read_raw, voltage);

}

// 定时器回调函数
void BatTimerCallback(TimerHandle_t xTimer)
{
  int read_raw;
  float voltage;
  char s[10]= "bat";
  char rx_buffer[5] = {0};
  BaseType_t xStaus;

  // char adc_buff[50] = {0};
  // ADC的结果转换成电压
  read_raw = adc1_get_raw(ADC1_TEST_CHANNEL);
  voltage = (float)read_raw / 4095;
  voltage = voltage * 5.02 * 0.96;
  sprintf(rx_buffer, "%.2f", voltage);
  my_strcat(s,rx_buffer); // 拼接两个字符串，结果保存在第一个字符串当中
  xStaus = xQueueSend(QHandle_tran, &s, 0);
  if (xStaus != pdPASS)
  {
    printf("send faild!\n");
  }
  else
  {
    printf("send done!\n");
  }
  printf("adc=%d,%.2fV\n", read_raw, voltage);
}

// 调用定时器，2秒采集一次电压
void battery_task(void *arg)
{
  TimerHandle_t xTimer1; // 定时器句柄
  adc_Init();
  xTimer1 = xTimerCreate("Timer1", pdMS_TO_TICKS(2000), pdTRUE, (void *)0, BatTimerCallback);
  xTimerStart(xTimer1, 0);
  // 定时器任务创建好之后就把自己杀了
  vTaskDelete(NULL);
  while (1)
  {
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}