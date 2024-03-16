/*
这个初始化文件里是一些变量，队列，还有flash这些的初始化
*/
#include "init.h"
#include "nvs_flash.h"

void init(void *arg)
{
  char rx_buffer[10] = {0};
  float rx_buffer2[18]={0};
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  QHandle_recv = xQueueCreate(30, sizeof(rx_buffer2)); // 长度，item宽度
  QHandle_tran = xQueueCreate(8, sizeof(rx_buffer));     // 长度，item宽度

  vTaskDelete(NULL);
  while (1)
  {
    vTaskDelay(1 / portTICK_RATE_MS);
  }
}
//字符串拼接
char *my_strcat(char *str1, char *str2)
{
    char *pt = str1;
    while(*str1!='\0') str1++;
    while(*str2!='\0') *str1++ = *str2++;
    *str1 = '\0';
    return pt;
}
