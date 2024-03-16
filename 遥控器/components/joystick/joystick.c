#include "joystick.h"
#include <driver/adc.h>
#include "led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "user_udp.h"

#include <math.h>

// 定义
// 注意，我们可以定义多个存储空间来存储对应的值
#define STORAGE_NAMESPACE "storage"

#define scale 0.525

#define N 20 // 滤波缓存数组大小

float scale_right_up = 0;
float scale_right_left = 0;

int up_down_static = 0;
int lef_right_static = 0;

bool FLAG = 0;

bool flag = 0;
bool flag2 = 0;
bool flag3 = 1;
bool remote_mode = 0; // 0是手动模式，1是定高模式

key_offset key_OFF_SET;

// key_OFF_SET.left_thr = 0;
// key_OFF_SET.left_right = 0;
// key_OFF_SET.right_up = 0;
// key_OFF_SET.right_left = 0;

int16_t temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
static uint16_t cnt = 0; // 判断油门值是否为0的计数变量

// 读取函数
/*
 *读取函数
 *para1:要读取的结构体指针。
 *para2:长度
 */
esp_err_t nvs_read_data(key_offset *Temp_st, uint32_t *len)
{
  nvs_handle_t nvs_handle;
  esp_err_t err;

  // 首先一部操作打开存储空间
  //  Open
  // para(空间名字（字符串），操作类型（只读还是读写），操作句柄)
  err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK)
  {
    printf("NVS_READ:存储空间打开失败\n");
    return err;
  }

  // 注意这一步十分重要，因为在NVS存储已知长度类型的数据时，我们可以明确的传入已知的长度。
  // 但是这个地方对于我们传入的数组或者说结构体，我们不知道明确长度，于是我们采用下面的操作来获取要读取的数据的长度。

  // Read the size of memory space required for blob
  size_t required_size = 0; // value will default to 0, if not set yet in NVS
  err = nvs_get_blob(nvs_handle, "Temp_st", NULL, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    return err;

  // 再来读取对应的值
  err = nvs_get_blob(nvs_handle, "Temp_st", Temp_st, &required_size);
  if (err == ESP_ERR_NVS_NOT_FOUND)
  {
    printf("NVS_READ:key doesn't exist\n");
    return err;
  }
  else if (err == ESP_ERR_NVS_INVALID_HANDLE)
  {
    printf("NVS_READ:handle has been closed or is NULL\n");
    return err;
  }
  else if (err == ESP_ERR_NVS_INVALID_NAME)
  {
    printf("NVS_READ:name doesn't satisfy constraints\n");
    return err;
  }
  else if (err == ESP_ERR_NVS_INVALID_LENGTH)
  {
    printf("NVS_READ:length is not sufficient to store data\n");
    return err;
  }
  else
  {
    printf("NVS_READ:读取成功\n");
  }

  // 关闭句柄
  nvs_close(nvs_handle);
  return ESP_OK;
}

/*
 *存储函数
 *para1:要读取的结构体指针。
 *para2:长度
 */
esp_err_t nvs_write_data(key_offset *Temp_st, uint32_t len)
{
  nvs_handle_t nvs_handle;
  esp_err_t err;

  printf("NVS_WRITE:存储wifi信息\n");
  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK)
  {
    printf("NVS_WRITE:存储空间打开失败\n");
    return err;
  }
  err = nvs_set_blob(nvs_handle, "Temp_st", Temp_st, len);
  if (err != ESP_OK)
  {
    printf("NVS_WRITE:存储空间存储失败\n");
    return err;
  }
  err = nvs_commit(nvs_handle);
  if (err != ESP_OK)
  {
    printf("NVS_WRITE:存储空间提交失败\n");
    return err;
  }
  nvs_close(nvs_handle);
  // if (err != ESP_OK)
  // {
  //   printf("NVS_WRITE:存储空间关闭失败\n");
  //   return err;
  // }
  return ESP_OK;
}

int16_t tran_buffer[4] = {
    0xaa,
    0,
    0,
    0,
};
// ADC所接的通道
void adc_Init(void)
{
  // 12位分辨率
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(LETF_UP_DOWN_CHANNEL, ADC_ATTEN_11db);
  adc1_config_channel_atten(LETF_LEFT_RIGHT_CHANNEL, ADC_ATTEN_11db);
  adc1_config_channel_atten(RIGHT_UP_DOWN_CHANNEL, ADC_ATTEN_11db);
  adc1_config_channel_atten(RIGHT_LEFT_RIGHT_CHANNEL, ADC_ATTEN_11db);
}

#define LPF_2_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz)*6.28f * (t)))) * ((in) - (out)))

void joystick_get_message(void)
{

  int16_t thr = 0, left = 0, right = 0, up = 0, down = 0;

  float animotion = 0;
  float temp01, temp02, temp03, temp04;
  bool twinkle_flag = 0;

  // animotion = (float)(4095 - adc1_get_raw(LETF_UP_DOWN_CHANNEL)) * 0.000244f;
  // temp1= (animotion*animotion)*1000;
  // printf("anni:%f\n",animotion*1000);
  // temp1 = (left_thr-adc1_get_raw(LETF_UP_DOWN_CHANNEL) )*scale;

  temp01 = (4095 - adc1_get_raw(LETF_UP_DOWN_CHANNEL)) * 0.17; // 500满油
  temp02 = adc1_get_raw(LETF_LEFT_RIGHT_CHANNEL) - key_OFF_SET.left_right;
  temp03 = adc1_get_raw(RIGHT_UP_DOWN_CHANNEL) - key_OFF_SET.right_up;             // roll
  temp04 = 4095 - adc1_get_raw(RIGHT_LEFT_RIGHT_CHANNEL) - key_OFF_SET.right_left; // pit

  //  printf("data2:%0.2f,%0.2f,%0.2f,%0.2f\n", temp01, temp02, temp03, temp04);

  // // 滤波
  LPF_2_(20, 0.001, temp02, temp2);
  LPF_2_(20, 0.001, temp03, temp3);
  LPF_2_(20, 0.001, temp04, temp4);


  // 限幅
  if (temp2 > 1800)
    temp2 = 1800;
  else if (temp2 < -1800)
    temp2 = -1800;
  else if (abs(temp2) < 40)
    temp2 = 0;

  if (temp3 > 1800)
    temp3 = 1800;
  else if (temp3 < -1800)
    temp3 = -1800;
  else if (abs(temp3) < 40)
    temp3 = 0;

  if (temp4 > 1800)
    temp4 = 1800;
  else if (temp4 < -1800)
    temp4 = -1800;
  else if (abs(temp4) < 40)
    temp4 = 0;
  // 判断模式
  if (temp2 == -1800) // 如果两个摇杆同时向内，则改变模式
  {
    vTaskDelay(1000 / portTICK_RATE_MS);
    if (temp2 == -1800)
    {
      remote_mode = 1;
      udp_send_data("P", 1); // 发送模式
    }
  }
  if (temp2 == 1800) // 如果两个摇杆同时向内，则改变模式
  {
    vTaskDelay(1000 / portTICK_RATE_MS);
    if (temp2 == 1800)
    {
      remote_mode = 0;
      udp_send_data("Q", 1); // 发送模式
    }
  }

  if (remote_mode)
  {
    if (temp01 > 500)
    {
      thr = 500 + key_OFF_SET.thr;
    }
  }
  else
  {
    if (temp01 > 0) // 如果测得的值大于基础油门值，则是往下运动的，发送油门值等于测得的值减去基础油门值
    {
      thr = temp01;
    }
  }

  // if (temp1 == 0)
  // {
  //   cnt++;
  //   printf("%d\n",cnt);
  //   if (cnt == 200) // 如果1s钟后油门值还是为0，那么就断开连接
  //   {
  //     if (temp1 == 0)
  //     {
  //       cnt = 0;
  //       FLAG = 0;
  //       flag=0;
  //       udp_send_data("L", 1);
  //       printf("thru is shut!\n");
  //     }
  //   }
  // }

  tran_buffer[1] = thr;
  tran_buffer[2] = temp3; // rol
  tran_buffer[3] = temp4; // pit

  printf("data:%d,%d,%d,%d\n", thr, temp3, temp4, remote_mode);
}

void calibrate_joystick()
{
  int temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;

  for (char i = 0; i < 240; i++)
  {
    temp1 += adc1_get_raw(LETF_UP_DOWN_CHANNEL);
    temp2 += adc1_get_raw(LETF_LEFT_RIGHT_CHANNEL);
    temp3 += adc1_get_raw(RIGHT_UP_DOWN_CHANNEL);
    temp4 += (4095 - adc1_get_raw(RIGHT_LEFT_RIGHT_CHANNEL));
  }
  key_OFF_SET.left_thr = temp1 / 240;
  key_OFF_SET.left_right = temp2 / 240;
  key_OFF_SET.right_up = temp3 / 240;
  key_OFF_SET.right_left = temp4 / 240;
}

// 摇杆发数据发送任务
void joystick(void *pvParameters)
{
  adc_Init();
  // calibrate_joystick();
  while (1)
  {
    joystick_get_message();

    if (FLAG)
    {
      udp_send_data((char *)tran_buffer, 8);
      // printf("data:%d,%d,%d,%d\n", thr, temp3, temp4, remote_mode);
    }
    vTaskDelay(20 / portTICK_RATE_MS);
  }
}

void led_message(void *pvParameters)
{
  initLed();
  while (1)
  {
    if (flag2 || remote_mode)
    {
      led_green(0); // 状态指示灯
    }
    else
    {
      led_green(1); // 状态指示灯
    }
    led_red(!flag);
    vTaskDelay(2 / portTICK_RATE_MS);
  }
}

void key_scan(void *pvParameters)
{

  initkey();
  adc_Init();
  while (1)
  {
    if (flag3)
    {
      if (!gpio_get_level(KEY8_IO)) // 连接按钮
      {
        vTaskDelay(100 / portTICK_RATE_MS);
        if (!gpio_get_level(KEY8_IO))
        {
          flag = !flag;
          if (flag)
          {
            // tran_buffer[6] = 0x8000; // 1000 0000 0000 0000
            // printf("d:%d\n", flag);
            udp_send_data("O", 1);
            cnt = 0;
            FLAG = 1;
          }
          else
          {
            // tran_buffer[6] = 0x4000; // 0100 0000 0000 0000
            // printf("d:%d\n", flag);
            udp_send_data("L", 1);
            FLAG = 0;
          }
        }
      }
    }
    if (!gpio_get_level(KEY7_IO)) // 急停按键，让油门值为0
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY7_IO))
      {
        flag2 = !flag2;
        flag3 = !flag3;
        if (flag2)
        {
          udp_send_data("S", 1);
          FLAG = 0;
        }
      }
    }

    // ---------------------------------微调---------------------------------------

    if (!gpio_get_level(KEY4_IO)) // 上微调
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY4_IO))
      {
        key_OFF_SET.right_up += 20;
        // printf("adjust:%d\n",temp3);
      }
    }

    if (!gpio_get_level(KEY1_IO)) // 下微调
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY1_IO))
      {
        key_OFF_SET.right_up -= 20;
        // printf("adjust:%d\n",temp3);
      }
    }

    if (!gpio_get_level(KEY3_IO)) // 左微调
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY3_IO))
      {
        key_OFF_SET.right_left += 20;
        // printf("adjust:%d\n",temp4);
      }
    }

    if (!gpio_get_level(KEY2_IO)) // 右微调
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY2_IO))
      {
        key_OFF_SET.right_left -= 20;
        // printf("adjust:%d\n",temp4);
      }
    }

    // 定高油门
    if (!gpio_get_level(KEY6_IO)) // 油门加
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY6_IO))
      {
        key_OFF_SET.thr += 10;
      }
    }

    if (!gpio_get_level(KEY5_IO)) // 油门减
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY5_IO))
      {
        key_OFF_SET.thr -= 10;
      }
    }

    // 保存按钮
    if (!gpio_get_level(KEY_SAVE_IO))
    {
      vTaskDelay(100 / portTICK_RATE_MS);
      if (!gpio_get_level(KEY_SAVE_IO))
      {
        nvs_write_data(&key_OFF_SET, 8);
      }
    }
    // led_green(!flag2); // 状态指示灯
    // led_red(!flag);
    vTaskDelay(2 / portTICK_RATE_MS);
  }
}
