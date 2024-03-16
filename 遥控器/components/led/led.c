#include "driver/gpio.h"
#include "led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ���ƺ��
void led_red(int on)
{
    if (on == LED_ON)
    {
        gpio_set_level(LED_RED_IO, LED_ON); // ����
    }
    else
    {
        gpio_set_level(LED_RED_IO, LED_OFF); // �ص�
    }
}

// �����̵�
void led_green(int on)
{
    if (on == LED_ON)
    {
        gpio_set_level(LED_GREEN_IO, LED_ON); // ����
    }
    else
    {
        gpio_set_level(LED_GREEN_IO, LED_OFF); // �ص�
    }
}

void initkey(void)
{
    gpio_pad_select_gpio(KEY8_IO);
    gpio_set_direction(KEY8_IO, GPIO_MODE_INPUT);
    // gpio_pad_select_gpio(LED_RED_IO);
    // gpio_pad_select_gpio(LED_RED_IO);
    // gpio_pad_select_gpio(LED_RED_IO);

    // gpio_pad_select_gpio(LED_RED_IO);
    // gpio_pad_select_gpio(LED_RED_IO);
    // gpio_pad_select_gpio(LED_RED_IO);
    // gpio_pad_select_gpio(LED_RED_IO);
}

// LED��ʼ��
void initLed()
{
    // ��ʼ���ʵƵ�IO�ڣ�
    // ��ƣ�GPIO32���̵ƣ�GPIO15�����ƣ�GPIO16
    gpio_pad_select_gpio(LED_RED_IO);
    gpio_pad_select_gpio(LED_GREEN_IO);
    gpio_set_direction(LED_RED_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN_IO, GPIO_MODE_OUTPUT);

    // �غ�������ɫ��
    // led_red(LED_ON);
    // led_green(LED_ON);
}

void led_twinkle(void)
{
    led_red(0);
    led_green(0);
    vTaskDelay(50 / portTICK_RATE_MS);
    led_red(1);
    led_green(1);
    vTaskDelay(50 / portTICK_RATE_MS);
    led_red(0);
    led_green(0);
    vTaskDelay(50 / portTICK_RATE_MS);
    led_red(1);
    led_green(1);
    vTaskDelay(50 / portTICK_RATE_MS);
}
