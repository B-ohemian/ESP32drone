#include <stdio.h>
#include <lwip/sockets.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
// #include "nvs_flash.h"
#include "esp_event.h"
#include "led.h"
#include "user_wifi.h"
#include "user_udp.h"
#include "joystick.h"



extern key_offset key_OFF_SET;

void app_main()
{
     
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_read_data(&key_OFF_SET,8);
    

#ifdef WIFI_AP
    wifi_init_ap();
#else
    wifi_init_sta();
#endif
    xTaskCreate(&joystick, "joystick", 4096, NULL, 1, NULL);
    xTaskCreate(&key_scan, "key_scan", 2048, NULL, 1, NULL);
    xTaskCreate(&led_message, "led_message", 2048, NULL, 1, NULL);

}
