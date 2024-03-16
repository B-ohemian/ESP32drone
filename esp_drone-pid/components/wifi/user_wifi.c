#include "user_wifi.h"
#include "user_udp.h"
#include "init.h"

ip4_addr_t s_ip_addr, s_gw_addr, s_netmask_addr;

// wifi�¼�������
// ctx     :��ʾ������¼����Ͷ�Ӧ��Я���Ĳ���
// event   :��ʾ������¼�����
// ESP_OK  : succeed
// ����    :ʧ��
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_AP_START:
    {
        tcpip_adapter_ip_info_t ipInfo;
        printf("\nwifi_softap_start\n");
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP, "uestc"));
        ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo));
        s_ip_addr = ipInfo.ip;
        s_gw_addr = ipInfo.gw;
        s_netmask_addr = ipInfo.netmask;

        // lcd_display(1);
    }
    break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        printf("station:" MACSTR " join, AID=%d.\r\n",
               MAC2STR(event->event_info.sta_connected.mac),
               event->event_info.sta_connected.aid);

        // �пͻ������ӣ�����������

        create_udp();
        break;

    case SYSTEM_EVENT_AP_STADISCONNECTED:
        printf("station:" MACSTR "leave, AID=%d.\r\n",
               MAC2STR(event->event_info.sta_disconnected.mac),
               event->event_info.sta_disconnected.aid);

        break;

    case SYSTEM_EVENT_STA_START:
        printf("sta start.\r\n");
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "uestc"));
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        printf("SYSTEM_EVENT_STA_CONNECTED.\r\n");
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        printf("\r\nip:%s.\r\n", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        printf("gw:%s.\r\n", ip4addr_ntoa(&event->event_info.got_ip.ip_info.gw));
        printf("netmask:%s.\r\n", ip4addr_ntoa(&event->event_info.got_ip.ip_info.netmask));

        s_ip_addr.addr = event->event_info.got_ip.ip_info.ip.addr;           // ����IP
        s_gw_addr.addr = event->event_info.got_ip.ip_info.gw.addr;           // ��������
        s_netmask_addr.addr = event->event_info.got_ip.ip_info.netmask.addr; // ��������
        // �������пͻ������ӣ�����������
        create_udp();
        xTaskNotify(xTask_LED, 0x03, eSetValueWithoutOverwrite);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xTaskNotify(xTask_LED, 0x04, eSetValueWithoutOverwrite);
        xTaskNotify(xTask_LED, 0x02, eSetValueWithoutOverwrite);
        // ���½���server
        break;

    default:
        break;
    }
    return ESP_OK;
}

// ����WIFI��AP
void wifi_init_ap()
{
    tcpip_adapter_init(); // tcp/IP����

#ifdef ESP32_STATIC_IP
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    tcpip_adapter_ip_info_t ipInfo;

    inet_pton(AF_INET, DEVICE_IP, &ipInfo.ip);
    inet_pton(AF_INET, DEVICE_IP, &ipInfo.gw);
    inet_pton(AF_INET, DEVICE_NETMASK, &ipInfo.netmask);
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo));
    tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP);
#endif

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL)); // ����wifi�¼��ص�����

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // wifiĬ�ϳ�ʼ��
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = DEFAULT_SSID,
            .ssid_len = 0,
            .max_connection = 1, // ���ֻ�ܱ�4��stationͬʱ����
            .password = DEFAULT_PWD,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));                   // ����wifi����ģʽΪAP
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config)); // ����AP����
    ESP_ERROR_CHECK(esp_wifi_start());                                  // ʹ��wifi
}

// ����WIFI��STA
void wifi_init_sta()
{
    tcpip_adapter_init(); // tcp/IP����
#ifdef ESP32_STATIC_IP
    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);
    tcpip_adapter_ip_info_t ipInfo;

    inet_pton(AF_INET, DEVICE_IP, &ipInfo.ip);
    inet_pton(AF_INET, DEVICE_GW, &ipInfo.gw);
    inet_pton(AF_INET, DEVICE_NETMASK, &ipInfo.netmask);
    tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
#endif

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL)); // ����wifi�¼��ص�����

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                    // wifiĬ�ϳ�ʼ��
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // NVS ���ڴ洢 WiFi ����.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID, //
            .password = DEFAULT_PWD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));               // ����wifi����ģʽΪSTA
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // ����AP����
    ESP_ERROR_CHECK(esp_wifi_start());                               // ʹ��wifi
}
