
#include "user_udp.h"
#include "led.h"

static struct sockaddr_in dest_addr;                  //远端地址
socklen_t dest_addr_socklen = sizeof(dest_addr);

static int udp_socket = 0;                          //连接socket
TaskHandle_t xUDPRecvTask = NULL;

extern bool FLAG;
//UDP发数据发送函数
void udp_send_data(char* data, int len)
{

    if(udp_socket>0){

        int err = sendto(udp_socket, data, len, 0, (struct sockaddr *)&dest_addr, dest_addr_socklen);
        if (err < 0) {
          printf( "Error occurred during sending: errno %d", errno);
        }
    }
}

//UDP接收任务
void udp_recv_data(void *pvParameters){
    socklen_t socklen = sizeof(dest_addr);
    uint8_t rx_buffer[1024] = {0};
    printf("create udp recv\n");

    //测试UDP是否OK
    //可以删除
    udp_send_data("udp test", strlen("udp test"));

    while (1)
    {
        int len = recvfrom(udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0,  (struct sockaddr *)&dest_addr, &dest_addr_socklen);
        if(len > 0){
            if(rx_buffer[0]=='D')
            {
                //处理收到的命令"on"
                led_red(LED_OFF);
                FLAG=0;
            }
            else
            {
                rx_buffer[len] = 0; //未尾增加"\0"
                printf("Received %d bytes: %s.\n", len, rx_buffer);

                //接收到的数据在rx_buffer里，长度为len字节
                //可以增加用户代码

                //在这个实验里，返回数据给发送端
                udp_send_data((char*)rx_buffer, len);
            }
        }
    }
}


//创建UDP连接
void udp_ini_client(void *pvParameters){

    //如果已经创建过，先关闭之前的
    if(udp_socket>0){
        close(udp_socket);
        udp_socket=0;
    }

    //第一步：创建socket
    udp_socket = socket(AF_INET,SOCK_DGRAM,0);
    printf("connect_socket:%d\n",udp_socket);

    if(udp_socket < 0){
       printf( "Unable to create socket: errno %d", errno);
       return;
    }

    //远端参数设置
    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255"); //目标地址默认使用广播
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);//目标端口
    printf("Socket created, sending to %s:%d", UDP_ADRESS, UDP_PORT);

    //ESP32模块参数设置
    struct sockaddr_in Loacl_addr;
    Loacl_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    Loacl_addr.sin_family = AF_INET;
    Loacl_addr.sin_port = htons(UDP_PORT); //设置本地端口

    uint8_t res = 0;
    //第二步：绑定bind
    res = bind(udp_socket,(struct sockaddr *)&Loacl_addr,sizeof(Loacl_addr));
    if(res != 0){
        printf("bind error\n");

    }

    //如果存在接收任务，先删除接收任务
    if(xUDPRecvTask != NULL){
        vTaskDelete(xUDPRecvTask);
        xUDPRecvTask = NULL;
    }

    //第三步：创建接收任务函数
    xTaskCreate(&udp_recv_data,"udp_recv_data",2048*2,NULL,10,&xUDPRecvTask);

    //删除当前任务
    vTaskDelete(NULL);
}

void create_udp()
{
    xTaskCreate(&udp_ini_client, "udp_ini_client", 4096, NULL, 5, NULL);
}


