
#include "user_udp.h"
#include "led.h"

static struct sockaddr_in dest_addr;                  //Զ�˵�ַ
socklen_t dest_addr_socklen = sizeof(dest_addr);

static int udp_socket = 0;                          //����socket
TaskHandle_t xUDPRecvTask = NULL;

extern bool FLAG;
//UDP�����ݷ��ͺ���
void udp_send_data(char* data, int len)
{

    if(udp_socket>0){

        int err = sendto(udp_socket, data, len, 0, (struct sockaddr *)&dest_addr, dest_addr_socklen);
        if (err < 0) {
          printf( "Error occurred during sending: errno %d", errno);
        }
    }
}

//UDP��������
void udp_recv_data(void *pvParameters){
    socklen_t socklen = sizeof(dest_addr);
    uint8_t rx_buffer[1024] = {0};
    printf("create udp recv\n");

    //����UDP�Ƿ�OK
    //����ɾ��
    udp_send_data("udp test", strlen("udp test"));

    while (1)
    {
        int len = recvfrom(udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0,  (struct sockaddr *)&dest_addr, &dest_addr_socklen);
        if(len > 0){
            if(rx_buffer[0]=='D')
            {
                //�����յ�������"on"
                led_red(LED_OFF);
                FLAG=0;
            }
            else
            {
                rx_buffer[len] = 0; //δβ����"\0"
                printf("Received %d bytes: %s.\n", len, rx_buffer);

                //���յ���������rx_buffer�����Ϊlen�ֽ�
                //���������û�����

                //�����ʵ����������ݸ����Ͷ�
                udp_send_data((char*)rx_buffer, len);
            }
        }
    }
}


//����UDP����
void udp_ini_client(void *pvParameters){

    //����Ѿ����������ȹر�֮ǰ��
    if(udp_socket>0){
        close(udp_socket);
        udp_socket=0;
    }

    //��һ��������socket
    udp_socket = socket(AF_INET,SOCK_DGRAM,0);
    printf("connect_socket:%d\n",udp_socket);

    if(udp_socket < 0){
       printf( "Unable to create socket: errno %d", errno);
       return;
    }

    //Զ�˲�������
    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255"); //Ŀ���ַĬ��ʹ�ù㲥
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);//Ŀ��˿�
    printf("Socket created, sending to %s:%d", UDP_ADRESS, UDP_PORT);

    //ESP32ģ���������
    struct sockaddr_in Loacl_addr;
    Loacl_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    Loacl_addr.sin_family = AF_INET;
    Loacl_addr.sin_port = htons(UDP_PORT); //���ñ��ض˿�

    uint8_t res = 0;
    //�ڶ�������bind
    res = bind(udp_socket,(struct sockaddr *)&Loacl_addr,sizeof(Loacl_addr));
    if(res != 0){
        printf("bind error\n");

    }

    //������ڽ���������ɾ����������
    if(xUDPRecvTask != NULL){
        vTaskDelete(xUDPRecvTask);
        xUDPRecvTask = NULL;
    }

    //����������������������
    xTaskCreate(&udp_recv_data,"udp_recv_data",2048*2,NULL,10,&xUDPRecvTask);

    //ɾ����ǰ����
    vTaskDelete(NULL);
}

void create_udp()
{
    xTaskCreate(&udp_ini_client, "udp_ini_client", 4096, NULL, 5, NULL);
}


