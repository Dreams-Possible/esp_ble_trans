#include"main.h"

void app_main(void)
{
    //串口初始化
    while(sp_uart_init()!=SP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //低功耗蓝牙初始化
    while(sp_ble_init()!=SP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    while(1)
    {
        //从串口输入
        char*data=sp_uart_read();
        //通过低功耗蓝牙发送
        sp_ble_send(data,strlen(data)+1);
        printf("sd_data=%s\n",data);
        //通过低功耗蓝牙接收
        char*data2=sp_ble_read();
        printf("rx_data=%s\n",data2);
    }
}
