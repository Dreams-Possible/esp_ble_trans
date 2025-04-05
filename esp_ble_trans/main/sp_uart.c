#include"sp_uart.h"

//配置
#define TX UART_PIN_NO_CHANGE
#define RX UART_PIN_NO_CHANGE
#define PORT UART_NUM_0
#define RATE 115200
#define STR_MAX 1*1024

//保存静态数据
typedef struct static_t
{
    char receive[STR_MAX+1];
    QueueHandle_t state;
    QueueHandle_t uart;
}static_t;
static static_t*static_data=NULL;

//串口后台接收
static void receive(void*arg);
//初始化串口
uint8_t sp_uart_init();
//读取串口数据
char*sp_uart_read();

//串口后台接收
static void receive(void*arg)
{
    while(1)
    {
        //阻塞直到有串口事件
        uart_event_t event={0};
        if(xQueueReceive(static_data->uart,&event,portMAX_DELAY))
        {
            //如果是收到数据
            if(event.type==UART_DATA)
            {
                //重新读取
                memset(static_data->receive,0,strlen(static_data->receive));
                uart_read_bytes(PORT,static_data->receive,event.size,portMAX_DELAY);
                //收到新串口数据
                uint8_t state=1;
                xQueueOverwrite(static_data->state,&state);
            }
        }
    }
}

//初始化串口
sp_t sp_uart_init()
{
    //申请资源
    static_data=malloc(sizeof(static_t));
    if(!static_data)
    {
        printf("uart: uart_init: memory out\n");
        return SP_FAIL;
    }
    memset(static_data,0,sizeof(static_t));
    static_data->state=xQueueCreate(1,sizeof(uint8_t));
    esp_err_t ret=ESP_OK;
    //配置串口参数
    uart_config_t config={0};
    config.baud_rate=RATE;
    config.data_bits=UART_DATA_8_BITS;
    config.parity=UART_PARITY_DISABLE;
    config.stop_bits=UART_STOP_BITS_1;
    config.flow_ctrl=UART_HW_FLOWCTRL_DISABLE;
    config.source_clk=UART_SCLK_DEFAULT;
    //安装串口驱动程序
    ret=uart_driver_install(PORT,STR_MAX,STR_MAX,1,&static_data->uart,0);
    //配置串口
    ret=uart_param_config(PORT,&config);
    //配置引脚
    ret=uart_set_pin(PORT,TX,RX,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    //串口初始化失败
    if(ret!=ESP_OK)
    {
        printf("uart: uart_init: init fail\n");
        return SP_FAIL;
    }
    //创建UART后台任务
    xTaskCreate(receive,"uart: receive",2048,NULL,16,NULL);
    //串口初始化成功
    printf("uart: uart_init: init ok\n");
    return SP_OK;
}

//读取串口数据
char*sp_uart_read()
{
    uint8_t state=0;
    xQueueReceive(static_data->state,&state,portMAX_DELAY);
    //返回读取的数据
    return static_data->receive;
}
