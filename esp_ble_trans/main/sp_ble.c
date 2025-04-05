#include"sp_ble.h"

//设备名称
#define BT_DEV_NAME "ESP_SPP_SERVER"
//广播名称
#define BT_ADVERT_NAME 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R', 'V', 'E', 'R'
//SPP服务UUID
#define GATT_SPP_UUID 0xAA01
//接收UUID
#define GATT_REC_UUID 0xBB00
//通知UUID
#define GATT_NTF_UUID 0xBB01
//SPP应用程序ID
#define ESP_SPP_APP_ID 0x01
//SPP收发最大缓冲区
#define STR_TX_MAX 1*1024
#define STR_RX_MAX 1*1024

//SPP服务的ID
static u16 spp_id = 0xffff;
//SPP的GATT事件ID
static esp_gatt_if_t spp_gatt_id = 0xff;
//通知状态
static bool ntf_state = false;
//连接状态
static bool connect_state = false;
//远程设备低功耗蓝牙地址
static esp_bd_addr_t remote_bt_addr = {0};

//GAP事件回调函数
static void gap_event_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t*param);
//GATT事件回调函数
static void gatts_event_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t*param);
//GATT服务回调函数
static void gatt_prof_event_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t*param);
//低功耗蓝牙初始化
sp_t sp_ble_init();
//低功耗蓝牙发送
sp_t sp_ble_send(char*data, u16 len);
//低功耗蓝牙接收
char*sp_ble_read();

//SPP服务类型
enum
{
    //SPP服务
    SPP_SERVER,
    //SPP数据接收声明
    SPP_REC_DEC,
    //SPP数据接收值
    SPP_REC_DATA,
    //SPP数据通知声明
    SPP_NTF_DEC,
    //SPP数据通知值
    SPP_NTF_DATA,
    //SPP数据通知客户端配置
    SPP_NTF_CFG,
    //SPP类型总数
    SPP_SUM,
};
//SPP服务类型列表
static u16 spp_type_list[SPP_SUM];

//GATT服务UUID
static const u16 gatt_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
//GATT声明UUID
static const u16 gatt_dec_uuid = ESP_GATT_UUID_CHAR_DECLARE;
//GATT通知UUID
static const u16 gatt_client_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//SPP接收值UUID
static const u16 spp_rec_data_uuid = GATT_REC_UUID;
//SPP通知值UUID
static const u16 spp_ntf_dec_uuid = GATT_NTF_UUID;
//SPP服务属性（SPP服务UUID）
static const u16 spp_service_uuid = GATT_SPP_UUID;
//SPP接收属性（可读和无响应写）
static const u8 spp_rec_dec_attr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;
//SPP通知属性（可读和通知）
static const u8 spp_ntf_dec_attr = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//SPP接收值储存
static const u8  spp_rec_data_buf[20] = {0x00};
//SPP通知值储存
static const u8  spp_ntf_data_buf[20] = {0x00};
//SPP通知配置储存
static const u8  spp_ntf_cfg_buf[2] = {0x00, 0x00};

//SPP服务GATT列表
static const esp_gatts_attr_db_t spp_gatt_table[SPP_SUM]=
{
    //SPP服务声明
    [SPP_SERVER]=
    {
        {ESP_GATT_AUTO_RSP}, 
        {ESP_UUID_LEN_16, (u8*)&gatt_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (u8*)&spp_service_uuid}
    },
    //SPP接收声明
    [SPP_REC_DEC]=
    {
        {ESP_GATT_AUTO_RSP}, 
        {ESP_UUID_LEN_16, (u8*)&gatt_dec_uuid, ESP_GATT_PERM_READ, 1, 1, (u8*)&spp_rec_dec_attr}
    },
    //SPP接收值
    [SPP_REC_DATA]=
    {
        {ESP_GATT_AUTO_RSP}, 
        {ESP_UUID_LEN_16, (u8*)&spp_rec_data_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 512, sizeof(spp_rec_data_buf), (u8*)spp_rec_data_buf}
    },
    //SPP通知声明
    [SPP_NTF_DEC]=
    {
        {ESP_GATT_AUTO_RSP}, 
        {ESP_UUID_LEN_16, (u8*)&gatt_dec_uuid, ESP_GATT_PERM_READ, 1, 1, (u8*)&spp_ntf_dec_attr}
    },
    //SPP通知值
    [SPP_NTF_DATA]=
    {
        {ESP_GATT_AUTO_RSP}, 
        {ESP_UUID_LEN_16, (u8*)&spp_ntf_dec_uuid, ESP_GATT_PERM_READ, 512, sizeof(spp_ntf_data_buf), (u8*)spp_ntf_data_buf}
    },
    //SPP通知配置
    [SPP_NTF_CFG]=
    {
        {ESP_GATT_AUTO_RSP}, 
        {ESP_UUID_LEN_16, (u8*)&gatt_client_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(u16), sizeof(spp_ntf_cfg_buf), (u8*)spp_ntf_cfg_buf}
    },
};

//低功耗蓝牙广播参数
static esp_ble_adv_params_t spp_adv_params=
{
    //广播最小间隔（单位：0.625ms），0x20约为20ms
    .adv_int_min        = 0x20,
    //广播最大间隔（单位：0.625ms），0x40约为40ms
    .adv_int_max        = 0x40,
    //广播类型：可连接的常规广播
    .adv_type           = ADV_TYPE_IND,
    //使用设备的公有地址
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //使用所有广播信道
    .channel_map        = ADV_CHNL_ALL,
    //允许任何设备扫描和连接
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

//SPP服务广播数据
static const u8 bt_advert_data[2+3+15+3]=
{
    //可被发现，BLE外设，不支持传统低功耗蓝牙
    1+1, ESP_BLE_AD_TYPE_FLAG, 0x06,
    //小端序存储，支持SPP服务
    1+2, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xF0, 0xAB,
     //设置广播名称
    1+14, ESP_BLE_AD_TYPE_NAME_CMPL, BT_ADVERT_NAME,
};


static void gatt_prof_event_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t*param);

//GATT事件信息
typedef struct gatt_profile_t
{
    //GATT事件回调函数
    esp_gatts_cb_t gatts_cb;
    //GATT事件ID
    u16 gatts_if;
}gatt_profile_t;
static gatt_profile_t gatt_profile=
{
    .gatts_cb = gatt_prof_event_cb,
    .gatts_if = ESP_GATT_IF_NONE,
};

//低功耗蓝牙收发数据
typedef struct ble_data_t
{
    QueueHandle_t state;
    // u16 rx_len;
    char tx[STR_TX_MAX+1];
    char rx[STR_RX_MAX+1];
}ble_data_t;
static ble_data_t*ble_data=NULL;

//GAP事件回调函数
static void gap_event_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t*param)
{
    switch(event)
    {
        //广播数据设置完成
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            //启动BLE广播
            esp_ble_gap_start_advertising(&spp_adv_params);
        break;
        break;
    default:
        break;
    }
}

//GATT事件回调函数
static void gatts_event_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t*param)
{
    //进入GATT事件回调
    if (gatt_profile.gatts_cb)
    {
        gatt_profile.gatts_cb(event,gatts_if, param);
    }
}

//GATT服务回调函数
static void gatt_prof_event_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t*param)
{
    esp_ble_gatts_cb_param_t*p_data = (esp_ble_gatts_cb_param_t*) param;  //事件参数

    switch (event)
    {
        //注册
        case ESP_GATTS_REG_EVT:
            //分配ID
            if (param->reg.status == ESP_GATT_OK)
            {
                gatt_profile.gatts_if = gatts_if;
            }else
            {
                return;
            }
            //设置设备名称
            esp_ble_gap_set_device_name(BT_DEV_NAME);
            //设置广播数据
            esp_ble_gap_config_adv_data_raw((u8*)bt_advert_data, sizeof(bt_advert_data));
            //创建属性表
            esp_ble_gatts_create_attr_tab(spp_gatt_table, gatts_if, SPP_SUM,0);
        break;
        //传输
        case ESP_GATTS_WRITE_EVT:
            //查找SPP服务器句柄
            u8 type=0xff;
            for(u8 f=0;f<SPP_SUM;++f)
            {
                if(p_data->write.handle == spp_type_list[f])
                {
                    type=f;
                    break;
                }
            }
            if(!p_data->write.is_prep)
            {
                //判断是否是通知配置
                if(type == SPP_NTF_CFG)
                {
                    if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00))
                    {
                        //启用通知
                        ntf_state = true;
                    } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00))
                    {
                        //禁用通知
                        ntf_state = false;
                    }
                }else
                if(type==SPP_REC_DATA)
                {
                    // //不是分片传输
                    // if(p_data->write.is_prep==false)
                    // {
                        u16 copy_len=0;
                        if(p_data->write.len<STR_TX_MAX)
                        {
                            copy_len=p_data->write.len;
                        }else
                        {
                            printf("mem out\n");
                            copy_len=STR_TX_MAX;
                        }
                        memcpy(ble_data->rx,p_data->write.value,copy_len);
                        // printf("once_data=%s\n",ble_data->rx);
                        uint8_t state=1;
                        xQueueOverwrite(ble_data->state,&state);
                    // }else
                    // //分片传输
                    // {
                    //     u16 copy_len=0;
                    //     if(ble_data->rx_len+p_data->write.len<STR_TX_MAX)
                    //     {
                    //         copy_len=p_data->write.len;
                    //     }else
                    //     {
                    //         printf("mem out\n");
                    //         copy_len=STR_TX_MAX-ble_data->rx_len;
                    //     }
                    //     memcpy(ble_data->rx+ble_data->rx_len,p_data->write.value,copy_len);
                    //     printf("spring_data=%s\n",ble_data->rx);
                    //     ble_data->rx_len+=p_data->write.len;
                    // }
                    // // printf((char*)(p_data->write.value), p_data->write.len);
                }
            }
        break;
        // //写入扩展
        // case ESP_GATTS_EXEC_WRITE_EVT:
        //     ble_data->rx_len=0;
        //     uint8_t state=1;
        //     printf("spring_data_over\n");
        //     xQueueOverwrite(ble_data->state,&state);
    	// break;
        //连接
        case ESP_GATTS_CONNECT_EVT:
            //更新状态
            spp_id = p_data->connect.conn_id;
            spp_gatt_id = gatts_if;
            connect_state = true;
            //保存远程低功耗蓝牙设备地址
            memcpy(&remote_bt_addr, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        break;
        //断开连接
        case ESP_GATTS_DISCONNECT_EVT:
        //更新状态
            connect_state = false;
            ntf_state = false;
            //重启广播
            esp_ble_gap_start_advertising(&spp_adv_params);
            //删除远程低功耗蓝牙设备地址
            memset(&remote_bt_addr,0,sizeof(esp_bd_addr_t));
        break;
        //创建属性表
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if(param->add_attr_tab.status != ESP_GATT_OK)
            {
                printf("craet attr err\n");
                return;
            }else
            if(param->add_attr_tab.num_handle != SPP_SUM)
            {
                printf("craet attr err\n");
                return;
            }else
            {
                //成功创建属性表，保存句柄
                memcpy(spp_type_list, param->add_attr_tab.handles, sizeof(spp_type_list));
                //启动服务
                esp_ble_gatts_start_service(spp_type_list[SPP_SERVER]);
            }
        break;
        default:
        break;
    }
}

//低功耗蓝牙初始化
sp_t sp_ble_init()
{

    //申请资源
    ble_data=malloc(sizeof(ble_data_t));
    if(!ble_data)
    {
        printf("mem err\n");
        return SP_FAIL;
    }
    memset(ble_data,0,sizeof(ble_data_t));
    ble_data->state=xQueueCreate(1,sizeof(uint8_t));
    esp_err_t ret=ESP_OK;
    //初始化NVS
    ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    //释放经典低功耗蓝牙内存（仅使用BLE）
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    //初始化低功耗蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    //启用BLE模式
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    //初始化Bluedroid协议栈
    ret = esp_bluedroid_init();
    //启用Bluedroid协议栈
    ret = esp_bluedroid_enable();
    //注册GATT事件回调
    esp_ble_gatts_register_callback(gatts_event_cb);
    //注册GAP事务回调
    esp_ble_gap_register_callback(gap_event_cb);
    //注册GATT应用程序
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);
    //设置MTU数据包最大量
    ret = esp_ble_gatt_set_local_mtu(512);
    if(ret==ESP_OK)
    {
        return SP_OK;
    }else
    {
        return SP_FAIL;
    }
}

//低功耗蓝牙发送
sp_t sp_ble_send(char*data, u16 len)
{
    if(len>STR_TX_MAX)
    {
        len=STR_TX_MAX;
    }
    memset(ble_data->tx,0,STR_TX_MAX);
    memcpy(ble_data->tx,data,len);
    //一次发送
    if(len<21)
    {
        if(esp_ble_gatts_send_indicate(spp_gatt_id, spp_id,spp_type_list[SPP_NTF_DATA],len,(u8*)ble_data->tx,false)==ESP_OK)
        {
            return SP_OK;
        }else
        {
            printf("send err\n");
            return SP_FAIL;
        }
    }
    //多次发送
    u16 part_all=(len+20-1)/20;
    u16 part_now=1;
    u8*buf=malloc(20*(sizeof(u8)));
    if(!buf)
    {
        printf("mem err\n");
        return SP_FAIL;
    }
    while(part_now<=part_all)
    {
        memset(buf,0,20);
        u16 copy_len=0;
        if(part_now<part_all)
        {
            copy_len=20;
        }else
        {
            copy_len=(len-(part_now-1)*20);
        }
        memcpy(buf,ble_data->tx+(part_now-1)*20,copy_len);
        if(esp_ble_gatts_send_indicate(spp_gatt_id,spp_id,spp_type_list[SPP_NTF_DATA],copy_len,buf,false)!=ESP_OK)
        {
            printf("send err\n");
            free(buf);
            return SP_FAIL;
        }
        part_now++;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    free(buf);
    return SP_OK;
}

//低功耗蓝牙接收
char*sp_ble_read()
{
    uint8_t state=0;
    xQueueReceive(ble_data->state,&state,portMAX_DELAY);
    //返回读取的数据
    return ble_data->rx;
}
