#pragma once

//头文件
#include"sp_sys.h"
#include<stdio.h>
#include<string.h>
#include"nvs_flash.h"
#include"esp_bt.h"
#include"esp_bt_main.h"
#include"esp_gap_ble_api.h"
#include"esp_gatts_api.h"
#include"esp_gatt_common_api.h"

//低功耗蓝牙初始化
sp_t sp_ble_init();
//低功耗蓝牙发送
sp_t sp_ble_send(char*data,u16 len);
//低功耗蓝牙接收
char*sp_ble_read();