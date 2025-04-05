#pragma once

//头文件
#include"sp_sys.h"
#include<stdio.h>
#include<string.h>
#include"driver/uart.h"

//初始化串口
sp_t sp_uart_init();
//读取串口数据
char*sp_uart_read();
