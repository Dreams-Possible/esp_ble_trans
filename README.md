# esp_ble_trans
一个使用ESP32低功耗蓝牙模拟串口服务的简单例程。
这个例程仅处于早期实现阶段，并且将来可能不会更新，仅供学习参考。
通过简化官方例程而来，添加了尽可能详细的中文注释，并实现了面向对象的接口函数。
由于ESPS3不支持经典蓝牙，只能使用BLE（低功耗蓝牙），因此有了这个例程。
通过BLE的GATT事务模拟串口的实现。
需要注意的是，它并非经典蓝牙的SPP串口，因此只能与支持BLE的设备连接，不支持标准蓝牙SPP协议。
由于协议不同，它无法与仅支持标准蓝牙的设备通信。设备应至少支持BLE。
由于通过GATT模拟SPP并非标准的蓝牙SPP实现，在通信过程中可能产生一些兼容性问题。
例程已经实现任意字节发送，经测试其他设备可按预期接收。
但对于ESP端的接收，根据发送设备客户端的处理方式不同，对于部分客户端，可实现完整的任意字节接收。
但对于另一部分客户端，数据会被分解成每20个字节的数据包接收，在这种情况下，这个例程的接口函数只会返回最后一个接收的数据包。
作者在注释部分已经尝试解决，但似乎仍未达到预期实现目的。
SPP服务UUID：aa01
TX UUID：bb01
RX UUID：bb00
