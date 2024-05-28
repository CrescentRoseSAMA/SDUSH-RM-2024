#ifndef SERIAL_H_
#define SERIAL_H_
#include "../AngleSolver/AngleSolver.hpp"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <string.h>
#include "Serial.h"
#include <iostream>

class Serial
{

private:
    int fd = -1; // 串口权柄

public:
    /**
     * @brief 发送数据
     * @return error_code
     */
    int send(double x, double y, double z, double distance, double pitch, double yaw, bool test);
    int send(DataPack &pack, bool test);

    /**
     * @brief 设置串口波特率，停止位，数据长度,打开串口
     * @return error_code
     */
    int uart_setup();

private:
    /**
     *  @brief  将长度数据转换为uchar型以供发送
     */
    unsigned char transformData(double In);
};

#endif