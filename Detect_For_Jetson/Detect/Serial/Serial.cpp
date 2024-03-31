#include "Serial.h"

int Serial::uart_setup()
{
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        printf("open dev fail!\n");
        fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0)
            return -1;
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
        printf("port is open\n");
    }
    struct termios options;

    // // 获取原有串口配置
    // if  (tcgetattr(fd, &options) < 0) {
    //     return -1;
    // }
    // 设置波特率
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // // 修改控制模式，保证程序不会占用串口
    // options.c_cflag  |=  CLOCAL;

    // // 修改控制模式，能够从串口读取数据
    // options.c_cflag  |=  CREAD;

    // // 不使用流控制
    // options.c_cflag &= ~CRTSCTS;

    // 设置数据位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 设置奇偶校验位
    options.c_cflag &= ~PARENB;
    // options.c_iflag &= ~INPCK;

    // 设置停止位
    options.c_cflag &= ~CSTOPB;

    // // 设置最少字符和等待时间
    // options.c_cc[VMIN] = 1;     // 读数据的最小字节数
    // options.c_cc[VTIME]  = 0;   //等待第1个数据，单位是10s

    // // 修改输出模式，原始数据输出
    // options.c_oflag &= ~OPOST;
    // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(INLCR | ICRNL);        // 不要回车和换行转换
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 不要软件流控制
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式

    // // 清空终端未完成的数据
    // tcflush(fd, TCIFLUSH);

    // 设置新属性
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        std::cout << fd << std::endl;
        return -1;
    }

    return 0;
}

int Serial::send(double x, double y, double z, double distance, double pitch, double yaw, bool test)
{
    int flag = 0;
    unsigned char input[16] = {0}; // 每个数据包发送13帧，第0帧为起始帧，第1-10帧数据帧，第11帧为校验帧，第12帧为结束帧
    /* 起始帧设为‘s’ */
    input[0] = 's';
    input[1] = 't';
    /* 数据帧，进行数据转换与装载 */
    input[2] = transformData(x);
    if (x < 0)
        input[8] = 1;
    input[3] = transformData(y);
    if (y < 0)
        input[9] = 1;
    input[4] = transformData(z);
    input[5] = transformData(distance);

    if (pitch < 0) // 设置正负标志（第五第六帧为角度，有正负）
    {
        input[6] = (unsigned char)(-pitch);
        input[10] = 1;
    }
    else
    {
        input[6] = (unsigned char)(pitch);
    }
    if (yaw < 0)
    {
        input[7] = (unsigned char)(-yaw);
        input[11] = 1;
    }
    else
    {
        input[7] = (unsigned char)(yaw);
    }
    input[12] = test;
    /* 校验帧对八帧数据进行校验 */
    for (int i = 2; i < 13; i++)
    {
        flag += input[i];
    }

    input[13] = (unsigned char)(flag / 10);

    /* 结束帧设为‘e’ */
    input[14] = 'e';
    input[15] = 'n';
    /* 发送数据 */
    if (write(fd, input, sizeof(input)) == -1)
        return 0;
    return -1;
}
unsigned char Serial::transformData(double In)
{
    if (In < 0)
    {
        In = -In;
    }
    if (In > 0 && In <= 20)
    {
        In = 1;
    }
    else if (In > 20)
    {
        In /= 20;
    }
    else
    {
        In = 0;
    }
    return (unsigned char)In;
}
