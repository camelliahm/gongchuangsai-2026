#include "serial.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <termios.h>

/**
 * @brief  打开设备并配成 8N1 非阻塞
 * @param  dev  设备节点（如 "/dev/ttyUSB0"）
 * @param  baud 波特率
 * @retval 句柄（fd）>=0 成功；<0 失败（设备不存在/无权限）
 * @author camelliahm
 */
int serial_Open(const char *dev, uint32_t baud)
{
    int fd;
    struct termios tio;
    speed_t speed;

    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        perror("serial: open");
        return -1;
    }

    memset(&tio, 0, sizeof(tio));

    switch (baud)
    {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:     speed = B115200; break;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    /* 8N1，无流控 */
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);   /* 原始模式 */
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;

    tio.c_cc[VMIN]  = 0;   /* 非阻塞：无数据立即返回 */
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
        perror("serial: tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

/**
 * @brief  非阻塞读
 * @param  fd  句柄
 * @param  buf 目标缓冲
 * @param  max 最多读的字节数
 * @retval 读到的字节数；0 = 无数据/失败
 * @author camelliahm
 */
int serial_Read(int fd, uint8_t *buf, int max)
{
    ssize_t n;

    if ((fd < 0) || (buf == NULL) || (max <= 0))
    {
        return 0;
    }

    n = read(fd, buf, (size_t)max);
    if (n < 0)
    {
        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
        {
            perror("serial: read");
        }
        return 0;
    }

    return (int)n;
}

/**
 * @brief  写
 * @param  fd  句柄
 * @param  buf 数据
 * @param  len 长度
 * @retval 实际写入字节数
 * @author camelliahm
 */
int serial_Write(int fd, const uint8_t *buf, int len)
{
    ssize_t n;

    if ((fd < 0) || (buf == NULL) || (len <= 0))
    {
        return 0;
    }

    n = write(fd, buf, (size_t)len);
    return (n < 0) ? 0 : (int)n;
}

/**
 * @brief  关闭
 * @param  fd 句柄
 * @author camelliahm
 */
void serial_Close(int fd)
{
    if (fd >= 0)
    {
        close(fd);
    }
}
