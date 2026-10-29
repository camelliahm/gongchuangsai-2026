#include "socketcan.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

static int s_fd = -1;	/* socket 描述符，-1 = 未打开 */

/**
 * @brief  初始化：打开 CAN 原始套接字并绑定接口
 * @param  ifname 接口名（"vcan0"/"can0"），可空（空则跳过绑定）
 * @retval true = 成功；false = 打开或绑定失败（接口不存在/无权限）
 * @note   本机无 vcan0 时失败是预期，上层须优雅降级（帧照常丢弃）
 * @author camelliahm
 */
bool socketcan_Init(const char *ifname)
{
    int fd;
    struct sockaddr_can addr;
    struct ifreq ifr;

    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0)
    {
        perror("socketcan: socket");
        return false;
    }

    /* 若非阻塞收发，配合上层轮询即可，不占线程 */
    if (ifname != NULL)
    {
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
        {
            perror("socketcan: ioctl(SIOCGIFINDEX)");
            close(fd);
            return false;
        }

        memset(&addr, 0, sizeof(addr));
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("socketcan: bind");
            close(fd);
            return false;
        }
    }

    s_fd = fd;
    return true;
}

/**
 * @brief  发送一帧
 * @param  frame 待发送帧
 * @retval true = 已写入内核；false = 失败或未初始化
 * @author camelliahm
 */
bool socketcan_Send(const SocketcanFrame_t *frame)
{
    struct can_frame cf;
    ssize_t n;

    if ((s_fd < 0) || (frame == NULL))
    {
        return false;
    }

    memset(&cf, 0, sizeof(cf));
    cf.can_id  = frame->id;
    cf.can_dlc = frame->dlc;
    memcpy(cf.data, frame->data, sizeof(cf.data));

    n = write(s_fd, &cf, sizeof(cf));
    return (n == (ssize_t)sizeof(cf));
}

/**
 * @brief  接收一帧（非阻塞）
 * @param  frame 输出帧
 * @retval true = 取到一帧；false = 无数据/失败/未初始化
 * @author camelliahm
 */
bool socketcan_Recv(SocketcanFrame_t *frame)
{
    struct can_frame cf;
    ssize_t n;

    if ((s_fd < 0) || (frame == NULL))
    {
        return false;
    }

    n = read(s_fd, &cf, sizeof(cf));
    if (n < 0)
    {
        /* EAGAIN/EWOULDBLOCK = 暂无数据，属正常，不打印 */
        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
        {
            perror("socketcan: read");
        }
        return false;
    }

    if (n < (ssize_t)sizeof(struct can_frame))
    {
        return false;
    }

    frame->id  = cf.can_id;
    frame->dlc = cf.can_dlc;
    memcpy(frame->data, cf.data, sizeof(cf.data));
    return true;
}

/**
 * @brief  关闭并释放
 * @author camelliahm
 */
void socketcan_Close(void)
{
    if (s_fd >= 0)
    {
        close(s_fd);
        s_fd = -1;
    }
}
