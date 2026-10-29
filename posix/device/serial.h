#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>
#include <stdbool.h>

/* serial 底座：把 Linux 串口（/dev/tty*）封装成与 input.h 对齐的最小接口。
 * 用 termios 配 115200 8N1，非阻塞读，替换 STM32 的 USART + DMA + IDLE 中断。
 * 多实例：每个通道一个句柄（fd），支持三路串口独立收发。 */

/* 打开设备并配好 termios（非阻塞），返回句柄（fd）；<0 表示失败 */
int serial_Open(const char *dev, uint32_t baud);

/* 非阻塞读：读到的字节数；无数据返回 0 */
int serial_Read(int fd, uint8_t *buf, int max);

/* 写：返回实际写入字节数 */
int serial_Write(int fd, const uint8_t *buf, int len);

/* 关闭 */
void serial_Close(int fd);

#endif
