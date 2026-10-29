/* input_stub.c —— POSIX 模拟层：input.c 空壳
 * 真实实现见 DEVICE/input.c（三路 USART + DMA + IDLE 中断 + 摁键 GPIO），本机不编译它。
 * 本机无串口硬件，字节管道恒为空；摁键不接，自启动倒计时不推进。 */
#include "input.h"

static InputSlot_t s_slot[INPUT_NUM];   /* 每通道对外的字节级状态 */

void input_Init(void)
{
    uint8_t i;

    for (i = 0u; i < INPUT_NUM; i++)
    {
        s_slot[i].online  = 0u;
        s_slot[i].lastTick = 0u;
        s_slot[i].rxCnt   = 0u;
        s_slot[i].lockCnt = 0u;
    }
}

void input_Step(void)
{
    /* 无串口、无摁键：空操作 */
}

uint16_t input_Read(InputCh_t ch, uint8_t *dst, uint16_t max)
{
    (void)ch;
    (void)dst;
    (void)max;
    return 0u;   /* 恒无数据 */
}

uint16_t input_Write(InputCh_t ch, const uint8_t *src, uint16_t len)
{
    (void)ch;
    (void)src;
    return len;   /* 假装发出去了 */
}

const InputSlot_t *input_GetSlot(InputCh_t ch)
{
    return (ch < INPUT_NUM) ? &s_slot[ch] : NULL;
}
