/* input.c 的 Linux 实现（serial 底座）
 * 替代 STM32 的 DEVICE/input.c（三路 USART + DMA + IDLE 中断 + 摁键 GPIO）。
 *
 * 与 STM32 版差异：
 *   - 物理收：USART DMA+IDLE 中断 → serial 轮询读（input_Step 里搬字节进环）
 *   - 物理发：USART_SendData → serial_Write
 *   - 摁键：GPIO 读 → 本机无键，恒不按下（自启动倒计时不推进，与 stub 一致）
 *   其余（环形缓冲、在线判定、摁键消抖逻辑）与 STM32 版一字不差。 */
#include "input.h"
#include "serial.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* 三路串口设备节点（M3 占位，按 LBC 实际枚举再改） */
static const char *const s_dev[INPUT_NUM] = {
    "/dev/ttyUSB0",   /* INPUT_VISION1 → 视觉模块 1 */
    "/dev/ttyUSB1",   /* INPUT_VISION2 → 视觉模块 2 */
    "/dev/ttyUSB2",   /* INPUT_SCREEN  → 串口屏 */
};

#define INPUT_RING_MASK     (INPUT_RING_SIZE - 1u)   /* 环形缓冲下标掩码 */

static int s_fd[INPUT_NUM];              /* 各通道串口句柄，<0 = 未打开 */

static uint8_t  s_ring[INPUT_NUM][INPUT_RING_SIZE]; /* 每通道环形缓冲 */
static volatile uint16_t s_head[INPUT_NUM];         /* 环形缓冲写指针（接收侧） */
static volatile uint16_t s_tail[INPUT_NUM];         /* 环形缓冲读指针（任务侧） */
static volatile uint16_t s_drop[INPUT_NUM];         /* 环满丢弃的字节数 */
static volatile uint16_t s_byte[INPUT_NUM];         /* 累计到达字节数 */

static InputSlot_t s_slot[INPUT_NUM];    /* 每通道对外的字节级状态 */

/*---- 摁键（自启动） ------------------------------------------------*/
/* 本机无物理摁键，恒不按下：自启动倒计时不推进。保留结构以便将来接 GPIO/键盘。 */
static struct {
    uint8_t  stable;     /* 消抖后的稳定电平：1 = 按下 */
    uint8_t  cnt;        /* 与 stable 相反的电平已持续的扫描次数 */
    uint8_t  wait;       /* 1 = 倒计时中，到点投启动事件 */
    uint32_t pressTick;  /* 按下被确认的时刻 */
} s_key;

/**
 * @brief  读摁键电平（本机无键，恒 0）
 * @author camelliahm
 */
static uint8_t input_KeyDown(void)
{
    return 0u;
}

/**
 * @brief  摁键扫描：消抖 → 按下即起倒计时 → 到点投 EV_ENABLE + EV_START
 * @note   本机无键，input_KeyDown 恒 0，此函数实际不触发，但逻辑与 STM32 版一致
 * @author camelliahm
 */
static void input_KeyStep(void)
{
    uint8_t down = input_KeyDown();

    if (down != s_key.stable)
    {
        s_key.cnt++;

        if (s_key.cnt >= KEY_DEBOUNCE_CNT)
        {
            s_key.stable = down;
            s_key.cnt    = 0u;

            if ((down != 0u) &&
                ((chassis.state == CHS_IDLE) || (chassis.state == CHS_READY)))
            {
                s_key.wait      = 1u;
                s_key.pressTick = xTaskGetTickCount();
            }
        }
    }
    else
    {
        s_key.cnt = 0u;
    }

    if ((s_key.wait != 0u) &&
        ((xTaskGetTickCount() - s_key.pressTick) >= pdMS_TO_TICKS(KEY_START_DELAY_MS)))
    {
        s_key.wait = 0u;

        if (chassis.state == CHS_IDLE)
        {
            contract_PostEvent(EV_ENABLE, 0);
            contract_PostEvent(EV_START, 0);
        }
        else if (chassis.state == CHS_READY)
        {
            contract_PostEvent(EV_START, 0);
        }
    }
}

/*---- 周期处理（任务上下文） ----------------------------------------*/

/**
 * @brief  从环形缓冲取一个字节
 * @retval true = 取到
 * @author camelliahm
 */
static bool input_RingPop(InputCh_t ch, uint8_t *b)
{
    if (s_tail[ch] == s_head[ch])
    {
        return false;
    }

    *b = s_ring[ch][s_tail[ch]];
    s_tail[ch] = (uint16_t)((s_tail[ch] + 1u) & INPUT_RING_MASK);
    return true;
}

/**
 * @brief  把串口新到字节搬进环形缓冲（替代 STM32 的 IDLE 中断搬运）
 * @param  ch 通道号
 * @author camelliahm
 */
static void input_Pump(InputCh_t ch)
{
    uint8_t buf[64];
    int n;
    int i;

    if (s_fd[ch] < 0)
    {
        return;
    }

    n = serial_Read(s_fd[ch], buf, (int)sizeof(buf));
    for (i = 0; i < n; i++)
    {
        uint16_t next = (uint16_t)((s_head[ch] + 1u) & INPUT_RING_MASK);

        if (next == s_tail[ch])
        {
            s_drop[ch]++;   /* 环满：丢新字节 */
            continue;
        }

        s_ring[ch][s_head[ch]] = buf[i];
        s_head[ch] = next;
        s_byte[ch] = (uint16_t)(s_byte[ch] + 1u);
    }
}

/**
 * @brief  刷新通道在线标志与丢弃计数
 * @param  ch 通道号
 * @note   在线 = 最近收到过字节
 * @author camelliahm
 */
static void input_RefreshState(InputCh_t ch)
{
    uint16_t byte = s_byte[ch];

    if (byte != s_slot[ch].rxCnt)
    {
        s_slot[ch].lastTick = xTaskGetTickCount();
    }
    s_slot[ch].rxCnt   = byte;
    s_slot[ch].lockCnt = s_drop[ch];

    if ((xTaskGetTickCount() - s_slot[ch].lastTick) <= pdMS_TO_TICKS(INPUT_TIMEOUT_MS))
    {
        s_slot[ch].online = 1u;
    }
    else
    {
        s_slot[ch].online = 0u;
    }
}

/*---- 对外接口 ----------------------------------------------------*/

/**
 * @brief  输入汇聚层初始化
 * @note   打开三路串口（失败静默降级）；复位通道状态与摁键
 * @author camelliahm
 */
void input_Init(void)
{
    uint8_t i;

    for (i = 0u; i < INPUT_NUM; i++)
    {
        s_fd[i] = serial_Open(s_dev[i], INPUT_BAUD);
        s_head[i] = 0u;
        s_tail[i] = 0u;
        s_drop[i] = 0u;
        s_byte[i] = 0u;

        s_slot[i].online   = 0u;
        s_slot[i].lastTick = 0u;
        s_slot[i].rxCnt    = 0u;
        s_slot[i].lockCnt  = 0u;
    }

    s_key.stable    = input_KeyDown();
    s_key.cnt       = 0u;
    s_key.wait      = 0u;
    s_key.pressTick = 0u;
}

/**
 * @brief  周期处理：搬字节进环、刷新在线标志、扫描摁键
 * @note   必须每周期调用（摁键消抖与自启动倒计时按调用推进）
 * @author camelliahm
 */
void input_Step(void)
{
    uint8_t i;

    for (i = 0u; i < INPUT_NUM; i++)
    {
        input_Pump((InputCh_t)i);
        input_RefreshState((InputCh_t)i);
    }

    input_KeyStep();
}

/**
 * @brief  取走原始字节
 * @param  ch  通道号
 * @param  dst 目标缓冲
 * @param  max 最多拷贝字节数
 * @retval 实际拷贝的字节数；0 = 无数据或参数非法
 * @author camelliahm
 */
uint16_t input_Read(InputCh_t ch, uint8_t *dst, uint16_t max)
{
    uint16_t n = 0u;

    if ((ch >= INPUT_NUM) || (dst == NULL))
    {
        return 0u;
    }

    while ((n < max) && input_RingPop(ch, &dst[n]))
    {
        n++;
    }

    return n;
}

/**
 * @brief  发送
 * @param  ch  通道号
 * @param  src 数据起始
 * @param  len 长度
 * @retval 实际发出的字节数；0 = 参数非法或未打开
 * @author camelliahm
 */
uint16_t input_Write(InputCh_t ch, const uint8_t *src, uint16_t len)
{
    if ((ch >= INPUT_NUM) || (src == NULL))
    {
        return 0u;
    }

    if (s_fd[ch] < 0)
    {
        return 0u;
    }

    return (uint16_t)serial_Write(s_fd[ch], src, (int)len);
}

/**
 * @brief  只读槽位
 * @retval 槽位指针；通道号非法时返回 NULL
 * @author camelliahm
 */
const InputSlot_t *input_GetSlot(InputCh_t ch)
{
    return (ch < INPUT_NUM) ? &s_slot[ch] : NULL;
}
