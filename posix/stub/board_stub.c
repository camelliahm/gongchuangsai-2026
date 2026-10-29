/* board_stub.c —— POSIX 模拟层：board.c 空壳
 * 真实实现见 BSP/board.c（STM32 时钟/GPIO/NVIC 配置），本机不编译它。 */
#include "board.h"

void nvic_init(void)   {}
void clock_init(void)  {}
void usart_init(void)  {}
void board_init(void)  {}
