/* board.cpp —— Linux 板级初始化空实现
 * 时钟 / GPIO / NVIC 在 Linux 上由内核接管，无需配置。 */
#include "board.h"

void nvic_init(void)   {}
void clock_init(void)  {}
void usart_init(void)  {}
void board_init(void)  {}
