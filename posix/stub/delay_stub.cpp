/* delay_stub.c —— POSIX 模拟层：delay.c 空壳
 * 真实实现见 DRIVERS/delay.c（依赖 SysTick），本机不编译它。 */
#include "delay.h"

void delay_us(__IO uint16_t us) { (void)us; }
void delay_ms(__IO uint16_t ms) { (void)ms; }
