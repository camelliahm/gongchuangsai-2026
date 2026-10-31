/* oled_stub.c —— POSIX 模拟层：oled.c 空壳
 * 真实实现见 CONTRACT/oled.c（SSD1306 软 I2C），本机不编译它。
 * OLED 只是给人看，本机模拟无需屏，全部空操作。 */
#include "oled.h"

void oled_Init(void)      {}
void oled_Clear(void)     {}
void oled_Refresh(void)   {}
void oled_ShowString(uint8_t line, uint8_t col, const char *str) { (void)line; (void)col; (void)str; }
void oled_ShowNum(uint8_t line, uint8_t col, int32_t num, uint8_t len) { (void)line; (void)col; (void)num; (void)len; }
void oled_ShowStatus(void) {}
void oled_Step(void)      {}
