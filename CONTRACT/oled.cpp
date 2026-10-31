/* oled.cpp —— OLED 状态屏（Linux 下空实现）
 * Linux 上无需 SSD1306 软 I2C 屏，状态显示由串口屏（IFACE/screen.cpp）承担，故全部空操作。 */
#include "oled.h"

void oled_Init(void)      {}
void oled_Clear(void)     {}
void oled_Refresh(void)   {}
void oled_ShowString(uint8_t line, uint8_t col, const char *str) { (void)line; (void)col; (void)str; }
void oled_ShowNum(uint8_t line, uint8_t col, int32_t num, uint8_t len) { (void)line; (void)col; (void)num; (void)len; }
void oled_ShowStatus(void) {}
void oled_Step(void)      {}
