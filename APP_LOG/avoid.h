#ifndef __AVOID_H
#define __AVOID_H

#include "contract.h"

/* 避障层：读视觉2 的障碍横向偏移，在 nav_Step() 之后追加横向让行分量。
 * 只有"左右偏移"，没有纵向距离 —— 收到障碍帧即视为障碍进入警戒区，
 * 帧超时未刷新即视为障碍消失。详见 README.md */

/*---- 参数 --------------------------------------------------------*/

//让行分量幅值(mm/s)：叠加在 nav 本段速度上，仍受逆运动学的轮速上限二次约束
#define AVOID_STRAFE_VEL		(150)
//障碍帧有效时长(ms)：超过此时长没有新帧即视为障碍消失；须大于视觉发帧周期
#define AVOID_HOLD_MS			(500u)
//偏移死区(cm)：|x| 小于此值视为障碍在正前方，固定往右绕，避免零点两侧来回摆
#define AVOID_DEADBAND_CM		(5)

//初始化：清避障层状态（由 contract_Init() 调用）
void avoid_Init(void);

//周期处理：在 nav_Step() 之后调用；只在 RUNNING 且本段属平移族时改写黑板 chassis.vx
void avoid_Step(void);

#endif
