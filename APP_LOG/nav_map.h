#ifndef __NAV_MAP_H
#define __NAV_MAP_H

#include "nav.h"
#include "nav_geom.h"

/*---- 启停区段角色（NAV_SEG_DOCK 的 value）----*/
#define NAV_DOCK_TO_QR		(0)		//起点 → 读码列
#define NAV_DOCK_TO_START	(1)		//原料区行 → 起点

/*---- 默认路线 ----------------------------------------*/

/* 初赛全程：读码 → 第一批（原料区取 / 粗加工区放 / 取回 / 暂存区放）
   → 回原料区取第二批（粗加工区放 / 取回 / 暂存区码垛）→ 回抽签选定的启停区
   作业姿态：车头与区域长边平行、区域位于车体右侧，换位靠沿机头前进，故跨区全走外围通道
   位号一律由任务码给出：NAV_SEG_RING 取 task[组][件]，NAV_SEG_STACK 取第一批同色物料的位
   段值全取 nav_geom.h 的常量，行尾注是段末机器人中心的场地坐标；转向正 = 逆时针
   首末两段是 NAV_SEG_DOCK，位移随抽签起点在运行时折算
   末段必须是 NAV_SEG_END */
static const NavSeg_t s_defaultMap[] = {	//地图
	/* 启停区 → 二维码板：读取任务码 */
	{ NAV_SEG_DOCK,    NAV_DOCK_TO_QR, NAV_MAP_SPEED_LANE, 0 },	//01 横移 1050 → (2250,1200)，机头正对二维码板
	{ NAV_SEG_DWELL,   NAV_MAP_DWELL_QR_MS, 0, 0 },	//02 停车读码 + 任务码上屏
	{ NAV_SEG_STRAFE,  (int16_t)(NAV_MAP_DOCK_QR_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_LANE, 0 },	//03 左移 829 → (2250,2029) 原料区行

	/* 第一批：原料区取料（任务码第 1 组） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//04 掉头 → 机头朝 -X，原料区在车右
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_X - NAV_MAP_DOCK_MAT_X), NAV_MAP_SPEED_LANE, 0 },	//05 前进 1050 → (1200,2029)
	{ NAV_SEG_HOME,    0, 0, NAV_HOME_TIMEOUT_MS },	//06 零点确认（装载的前置条件）
	{ NAV_SEG_ALIGN,   0, 0, NAV_ALIGN_TIMEOUT_MS },	//07 视觉对准（视觉模块 1）
	{ NAV_SEG_LOAD,    (int16_t)NAV_TASK_G_ORDER1, 0, NAV_LOAD_TIMEOUT_MS },	//08 三轮装载，每轮等色 → 对准 → 抓取

	/* 粗加工区：放第一批（任务码第 2 组） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//09 掉头 → 机头朝 +X，粗加工区在车右
	{ NAV_SEG_STRAFE,  (int16_t)(NAV_MAP_DOCK_MAT_Y - NAV_MAP_DOCK_ROU_Y), NAV_MAP_SPEED_LANE, 0 },	//10 右移 1733 → (1200,296)
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//11 前进到第 1 件的位号
	{ NAV_SEG_UNLOAD,  1, 0, NAV_UNLOAD_TIMEOUT_MS },	//12 卸第 1 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//13 前进到第 2 件的位号
	{ NAV_SEG_UNLOAD,  2, 0, NAV_UNLOAD_TIMEOUT_MS },	//14 卸第 2 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//15 前进到第 3 件的位号
	{ NAV_SEG_UNLOAD,  3, 0, NAV_UNLOAD_TIMEOUT_MS },	//16 卸第 3 槽

	/* 粗加工区：取回第一批（同一组位号，按序装回物料盘） */
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//17 前进到第 1 件的位号
	{ NAV_SEG_RELOAD,  1, 0, NAV_RELOAD_TIMEOUT_MS },	//18 取回第 1 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//19 前进到第 2 件的位号
	{ NAV_SEG_RELOAD,  2, 0, NAV_RELOAD_TIMEOUT_MS },	//20 取回第 2 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//21 前进到第 3 件的位号
	{ NAV_SEG_RELOAD,  3, 0, NAV_RELOAD_TIMEOUT_MS },	//22 取回第 3 件
	{ NAV_SEG_RING_BASE, 0, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//23 回 2 号位 → (1200,296)

	/* 暂存区：放第一批（任务码第 2 组、与粗加工区同号） */
	{ NAV_SEG_TURN,    -NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//24 右转 90° → 机头朝 -Y，暂存区在车右
	{ NAV_SEG_STRAFE,  (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_LANE, 0 },	//25 右移 904 → (296,296) 暂存区列
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_DOCK_BUF_Y), NAV_MAP_SPEED_LANE, 0 },	//26 后退 904 → (296,1200)
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//27 前进到第 1 件的位号
	{ NAV_SEG_UNLOAD,  1, 0, NAV_UNLOAD_TIMEOUT_MS },	//28 卸第 1 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//29 前进到第 2 件的位号
	{ NAV_SEG_UNLOAD,  2, 0, NAV_UNLOAD_TIMEOUT_MS },	//30 卸第 2 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//31 前进到第 3 件的位号
	{ NAV_SEG_UNLOAD,  3, 0, NAV_UNLOAD_TIMEOUT_MS },	//32 卸第 3 槽
	{ NAV_SEG_RING_BASE, 0, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//33 回 2 号位 → (296,1200)

	/* 回原料区：取第二批（任务码第 3 组） */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_BUF_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_LANE, 0 },	//34 后退 829 → (296,2029) 原料区行
	{ NAV_SEG_TURN,    -NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//35 右转 90° → 机头朝 -X
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_BUF_X - NAV_MAP_DOCK_MAT_X), NAV_MAP_SPEED_LANE, 0 },	//36 后退 904 → (1200,2029) 原料区在车右
	{ NAV_SEG_HOME,    0, 0, NAV_HOME_TIMEOUT_MS },	//37 零点确认（已标零则本段秒过）
	{ NAV_SEG_ALIGN,   0, 0, NAV_ALIGN_TIMEOUT_MS },	//38 视觉对准
	{ NAV_SEG_LOAD,    (int16_t)NAV_TASK_G_ORDER2, 0, NAV_LOAD_TIMEOUT_MS },	//39 三轮装载（第二批顺序）

	/* 粗加工区：放第二批（任务码第 4 组） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//40 掉头 → 机头朝 +X，粗加工区在车右
	{ NAV_SEG_STRAFE,  (int16_t)(NAV_MAP_DOCK_MAT_Y - NAV_MAP_DOCK_ROU_Y), NAV_MAP_SPEED_LANE, 0 },	//41 右移 1733 → (1200,296)
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//42 前进到第 1 件的位号
	{ NAV_SEG_UNLOAD,  1, 0, NAV_UNLOAD_TIMEOUT_MS },	//43 卸第 1 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//44 前进到第 2 件的位号
	{ NAV_SEG_UNLOAD,  2, 0, NAV_UNLOAD_TIMEOUT_MS },	//45 卸第 2 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//46 前进到第 3 件的位号
	{ NAV_SEG_UNLOAD,  3, 0, NAV_UNLOAD_TIMEOUT_MS },	//47 卸第 3 槽

	/* 粗加工区：取回第二批（同一组位号） */
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//48 前进到第 1 件的位号
	{ NAV_SEG_RELOAD,  1, 0, NAV_RELOAD_TIMEOUT_MS },	//49 取回第 1 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//50 前进到第 2 件的位号
	{ NAV_SEG_RELOAD,  2, 0, NAV_RELOAD_TIMEOUT_MS },	//51 取回第 2 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//52 前进到第 3 件的位号
	{ NAV_SEG_RELOAD,  3, 0, NAV_RELOAD_TIMEOUT_MS },	//53 取回第 3 件
	{ NAV_SEG_RING_BASE, 0, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//54 回 2 号位 → (1200,296)

	/* 暂存区：第二批码垛（叠到第一批同色物料之上） */
	{ NAV_SEG_TURN,    -NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//55 右转 90° → 机头朝 -Y，暂存区在车右
	{ NAV_SEG_STRAFE,  (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_LANE, 0 },	//56 右移 904 → (296,296) 暂存区列
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_DOCK_BUF_Y), NAV_MAP_SPEED_LANE, 0 },	//57 后退 904 → (296,1200)
	{ NAV_SEG_STACK,   0, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//58 前进到第 1 件的码垛位
	{ NAV_SEG_STACKED, 1, 0, NAV_UNLOAD_TIMEOUT_MS },	//59 码垛第 1 槽
	{ NAV_SEG_STACK,   1, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//60 前进到第 2 件的码垛位
	{ NAV_SEG_STACKED, 2, 0, NAV_UNLOAD_TIMEOUT_MS },	//61 码垛第 2 槽
	{ NAV_SEG_STACK,   2, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//62 前进到第 3 件的码垛位
	{ NAV_SEG_STACKED, 3, 0, NAV_UNLOAD_TIMEOUT_MS },	//63 码垛第 3 槽
	{ NAV_SEG_RING_BASE, 0, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//64 回 2 号位 → (296,1200)

	/* 回抽签选定的启停区 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_BUF_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_LANE, 0 },	//65 后退 829 → (296,2029) 原料区行
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//66 左转 90° → 机头朝 +X
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_LANE, 0 },	//67 前进 1954 → (2250,2029)
	{ NAV_SEG_DOCK,    NAV_DOCK_TO_START, NAV_MAP_SPEED_DOCK, 0 },	//68 横移到抽签起点 → (2250,2250) 或 (2250,150)
	{ NAV_SEG_END,     0, 0, 0 },	//69 终点
};

#define NAV_DEFAULT_NUM		((uint16_t)(sizeof(s_defaultMap) / sizeof(s_defaultMap[0])))	//航段数

#endif
