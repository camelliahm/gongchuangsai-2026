#ifndef __NAV_MAP_H
#define __NAV_MAP_H

#include "nav.h"

/* 场地地图（智能搬运初赛赛场）：几何常量 + 默认路线表，纯数据无逻辑
   坐标系：原点 = 场地左上角（俯视同图 1），+X 右，+Y 下，单位 mm；与里程计坐标系不是一回事
   ⚠ 原料区 X / 二维码板 Y 现场随机，本表取中值。几何来源与换算见 README §6 */

/*---- 场地与十字车道 ----*/
#define NAV_MAP_FIELD_MM		(2400)						/* 场地边长 */
#define NAV_MAP_FIELD_HALF		(NAV_MAP_FIELD_MM / 2)		/* 1200：十字中心线 */
#define NAV_MAP_LANE_MM			(400)						/* 十字灰色行车道宽 */
#define NAV_MAP_LANE_HALF		(NAV_MAP_LANE_MM / 2)

/*---- 四块淡黄区（禁止进入）----*/
#define NAV_MAP_YELLOW_MM		(450)						/* 边长；四块中心对称，夹出十字车道 */

/*---- 启停区（300×300，右上 / 右下角各一块）----*/
#define NAV_MAP_START_MM		(300)
#define NAV_MAP_START1_X0		(NAV_MAP_FIELD_MM - NAV_MAP_START_MM)	/* 2100 */
#define NAV_MAP_START1_Y0		(0)
#define NAV_MAP_START2_Y0		(NAV_MAP_FIELD_MM - NAV_MAP_START_MM)	/* 2100 */

/*---- 原料区（顶边电动转盘，φ300，台面高 80~100）----*/
#define NAV_MAP_MAT_D_MM		(300)						/* 转盘直径 */
#define NAV_MAP_MAT_CX_MIN		(1100)						/* 圆心 X 下限（现场随机） */
#define NAV_MAP_MAT_CX_MAX		(1300)						/* 圆心 X 上限（现场随机） */
#define NAV_MAP_MAT_CX_NOM		((NAV_MAP_MAT_CX_MIN + NAV_MAP_MAT_CX_MAX) / 2)
#define NAV_MAP_MAT_CY_NOM		(0)							/* 圆心落在顶边线上 */

/*---- 暂存区（左边，580×150 竖向居中，3 个圆环）----*/
#define NAV_MAP_BUF_L_MM		(580)						/* 长，沿 Y */
#define NAV_MAP_BUF_W_MM		(150)						/* 宽，沿 X */
#define NAV_MAP_BUF_X0			(0)							/* 贴左边线 */
#define NAV_MAP_BUF_Y0			(NAV_MAP_FIELD_HALF - NAV_MAP_BUF_L_MM / 2)	/* 910 */
#define NAV_MAP_RING_PITCH_MM	(150)						/* 圆环间距（1→2→3 环） */

/*---- 粗加工区（底边，580×150 横向居中，3 个圆环）----*/
#define NAV_MAP_ROU_X0			(NAV_MAP_FIELD_HALF - NAV_MAP_BUF_L_MM / 2)	/* 910 */
#define NAV_MAP_ROU_Y0			(NAV_MAP_FIELD_MM - NAV_MAP_BUF_W_MM)		/* 2250 */

/*---- 二维码板（右边墙，A4 横放，QR 80×80 居中）----*/
#define NAV_MAP_QR_Y_MIN		(1100)						/* 板心 Y 下限（现场随机） */
#define NAV_MAP_QR_Y_MAX		(1300)						/* 板心 Y 上限（现场随机） */
#define NAV_MAP_QR_Y_NOM		(NAV_MAP_FIELD_HALF)
#define NAV_MAP_QR_MM			(80)						/* 二维码边长 */

/*---- 障碍物与物料（摘自命题要求的物理尺寸）----*/
#define NAV_MAP_OBST_D_MM		(50)						/* 黑色模拟障碍物 φ50×100 */
#define NAV_MAP_OBST_H_MM		(100)
#define NAV_MAP_ITEM_D_MM		(80)						/* 物料最大尺寸 */
#define NAV_MAP_ITEM_M_G		(100)						/* 物料最大质量(g) */

/*---- 停靠点：机器人中心应到达的场地坐标 ----*/
#define NAV_MAP_DOCK_HALF_MM	(150)						/* 车体半宽；停靠时车身外缘贴区域边界 */

#define NAV_MAP_CROSS_X			NAV_MAP_FIELD_HALF		/* 1200：十字交点 */
#define NAV_MAP_CROSS_Y			NAV_MAP_FIELD_HALF

#define NAV_MAP_DOCK_START1_X	(NAV_MAP_START1_X0 + NAV_MAP_START_MM / 2)	/* 2250 */
#define NAV_MAP_DOCK_START1_Y	(NAV_MAP_START1_Y0 + NAV_MAP_START_MM / 2)	/* 150 */
#define NAV_MAP_DOCK_START2_Y	(NAV_MAP_START2_Y0 + NAV_MAP_START_MM / 2)	/* 2250 */

#define NAV_MAP_DOCK_QR_X		(NAV_MAP_FIELD_MM - NAV_MAP_DOCK_HALF_MM)	/* 2250 */
#define NAV_MAP_DOCK_QR_Y		NAV_MAP_QR_Y_NOM							/* 1200 */

#define NAV_MAP_DOCK_MAT_X		NAV_MAP_MAT_CX_NOM							/* 1200 */
#define NAV_MAP_DOCK_MAT_Y		(NAV_MAP_MAT_D_MM / 2 + NAV_MAP_DOCK_HALF_MM)	/* 300 */

#define NAV_MAP_DOCK_ROU_X		NAV_MAP_CROSS_X								/* 1200 */
#define NAV_MAP_DOCK_ROU_Y		(NAV_MAP_ROU_Y0 - NAV_MAP_DOCK_HALF_MM)		/* 2100 */

#define NAV_MAP_DOCK_BUF_X		(NAV_MAP_BUF_W_MM + NAV_MAP_DOCK_HALF_MM)	/* 300 */
#define NAV_MAP_DOCK_BUF_Y		NAV_MAP_CROSS_Y								/* 1200 */

/*---- 航段量纲换算（供下表引用）----*/
#define NAV_MAP_TURN_90			(900)						/* 90.0 度，0.1 度单位，正 = 逆时针 */
#define NAV_MAP_TURN_180		(1800)						/* 180.0 度 */

#define NAV_MAP_SPEED_LANE		(300)						/* 车道直行/横移速度(mm/s) */
#define NAV_MAP_SPEED_DOCK		(150)						/* 进位段速度(mm/s) */

#define NAV_MAP_DWELL_QR_MS		(800)						/* 读二维码 + 任务码上屏 */
#define NAV_MAP_DWELL_GRAB_MS	(3000)						/* 抓取机构动作（占位，待机构确定） */
#define NAV_MAP_DWELL_PLACE_MS	(3000)						/* 放置机构动作（占位，待机构确定） */

/*---- 默认路线 ----------------------------------------*/

/* 初赛单程路线（启停区 1 → 读码 → 原料区 → 粗加工区 → 暂存区 → 启停区 1）
   段值全取上表常量，行注是段末机器人中心的场地坐标
   ⚠ 前提：起点 = 启停区 1 中心、机头朝 +X；转向正 = 逆时针（实车反向则 TURN 段取负）
   末段必须是 NAV_SEG_END */
static const NavSeg_t s_defaultMap[] = {
	/* ① 右移 1050 → (2250,1200)，机头正对右侧二维码板 */
	{ NAV_SEG_STRAFE,  (int16_t)(NAV_MAP_DOCK_QR_Y - NAV_MAP_DOCK_START1_Y), NAV_MAP_SPEED_LANE,  0 },
	/* ② 停车读二维码（视觉模块 2）并把任务码上串口屏 */
	{ NAV_SEG_DWELL,   NAV_MAP_DWELL_QR_MS, 0, 0 },
	/* ③ 后退 1050 → (1200,1200)，退到十字交点 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_QR_X), NAV_MAP_SPEED_LANE, 0 },
	/* ④ 左转 90° → 机头朝 -Y（场地顶边） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },
	/* ⑤ 前进 900 → (1200,300)，正对原料区转盘 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_DOCK, 0 },
	/* ⑥ 抓取第 1 批 3 个物料（每次 1 个，机构动作占位） */
	{ NAV_SEG_DWELL,   NAV_MAP_DWELL_GRAB_MS, 0, 0 },
	/* ⑦ 掉头 180° → 机头朝 +Y（场地底边） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },
	/* ⑧ 前进 1800 → (1200,2100)，正对粗加工区 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_LANE, 0 },
	/* ⑨ 按任务码顺序放入粗加工区 1~3 号圆环 */
	{ NAV_SEG_DWELL,   NAV_MAP_DWELL_PLACE_MS, 0, 0 },
	/* ⑩ 掉头 180° → 机头朝 -Y */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },
	/* ⑪ 前进 900 → (1200,1200)，回到十字交点 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_CROSS_Y), NAV_MAP_SPEED_LANE, 0 },
	/* ⑫ 左转 90° → 机头朝 -X（场地左边） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },
	/* ⑬ 前进 900 → (300,1200)，正对暂存区 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_DOCK, 0 },
	/* ⑭ 放入暂存区 1~3 号圆环 */
	{ NAV_SEG_DWELL,   NAV_MAP_DWELL_PLACE_MS, 0, 0 },
	/* ⑮ 掉头 180° → 机头朝 +X */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },
	/* ⑯ 前进 1950 → (2250,1200)，沿横向车道回到右端 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_LANE, 0 },
	/* ⑰ 左转 90° → 机头朝 -Y */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },
	/* ⑱ 前进 1050 → (2250,150)，回到启停区 1 */
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_Y - NAV_MAP_DOCK_START1_Y), NAV_MAP_SPEED_DOCK, 0 },
	{ NAV_SEG_END,     0, 0, 0 },
};

#define NAV_DEFAULT_NUM		((uint16_t)(sizeof(s_defaultMap) / sizeof(s_defaultMap[0])))

#endif
