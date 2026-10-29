#ifndef __NAV_MAP_H
#define __NAV_MAP_H

#include "nav.h"

/* 场地地图（智能搬运初赛赛场）：几何常量 + 默认路线表，纯数据无逻辑
   坐标系：原点 = 场地左上角（俯视同图 1），+X 右，+Y 下，单位 mm；与里程计坐标系不是一回事
   ⚠ 原料区 X / 二维码板 Y 现场随机，本表取中值。几何来源与换算见 README §6 */

/*---- 场地与十字车道 ----*/
#define NAV_MAP_FIELD_MM		(2400)						//场地边长
#define NAV_MAP_FIELD_HALF		(NAV_MAP_FIELD_MM / 2)		//1200 十字中心线
#define NAV_MAP_LANE_MM			(400)						//十字灰色行车道宽
#define NAV_MAP_LANE_HALF		(NAV_MAP_LANE_MM / 2)		//车道半宽

/*---- 四块淡黄区（禁止进入）----*/
#define NAV_MAP_YELLOW_MM		(450)						//边长；四块中心对称，夹出十字车道

/*---- 启停区（300×300，右上 / 右下角各一块）----*/
#define NAV_MAP_START_MM		(300)									//启停区边长
#define NAV_MAP_START1_X0		(NAV_MAP_FIELD_MM - NAV_MAP_START_MM)	//2100
#define NAV_MAP_START1_Y0		(0)										//启停区1 上边 Y（贴场地顶边）
#define NAV_MAP_START2_Y0		(NAV_MAP_FIELD_MM - NAV_MAP_START_MM)	//2100

/*---- 原料区（顶边电动转盘，φ300，台面高 80~100）----*/
#define NAV_MAP_MAT_D_MM		(300)						//转盘直径
#define NAV_MAP_MAT_CX_MIN		(1100)						//圆心 X 下限（现场随机）
#define NAV_MAP_MAT_CX_MAX		(1300)						//圆心 X 上限（现场随机）
#define NAV_MAP_MAT_CX_NOM		((NAV_MAP_MAT_CX_MIN + NAV_MAP_MAT_CX_MAX) / 2)	//原料区圆心 X 中值
#define NAV_MAP_MAT_CY_NOM		(0)							//圆心落在顶边线上

/*---- 暂存区（左边，580×150 竖向居中，3 个圆环）----*/
#define NAV_MAP_BUF_L_MM		(580)						//长，沿 Y
#define NAV_MAP_BUF_W_MM		(150)						//宽，沿 X
#define NAV_MAP_BUF_X0			(0)							//贴左边线
#define NAV_MAP_BUF_Y0			(NAV_MAP_FIELD_HALF - NAV_MAP_BUF_L_MM / 2)	//910
#define NAV_MAP_RING_PITCH_MM	(150)						//圆环间距（1→2→3 环）

/*---- 粗加工区（底边，580×150 横向居中，3 个圆环）----*/
#define NAV_MAP_ROU_X0			(NAV_MAP_FIELD_HALF - NAV_MAP_BUF_L_MM / 2)	//910
#define NAV_MAP_ROU_Y0			(NAV_MAP_FIELD_MM - NAV_MAP_BUF_W_MM)		//2250

/*---- 二维码板（右边墙，A4 横放，QR 80×80 居中）----*/
#define NAV_MAP_QR_Y_MIN		(1100)						//板心 Y 下限（现场随机）
#define NAV_MAP_QR_Y_MAX		(1300)						//板心 Y 上限（现场随机）
#define NAV_MAP_QR_Y_NOM		(NAV_MAP_FIELD_HALF)		//二维码板板心 Y 中值
#define NAV_MAP_QR_MM			(80)						//二维码边长

/*---- 障碍物与物料（摘自命题要求的物理尺寸）----*/
#define NAV_MAP_OBST_D_MM		(50)						//黑色模拟障碍物 φ50×100
#define NAV_MAP_OBST_H_MM		(100)						//障碍物高度
#define NAV_MAP_ITEM_D_MM		(80)						//物料最大尺寸
#define NAV_MAP_ITEM_M_G		(100)						//物料最大质量(g)

/*---- 停靠点：机器人中心应到达的场地坐标 ----*/
#define NAV_MAP_DOCK_HALF_MM	(150)						//车体半宽；停靠时车身外缘贴区域边界

#define NAV_MAP_CROSS_X			NAV_MAP_FIELD_HALF		//1200 十字交点
#define NAV_MAP_CROSS_Y			NAV_MAP_FIELD_HALF		//十字交点 Y

#define NAV_MAP_DOCK_START1_X	(NAV_MAP_START1_X0 + NAV_MAP_START_MM / 2)	//2250
#define NAV_MAP_DOCK_START1_Y	(NAV_MAP_START1_Y0 + NAV_MAP_START_MM / 2)	//150
#define NAV_MAP_DOCK_START2_Y	(NAV_MAP_START2_Y0 + NAV_MAP_START_MM / 2)	//2250

#define NAV_MAP_DOCK_QR_X		(NAV_MAP_FIELD_MM - NAV_MAP_DOCK_HALF_MM)	//2250
#define NAV_MAP_DOCK_QR_Y		NAV_MAP_QR_Y_NOM							//1200

#define NAV_MAP_DOCK_MAT_X		NAV_MAP_MAT_CX_NOM							//1200
#define NAV_MAP_DOCK_MAT_Y		(NAV_MAP_MAT_D_MM / 2 + NAV_MAP_DOCK_HALF_MM)	//300

#define NAV_MAP_DOCK_ROU_X		NAV_MAP_CROSS_X								//1200
#define NAV_MAP_DOCK_ROU_Y		(NAV_MAP_ROU_Y0 - NAV_MAP_DOCK_HALF_MM)		//2100

#define NAV_MAP_DOCK_BUF_X		(NAV_MAP_BUF_W_MM + NAV_MAP_DOCK_HALF_MM)	//300
#define NAV_MAP_DOCK_BUF_Y		NAV_MAP_CROSS_Y								//1200

/*---- 航段量纲换算（供下表引用）----*/
#define NAV_MAP_TURN_90			(900)						//90.0 度，0.1 度单位，正 = 逆时针
#define NAV_MAP_TURN_180		(1800)						//180.0 度

#define NAV_MAP_SPEED_LANE		(300)						//车道直行/横移速度(mm/s)
#define NAV_MAP_SPEED_DOCK		(150)						//进位段速度(mm/s)

#define NAV_MAP_DWELL_QR_MS		(800)						//读二维码 + 任务码上屏

/*---- 目标区位置（圆环）横移 ----*/
/* 车停在区域入口停靠点时正对哪个位号、位号增大朝车体哪一侧，取决于现场两区的摆位与编号方向 */
#define NAV_MAP_RING_DOCK_NUM	(2)							//入口停靠点正对的位号(1~3)  ⚠ 占位值，待实测
#define NAV_MAP_RING_DIR		(-1)						//位号 +1 时的车体横移方向：+1 = 右移，−1 = 左移  ⚠ 占位值，待实测

/*---- 默认路线 ----------------------------------------*/

/* 初赛全程：启停区 1 → 读码 → 第一批（原料区取 / 粗加工区放 / 取回 / 暂存区放）
   → 回原料区取第二批（粗加工区放 / 取回 / 暂存区码垛）→ 启停区 1
   位号一律由任务码给出：NAV_SEG_RING 取 task[组][件]，NAV_SEG_STACK 取第一批同色物料的位
   段值全取上表常量，行尾注是段末机器人中心的场地坐标
   ⚠ 前提：起点 = 启停区 1 中心、机头朝 +X；转向正 = 逆时针（实车反向则 TURN 段取负）
   末段必须是 NAV_SEG_END */
static const NavSeg_t s_defaultMap[] = {	//地图
	/* 启停区 1 → 二维码板：读取任务码 */
	{ NAV_SEG_STRAFE,  (int16_t)(NAV_MAP_DOCK_QR_Y - NAV_MAP_DOCK_START1_Y), NAV_MAP_SPEED_LANE, 0 },	//01 右移 1050 → (2250,1200)，机头正对二维码板
	{ NAV_SEG_DWELL,   NAV_MAP_DWELL_QR_MS, 0, 0 },	//02 停车读码 + 任务码上屏
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_QR_X), NAV_MAP_SPEED_LANE, 0 },	//03 后退 1050 → (1200,1200) 十字
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//04 左转 90° → 机头朝 -Y
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_DOCK, 0 },	//05 前进 900 → (1200,300) 正对原料区

	/* 第一批：原料区取料（任务码第 1 组） */
	{ NAV_SEG_HOME,    0, 0, NAV_HOME_TIMEOUT_MS },	//06 升降轴回零（装载的前置条件）
	{ NAV_SEG_ALIGN,   0, 0, NAV_ALIGN_TIMEOUT_MS },	//07 视觉对准（视觉模块 1）
	{ NAV_SEG_LOAD,    (int16_t)NAV_TASK_G_ORDER1, 0, NAV_LOAD_TIMEOUT_MS },	//08 三轮装载，每轮等色 → 对准 → 抓取

	/* 粗加工区：放第一批（任务码第 2 组） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//09 掉头 → 机头朝 +Y
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_LANE, 0 },	//10 前进 1800 → (1200,2100) 正对粗加工区
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//11 横移到第 1 件的位号
	{ NAV_SEG_UNLOAD,  1, 0, NAV_UNLOAD_TIMEOUT_MS },	//12 卸第 1 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//13 横移到第 2 件的位号
	{ NAV_SEG_UNLOAD,  2, 0, NAV_UNLOAD_TIMEOUT_MS },	//14 卸第 2 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//15 横移到第 3 件的位号
	{ NAV_SEG_UNLOAD,  3, 0, NAV_UNLOAD_TIMEOUT_MS },	//16 卸第 3 槽

	/* 粗加工区：取回第一批（同一组位号，按序装回物料盘） */
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//17 横移到第 1 件的位号
	{ NAV_SEG_RELOAD,  1, 0, NAV_RELOAD_TIMEOUT_MS },	//18 取回第 1 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//19 横移到第 2 件的位号
	{ NAV_SEG_RELOAD,  2, 0, NAV_RELOAD_TIMEOUT_MS },	//20 取回第 2 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//21 横移到第 3 件的位号
	{ NAV_SEG_RELOAD,  3, 0, NAV_RELOAD_TIMEOUT_MS },	//22 取回第 3 件

	/* 暂存区：放第一批（任务码第 2 组、与粗加工区同号） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//23 掉头 → 机头朝 -Y
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_CROSS_Y), NAV_MAP_SPEED_LANE, 0 },	//24 前进 900 → (1200,1200)
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//25 左转 90° → 机头朝 -X
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_DOCK, 0 },	//26 前进 900 → (300,1200) 正对暂存区
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//27 横移到第 1 件的位号
	{ NAV_SEG_UNLOAD,  1, 0, NAV_UNLOAD_TIMEOUT_MS },	//28 卸第 1 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//29 横移到第 2 件的位号
	{ NAV_SEG_UNLOAD,  2, 0, NAV_UNLOAD_TIMEOUT_MS },	//30 卸第 2 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS1 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//31 横移到第 3 件的位号
	{ NAV_SEG_UNLOAD,  3, 0, NAV_UNLOAD_TIMEOUT_MS },	//32 卸第 3 槽

	/* 回原料区：取第二批（任务码第 3 组） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//33 掉头 → 机头朝 +X
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_LANE, 0 },	//34 前进 900 → (1200,1200)
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//35 左转 90° → 机头朝 -Y
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_CROSS_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_DOCK, 0 },	//36 前进 900 → (1200,300) 正对原料区
	{ NAV_SEG_HOME,    0, 0, NAV_HOME_TIMEOUT_MS },	//37 升降轴回零（已回零则本段秒过）
	{ NAV_SEG_ALIGN,   0, 0, NAV_ALIGN_TIMEOUT_MS },	//38 视觉对准
	{ NAV_SEG_LOAD,    (int16_t)NAV_TASK_G_ORDER2, 0, NAV_LOAD_TIMEOUT_MS },	//39 三轮装载（第二批顺序）

	/* 粗加工区：放第二批（任务码第 4 组） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//40 掉头 → 机头朝 +Y
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_DOCK_MAT_Y), NAV_MAP_SPEED_LANE, 0 },	//41 前进 1800 → (1200,2100) 正对粗加工区
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//42 横移到第 1 件的位号
	{ NAV_SEG_UNLOAD,  1, 0, NAV_UNLOAD_TIMEOUT_MS },	//43 卸第 1 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//44 横移到第 2 件的位号
	{ NAV_SEG_UNLOAD,  2, 0, NAV_UNLOAD_TIMEOUT_MS },	//45 卸第 2 槽
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//46 横移到第 3 件的位号
	{ NAV_SEG_UNLOAD,  3, 0, NAV_UNLOAD_TIMEOUT_MS },	//47 卸第 3 槽

	/* 粗加工区：取回第二批（同一组位号） */
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 0), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//48 横移到第 1 件的位号
	{ NAV_SEG_RELOAD,  1, 0, NAV_RELOAD_TIMEOUT_MS },	//49 取回第 1 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 1), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//50 横移到第 2 件的位号
	{ NAV_SEG_RELOAD,  2, 0, NAV_RELOAD_TIMEOUT_MS },	//51 取回第 2 件
	{ NAV_SEG_RING,    (int16_t)(NAV_TASK_G_POS2 * 10 + 2), NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//52 横移到第 3 件的位号
	{ NAV_SEG_RELOAD,  3, 0, NAV_RELOAD_TIMEOUT_MS },	//53 取回第 3 件

	/* 暂存区：第二批码垛（叠到第一批同色物料之上） */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//54 掉头 → 机头朝 -Y
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_ROU_Y - NAV_MAP_CROSS_Y), NAV_MAP_SPEED_LANE, 0 },	//55 前进 900 → (1200,1200)
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//56 左转 90° → 机头朝 -X
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_CROSS_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_DOCK, 0 },	//57 前进 900 → (300,1200) 正对暂存区
	{ NAV_SEG_STACK,   0, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//58 横移到第 1 件的码垛位
	{ NAV_SEG_STACKED, 1, 0, NAV_UNLOAD_TIMEOUT_MS },	//59 码垛第 1 槽
	{ NAV_SEG_STACK,   1, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//60 横移到第 2 件的码垛位
	{ NAV_SEG_STACKED, 2, 0, NAV_UNLOAD_TIMEOUT_MS },	//61 码垛第 2 槽
	{ NAV_SEG_STACK,   2, NAV_MAP_SPEED_DOCK, NAV_RING_TIMEOUT_MS },	//62 横移到第 3 件的码垛位
	{ NAV_SEG_STACKED, 3, 0, NAV_UNLOAD_TIMEOUT_MS },	//63 码垛第 3 槽

	/* 回启停区 1 */
	{ NAV_SEG_TURN,    NAV_MAP_TURN_180, 0, NAV_TURN_TIMEOUT_MS },	//64 掉头 → 机头朝 +X
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_X - NAV_MAP_DOCK_BUF_X), NAV_MAP_SPEED_LANE, 0 },	//65 前进 1950 → (2250,1200)
	{ NAV_SEG_TURN,    NAV_MAP_TURN_90, 0, NAV_TURN_TIMEOUT_MS },	//66 左转 90° → 机头朝 -Y
	{ NAV_SEG_FORWARD, (int16_t)(NAV_MAP_DOCK_QR_Y - NAV_MAP_DOCK_START1_Y), NAV_MAP_SPEED_DOCK, 0 },	//67 前进 1050 → (2250,150) 回到启停区 1
	{ NAV_SEG_END,     0, 0, 0 },	//68 终点
};

#define NAV_DEFAULT_NUM		((uint16_t)(sizeof(s_defaultMap) / sizeof(s_defaultMap[0])))	//航段数

#endif
