#ifndef __NAV_PATH_H
#define __NAV_PATH_H

#include <stdint.h>

//可行驶图：节点 = 走廊上的关键点（作业区停靠点 / 拐角 / 十字中心），边 = 一段直走廊
//本模块只回答"经过哪些节点、多远、要转几次"，不含车体动作；航段展开在 nav 层完成

/*---- 节点编号（坐标见 nav_path.c，取自 nav_map.h 的停靠点）----*/
typedef enum {
	NAV_NODE_MAT = 0,	//原料区停靠点
	NAV_NODE_RT,		//右上角：原料区行与二维码列的交点
	NAV_NODE_QR,		//二维码读码点
	NAV_NODE_RB,		//右下角：粗加工区行与二维码列的交点
	NAV_NODE_ROU,		//粗加工区停靠点
	NAV_NODE_LB,		//左下角：粗加工区行与暂存区列的交点
	NAV_NODE_BUF,		//暂存区停靠点
	NAV_NODE_LT,		//左上角：原料区行与暂存区列的交点
	NAV_NODE_CROSS,		//十字中心
	NAV_NODE_NUM		//节点数
} NavNode_t;

/*---- 世界方向（与场地坐标系同向：+X → +Y 为逆时针）----*/
#define NAV_DIR_PX	(0u)	//+X
#define NAV_DIR_PY	(1u)	//+Y
#define NAV_DIR_NX	(2u)	//-X
#define NAV_DIR_NY	(3u)	//-Y
#define NAV_DIR_NUM	(4u)	//方向数

/*---- 参数 ----*/
#define NAV_PATH_NODE_MAX	(9u)		//路径节点上限：最短路无重复节点，故等于节点数
#define NAV_PATH_W90_MM		(5700u)		//一次 90° 转向的等效里程(mm)：19 s × 300 mm/s
#define NAV_PATH_W180_MM	(7200u)		//一次 180° 转向的等效里程(mm)：24 s × 300 mm/s
#define NAV_COST_INF		(0xFFFFFFFFu)	//代价无穷
#define NAV_EDGE_NONE		(0xFFu)		//查不到对应的边

//边：一条直走廊，dir 为由 a 指向 b 的世界方向
typedef struct {
	uint8_t  a;		//端点 A 的节点号
	uint8_t  b;		//端点 B 的节点号
	uint8_t  dir;	//由 A 指向 B 的世界方向 NAV_DIR_*
	uint16_t len;	//边长(mm)
} NavEdge_t;

//节点坐标（场地坐标系，mm）；节点号越界时返回 0
int16_t nav_PathNodeX(uint8_t node);
int16_t nav_PathNodeY(uint8_t node);

//边表规模与取边
uint8_t nav_PathEdgeNum(void);
const NavEdge_t *nav_PathEdge(uint8_t idx);

//查边号：a 与 b 不相邻时返回 NAV_EDGE_NONE
uint8_t nav_PathEdgeOf(uint8_t a, uint8_t b);

//找路：from → to，blocked 的 bit i = 第 i 条边不可走
//headDir 为车当前机头的世界方向，用于给路径首段的转向定价
//outPath 写入节点序列（含首尾），返回节点数；0 = 无可行路径或缓冲不够
uint8_t nav_PathFind(uint8_t from, uint8_t to, uint16_t blocked, uint8_t headDir,
					 uint8_t *outPath, uint8_t maxNodes);

#endif
