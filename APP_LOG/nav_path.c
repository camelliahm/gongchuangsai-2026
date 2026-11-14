#include "nav_path.h"
#include "nav_geom.h"
#include <stddef.h>

//可行驶图：节点坐标 + 边表 + 带转向代价的最短路
//图的规模是给定的（9 节点 12 边），故不用堆、不建邻接表，直接遍历，5 ms 周期内跑完

/*---- 内部宏 ----------------------------------------*/
#define NAV_ST_NUM		(NAV_NODE_NUM * NAV_DIR_NUM)	//状态数 = 节点 × 进入方向
#define NAV_PREV_NONE	(0xFFu)							//无前驱状态

/*---- 图（纯数据）----------------------------------------*/

//节点坐标：一律取 nav_map.h 的常量，图纸与代码不会各说一套
static const int16_t s_nodeX[NAV_NODE_NUM] = {
	NAV_MAP_DOCK_MAT_X,		//NAV_NODE_MAT
	NAV_MAP_DOCK_QR_X,		//NAV_NODE_RT
	NAV_MAP_DOCK_QR_X,		//NAV_NODE_QR
	NAV_MAP_DOCK_QR_X,		//NAV_NODE_RB
	NAV_MAP_DOCK_ROU_X,		//NAV_NODE_ROU
	NAV_MAP_DOCK_BUF_X,		//NAV_NODE_LB
	NAV_MAP_DOCK_BUF_X,		//NAV_NODE_BUF
	NAV_MAP_DOCK_BUF_X,		//NAV_NODE_LT
	NAV_MAP_CROSS_X,		//NAV_NODE_CROSS
};

static const int16_t s_nodeY[NAV_NODE_NUM] = {
	NAV_MAP_DOCK_MAT_Y,		//NAV_NODE_MAT
	NAV_MAP_DOCK_MAT_Y,		//NAV_NODE_RT
	NAV_MAP_DOCK_QR_Y,		//NAV_NODE_QR
	NAV_MAP_DOCK_ROU_Y,		//NAV_NODE_RB
	NAV_MAP_DOCK_ROU_Y,		//NAV_NODE_ROU
	NAV_MAP_DOCK_ROU_Y,		//NAV_NODE_LB
	NAV_MAP_DOCK_BUF_Y,		//NAV_NODE_BUF
	NAV_MAP_DOCK_MAT_Y,		//NAV_NODE_LT
	NAV_MAP_CROSS_Y,		//NAV_NODE_CROSS
};

//边表：外环 8 条 + 十字 4 条，覆盖全部可行驶走廊
static const NavEdge_t s_edge[] = {
	{ NAV_NODE_MAT,   NAV_NODE_RT,    NAV_DIR_PX, (uint16_t)(NAV_MAP_DOCK_QR_X  - NAV_MAP_DOCK_MAT_X)  },	//00 原料区行东段
	{ NAV_NODE_RT,    NAV_NODE_QR,    NAV_DIR_NY, (uint16_t)(NAV_MAP_DOCK_MAT_Y - NAV_MAP_DOCK_QR_Y)   },	//01 二维码列上段
	{ NAV_NODE_QR,    NAV_NODE_RB,    NAV_DIR_NY, (uint16_t)(NAV_MAP_DOCK_QR_Y  - NAV_MAP_DOCK_ROU_Y)  },	//02 二维码列下段
	{ NAV_NODE_RB,    NAV_NODE_ROU,   NAV_DIR_NX, (uint16_t)(NAV_MAP_DOCK_QR_X  - NAV_MAP_DOCK_ROU_X)  },	//03 粗加工区行东段
	{ NAV_NODE_ROU,   NAV_NODE_LB,    NAV_DIR_NX, (uint16_t)(NAV_MAP_DOCK_ROU_X - NAV_MAP_DOCK_BUF_X)  },	//04 粗加工区行中段
	{ NAV_NODE_LB,    NAV_NODE_BUF,   NAV_DIR_PY, (uint16_t)(NAV_MAP_DOCK_BUF_Y - NAV_MAP_DOCK_ROU_Y)  },	//05 暂存区列下段
	{ NAV_NODE_BUF,   NAV_NODE_LT,    NAV_DIR_PY, (uint16_t)(NAV_MAP_DOCK_MAT_Y - NAV_MAP_DOCK_BUF_Y)  },	//06 暂存区列上段
	{ NAV_NODE_LT,    NAV_NODE_MAT,   NAV_DIR_PX, (uint16_t)(NAV_MAP_DOCK_MAT_X - NAV_MAP_DOCK_BUF_X)  },	//07 原料区行西段
	{ NAV_NODE_MAT,   NAV_NODE_CROSS, NAV_DIR_NY, (uint16_t)(NAV_MAP_DOCK_MAT_Y - NAV_MAP_CROSS_Y)     },	//08 十字竖线北段
	{ NAV_NODE_CROSS, NAV_NODE_ROU,   NAV_DIR_NY, (uint16_t)(NAV_MAP_CROSS_Y    - NAV_MAP_DOCK_ROU_Y)  },	//09 十字竖线南段
	{ NAV_NODE_CROSS, NAV_NODE_BUF,   NAV_DIR_NX, (uint16_t)(NAV_MAP_CROSS_X    - NAV_MAP_DOCK_BUF_X)  },	//10 十字横线西段
	{ NAV_NODE_CROSS, NAV_NODE_QR,    NAV_DIR_PX, (uint16_t)(NAV_MAP_DOCK_QR_X  - NAV_MAP_CROSS_X)     },	//11 十字横线东段
};

#define NAV_EDGE_NUM	((uint8_t)(sizeof(s_edge) / sizeof(s_edge[0])))	//边数

/*---- 运行期缓冲（找路时复用，单线程调用）----------------------------------------*/
static uint32_t s_cost[NAV_ST_NUM];			//各状态的最优代价（里程 + 转向等效里程）
static uint8_t  s_prev[NAV_ST_NUM];			//各状态的前驱状态
static uint8_t  s_visit[NAV_ST_NUM];		//1 = 该状态的最优代价已确定
static uint8_t  s_nodeBuf[NAV_PATH_NODE_MAX];	//回溯得到的节点序列（倒序）

/**
 * @brief  两次行进方向之间的转向等效代价
 * @param  d1 前一方向（NAV_DIR_*）
 * @param  d2 后一方向
 * @retval 等效里程(mm)：同向 0，90° 记 W90，180° 记 W180
 * @author camelliahm
 */
static uint32_t nav_TurnCost(uint8_t d1, uint8_t d2)
{
	uint8_t diff = (uint8_t)((d2 + (uint8_t)NAV_DIR_NUM - d1) % (uint8_t)NAV_DIR_NUM);

	if (diff == 0u)
	{
		return 0u;
	}

	if (diff == 2u)
	{
		return NAV_PATH_W180_MM;
	}

	return NAV_PATH_W90_MM;
}

/*---- 图查询 ----------------------------------------*/

int16_t nav_PathNodeX(uint8_t node)
{
	return (node < (uint8_t)NAV_NODE_NUM) ? s_nodeX[node] : 0;
}

int16_t nav_PathNodeY(uint8_t node)
{
	return (node < (uint8_t)NAV_NODE_NUM) ? s_nodeY[node] : 0;
}

uint8_t nav_PathEdgeNum(void)
{
	return NAV_EDGE_NUM;
}

const NavEdge_t *nav_PathEdge(uint8_t idx)
{
	return (idx < NAV_EDGE_NUM) ? &s_edge[idx] : NULL;
}

/**
 * @brief  查两节点之间的边号
 * @param  a 端点节点号
 * @param  b 另一端点节点号
 * @retval 边号；不相邻时返回 NAV_EDGE_NONE
 * @author camelliahm
 */
uint8_t nav_PathEdgeOf(uint8_t a, uint8_t b)
{
	uint8_t e;

	for (e = 0u; e < NAV_EDGE_NUM; e++)
	{
		if (((s_edge[e].a == a) && (s_edge[e].b == b))
				|| ((s_edge[e].a == b) && (s_edge[e].b == a)))
		{
			return e;
		}
	}

	return NAV_EDGE_NONE;
}

/**
 * @brief  在图上找一条从 from 到 to 的走法
 * @param  from    起点节点号
 * @param  to      终点节点号
 * @param  blocked 堵边位图：bit i = 1 表示第 i 条边不可走
 * @param  headDir 车当前机头的世界方向，首段需要转向时由此计价
 * @param  outPath 输出节点序列（含首尾）
 * @param  maxNodes outPath 的容量（节点数）
 * @retval 节点数；0 = 无可行路径 / 入参越界 / 缓冲不够
 * @note   代价 = 里程 + 转向等效里程，故两条等长走法会选转向少的；
 *         状态取"节点 × 进入方向"，转向代价才可累加（只按节点记会丢掉方向信息）
 * @author camelliahm
 */
uint8_t nav_PathFind(uint8_t from, uint8_t to, uint16_t blocked, uint8_t headDir,
					 uint8_t *outPath, uint8_t maxNodes)
{
	uint8_t  i;
	uint8_t  best;
	uint8_t  node = 0u;
	uint8_t  dir = 0u;
	uint8_t  e;
	uint8_t  nb;
	uint8_t  nd;
	uint8_t  ns;
	uint8_t  n = 0u;
	uint32_t cost;
	uint32_t cand;

	if ((from >= (uint8_t)NAV_NODE_NUM) || (to >= (uint8_t)NAV_NODE_NUM)
			|| (headDir >= (uint8_t)NAV_DIR_NUM) || (outPath == NULL)
			|| (maxNodes == 0u))
	{
		return 0u;
	}

	for (i = 0u; i < (uint8_t)NAV_ST_NUM; i++)
	{
		s_cost[i]  = NAV_COST_INF;
		s_prev[i]  = NAV_PREV_NONE;
		s_visit[i] = 0u;
	}

	s_cost[(uint8_t)(from * NAV_DIR_NUM + headDir)] = 0u;

	//Dijkstra：每次取未确定状态里代价最小的一个，松弛它的全部邻边
	for (;;)
	{
		best = NAV_PREV_NONE;
		cost = NAV_COST_INF;

		for (i = 0u; i < (uint8_t)NAV_ST_NUM; i++)
		{
			if ((s_visit[i] == 0u) && (s_cost[i] < cost))
			{
				cost = s_cost[i];
				best = i;
			}
		}

		if (best == NAV_PREV_NONE)
		{
			return 0u;		//没有可达状态：起点与终点被堵边分开
		}

		s_visit[best] = 1u;
		node = (uint8_t)(best / NAV_DIR_NUM);
		dir  = (uint8_t)(best % NAV_DIR_NUM);

		if (node == to)
		{
			break;			//第一次弹出终点状态即为最优
		}

		for (e = 0u; e < NAV_EDGE_NUM; e++)
		{
			if (((blocked >> e) & 1u) != 0u)
			{
				continue;
			}

			if (s_edge[e].a == node)
			{
				nb = s_edge[e].b;
				nd = s_edge[e].dir;
			}
			else if (s_edge[e].b == node)
			{
				nb = s_edge[e].a;
				nd = (uint8_t)((s_edge[e].dir + (uint8_t)NAV_DIR_NUM / 2u) % (uint8_t)NAV_DIR_NUM);
			}
			else
			{
				continue;
			}

			ns   = (uint8_t)(nb * NAV_DIR_NUM + nd);
			cand = cost + (uint32_t)s_edge[e].len + nav_TurnCost(dir, nd);

			if (cand < s_cost[ns])
			{
				s_cost[ns] = cand;
				s_prev[ns] = best;
			}
		}
	}

	//回溯：方向只是边上的标记，节点才是路径
	while ((best != NAV_PREV_NONE) && (n < (uint8_t)NAV_PATH_NODE_MAX))
	{
		s_nodeBuf[n] = (uint8_t)(best / NAV_DIR_NUM);
		n++;
		best = s_prev[best];
	}

	if ((n == 0u) || (n > maxNodes))
	{
		return 0u;
	}

	for (i = 0u; i < n; i++)
	{
		outPath[i] = s_nodeBuf[(uint8_t)(n - 1u - i)];
	}

	return n;
}
