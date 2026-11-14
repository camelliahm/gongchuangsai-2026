#ifndef __NAV_GEOM_H
#define __NAV_GEOM_H

/* 场地几何常量（智能搬运初赛赛场）：纯数据无逻辑
   供 nav_map.h 的路线表与 nav_path.c 的走廊图共用，故单独成文件、不依赖任何类型定义
   坐标系：原点 = 场地左下角，+X 右，+Y 上，单位 mm；与里程计坐标系不是一回事
   ⚠ 原料区 X / 二维码板 Y 现场随机，本表取中值 */

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
#define NAV_MAP_START1_Y0		(NAV_MAP_FIELD_MM - NAV_MAP_START_MM)	//2100 启停区1 下边 Y（贴场地顶边）
#define NAV_MAP_START2_Y0		(0)										//启停区2 下边 Y（贴场地底边）

/*---- 原料区（顶边电动转盘，φ300，台面高 80~100）----*/
#define NAV_MAP_MAT_D_MM		(300)						//转盘直径
#define NAV_MAP_MAT_CX_NOM		NAV_MAP_FIELD_HALF			//1200 圆心 X（赛题给 1100~1300 随机，取中值）
#define NAV_MAP_MAT_CY_NOM		(NAV_MAP_FIELD_MM)			//2400 圆心落在顶边线（Y 最大侧）

/*---- 暂存区（左边，580×150 竖向居中，3 个靶位）----*/
#define NAV_MAP_BUF_L_MM		(580)						//长，沿 Y
#define NAV_MAP_BUF_W_MM		(150)						//宽，沿 X
#define NAV_MAP_BUF_X0			(0)							//贴左边线
#define NAV_MAP_BUF_Y0			(NAV_MAP_FIELD_HALF - NAV_MAP_BUF_L_MM / 2)	//910

/*---- 粗加工区（底边，580×150 横向居中，3 个靶位）----*/
#define NAV_MAP_ROU_X0			(NAV_MAP_FIELD_HALF - NAV_MAP_BUF_L_MM / 2)	//910
#define NAV_MAP_ROU_Y0			(0)											//贴场地底边线

/*---- 二维码板（右边墙，A4 横放，QR 80×80 居中）----*/
#define NAV_MAP_QR_Y_NOM		NAV_MAP_FIELD_HALF			//1200 板心 Y（赛题给 1100~1300 随机，取中值）
#define NAV_MAP_QR_MM			(80)						//二维码边长

/*---- 障碍物与物料（摘自命题要求的物理尺寸）----*/
#define NAV_MAP_OBST_D_MM		(50)						//黑色模拟障碍物 φ50×100
#define NAV_MAP_OBST_H_MM		(100)						//障碍物高度
#define NAV_MAP_ITEM_D_MM		(80)						//物料最大尺寸
#define NAV_MAP_ITEM_M_G		(100)						//物料最大质量(g)

/*---- 车体与抓取点 ----*/
/* 车体坐标系：原点 = 车心，+X 车头，+Y 车右；抓取点在车长中点、车体右边界外 100 mm */
#define NAV_MAP_CAR_L_MM		(300)						//车长（沿机头）
#define NAV_MAP_CAR_W_MM		(242)						//车宽（横向）
#define NAV_MAP_GRIP_OUT_MM		(100)						//抓取点距车体右边界的距离
#define NAV_MAP_GRIP_REACH_MM	(NAV_MAP_CAR_W_MM / 2 + NAV_MAP_GRIP_OUT_MM)	//221 车心到抓取点

/*---- 停靠点：机器人中心应到达的场地坐标 ----*/
/* 作业时区域长边与机头平行、区域位于车体右侧，故 停靠点 = 靶位中心 ∓ GRIP_REACH */
#define NAV_MAP_CROSS_X			NAV_MAP_FIELD_HALF		//1200 十字交点
#define NAV_MAP_CROSS_Y			NAV_MAP_FIELD_HALF		//十字交点 Y

#define NAV_MAP_DOCK_START1_X	(NAV_MAP_START1_X0 + NAV_MAP_START_MM / 2)	//2250
#define NAV_MAP_DOCK_START1_Y	(NAV_MAP_START1_Y0 + NAV_MAP_START_MM / 2)	//2250
#define NAV_MAP_DOCK_START2_Y	(NAV_MAP_START2_Y0 + NAV_MAP_START_MM / 2)	//150

#define NAV_MAP_DOCK_QR_X		(NAV_MAP_FIELD_MM - NAV_MAP_CAR_L_MM / 2)	//2250 车头顶右墙
#define NAV_MAP_DOCK_QR_Y		NAV_MAP_QR_Y_NOM							//1200

#define NAV_MAP_DOCK_MAT_X		NAV_MAP_MAT_CX_NOM											//1200
#define NAV_MAP_DOCK_MAT_Y		(NAV_MAP_MAT_CY_NOM - NAV_MAP_MAT_D_MM / 2 - NAV_MAP_GRIP_REACH_MM)	//2029 盘缘在车右

#define NAV_MAP_DOCK_ROU_X		NAV_MAP_CROSS_X													//1200
#define NAV_MAP_DOCK_ROU_Y		(NAV_MAP_ROU_Y0 + NAV_MAP_BUF_W_MM / 2 + NAV_MAP_GRIP_REACH_MM)	//296 区中线在车右

#define NAV_MAP_DOCK_BUF_X		(NAV_MAP_BUF_X0 + NAV_MAP_BUF_W_MM / 2 + NAV_MAP_GRIP_REACH_MM)	//296 区中线在车右
#define NAV_MAP_DOCK_BUF_Y		NAV_MAP_CROSS_Y													//1200

/*---- 航段量纲换算（供路线表引用）----*/
#define NAV_MAP_TURN_90			(900)						//90.0 度，0.1 度单位，正 = 逆时针
#define NAV_MAP_TURN_180		(1800)						//180.0 度

#define NAV_MAP_SPEED_LANE		(300)						//车道直行/横移速度(mm/s)
#define NAV_MAP_SPEED_DOCK		(150)						//进位段速度(mm/s)

#define NAV_MAP_DWELL_QR_MS		(800)						//读二维码 + 任务码上屏

/*---- 目标区靶位（位号）----*/
/* 暂存区与粗加工区各 3 个靶位，沿 580 长边等距 150；区域中心即 2 号靶位 */
/* 作业时区域长边与机头平行，故位号 +1 = 车体前进 150 */
#define NAV_MAP_RING_DOCK_NUM	(2)							//入口停靠点正对的位号
#define NAV_MAP_RING_STEP_MM	(150)						//位号 +1 对应的车体前进量

#endif
