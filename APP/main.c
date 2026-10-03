#include "board.h"
#include "can.h"
#include "chassis.h"
#include "FreeRTOS.h"
#include "task.h"

/**********************************************************
***	任务表
***	任务体一律实现在各自模块（如 APP/chassis.c），本文件只负责
***	"声明式"地罗列任务并统一创建，不承载任何业务逻辑。
***	任务划分与控制环周期取值的依据见工程根目录 README.md。
**********************************************************/
typedef struct {
	TaskFunction_t entry;		/* 任务体 */
	const char    *name;		/* 任务名，调试器里可见 */
	uint16_t       stack;		/* 栈深度（字） */
	UBaseType_t    prio;		/* 优先级，数值越大越高 */
} TaskDesc_t;

static const TaskDesc_t s_tasks[] = {
	{ chassis_CtrlTask,    "CTRL",   CHASSIS_CTRL_TASK_STACK,  CHASSIS_CTRL_TASK_PRIO  },
	{ chassis_CanRecvTask, "CAN_RX", CHASSIS_CAN_TASK_STACK,   CHASSIS_CAN_TASK_PRIO   },
	{ chassis_StateTask,   "STATE",  CHASSIS_STATE_TASK_STACK, CHASSIS_STATE_TASK_PRIO },
};

#define TASK_NUM	(sizeof(s_tasks) / sizeof(s_tasks[0]))

/*----------------------------------------------------------*/
/*  FreeRTOS 钩子函数                                          */
/*----------------------------------------------------------*/

/**
 * @brief  栈溢出钩子
 * @param  xTask 溢出任务的句柄
 * @param  pcTaskName 溢出任务名
 * @note   configCHECK_FOR_STACK_OVERFLOW = 2 时必须实现
 * @author camelliehm
 */
void vApplicationStackOverflowHook(struct tskTaskControlBlock *xTask, char *pcTaskName)
{
	(void)xTask;		/* 栈溢出，停在此处便于调试观察是哪个任务 */
	(void)pcTaskName;

	taskDISABLE_INTERRUPTS();
	for(;;)
	{

	}
}

/*----------------------------------------------------------*/
/*  应用入口                                                    */
/*----------------------------------------------------------*/

/**
 * @brief  按任务表统一创建任务
 * @author camelliehm
 */
static void App_TaskCreate(void)
{
	UBaseType_t i;

	for (i = 0; i < TASK_NUM; i++)
	{
		BaseType_t xRet = xTaskCreate(s_tasks[i].entry,
									  s_tasks[i].name,
									  s_tasks[i].stack,
									  NULL,
									  s_tasks[i].prio,
									  NULL);
		configASSERT(xRet == pdPASS);
	}
}

/**
 * @brief  MAIN函数
 * @note   delay_ms()/delay_us() 会重新配置并关闭 SysTick，
 *         调度器启动后禁止使用，请改用 vTaskDelay()。
 * @author camelliehm
 */
int main(void)
{
	board_init();                          // ① 硬件初始化（NVIC、时钟、GPIO、CAN）
	configASSERT(can_RtosInit() == true);  // ② 创建 CAN 接收队列与发送互斥锁（须在创建任务前）
	chassis_Init();                        // ③ 创建状态机事件队列
	App_TaskCreate();                      // ④ 按任务表创建所有任务
	vTaskStartScheduler();                 // ⑤ 启动调度器（不再返回）
	while(1)
	{

	}
}
