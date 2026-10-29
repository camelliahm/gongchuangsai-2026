#include "board.h"
#include "can.h"
#include "contract.h"
#include "servo.h"
#include "input.h"
#include "vision.h"
#include "screen.h"
#include "oled.h"
#include "nav.h"
#include "FreeRTOS.h"
#include "task.h"

/**********************************************************
***	任务表
***	任务体一律实现在各自模块（如 APP/contract.c），本文件只负责
***	"声明式"地罗列任务并统一创建，不承载任何业务逻辑。
***	任务划分与控制环周期取值的依据见工程根目录 README.md。
**********************************************************/
typedef struct {
	TaskFunction_t entry;		//任务体
	const char    *name;		//任务名，调试器里可见
	uint16_t       stack;		//栈深度（字）
	UBaseType_t    prio;		//优先级，数值越大越高
} TaskDesc_t;

static const TaskDesc_t s_tasks[] = {
	{ contract_CtrlTask,    "CTRL",   CHASSIS_CTRL_TASK_STACK,  CHASSIS_CTRL_TASK_PRIO  },
	{ contract_CanRecvTask, "CAN_RX", CHASSIS_CAN_TASK_STACK,   CHASSIS_CAN_TASK_PRIO   },
	{ contract_StateTask,   "STATE",  CHASSIS_STATE_TASK_STACK, CHASSIS_STATE_TASK_PRIO },
};

#define TASK_NUM	(sizeof(s_tasks) / sizeof(s_tasks[0]))	//任务表项数

/*---- FreeRTOS 钩子函数 -------------------------------*/

/**
 * @brief  栈溢出钩子
 * @param  xTask 溢出任务的句柄
 * @param  pcTaskName 溢出任务名
 * @note   configCHECK_FOR_STACK_OVERFLOW = 2 时必须实现
 * @author camelliahm
 */
void vApplicationStackOverflowHook(struct tskTaskControlBlock *xTask, char *pcTaskName)
{
	(void)xTask;		//栈溢出，停在此处便于调试观察是哪个任务
	(void)pcTaskName;

	taskDISABLE_INTERRUPTS();
	for(;;)
	{

	}
}

/*---- 应用入口 ----------------------------------------*/

/**
 * @brief  按任务表统一创建任务
 * @author camelliahm
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
 * @brief  全部初始化，按依赖顺序集中调用
 * @note   board_init() 须最先执行（时钟与 GPIO 由它打开）；
 *         创建队列/互斥锁的调用须排在建立任务之前。
 * @author camelliahm
 */
static void ALL_Init(void)
{
	board_init();                          // 硬件初始化（NVIC、时钟、GPIO、CAN）
	servo_Init();                          // 三路舵机 PWM（TIM3 CH1~CH3）
	input_Init();                          // 三条串口（USART1 / USART3 / USART2）+ 摁键 GPIO
	vision_Init();                         // 视觉 1 / 2 帧槽位归零（须在 input_Init 之后）
	screen_Init();                         // 串口屏上报槽位归零（USART2 硬件在 input_Init）
	oled_Init();                           // OLED 状态屏（软件 I2C）
	nav_Init();                            // 编排层：装载默认地图，不自动启动
	configASSERT(can_RtosInit() == true);  // CAN 接收队列与发送互斥锁
	contract_Init();                       // 状态机事件队列
	App_TaskCreate();                      // 按任务表创建所有任务
}

/**
 * @brief  MAIN函数
 * @note   delay_ms()/delay_us() 会重新配置并关闭 SysTick，
 *         调度器启动后禁止使用，请改用 vTaskDelay()。
 * @author camelliahm
 */
int main(void)
{
	ALL_Init();                            // 初始化
	vTaskStartScheduler();                 // 启动调度器（不再返回）

	while(1)
	{

	}
}
