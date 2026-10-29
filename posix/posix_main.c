/* posix_main.c —— POSIX 模拟层入口
 * 与 APP/main.c 同构：任务表 + 统一创建 + 调度器启动。
 * 差异：不调 board_init()（无硬件）、不调 stm32f4xx_it 相关；
 * 其余初始化顺序与实机一致。 */
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

typedef struct {
    TaskFunction_t entry;       /* 任务体 */
    const char    *name;        /* 任务名 */
    uint16_t       stack;       /* 栈深度（字） */
    UBaseType_t    prio;        /* 优先级 */
} TaskDesc_t;

static const TaskDesc_t s_tasks[] = {
    { contract_CtrlTask,    "CTRL",   CHASSIS_CTRL_TASK_STACK,  CHASSIS_CTRL_TASK_PRIO  },
    { contract_CanRecvTask, "CAN_RX", CHASSIS_CAN_TASK_STACK,   CHASSIS_CAN_TASK_PRIO   },
    { contract_StateTask,   "STATE",  CHASSIS_STATE_TASK_STACK, CHASSIS_STATE_TASK_PRIO },
};

#define TASK_NUM    (sizeof(s_tasks) / sizeof(s_tasks[0]))

/**
 * @brief  栈溢出钩子（POSIX 模拟下打印并停下）
 * @author camelliahm
 */
void vApplicationStackOverflowHook(struct tskTaskControlBlock *xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for(;;) {}
}

static void App_TaskCreate(void)
{
    UBaseType_t i;

    for (i = 0; i < TASK_NUM; i++)
    {
        BaseType_t xRet = xTaskCreate(s_tasks[i].entry, s_tasks[i].name,
                                      s_tasks[i].stack, NULL,
                                      s_tasks[i].prio, NULL);
        configASSERT(xRet == pdPASS);
    }
}

static void ALL_Init(void)
{
    board_init();                          /* POSIX 空壳 */
    servo_Init();
    input_Init();
    vision_Init();
    screen_Init();
    oled_Init();
    nav_Init();
    configASSERT(can_RtosInit() == true);
    contract_Init();
    App_TaskCreate();
}

int main(void)
{
    ALL_Init();
    vTaskStartScheduler();

    while(1) {}
}
