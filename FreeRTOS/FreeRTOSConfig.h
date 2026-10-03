#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"

#define configUSE_PREEMPTION                         1      /* 1=抢占式调度：高优先级任务就绪即刻抢占当前任务 */
#define configUSE_IDLE_HOOK                          0      /* 1=启用空闲任务钩子 vApplicationIdleHook()，本工程未使用 */
#define configUSE_TICK_HOOK                          0      /* 1=启用每个节拍的钩子 vApplicationTickHook()，本工程未使用 */
#define configCPU_CLOCK_HZ                           ( SystemCoreClock )  /* 内核时钟频率，取 SystemCoreClock（本工程 168 MHz） */
#define configTICK_RATE_HZ                           ( ( TickType_t ) 1000 )  /* 系统节拍频率 1000 Hz，即节拍周期 1 ms */
#define configMAX_PRIORITIES                         8      /* 最大优先级数，合法优先级 0~7（数值越大优先级越高） */
#define configMINIMAL_STACK_SIZE                     ( ( uint16_t ) 128 )  /* 空闲任务栈深度，单位=字，Cortex-M4 上 128 字 = 512 字节 */
#define configTOTAL_HEAP_SIZE                        ( ( size_t ) ( 16 * 1024 ) )  /* heap_4 堆总大小 16 KB，任务栈/队列/信号量/定时器均由此分配 */
#define configMAX_TASK_NAME_LEN                      16     /* 任务名最大长度（含字符串结束符） */
#define configUSE_TRACE_FACILITY                     0      /* 1=启用可视化追踪与 uxTaskGetSystemState()，关闭可省 RAM */
#define configUSE_16_BIT_TICKS                       0      /* 0=用 32 位节拍计数；32 位 MCU 必须为 0 */
#define configIDLE_SHOULD_YIELD                      1      /* 1=有同优先级任务就绪时，空闲任务立刻让出 CPU */
#define configUSE_MUTEXES                            1      /* 1=启用互斥量（本工程 CAN 发送互斥 can_TxLock 依赖它） */
#define configUSE_RECURSIVE_MUTEXES                  1      /* 1=启用递归互斥量（同一任务可重复获取） */
#define configUSE_COUNTING_SEMAPHORES                1      /* 1=启用计数信号量 */
#define configQUEUE_REGISTRY_SIZE                    8      /* 队列注册表容量，仅调试/可视化用，>0 才能按名查队列 */
#define configCHECK_FOR_STACK_OVERFLOW               2      /* 栈溢出检测：2=方法2（需实现 vApplicationStackOverflowHook，见 main.c） */
#define configUSE_MALLOC_FAILED_HOOK                 0      /* 1=启用内存分配失败钩子 vApplicationMallocFailedHook() */
#define configUSE_APPLICATION_TASK_TAG               0      /* 1=启用任务标签与 xTaskCallApplicationTaskHook() */
#define configUSE_TIMERS                             1      /* 1=启用软件定时器（会自动创建定时器服务任务） */
#define configTIMER_TASK_PRIORITY                    ( configMAX_PRIORITIES - 1 )  /* 定时器服务任务优先级 = 7，高于三个业务任务（4/3/2） */
#define configTIMER_QUEUE_LENGTH                     8      /* 定时器命令队列长度，可缓存的定时器命令条数 */
#define configTIMER_TASK_STACK_DEPTH                 ( configMINIMAL_STACK_SIZE )  /* 定时器服务任务栈深度，此处等于最小栈 128 字 */
#define configSUPPORT_DYNAMIC_ALLOCATION             1      /* 1=允许动态创建任务/队列（xTaskCreate 等可用） */
#define configSUPPORT_STATIC_ALLOCATION              0      /* 0=关闭静态创建（xTaskCreateStatic 等不可用） */

#define configPRIO_BITS                              4      /* NVIC 抢占优先级位数，STM32F4 用 4 位（共 16 级） */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15     /* 内核可管理的最低中断优先级（数值 15 = 优先级最低） */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5      /* 可调用 FreeRTOS API 的中断优先级下限：优先级号 >= 5 的中断才能调 FromISR 版 API */
#define configKERNEL_INTERRUPT_PRIORITY              ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )  /* 内核中断优先级，写入 NVIC 时需左移 (8 - configPRIO_BITS) 位 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY         ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )  /* 中断优先级阈值，写入 NVIC 的移位值；高于此优先级的中断不得调 FreeRTOS API */

#define configASSERT( x )                            if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }  /* 断言失败即关中断并死循环，便于调试器定位出错行 */

#define INCLUDE_vTaskPrioritySet                     1      /* 编译进 vTaskPrioritySet()（运行时改任务优先级） */
#define INCLUDE_uxTaskPriorityGet                    1      /* 编译进 uxTaskPriorityGet()（查询任务优先级） */
#define INCLUDE_vTaskDelete                          1      /* 编译进 vTaskDelete()（删除任务） */
#define INCLUDE_vTaskCleanUpResources                0      /* 编译进 vTaskCleanUpResources()，通常不用 */
#define INCLUDE_vTaskSuspend                         1      /* 编译进 vTaskSuspend()/vTaskResume()（挂起与恢复任务） */
#define INCLUDE_vTaskDelay                           1      /* 编译进 vTaskDelay()（相对延时） */
#define INCLUDE_xTaskDelayUntil                      1      /* 编译进 xTaskDelayUntil()（CTRL 任务 5 ms 绝对周期依赖它） */
#define INCLUDE_xTaskGetSchedulerState               1      /* 编译进 xTaskGetSchedulerState()（查询调度器状态） */
#define INCLUDE_xTaskGetCurrentTaskHandle            1      /* 编译进 xTaskGetCurrentTaskHandle()（获取当前任务句柄） */
#define INCLUDE_xTaskGetIdleTaskHandle               1      /* 编译进 xTaskGetIdleTaskHandle()（获取空闲任务句柄） */
#define INCLUDE_xTaskGetHandle                       1      /* 编译进 xTaskGetHandle()（按任务名查句柄） */
#define INCLUDE_xTaskAbortDelay                      1      /* 编译进 xTaskAbortDelay()（中止任务的阻塞延时） */

#define xPortPendSVHandler                           PendSV_Handler  /* 把 PendSV_Handler 映射到 port 层实现；stm32f4xx_it.c 不得再定义该函数 */
#define vPortSVCHandler                              SVC_Handler  /* 把 SVC_Handler 映射到 port 层实现；stm32f4xx_it.c 不得再定义该函数 */

/**********************************************************
***	钩子函数声明
***	configCHECK_FOR_STACK_OVERFLOW = 2 时，必须实现
***	vApplicationStackOverflowHook()，实现在 APP/main.c
***	此处用前置声明，避免依赖 FreeRTOS.h 的类型定义顺序
**********************************************************/
struct tskTaskControlBlock;
void vApplicationStackOverflowHook( struct tskTaskControlBlock * xTask, char * pcTaskName );

#endif
