/* FreeRTOSConfig.h —— POSIX 模拟层专用配置
 * 从 FreeRTOS/FreeRTOSConfig.h 裁剪而来：去掉 stm32f4xx.h 依赖、
 * NVIC/PendSV/SVC/SysTick 等 ARM 特定项，其余数值保持一致，
 * 保证本机模拟与 STM32 实机的任务/队列/时序语义一致。 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION                         1      /* 1=抢占式调度：高优先级任务就绪即刻抢占当前任务 */
#define configUSE_IDLE_HOOK                          0
#define configUSE_TICK_HOOK                          0
#define configCPU_CLOCK_HZ                           ( 100000000UL )  /* POSIX 模拟下无真实时钟，仅占位 */
#define configTICK_RATE_HZ                           ( ( TickType_t ) 1000 )  /* 1 ms 节拍，与实机一致 */
#define configMAX_PRIORITIES                         8
#define configMINIMAL_STACK_SIZE                     ( ( uint16_t ) 128 )
#define configTOTAL_HEAP_SIZE                        ( ( size_t ) ( 16 * 1024 ) )
#define configMAX_TASK_NAME_LEN                      16
#define configUSE_TRACE_FACILITY                     0
#define configUSE_16_BIT_TICKS                       0
#define configIDLE_SHOULD_YIELD                      1
#define configUSE_MUTEXES                            1
#define configUSE_RECURSIVE_MUTEXES                  1
#define configUSE_COUNTING_SEMAPHORES                1
#define configQUEUE_REGISTRY_SIZE                    8
#define configCHECK_FOR_STACK_OVERFLOW               2
#define configUSE_MALLOC_FAILED_HOOK                 0
#define configUSE_APPLICATION_TASK_TAG               0
#define configUSE_TIMERS                             1
#define configTIMER_TASK_PRIORITY                    ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                     8
#define configTIMER_TASK_STACK_DEPTH                 ( configMINIMAL_STACK_SIZE )
#define configSUPPORT_DYNAMIC_ALLOCATION             1
#define configSUPPORT_STATIC_ALLOCATION              0

#define configASSERT( x )                            if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

#define INCLUDE_vTaskPrioritySet                     1
#define INCLUDE_uxTaskPriorityGet                    1
#define INCLUDE_vTaskDelete                          1
#define INCLUDE_vTaskCleanUpResources                0
#define INCLUDE_vTaskSuspend                         1
#define INCLUDE_vTaskDelay                           1
#define INCLUDE_xTaskDelayUntil                      1
#define INCLUDE_xTaskGetSchedulerState               1
#define INCLUDE_xTaskGetCurrentTaskHandle            1
#define INCLUDE_xTaskGetIdleTaskHandle               1
#define INCLUDE_xTaskGetHandle                       1
#define INCLUDE_xTaskAbortDelay                      1

/* 钩子函数声明 */
struct tskTaskControlBlock;
void vApplicationStackOverflowHook( struct tskTaskControlBlock * xTask, char * pcTaskName );

#endif
