#ifndef __FIFO_H
#define __FIFO_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FIFO_SIZE		128		//FIFO 容量（字节）
typedef struct {
	uint16_t buffer[FIFO_SIZE];	//数据缓冲
	__IO uint8_t ptrWrite;		//写指针
	__IO uint8_t ptrRead;		//读指针
} FIFO_t;

extern __IO FIFO_t rxFIFO;

void fifo_initQueue(void);
void fifo_enQueue(uint16_t data);
uint16_t fifo_deQueue(void);
bool fifo_isEmpty(void);
uint16_t fifo_queueLength(void);

#ifdef __cplusplus
}
#endif

#endif
