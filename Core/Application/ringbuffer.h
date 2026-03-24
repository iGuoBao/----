#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H
#include "math.h"
#include "string.h"
#include "main.h"
#define RING_BUFFER_SIZE 128  // 缓冲区容量（需为2的幂以优化取模运算）

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];  // 存储区
    volatile uint16_t head;            // 写指针（需volatile防止编译器优化）
    volatile uint16_t tail;            // 读指针
} RingBuffer;


extern RingBuffer sevenway_buffer;

void RingBuffer_Init(RingBuffer *rb);
uint8_t RingBuffer_IsEmpty(RingBuffer *rb);
uint8_t RingBuffer_IsFull(RingBuffer *rb) ;
uint8_t RingBuffer_Push(RingBuffer *rb, uint8_t data) ;
uint16_t RingBuffer_Pop(RingBuffer *rb, uint8_t *dst, uint16_t len);

#endif
