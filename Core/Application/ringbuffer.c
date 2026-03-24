#include "ringbuffer.h"


RingBuffer sevenway_buffer;


void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    memset(rb->buffer, 0, RING_BUFFER_SIZE);
}


// 缓冲区是否为空
uint8_t RingBuffer_IsEmpty(RingBuffer *rb) {
    return (rb->head == rb->tail);
}

// 缓冲区是否为满
uint8_t RingBuffer_IsFull(RingBuffer *rb) {
    return ((rb->head + 1) & (RING_BUFFER_SIZE - 1)) == rb->tail;  // 保留一个空位判断法[7](@ref)
}

uint8_t RingBuffer_Push(RingBuffer *rb, uint8_t data) {
    uint32_t primask = __get_PRIMASK();  // 保存中断状态
    __disable_irq();                     // 关中断
    
    if (RingBuffer_IsFull(rb)) {
        __set_PRIMASK(primask);          // 恢复中断状态
        return 0;
    }
    
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) & (RING_BUFFER_SIZE - 1);
    
    __set_PRIMASK(primask);
    return 1;
}

uint16_t RingBuffer_Pop(RingBuffer *rb, uint8_t *dst, uint16_t len) {
    uint16_t available = (rb->head >= rb->tail) ? 
                        (rb->head - rb->tail) : 
                        (RING_BUFFER_SIZE - rb->tail + rb->head);
    len = fmin(len, available);

    // 分两段拷贝处理回绕
    uint16_t first_chunk = RING_BUFFER_SIZE - rb->tail;
    if (len > first_chunk) {
        memcpy(dst, &rb->buffer[rb->tail], first_chunk);
        memcpy(dst + first_chunk, rb->buffer, len - first_chunk);
    } else {
        memcpy(dst, &rb->buffer[rb->tail], len);
    }
    
    rb->tail = (rb->tail + len) & (RING_BUFFER_SIZE - 1);
    return len;
}

