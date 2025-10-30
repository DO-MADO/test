/* circular_queue.h */
#ifndef __CIRCULAR_QUEUE_H
#define __CIRCULAR_QUEUE_H

#include "main.h" // element 타입을 위해 uint8_t 등이 필요

/* ----------------------------------------------------
 * 큐 설정 (ver7에 맞게 수정)
 * ---------------------------------------------------- */
 
// 1. 큐의 1개 아이템 = 1바이트
typedef uint8_t element;

// 2. 큐의 최대 크기 (수신 버퍼 크기와 동일하게 설정)
//    main.c의 RDV2_RX_BUFSZ (1024)와 맞춥니다.
#define MAX_QUEUE_SIZE 1024 

/* ----------------------------------------------------
 * 큐 구조체 정의
 * ---------------------------------------------------- */
typedef struct {
    element queue[MAX_QUEUE_SIZE];
    volatile int front;   // ★
    volatile int rear;    // ★
} QueueType;

/* ----------------------------------------------------
 * 큐 함수 프로토타입
 * ---------------------------------------------------- */
void init(QueueType *q);
int is_empty(QueueType *q);
int is_full(QueueType *q);
void enqueue(QueueType *q, element item);
element dequeue(QueueType *q);

#endif