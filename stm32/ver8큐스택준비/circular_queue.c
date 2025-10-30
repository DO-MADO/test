/* circular_queue.c */
#include "circular_queue.h"
#include <stdio.h> // printf, fprintf 등 (디버그용)

// 에러 핸들러 (참고용 코드와 동일)
void error(char *message){
    fprintf(stderr,"//%s\n",message);
    //printf(" ERROR!!!!!");
}

void init(QueueType *q){
    q->front = 0;
    q->rear = 0;
} 

int is_empty(QueueType *q){
    return q->front == q->rear;
}

int is_full(QueueType *q){
    return (q->rear+1) % MAX_QUEUE_SIZE == q->front;
}

void enqueue(QueueType *q, element item){
    int next = (q->rear + 1) % MAX_QUEUE_SIZE;
    if (next == q->front) {
        // FULL → 가장 오래된 것 버림
        q->front = (q->front + 1) % MAX_QUEUE_SIZE;
    }
    q->rear = next;
    q->queue[q->rear] = item;
}

element dequeue(QueueType *q){
    if (is_empty(q)){
        error("q is empty");
        //printf("q is empty");
        return 0; // 비어있으면 0 반환
    }
    q->front = (q->front+1) % MAX_QUEUE_SIZE;
    return q->queue[q->front];
}