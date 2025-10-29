#ifndef __CIRCULAR_QUEUE_H
#define __CIRCULAR_QUEUE_H

#include "circular_queue.h"


/*

//USART 
QueueType q1,q2,q3,q4;


*/


//
void error(char *message){
    fprintf(stderr,"//%s\n",message);
    //printf(" ERROR!!!!!");
    //exit(1);
}
// 초기 front와 rear를 0으로 하는 이유는
// empty와 full을 구분하기 위한 index 0을 비우기 위해서다.
void init(QueueType *q){
    q->front = 0;
    q->rear = 0;
    //printf(" Q init!!!!!");
} 

int is_empty(QueueType *q){
    return q->front == q->rear;
}

// % 연산자는 제한된 index를 반복하기 위해서 사용한다
int is_full(QueueType *q){
    return (q->rear+1) % MAX_QUEUE_SIZE == q->front;
}

void enqueue(QueueType *q, element item){
    if (is_full(q)){
        error("q is full");
        printf("enqueue q is full");
        init(q);
        //exit(1);
        
    }
    q->rear = (q->rear+1) % MAX_QUEUE_SIZE;
    q->queue[q->rear] = item;
}

element dequeue(QueueType *q){
    if (is_empty(q)){
        error("q is empty");
        //printf("q is empty");
        //exit(1);
return 0;
    }
    q->front = (q->front+1) % MAX_QUEUE_SIZE;
    return q->queue[q->front];
}

element peek(QueueType *q){
    if (is_full(q)) {
        error("q is full");
        printf("peek q is full");
        //exit(1);
    }
    return q->queue[(q->front+1) % MAX_QUEUE_SIZE];
}

#endif