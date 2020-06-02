#ifndef __QUEUE_H
#define __QUEUE_H

#define RX_QUEUE_SIZE 200

struct queue 
{
	char buffer[RX_QUEUE_SIZE];
	int front;
};

void enqueue(struct queue *qu, char element) {
	if (qu->front < RX_QUEUE_SIZE)
	{
		qu->buffer[qu->front] = element;
		++qu->front;
	}
}

void dequeue(struct queue *qu, char result[100])
{
	int i = 0;
	if (qu->front > 0)
	{
		for (; i < qu->front; ++i) 
		{
			result[i] = qu->buffer[i];
		}
		qu->front = 0;
	}
}

#endif
