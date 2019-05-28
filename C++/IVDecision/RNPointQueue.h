#pragma once
#define QUEUE_SIZE 100 


template <class T>  
struct NODE  
{  
	NODE<T> *next;  
	T data;  
};

template <class T> 
class CRNPointQueue
{
public:
	CRNPointQueue(void);
	~CRNPointQueue(void);

public:
	NODE<T> *front;	//指向队头  
	NODE<T> *rear;	//指向队尾

public:

	T Pop();
	void Push(T e);
	T Front();
	T Back();
	int Size();
	BOOL Empty();

};

template <class T>  
CRNPointQueue<T>::CRNPointQueue(void)
{
	NODE<T> *p =new NODE<T>; 
	//p->data = NULL;  
	p->next = NULL;  
	front = p;  
	rear = p;  
}

template <class T>
CRNPointQueue<T>::~CRNPointQueue(void)
{
}

template <class T>  
void CRNPointQueue<T>::Push(T e)
{
	NODE<T> *p =new NODE<T>; 
	if (Size() >= QUEUE_SIZE)
	{
		Pop();
	}
	p->data = e;  
	p->next = NULL;  
	rear->next = p;  
	rear = p;  
}

template <class T>  
T CRNPointQueue<T>::Pop()
{
	T e;
	memset(&e,0,sizeof(e));
	if(front==rear) 
		return e; 
	NODE<T> *p; 
	p=front; 

	e=p->data; 
	front=p->next; 
	if(rear==p) rear=front; 
	free(p);
	return e;
}

template <class T>  
T CRNPointQueue<T>::Front()
{
	T e;
	memset(&e,0,sizeof(e));
	if (front == rear)
	{
		return e;
	}
	else
	{
		NODE<T> *p = front->next;  
		e =  p->data;
		return e; 
	}
}

template <class T>  
T CRNPointQueue<T>::Back()
{
	T e;
	memset(&e,0,sizeof(e));
	if (front == rear)  
	{  
		return e;  
	}  
	else  
	{  
		e = rear->data;
		return e;  
	}  
}

template <class T>  
int CRNPointQueue<T>::Size()
{
	int count(0);  

	NODE<T> *p = front;  

	while (p != rear)  
	{  
		p = p->next;  
		count++;  
	}  
	return count;  
}

template<class T>
BOOL CRNPointQueue<T>::Empty()
{
	if (front == rear)  
	{  
		return TRUE;  
	}  
	else  
	{  
		return FALSE;  
	}  
}

