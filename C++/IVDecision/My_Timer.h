#pragma once
#ifndef __MyTimer_H__
#define __MyTimer_H__
#include <windows.h>
class My_Timer
{
private:
	int _freq;
	LARGE_INTEGER _begin;
	LARGE_INTEGER _end;
public:
	My_Timer(void);
	 long costTime;            // ���ѵ�ʱ��(��ȷ��΢��)
	 void Start() ;            //��ʼ��ʱ
	 void End();              //������ʱ
	 void Reset() ;           //��ʱ������
public:
	~My_Timer(void);
};
#endif
