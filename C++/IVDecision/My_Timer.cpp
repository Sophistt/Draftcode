#include "StdAfx.h"
#include "My_Timer.h"

My_Timer::My_Timer(void)
{
	LARGE_INTEGER tmp;
	QueryPerformanceFrequency(&tmp);
	_freq = tmp.QuadPart;
	costTime = 0;
}

My_Timer::~My_Timer(void)
{
}

void My_Timer:: Start()            // ��ʼ��ʱ
{
	QueryPerformanceCounter(&_begin);
}

void My_Timer:: End()                // ������ʱ
{
	QueryPerformanceCounter(&_end);
	costTime = (long)((_end.QuadPart - _begin.QuadPart) * 1000000 / _freq);
}

void My_Timer:: Reset()            // ��ʱ��0
{
	costTime = 0;
}
