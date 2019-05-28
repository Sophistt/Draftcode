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

void My_Timer:: Start()            // 开始计时
{
	QueryPerformanceCounter(&_begin);
}

void My_Timer:: End()                // 结束计时
{
	QueryPerformanceCounter(&_end);
	costTime = (long)((_end.QuadPart - _begin.QuadPart) * 1000000 / _freq);
}

void My_Timer:: Reset()            // 计时清0
{
	costTime = 0;
}
