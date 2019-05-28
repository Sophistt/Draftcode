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
	 long costTime;            // 花费的时间(精确到微秒)
	 void Start() ;            //开始计时
	 void End();              //结束计时
	 void Reset() ;           //计时器重置
public:
	~My_Timer(void);
};
#endif
