#pragma once
#include "intersectionstate.h"

class CCheckSign :
	public CIntersectionState
{
public:
	CCheckSign(void);
	void CheckLightDrive();
    void IntersectionStopWaitDrive();
    void PassIntersectionDirve();
    void CheckSignDirve();
public:
	~CCheckSign(void);
};
