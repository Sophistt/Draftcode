#pragma once
#include "intersectionstate.h"

class CIntersectionStopWait :
	public CIntersectionState
{
public:
	CIntersectionStopWait(void);
	void CheckLightDrive();
    void IntersectionStopWaitDrive();
    void PassIntersectionDirve();
    void CheckSignDirve();
public:
	~CIntersectionStopWait(void);
};
