#pragma once
#include "vehicle.h"
class CIntersectionContext;
class CIntersectionState :
	public CVehicle
{
public:
	CIntersectionState(void);
	void SetIntersectionContext(CIntersectionContext *IntersectionContext);
	virtual void CheckLightDrive() = 0;
    virtual void IntersectionStopWaitDrive() = 0;
    virtual void PassIntersectionDirve() = 0;
    virtual void CheckSignDirve() = 0;
protected:
	CIntersectionContext *m_IntersectionContext;
public:
	~CIntersectionState(void);
};
