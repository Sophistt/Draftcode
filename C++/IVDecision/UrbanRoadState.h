#pragma once
#include "vehicle.h"
class CUrbanRoadContext;
class CUrbanRoadState :
	public CVehicle
{
public:
	CUrbanRoadState(void);
	void SetUrbanRoadContext(CUrbanRoadContext *UrbanRoadContext);
	virtual void LaneDrive() = 0;
    virtual void StopWaitDrive() = 0;
    virtual void DistKeeperDirve() = 0;
    virtual void ChangeLaneDirve() = 0;
	virtual void ApproachExitDirve()  =0;
protected:
    CUrbanRoadContext *m_UrbanRoadContext;
public:
	~CUrbanRoadState(void);
};



