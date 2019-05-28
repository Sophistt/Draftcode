#pragma once
#include "UrbanRoadState.h"
#include "ApproachExit.h"
#include "ChangeLane.h"
#include "DistKeeper.h"
#include "LaneDriver.h"
#include "StopWait.h"
class CUrbanRoadContext
{
public:
	CUrbanRoadContext(void);
	static CApproachExit *pApproachExitState;
    static CChangeLane *pChangeLaneState;
    static CDistKeeper *pDistKeeperState;
    static CLaneDriver *pLaneDriverState;
	static CStopWait *pStopWaitState;
    CUrbanRoadState * GetUrbanRoadState();
    void SetUrbanRoadState(CUrbanRoadState *pUrbanRoadState);
	void LaneDrive();
    void StopWaitDrive();
    void DistKeeperDirve();
    void ChangeLaneDirve();
	void ApproachExitDirve();

private:
	CUrbanRoadState *m_pUrbanRoadState;



public:
	~CUrbanRoadContext(void);
};
