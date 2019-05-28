#pragma once
#include "UrbanRoadState.h"

class CStopWait :
	public CUrbanRoadState
{
public:
	CStopWait(void);
	void LaneDrive();
    void StopWaitDrive();
    void DistKeeperDirve();
    void ChangeLaneDirve();
	void ApproachExitDirve();
	int Stop(CvPoint2D64f m_gps);
private:
	CvPoint2D64f m_gps;
public:
	~CStopWait(void);
};
