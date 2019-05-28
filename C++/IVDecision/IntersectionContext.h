#pragma once
#include "IntersectionState.h"
#include "IntersectionStopWait.h"
#include "PassIntersection.h"
#include "CheckLight.h"
#include "CheckSign.h"
class CIntersectionContext
{
public:
	CIntersectionContext(void);
	static CIntersectionStopWait *pIntersectionStopWaitState;
    static CPassIntersection *pPassIntersectionState;
    static CCheckLight *pCheckLightState;
    static CCheckSign *pCheckSighState;
    CIntersectionState * GetIntersectionState();
    void SetIntersectionState(CIntersectionState *pIntersectionState);
	void CheckLightDrive();
    void IntersectionStopWaitDrive();
    void PassIntersectionDirve();
    void CheckSignDirve();
private:
	CIntersectionState *m_pIntersectionState;
public:
	~CIntersectionContext(void);
};
