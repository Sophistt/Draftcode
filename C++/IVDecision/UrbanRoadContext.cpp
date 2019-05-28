#include "StdAfx.h"
#include "UrbanRoadContext.h"
CApproachExit* CUrbanRoadContext::pApproachExitState = NULL;
CChangeLane* CUrbanRoadContext::pChangeLaneState = NULL;
CDistKeeper* CUrbanRoadContext::pDistKeeperState = NULL;
CLaneDriver* CUrbanRoadContext::pLaneDriverState = NULL;
CStopWait* CUrbanRoadContext::pStopWaitState = NULL;
CUrbanRoadContext::CUrbanRoadContext(void)
{
	m_pUrbanRoadState = NULL;
	pApproachExitState = new CApproachExit();
	pChangeLaneState = new CChangeLane();
	pDistKeeperState = new CDistKeeper();
	pLaneDriverState = new CLaneDriver();
	pStopWaitState = new CStopWait();
}

CUrbanRoadContext::~CUrbanRoadContext(void)
{
	delete pApproachExitState;
	pApproachExitState = NULL;
	delete pChangeLaneState;
	pChangeLaneState = NULL;
	delete pDistKeeperState;
	pDistKeeperState = NULL;
	delete pLaneDriverState;
	pLaneDriverState = NULL;
	delete pStopWaitState;
	pStopWaitState = NULL;
}
CUrbanRoadState * CUrbanRoadContext::GetUrbanRoadState()
{
    return m_pUrbanRoadState;
}

void CUrbanRoadContext::SetUrbanRoadState(CUrbanRoadState *pUrbanRoadState)
{
    this->m_pUrbanRoadState = pUrbanRoadState;
    this->m_pUrbanRoadState->SetUrbanRoadContext(this);
}

void CUrbanRoadContext::ApproachExitDirve()
{
    this->m_pUrbanRoadState->ApproachExitDirve();
}

void CUrbanRoadContext::ChangeLaneDirve()//lr:×ó±ß»òÕßÓÒ±ß
{
    this->m_pUrbanRoadState->ChangeLaneDirve();
}

void CUrbanRoadContext::DistKeeperDirve()
{
    this->m_pUrbanRoadState->DistKeeperDirve();
}

void CUrbanRoadContext::StopWaitDrive()
{
    this->m_pUrbanRoadState->StopWaitDrive();
}
void CUrbanRoadContext::LaneDrive()
{
    this->m_pUrbanRoadState->LaneDrive();
}
//void CUrbanRoadContext::set(a)
