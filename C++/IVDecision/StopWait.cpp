#include "StdAfx.h"
#include "StopWait.h"
#include "UrbanRoadContext.h"
CStopWait::CStopWait(void)
{
}

CStopWait::~CStopWait(void)
{
}
void CStopWait::ChangeLaneDirve()
{
	this->CUrbanRoadState::m_UrbanRoadContext->SetUrbanRoadState(CUrbanRoadContext::pChangeLaneState);
	this->CUrbanRoadState::m_UrbanRoadContext->GetUrbanRoadState()->ChangeLaneDirve();
}
void CStopWait::DistKeeperDirve()
{
	this->CUrbanRoadState::m_UrbanRoadContext->SetUrbanRoadState(CUrbanRoadContext::pDistKeeperState);
	this->CUrbanRoadState::m_UrbanRoadContext->GetUrbanRoadState()->DistKeeperDirve();
}
void CStopWait::StopWaitDrive()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "停车";
	app->critical_section.Lock();//锁住
	m_gps = app->GPS_Point;	
	app->critical_section.Unlock();//解锁
	app->critical_planningroad.Lock();
	cvClearSeq( planning_road );
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( planning_road, &m_gps);
	}
	app->critical_planningroad.Unlock();
	SetEvent(app->m_hEvent);//路径更新后，发送消息
	Sleep(1000);
}
void CStopWait::LaneDrive()
{
	this->CUrbanRoadState::m_UrbanRoadContext->SetUrbanRoadState(CUrbanRoadContext::pLaneDriverState);
	this->CUrbanRoadState::m_UrbanRoadContext->GetUrbanRoadState()->LaneDrive();
}
void CStopWait::ApproachExitDirve()
{
	this->CUrbanRoadState::m_UrbanRoadContext->SetUrbanRoadState(CUrbanRoadContext::pApproachExitState);
	this->CUrbanRoadState::m_UrbanRoadContext->GetUrbanRoadState()->ApproachExitDirve();
}
int CStopWait::Stop(CvPoint2D64f m_gps)
{
	
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	return 0;
}