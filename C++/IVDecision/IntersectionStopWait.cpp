#include "StdAfx.h"
#include "IntersectionStopWait.h"
#include "IntersectionContext.h"
CIntersectionStopWait::CIntersectionStopWait(void)
{
}

CIntersectionStopWait::~CIntersectionStopWait(void)
{
}
void CIntersectionStopWait::CheckLightDrive()
{}
void CIntersectionStopWait::IntersectionStopWaitDrive()
{}
void CIntersectionStopWait::PassIntersectionDirve()
{
	this->CIntersectionState::m_IntersectionContext->SetIntersectionState(CIntersectionContext::pPassIntersectionState);
	this->CIntersectionState::m_IntersectionContext->GetIntersectionState()->PassIntersectionDirve();
}
void CIntersectionStopWait::CheckSignDirve()
{}