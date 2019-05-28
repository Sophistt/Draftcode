#include "StdAfx.h"
#include "IntersectionContext.h"
CIntersectionStopWait *CIntersectionContext::pIntersectionStopWaitState = NULL;
CPassIntersection *CIntersectionContext::pPassIntersectionState = NULL;
CCheckLight *CIntersectionContext::pCheckLightState = NULL;
CCheckSign *CIntersectionContext::pCheckSighState = NULL;
CIntersectionContext::CIntersectionContext(void)
{
	m_pIntersectionState = NULL;
	pIntersectionStopWaitState = new CIntersectionStopWait();
	pPassIntersectionState = new CPassIntersection();
	pCheckLightState = new CCheckLight();
	pCheckSighState = new CCheckSign();
}

CIntersectionContext::~CIntersectionContext(void)
{
	delete pIntersectionStopWaitState;
	pIntersectionStopWaitState = NULL;
	delete pPassIntersectionState;
	pPassIntersectionState = NULL;
	delete pCheckLightState;
	pCheckLightState = NULL;
	delete pCheckSighState;
	pCheckSighState = NULL;
}

CIntersectionState * CIntersectionContext::GetIntersectionState()
{
	return m_pIntersectionState;
}
void CIntersectionContext::SetIntersectionState(CIntersectionState *pIntersectionState)
{
	m_pIntersectionState = pIntersectionState;
}
void CIntersectionContext::IntersectionStopWaitDrive()
{
	this->pIntersectionStopWaitState->IntersectionStopWaitDrive();
}
void CIntersectionContext::PassIntersectionDirve()
{
	this->pPassIntersectionState->PassIntersectionDirve();
}
void CIntersectionContext::CheckLightDrive()
{
	this->pCheckLightState->CheckLightDrive();
}
void CIntersectionContext::CheckSignDirve()
{
	this->pCheckSighState->CheckSignDirve();
}