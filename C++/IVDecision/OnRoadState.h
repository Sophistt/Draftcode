#pragma once
//class COnRoadContext
class COnRoadState
{
public:
	COnRoadState(void);
//    void SetContext(COnRoadContext *OnRoadContext);
    virtual void UrbanRoadDrive() = 0;
    virtual void UTurnDirve() = 0;
    virtual void DistKeeperDirve() = 0;
    virtual void ChangeLaneDirve() = 0;
protected:
//    COnRoadContext *m_OnRoadContext;
public:
	virtual ~COnRoadState(void);
};
