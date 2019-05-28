#pragma once
#include "vehicle.h"
#include "GetGPSData.h"
#include "ComuLcm.h"
#include "vehicle.h"

class new_uturn :
	public CVehicle
{
public:
	new_uturn(void);
	void UturnPath();
	void SearchMidPoint(I_Map *map,CvPoint2D64f up_point,CvPoint2D64f down_point,bool up_geli,bool down_geli,double k,CvPoint2D64f &MidPoint);
	void SearchLine(CvPoint2D64f (&line)[512],int &l_num);
	void CountLine(CvPoint2D64f *line,int l_num,double &k,double &b);
	CvPoint2D64f CountPedal(CvPoint2D64f ori,CvPoint2D64f MidPoint,double k);
	void ChaZhi(CvPoint2D64f pedal,CvPoint2D64f pointt1,CvPoint2D64f point2,CvPoint2D64f MidPoint,CvPoint2D64f (&upoint)[11]);
	bool MovePoint(CvPoint2D64f *pathpoint,I_Map *map);

public:
	~new_uturn(void);
private:
	ComuLcm uturn_state;
	bool down_flag;
	bool up_flag;
};
