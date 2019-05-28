#pragma once

#include "vehicle.h"
#include "GetGPSData.h"
#include "ComuLcm.h"

class auturn:public CVehicle
{
public:
	auturn(void);
	void ReturnUTurnPath();
	bool SearchUturnObstacleMoveWay(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	double Countk(CvPoint2D64f *line,int l_num);
	CvPoint2D64f CountChuiZu(CvPoint2D64f Rndf_MapPoint[],I_Map *map);
	bool SearchUturnObstacleMoveWay1(CvPoint Rndf_MapPoint[],I_Map *map);
	CvPoint2D64f SeachHuokou(I_Map *map,int&xia_y);
	bool SearchKoingBai(I_Map *map);

public:
	~auturn(void);
private:
	ComuLcm uturn_state;
};
