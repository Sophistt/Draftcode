#pragma once
#include "vehicle.h"
#include "GetGPSData.h"
#include "ComuLcm.h"
#include "LoadRNDF.h"
class CUTurn :
	public CVehicle
{
public:
	CUTurn(void);
	void ReturnUTurnPath(CvSeq *gp,int seq,CvPoint2D64f (&uturnpath)[200]);
	bool SearchUTurnObstacleMoveWay(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	//double Countk(CvPoint2D64f *line,int l_num);
	CvPoint2D64f CountChuiZu(CvPoint2D64f ori,CvPoint2D64f MidPoint,double k);
	bool SearchUTurnObstacleMoveWay1(CvPoint2D64f Rndf_MapPoint[],I_Map *map);
	bool SearchUTurnObstacleMoveWay1_rightside(CvPoint2D64f Rndf_MapPoint[],I_Map *map);
	bool SearchUTurnObstacleMoveWay1_rightsidethird(CvPoint2D64f Rndf_MapPoint[],I_Map *map);
	CvPoint2D64f SearchHuokou(I_Map *map,int&xia_y,int&xia_x,int lane_num,CvPoint2D64f *rndfpath);
	bool SearchKongBai(I_Map *map);
	CvPoint2D64f SearchZuoKongBai(I_Map *map,double k);
	void ReturnUTurnKongBaiPath();
	void ReturnUTurnPathRNDF(int seq,CvPoint2D64f (&upathyc)[200],CvPoint2D64f (&upoint)[4],int lane_num);
	CLoadRNDF RNDFGPS;

	double xdisini;
	int xia_y_pub;
public:
	~CUTurn(void);
	int YanChang(CvPoint2D64f m_gps, double m_dir,CvPoint2D64f *op, double dir,CvPoint2D64f *yanp);
	void ReturnUTurnPath1(CvSeq *gp,int seq,CvPoint2D64f (&upathyc)[200]);
private:
	ComuLcm uturn_state;
};
