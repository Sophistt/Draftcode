#pragma once
#include "vehicle.h"
#include "IVDecision.h"

typedef struct MAPPOINT_star
{
	bool obflag;
	int x;
	int y;
	float g,h,f;
	float d[8];
	struct MAPPOINT_star *par;
	struct MAPPOINT_star *next;
	struct MAPPOINT_star *nearnode[8];
}*PMAPPOINT_star;

class PathPlan:
	public CVehicle
{
public:
	PathPlan(void);
	~PathPlan(void);

	CvMemStorage* storage_road180;
	CvSeq* plan_road;
	CvPoint2D64f his_ap;
	CvPoint2D64f his_ap_tmp;
	double his_hiscost[23];
	double changedis;
	bool bpathfound;

	int KeepLane_lukouwan(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int lukoufx);

	int KeepLanewan(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],bool byulukouzhiflag);

	int AstarPathPlan(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200]);

	int KeepLane(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],bool byulukouzhiflag);
	int KeepLane_lukou(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int lukoufx);
	int KeepLane_lukou1(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int lukoufx);
	void Hermite(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int l1,int l2);
	double ObstacleCost(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range);
	double ObstacleCost_lukou(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range);
	double ObstacleCost_lukouwan(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range);
	int Cross1(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200]);
	int Cross2(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200]);
	int ExceptionPath(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200]);
	double SearchFrontOb(I_Map *map,double range);
	double ObstacleDist(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n);

	int YanChang(CvPoint2D64f m_gps, double m_dir,CvPoint2D64f *op, double dir,CvPoint2D64f *yanp);
	int KeepLane2(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])/*±£³Ö³µµÀ */;
	double ObstacleCost2(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range);
	double SearchFrontObCross(I_Map *map,double range,int n)/*正前 絩ange米内有无障碍 */;
	double SearchObCircle(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	double ObstacleCost3(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range);
	int ExceptionPath2(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])/*异常轨迹 */;

	struct MAPPOINT_star *open,*close;
	bool existopen(struct MAPPOINT_star *Eo,struct MAPPOINT_star *noop);
	bool existclose(struct MAPPOINT_star *Ec,struct MAPPOINT_star *nocl);
	void addopen(struct MAPPOINT_star *pno);
	int deleteopen(struct MAPPOINT_star *pyes);
	void addclose(struct MAPPOINT_star *cno);
	struct MAPPOINT_star *SearchBest();
	void Sort() ;
	
};
