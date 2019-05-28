#pragma once
#include "vehicle.h"
#include "CAGD_NURBS.h"
typedef CArray<double,double>CDoubleArray;

class CLaneDriver :
	public CVehicle
{
public:
	CLaneDriver(void);
	int LaneDrive(int &num,CvPoint2D64f (&MPoint)[512]);
	void zFitLanesCenter(IplImage *src, IplImage *dst);
	void ProcCenterLane(I_Map *map_src,I_Map *map_dst);
	int KeepLane(CvPoint2D64f rPosition,double rDirection,CvPoint2D64f (&Lane_MidGpsPoint)[200]);
	
	int SearchMidLane(I_Map *map,CvPoint2D64f MidPoint[]);
	int SearchFromMidLane(I_Map *map,CvPoint2D64f *MidPoint,int &num);
	int YanChang(CvPoint2D64f midpoint[],int length,CvPoint2D64f (&shift_point)[200]);
	bool SearchMidLane1(I_Map *map,CvPoint2D64f &StartPoint,int count1);
	bool GetLaneMap(I_Map &ori_map,I_Map &nwmap);
	bool GetLaneMap1(I_Map &ori_map);
	int getRndfWjDir(I_Map &map, CvPoint2D64f m_gps,double dir,double &goal_dir,CvPoint2D64f *cpath,int &num);
	BOOL CalculateCurveParameter(CDoubleArray *X,CDoubleArray *Y,long M,long N,CDoubleArray *A);
	int Calculate2cicurveparam(CvPoint2D64f p1,CvPoint2D64f p2,CvPoint2D64f p3,double (&param)[3]);
	int SearchNiXing(CvPoint2D64f *waypoint);
	int Yanchangs(double lanedir,double dir,double s,CvPoint2D64f v,CvPoint2D64f &aim);
	int GetPointfromNearMap(CvPoint2D64f *path,int num,CvPoint2D64f cent,double s,	CvPoint2D64f &res);
	int GetPointfromNearGPS(CvPoint2D64f *path,int num,CvPoint2D64f m_gps,double s,CvPoint2D64f &res);
	int GetPathDirGPS(CvPoint2D64f *CrossPath,int num,CvPoint2D64f m_gps,double dir,double &goal_dir);
	int GetPathDirMap(CvPoint2D64f *CrossPath,int num,CvPoint2D64f m_gps,double dir,double &goal_dir);
	int Get2Curve(CvPoint2D64f p1,CvPoint2D64f p2,CvPoint2D64f p3,CvPoint2D64f (&curve)[200]);
	int GetPointNumfromNearGPS(CvPoint2D64f *path,int num,CvPoint2D64f m_gps,double s);
	int SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down);
	int SearchObstacle_laneyichang(CvPoint2D64f GPSpoint[],I_Map *map);

	int SearchObstacle_lanenotusedcheck(CvPoint2D64f GPSpoint[],I_Map *map,double lengtherr);

	int SearchMoveStaticObstacleGps(CvPoint2D64f *waypoint);
	int SearchObstacleLuyan(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down);
	int SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,double w);
	int SearchObstacle1(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_frontnear(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down);
	int SearchObstacle1_leftnear(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);

	int SearchObstacle1_leftnear_new(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int upnew,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_rightnear_new(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int upnew,int up,int down/*,int &ob_x*/,double &ob_y);

	int SearchObstacle1_leftlaneobdist(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_rightlaneobdist(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);

	int SearchObstacle1_rightnear(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_leftnextlane(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_rightnextlane(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_lanechange(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_lanechangenot(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down);
	int SearchObstacle1_vehlanechange(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_vehlanechange111(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_lanechange_left(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_lanechange_mid(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle1_VelMapPath_left(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left);
	int SearchObstacle1_VelMapPath_right(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left);

	int SearchObstacle1_VelMapPath_leftini(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left);
	int SearchObstacle1_VelMapPath_rightini(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left);

	int SearchObstacle1_lanechange_right(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle_leftedge(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle_rightedge(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle_leftedge_rightside(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle_rightedge_rightside(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle111(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int SearchObstacle222(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y);
	int CLaneDriver::SearchObstacleMoveStill(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_x,int &ob_y);
	int CLaneDriver::SearchMoveStaticObstacleGps1(CvPoint2D64f *waypoint);

	void SearchObstacle1_lchgconcheck(CvPoint2D64f GPSpoint[],I_Map *map,double &ob_y_front,double &ob_y_rear);

	double y_left_front[5];
	double y_left_rear[5];
	double y_right_front[5];
	double y_right_rear[5];
	double y_middle_front[5];

	int timer_left_front[5];
	int timer_left_rear[5];
	int timer_right_front[5];
	int timer_right_rear[5];
	int timer_middle_front[5];

	double spdrelmidfront;
	double spdrelleftfront;
	double spdrelleftrear;
	double spdrelrightfront;
	double spdrelrightrear;

	//I_Map *lane_mid_map;
private:
	CvPoint2D64f Lane_MidGpsPoint[200];
	CvPoint2D64f his_MidPoint[200];
	CvPoint2D64f MidPoint[200];
	//CvPoint2D64f *MidPoint;
	CvPoint2D64f shift_MidPoint[200];
	CvPoint2D64f lane_gps;
	double lane_orientation;
	CvPoint2D64f ori_LaneMidGpsPoint[200];
	//I_Map *lane_Map;
	//static I_Map *vel_Map;
	I_Map *lane_perception_map;
	CvPoint2D64f TempPoint_Vehicle[501];
	CvPoint2D64f TempPoint[500];
	int search_res;
	I_Map *temp_map;
	int midd_num;
	CvPoint2D64f shift_midmap[512];
//	CvPoint2D64f *MidPoint1;
	CvPoint2D64f MidPoint1[512];
	double nearmove_obcount;
	double stillmove_obcount;
	int count_ob1;
	int count_ob2;
	int b,c;
	CvPoint2D64f exe_leftpoint[200];
	CvPoint2D64f exe_rightpoint[200];
public:
	~CLaneDriver(void);
	int getRndfWjMapPath(I_Map &map,CvPoint2D64f (&cpath)[512],int &num);
	double GetMinDis(CvPoint2D64f *path,int num,CvPoint2D64f m_gps);
	int GetMinDisPointGPS(CvPoint2D64f *path,int num,CvPoint2D64f m_gps);
	bool SearchObstacleLukou(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	bool SearchPeople(CvPoint2D64f GPSpoint[],I_Map *map,int width,int up,int &dis);
	bool GetLaneMap2(I_Map &ori_map);
	int GetMinNum(CvPoint2D64f *path,int num,CvPoint2D64f m_gps);
	void CLaneDriver::GetBSpline1(LEAD *m_vgpspoints,int gpssize, CvPoint2D64f *curve);
	double SearchFrontOb(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	double SearchRearOb(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	double SearchRearObForApa(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	double SearchrightFrontOb(I_Map *map,double range);
	double SearchleftFrontOb(I_Map *map,double range);
	int SearchkongbaiFrontOb(I_Map *map,double range);

	int SearchkongbaiFrontObnew(CvPoint2D64f GPSpoint[],I_Map *map,double range);

	double SearchFrontTurnVehOb(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	int SearchObstacle2(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y, int dir)/*靠边，uturn检测 */;
	bool boblefthis;
	bool bobrighthis;
	bool boblefthispre;
	bool bobrighthispre;

	bool bwaitlchg;

	int bobleftlchgnotalooweddisp;
	int bobrightlchgnotalooweddisp;
	int bobfrontlchgnotalooweddisp;
	int bobfrontlchgreqdisp;

	int up_sidelaneleft;
	int up_sidelaneright;
};
