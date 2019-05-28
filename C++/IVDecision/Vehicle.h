#pragma once
#include "structhead.h"
#include "IVDecision.h"
#include "GetGPSData.h"
#include "ComuLcm.h"
class CVehicle
{
public:
	CVehicle(void);
public:
	~CVehicle(void);
	virtual void setEnable(bool option);
	virtual bool getEnable();
	//5.31日志记录功能
	//virtual void setName(const char *name, bool hash); //任务命名
    //virtual const char *getName() const;

	//5.31车辆初始化
	
	virtual void setPerceptionNetwork (void); //建立数据连接。
	void setSignalNetwork (void) ;
	virtual I_Map* getPerceptionMap();
	//virtual void setRoadNet (const char *rndf_name, bool hash); //建立路网模型。
	bool setStartState (CDriveState *state=NULL); //设置车辆起始状态（路上/路口、路点号、方向、速度等）Return SUCCESS or FAILURE

	
	void ArrayFuZhi(CvPoint2D64f a,CvPoint2D64f *b);
	void ArrayFuZhi(CvPoint2D64f *a,CvPoint2D64f *b);
	int GetSendGps(CvPoint2D64f m_gps,double m_gpsdir,CvPoint2D64f *MidPoint,CvPoint2D64f (&MidGpsPoint)[200],int num);
	void Bezier(CvPoint2D64f p[],int n,CvPoint2D64f *MPoint,int m);
	void Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[200]);
	void Bezierlukou(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[200]);
	void Beziercazhi(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[200]);
	
	double ParaDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps);
	int Stop();
	double VertDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps);
	double LevelDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps);

	
	int MoveLeft(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length);
	int MoveLeft(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length,CvPoint2D64f &err);
	int MoveRight(CvPoint2D64f WayPoint[],CvPoint2D64f RightWayPoint[],double length);
	I_Map* getVelMap();
	I_Map* getLaneMidMap();
	CvSeq* getPlanRoad();
	static CvSeq* global_path;
	CvPoint2D64f globalendpoint;
	void seq_fuzhi(int object);
	static CGetGPSData m_GpsData;
	double rad(double d) { return d * PI / 180.0; };
	void PathFuZhi(CvPoint2D64f goal,CvPoint2D64f *b);
	int MoveLeft1(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length);
	int MoveRight2(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length);
	int MoveLeft2(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length);
	int MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f APoint[],int num,CvPoint2D64f WayPoint[]);
	int MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f WayPoint[]);
	double Countk(CvPoint2D64f *line,int l_num);
	int MoveWay(CvPoint2D64f m_gps, CvPoint2D64f WayPoint[],int num);
	int MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f WayPoint[],CvPoint2D64f &err);
	int MoveLeftDir(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length,double dir1);
	int MoveLeftDir(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length,double dir1,CvPoint2D64f &err);
	double returnDist(LEAD a, CvPoint2D64f b,double dir);
	static int seq_num;
	static int fx;
	bool whole_range;


	I_Map* getXinMap();
	I_Map* getDangerMap();
	I_Map* getVMap();

	I_Map* getDyMap();
	static I_Map* dy_map;
	static I_Map* map_ts;//时空障碍物栅格图
	static I_Map* map_v;
	static I_Map* map_dir;
	static I_Map* map_dan;
	static I_Map* map_ts1;

	static v_Map* map_v_x;
	static v_Map* map_v_y;
	static I_Map* static_Map;
	static I_Map* dynamic_Map;
	static v_Map* dynamicmap_v_x;
	static v_Map* dynamicmap_v_y;

	static I_Map* lukou_Map;
	static v_Map* lukou_v_x;
	static v_Map* lukou_v_y;

protected:
	static CvMemStorage* storage_road;
	static I_Map *vel_Map;
	static I_Map *vel_Map_path;
	static CvSeq* planning_road;
	static I_Map *lane_mid_map;
	static CvPoint2D64f MidGpsPoint[200];
	static CvPoint2D64f search_obGpsPoint[200];
	static CvPoint2D64f IntersectionMidGpsPoint[200];
	static int mission_num;
	static int mission_num2;
	static LEAD *LeadPoint;
	static LEAD *MissionPoint;	
	
	static CvPoint2D64f OriGpsPoint[200];
	static int vehicleob_x;
	static int vehicleob_y;

	static I_Map *realtime_Map;
	static CvPoint2D64f realtime_Gps;
	static double realtime_Dir;
	static bool way_out;
	static int intersection_obnum;
	//static CvPoint2D64f MidPoint1[512];
private:
	CvMemStorage* veiw_storage_road;
	CvSeq* view_vechile_planning_road;
	I_Map *view_vechile_mid_map;
	I_Map *view_vechile_vel_map;
	I_Map *view_vechile_ob_map;
	I_Map *view_vechile_dy_map;

	bool vehicle_upstate;
	CvPoint2D64f vehicle_pos;
	double vehicle_orientation;
	double vehicle_speed;
	
	ComuLcm perception_network;
	double vehicle_distance;
	I_Map *perception_map;
	double vehicle_angle;
	Map_Point map_point;
	CvPoint2D64f doumap_point;
	int vehicle_seq_num;
};
