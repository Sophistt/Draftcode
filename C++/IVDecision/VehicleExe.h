#pragma once
#include "IVDecision.h"
#include "Astar.h"
#include "vehicle.h"
#include "ApproachExit.h"
#include "ChangeLane.h"
#include "DistKeeper.h"
#include "LaneDriver.h"
#include "StopWait.h"
//#include "IVExeNew.h"
#include "Control.h"
#include "PassIntersection.h"
#include "LoadRNDF.h"
#include "LoadMDF.h"
#include "ComuLcm.h"
#include "UTurn.h"
#include "PathPlan.h"
#include "TSmap.h"
#include "CANBus.h"
#include "DynamicObFuse.h"
#include "My_timer.h"
#include <time.h>
//*******添加ADASIS数据处理线程*******//
//#include "ADASISDataReceiver.h"
//*******添加ADASIS数据处理线程*******//

#define WM_INTERVAL_EXE WM_USER+9998

#define IVTASK_LANE  5001                          // 路上
#define IVTASK_INTERSECTION  5002                  // 路口 
#define IVTASK_APPROINTERSECTION  5005             // 靠近路口

// 后面两个任务基本没用
#define IVTASK_STOPLINE  5003
#define IVTASK_PARKING  5004

/*
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
*/

//CDoubleArray B,W,S;
class CVehicleExe:
	public CVehicle
{
public:
	//CADASISDataReceiver ADASIS_Receiver;
	//CADASISDataReceiver ADASISReceiver;

	CLoadRNDF RNDFGPS;
	CLoadMDF MDF;

	CVehicleExe(void);
	void StartProc();
	DWORD ProcThread();
	static DWORD theProcThread(LPVOID lpParam);

	void StartCtrl();
	DWORD CtrlThread();
	static DWORD theCtrlThread(LPVOID lpParam);
	
	void StartPath();
	DWORD PathThread();
	static DWORD thePathThread(LPVOID lpParam);
	
	void StartLight();
	DWORD LightThread();
	static DWORD theLightThread(LPVOID lpParam);

	//*******添加ADASIS数据处理线程*******//
	//void StartADASIS();
	//DWORD ADASISThread();
	//static DWORD theADASISThread(LPVOID lpParam);
	//*******添加ADASIS数据处理线程*******//

	int GetMissionNum();
	int GetMissionPoint(LEAD lead[],int num);
	bool first_state;
	int PassMissionPoint(CvPoint2D64f m_gps,double dir);
	//bool SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	bool SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int &ob_y);
	int SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	int SearchLeftRightObstacle();
	int InitExe();
	bool SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int &ob_y,int ob_width);
	bool GetSignal(int (&sign)[4]);
	void WaitForSignal();
	void Gettestmap(I_Map &map);
	int GetFangxiang1();
	void SpeedDecision();
	void CVehicleExe::SetSpeedInitState(double temp);
	bool getAimPoint(int num,CvPoint2D64f (&MidPoint)[512],CvPoint2D64f &Point);
	double getAimDir(int num,CvPoint2D64f *MidPoint1,double dir);
	double Kalman(double observe,double his,double &optimal );
	void GetPathString(CvPoint2D64f* path,CString &str,int num,int width,int lane_num);
	bool solvesegment_num1(double x,double y,double dir);//根据无人车当前GPS坐标求其所在车道号
	bool solvesegment_num2(double x1,double y1,double x2,double y2);//根据两点求其所在车道号
	bool solvesegment_num3(double x,double y,double dir);
	double SearchPointToLine(CvPoint2D64f m_Gps,double realtime_Dir);
	int GetPathSeq(CvSeq *path,MAPPOINTexe *p);
	int WaitForSignal1();
	int SendtoPerc();
	int GetFangxiang(CvSeq *gp);
	int SearchLeftRightObstacleGps(CvPoint2D64f *waypoint);
	CvPoint2D64f wj_bezier[200];
	CvPoint2D64f gps_wjbezier[200];
	CvPoint2D64f CrossPath[200];
	CvPoint2D64f rndfbezier[200];
	CvPoint2D64f rndfbezier_lchgdst[200];
	CvPoint2D64f rndfbezierchadian[1991];
	CvPoint2D64f Rndf_MapPointchadian[1991];
	CvPoint2D64f rndfbezierchadian_path[1991];
	CvPoint2D64f Rndf_MapPointchadian_path[1991];
	CvPoint2D64f rndfpath[512];
	CvPoint2D64f prebezier[200];
	double roadcurve[200];
	double roadcurvespdlim[200];
	CvPoint2D64f ob_tempoint;
	double lanechangedistmp;
	int ob_res;
	int lanechangecount;
	int lane_judge;
	int rndf_num;
	int inter_res;
	bool applanechang;
	int yulukoucount;
	CvPoint2D64f temprndf[200];
	bool xz_flag;
////变量
	//int seq_num;
	HANDLE m_MissionEvent;
	//I_Map *vel_Map
	CvMemStorage* storage_path;
	CvSeq* plann_road;
//	CvSeq* global_path;
	CvPoint2D64f aim_point;
	CvPoint2D64f  gps_aimpoint;
	CvPoint2D64f  hisgpsaimpoint;
	CvPoint2D64f  gps_aimpointforrecord;
	double hisgpsaimddir;
	double gps_aimdir;

	CvPoint2D64f  gps_aimpointinidata;
	double gps_aimdirinidata;

	CvPoint2D64f  gps_aimpoint_new[3];
	double gps_aimdir_new[3];

	double aim_dis;
	PathPlan pathplan;
	Point3DInt startp;
	Point3DInt endp;
	CvPoint2D64f show_midpoint;
	double real_dir;
	CvPoint2D64f dirbezier[200];
	int solvenearpointnum;
	double hdl_dir;
	double lanehdldir;
	double lanechang_dis;
	bool lanechang_flag;
	bool able_ChangeLane;

	double lanechangespd;
	double lukoulockspd;
	
	double fasterobspdini;
	bool bfasterobacv;
	bool bobfisrstapper;
	int ob_dis_now;
	int ob_dis_pre;
	int ob_dis_prepre;
	int ob_loss_count;

	int ob_dis_now_timer;
	int ob_dis_pre_timer;
	int ob_dis_prepre_timer;
	
	double fasterobspdini_lchg;
	bool bfasterobacv_lchg;
	bool bobfisrstapper_lchg;
	int ob_dis_now_lchg;
	int ob_dis_pre_lchg;
	int ob_dis_prepre_lchg;
	int ob_loss_count_lchg;

	int ob_dis_now_lchg_timer;
	int ob_dis_pre_lchg_timer;
	int ob_dis_prepre_lchg_timer;
	
	double fasterobspdini_path;
	bool bfasterobacv_path;
	bool bobfisrstapper_path;
	int ob_dis_now_path;
	int ob_dis_pre_path;
	int ob_dis_prepre_path;
	int ob_loss_count_path;

	int ob_dis_now_path_timer;
	int ob_dis_pre_path_timer;
	int ob_dis_prepre_path_timer;

	double  spdacctemp;

	bool brakeflagchange;

	double  spdobaim;

	int light_res_HIS;

	int count_obb1;
	int count_ob_flag;

	bool bholdbrake;

	CvPoint2D64f upoint[4];

	CvPoint2D64f upointyulukou[4];

	CvPoint2D64f globalstrt;

	CLaneDriver lanedriverstate;

private:
	CvPoint2D64f his_MidGpsPoint[200];
	HANDLE  m_hThreadProc; 
	HANDLE  m_hThreadCtrl;
	
	DWORD dwCtrlThreadId;
	HANDLE  m_hThreadPath;
	DWORD dwPathThreadId;
	HANDLE  m_hThreadLight;
	DWORD dwLightThreadId;
	HANDLE m_pathevent;
	//*******添加ADASIS数据处理线程*******//
	HANDLE m_hThreadADASIS;
	DWORD dwADASISThreadId;
	
	//int mission_num;
	//LEAD *LeadPoint;
	LEAD StartPoint;
	CvPoint2D64f exe_gps;
	CvPoint2D64f exe_orientation;
	int pass_count;
	int m_task;
	int m_task_history;
	bool approachgoal_flag;
	CvPoint2D64f *gps_road;
	//CUrbanRoadContext urbanroadcontext;
	//CIntersectionContext intersectioncontext;
	CApproachExit approachexitstate;
	CChangeLane changelanestate;
	//CLaneDriver lanedriverstate;
	CDistKeeper distkeeperstate;
	CPassIntersection passintersectionstate;

	int	ob;
	int ob_x;
	int ob_y;
	int ob_speed;
	bool sendflag;
	
	HANDLE m_hEvent;
//	CIVExeNew exe_old;
	CvMemStorage* storage_road1;
	CvPoint2D64f exe_leftpoint[200];
	CvPoint2D64f exe_rightpoint[200];
	bool ob_left;
	bool ob_right;
	bool init_res;
	MSG   msgThreadData;
	//bool way_out;
	double approach_dis;
	ComuLcm road_state;
	CUTurn uTurn;
	int tongji;
	bool interob;
	//*********//
	int count_tongji;
	

	CvPoint2D64f his_Gps[10];
	bool first_Road;
	bool first_ApproIntersection;
	bool first_Intersection;
public:
	CControl control;
	bool startredgreen;
	~CVehicleExe(void);

public:
	CAstar astar;
	void solvewaypointnum();
	MAPPOINT* solvepoint(double x,double y);
	MAPPOINT* solvepoint2(double x,double y,double dir);
	MAPPOINTexe* solvepoint3(double x,double y,double dir,MAPPOINTexe* path11);
	void point2point();
	void planning(int m1,int n1,int p1,int m2,int n2,int p2);
	struct MAPPOINT *start_point,*end_point;
	struct MAPPOINT *path;
	struct MAPPOINTexe *pathexe;
	int pathpointnum,pathpointnumexe;
	void solveID(MAPPOINT* thepoint);
	void solveID(MAPPOINTexe* thepoint);
	void solveID(CvPoint2D64f thepoint);
	int ID1,ID2,ID3;

	int waypointnum;
	int PassMissionPointbyRNDF(CvPoint2D64f m_gps,double dir);
	int GetSeqNum(CvPoint2D64f gps,double dir);
	void GetGlobalPath();
	bool SearchObstacleMap(CvPoint2D64f *maplane,int num,I_Map *map,int up,int down);
	bool SearchObstacleMaptmp(CvPoint2D64f *maplane,int num,I_Map *map,int up,int down);
	int Calculate2cicurve(int id1,int id2,int id3,CvPoint2D64f (&curve)[200],CvPoint2D64f err);
	int AppLaneChange(int fx,double dn,CvPoint2D64f *point,CvPoint2D64f aimpoint,double dir,int &change,CvPoint2D64f &pianyi);
	int Setspeed(int drive_state,int drive_obstacle);
	int Getrndfbezier(CvPoint2D64f &pianyi);
	int GetLanebezier(CvPoint2D64f realtime_Gps,double realtime_Dir,CvPoint2D64f &pianyi,double yumiao);
	int ZhiDaoLane(int &ob_num,int &count_move,CvPoint2D64f &obvegps);
	int WanDaoLane(int &ob_num,int &count_move);
	int SearchObatAppro(int up,int down,CvPoint2D64f *temp,CvPoint2D64f m_gps,double dir,int left_right);
	int LaneInit();
	CvPoint2D64f pianyi;
	double hxdis;
	double hxdishis;
	LEAD InterStartPoint;

	double max_speed;
	double road_speed;
	double cross_speed1,cross_speed2;
	double obstacle_speed;

	//*******813*********//
	double temp_max;
	double temp_road;
	double temp_cross1,temp_cross2;
	double temp_obstacle;
	//*******813*******//

	public:
		void StartUdpRcv();
		DWORD UdpRcvThread();
		static DWORD theUdpRcvThread(LPVOID lpParam);
		HANDLE m_hUdpRcvThread;
		DWORD dwUdpRcvThreadId;

		void StartIbeo();
		DWORD IbeoThread();
		static DWORD theIbeoThread(LPVOID lpParam);
		HANDLE m_hIbeoThread;
		DWORD dwIbeoThreadId;

		void StartIbeo2();
		DWORD IbeoThread2();
		static DWORD theIbeoThread2(LPVOID lpParam);
		HANDLE m_hIbeoThread2;
		DWORD dwIbeoThreadId2;
		
		void StartIbeoMap();
		DWORD IbeoMapThread();
		static DWORD theIbeoMapThread(LPVOID lpParam);
		HANDLE m_hIbeoMapThread;
		DWORD dwIbeoMapThreadId;
		HANDLE m_IbeoMapEvent;

		//SOCKET server;//一个结构体，用于调用函数WSAStartup时作为参数
		//WSADATA wsaData;
		//sockaddr_in local;//存放ip地址的结构体
		
		bool SuccSocket;

		//SOCKET server2;//一个结构体，用于调用函数WSAStartup时作为参数
		//WSADATA wsaData2;
		//sockaddr_in local2;//存放ip地址的结构体
	
		bool SuccSocket2;



		int iObject1;//物体数量
		ObjectWT ObjectWT1[200];//预先定义200个物体

		int iObject1test1;
		ObjectWT ObjectWT1test1[200];

		int iObject1test2;
		ObjectWT ObjectWT1test2[200];

		int iObject1test3;
		ObjectWT ObjectWT1test3[200];

		int iObject1test4;
		ObjectWT ObjectWT1test4[200];

		int iObject1test5;
		ObjectWT ObjectWT1test5[200];

		int iObject1test6;
		ObjectWT ObjectWT1test6[200];

		int iObject1test7;
		ObjectWT ObjectWT1test7[200];




		int iObject2;//物体数量
		ObjectWT ObjectWT2[200];//预先定义200个物体

		ObjectWT ObjectWTfront[200];
		int frontnum;
		ObjectWT ObjectWTtrail[200];
		int trailnum;

		int iObject2test1;
		ObjectWT ObjectWT2test1[200];

		int iObject2test2;
		ObjectWT ObjectWT2test2[200];

		int iObject2test3;
		ObjectWT ObjectWT2test3[200];

		int iObject2test4;
		ObjectWT ObjectWT2test4[200];

		int iObject2test5;
		ObjectWT ObjectWT2test5[200];

		int iObject2test6;
		ObjectWT ObjectWT2test6[200];

		int iObject2test7;
		ObjectWT ObjectWT2test7[200];


		void ClearObject();//清除前一真数据
		void ClearObject2();//清除前一真数据
		void ConnectInit(SOCKET &server,CString strIP);
		bool SearchObstacleLukou(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
		int GetNearWPoint(int id1,int id2,int id3,CvPoint2D64f m_gps,double m_dir);
		int GetCross_newexe2(CvPoint2D64f (&IntersectionPath)[200]);

		DynamicObFuse fuser;

		//I_Map* dy_map;//动态图 


		TSmap tsmap;
		NewMap* new_map;
		I_Map* map_id;//障碍物列表编号图
		I_Map* map_co;//碰撞分析图 
		I_Map *map3;


		bool interobflag;
		CvPoint2D64f interp;
		void WaitForSignal2();
		int GetMissionPoint2(LEAD lead[],int num);
		int GetMissionNum2( );
		bool JudgeBetweenTwoDis(CvPoint2D64f goal,double up, double down,CvPoint2D64f m_gps,double dir);
		double SearchFrontPeople(I_Map *map,double up,double width)/*正前 絩ange米内有无障碍 */;
		double SearchYulukouOb(I_Map *map,double range);
		double SearchFrontVehicle(I_Map *map,double up,double width)/*正前 絩ange米内有无障碍 */;
		int GetPartGlobalPath(CvPoint2D64f *curve);
		int GetGlobalPathSeq(CvPoint2D64f *path,int num);
		int ReturnPartPoints(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err);
		int ReturnPartPoints_lanechg(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err);
		int ReturnPartPoints_lukou(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err);
		int ReturnPartPoints_Ini(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err);
		int his_state;
        int Yulukou_yanchang(CvPoint2D64f (&rndf)[200],double length);
		void UpDatePath();
		int PickUpIntersectionPoints(int *c, int &b);
		CvPoint2D64f ori_rndf[200];
		int OnRoadDrive(int &ob_num,int &count_move,bool is_avoidobstacle);
		void yulukoujiansu();
		void lukoujiashi();
		void zhixinglukoujiashi();
		int ReturnPartPointsSeqNum(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err);
		void ReturnNeedPoints(CvSeq *adaspoint,CvPoint2D64f m_gps,double dir,double s);
		int ReturnPartPointsSeqNumOnTimer(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200]);
		int CVehicleExe::ApproIntersectionDrive(int &ob_num,int &count_move,bool is_avoidobstacle);
		int JudgeIntersectionFx(CvPoint2D64f &stop_point);
		int RangeLUkouFx();
		CCANBus can_bus;
		int InterSecNum;
		int InterSecPoints[500];
		int *InterSecPointOrder;
		bool bcurveactive;
		int lanelostcnt;
		int obleftlosscnt;
		int obrightlosscnt;
		int obpianyidir;
		int width_L;
		int width_R;
		int lukou_fx;
		int laneturndircnt;
		CvPoint2D64f yulukoupoint;
		int countlast;
		double latpianyi_lanechgstrt;
		double lonpianyi_lanechgstrt;

		int approachyulukou;//1=left,2=right;

		bool yulukounolanechange;
		bool zhongdiannolanechange;

		bool yulukounolaneassist;

		int m_taskpublic;

		bool bforcedirectrun;

		CvPoint2D64f raohuistart;

		bool blahuirndf;
		bool blanechangeend;
		bool blanechangeendyulukou;

		double stopmiddledis;

		bool bspecialuturnyulukou;

		int bspecialbiandaoshu;

		bool enterlukou;

		int specialbiandaoendcnt;

		CvPoint2D64f gps_aimpointhis;

		bool bfeijidongche;

		bool blabianqiansureq;

		bool SearchUTurnObstacleMoveWay1_rightsidespecial(CvPoint2D64f Rndf_MapPoint[],I_Map *map);

		bool blanesync;

		bool bleftchgallowed;
		bool brightchgallowed;

		bool bxingrenzixingche;
		double xingrendist;
		int x_right_xingren[15];//10
		double y_right_xingren[15];//10
		int x_left_xingren[15];//10
		double y_left_xingren[15];//10
		int x_right_xingrennum;
		int x_left_xingrennum;

		bool bhardraozhang;

		CvPoint2D64f rndfbezierlanechangeinitialdisp[200];

		SOCKET RcvFromContrlSocket;
		sockaddr_in RcvFromControlAddrSrv;

		SOCKET SendToContrlSocket;
		sockaddr_in ControlAddrSrv;
		void StartUdpSent();
		int UdpCnt;
		double lastacc;

		bool bspecialzhilukoutingche;

		int bchelianggenchi;

		CvPoint2D64f redlampchk;
		CvPoint2D64f redlampstrtchk;

		bool bspecialraozhangyulukou;

		int bguzhangcheliang;

		int uturndaochecnt1;
		int uturndaochecnt2;
		int uturndaochecnt3;
		int uturndaochecnt4;
		int uturndaochecnt5;
		int uturndaochecnt6;

		CvPoint2D64f pianyifulu;

		bool bfulujiansu;

		int qidongcnt1;
		int qidongcnt2;
		int qidongcnt3;
		int qidongcnt4;
		int qidongcnt5;
		int qidongcnt6;

		int yinhangdianstatuscnt1;
		int yinhangdianstatuscnt2;
		int yinhangdianstatuscnt3;
		int yinhangdianstatuscnt4;
		int yinhangdianstatuscnt5;
		int yinhangdianstatuscnt6;

		int rouchestatustimerstrt1;
		int rouchestatustimerstrt2;
		int rouchestatustimerstrt3;
		int rouchestatustimerstrt4;
		int rouchestatustimerstrt5;
		int rouchestatustimerstrt6;
		int rouchestatustimerstrt7;
		int rouchestatustimerstrt8;
		int rouchestatustimerstrt9;
		int rouchestatustimerstrt10;
		double rouchestatusdaochedist;
		double rouchestatusyoudamanzoudist;
		double rouchestatuszuodamanzoudist;
		double rouchestatuszhizoudist;

		CvPoint2D64f ApaStrt;
		CvPoint2D64f ApaEnd;

		int ApaStatustimerstrt2;
		int ApaStatustimerstrt3;
		int ApaStatustimerstrt4;
		int ApaStatustimerstrt5;
		int ApaStatustimerstrt6;
		int ApaStatustimerstrt7;
		int ApaStatustimerstrt8;
		int ApaStatustimerstrt9;
		int ApaStatustimerstrt10;
		int ApaStatustimerstrt11;
		double youdamandaochedist;
		double zhidaochedist;
		double zhiqianjinApadist;

		int PXApaStatustimerstrt2;
		int PXApaStatustimerstrt3;
		int PXApaStatustimerstrt4;
		int PXApaStatustimerstrt5;
		int PXApaStatustimerstrt6;
		int PXApaStatustimerstrt7;
		int PXApaStatustimerstrt8;
		int PXApaStatustimerstrt9;
		int PXApaStatustimerstrt10;
		int PXApaStatustimerstrt11;
		int PXApaStatustimerstrt12;
		int PXApaStatustimerstrt13;
		int PXApaStatustimerstrt14;
		int PXApaStatustimerstrt15;
		int PXApaStatustimerstrt16;
		int PXApaStatustimerstrt17;
		int PXApaStatustimerstrt18;
		int PXApaStatustimerstrt19;
		double PXyoudamandaochedist;
		double PXzhidaochedist;
		double PXzuodamandaochedist;
		double PXzuodamanqianjindist;
		double PXzhiqianjindist;
		double PXyoudamanqianjindist;

		bool blanechgendly;
		int blanechgendlycnt;

		bool blaneusedisable;
		
		bool blanechginiobbrake;
		CvPoint2D64f lanechginiendpnt;

		CvPoint2D64f MidGpsPointinidata[200];
		bool bMidGpsPointinidataVd;

		double lanemovedist;
		int lanechangenumtmp;

		double lchghxdist;

		bool bhaikanglaneused;

		int haikanglanecnt;

		int spdcurrentpnt;
		int spdintersection;
		bool bspdlimitused;

		double halfwidth;

		double spdsentlast;

		bool bRturnLightChk;
		bool bLturnLightChk;
		bool bZturnLightChk;

		bool bobwaitlanechg;

		bool bspeciallukoustop;

	private:
		bool VelListenned;
		bool VelListenned2;
};
