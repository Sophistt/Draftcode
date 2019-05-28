#pragma once

#include "GetGPSData.h"
#include "SocketTransfer.h"
#include "LoadRNDF.h"
#include "LoadMDF.h"
//#include "IVLocalPath.h"
#include "pid_reg3.h"
#include "ComuLcm.h"
//#include "IVKeepLane.h"
#include "Vehicle.h"
#define WM_INTERVAL_EXE WM_USER+9998

#define IVTASK_LANE  5001
#define IVTASK_CROSS  5002
#define IVTASK_STOPLINE  5003
#define IVTASK_PARKING  5004
//触须
const double vehicle_r[81] = { -1377.5,-459.167,-196.786,-137.75,-105.962,-76.5278,-62.6136,-55.1,-51.0185,-45.9167,-43.0469,-41.7424,-38.2639,-35.3205,-31.3068,-27.55,-25.5093,-23.75,-22.2177,-20.5597,-18.3667,-17.6603,-16.7988,-15.6534,-14.6543,-13.775,-12.9953,-11.9783,-11.3843,-11.02,-10.2037,-9.18333,-8.60938,-7.87143,-7.44595,-6.8875,-6.55952,-6.12222,-5.8617,-5.6,5.51,5.8617,6.12222,6.55952,6.8875,7.44595,7.87143,8.60938,9.18333,10.2037,11.02,11.3843,11.9783,12.9953,13.775,14.6543,15.6534,16.7988,17.6603,18.3667,20.5597,22.2177,23.75,25.5093,27.55,31.3068,35.3205,38.2639,41.7424,43.0469,45.9167,51.0185,55.1,62.6136,76.5278,105.962,137.75,196.786,459.167,1377.5,100000 };
const int tentacle_code[81] = { -4000,-12000,-28000,-40000,-52000,-72000,-88000,-100000,-108000,-120000,-128000,-132000,-144000,-156000,-176000,-200000,-216000,-232000,-248000,-268000,-300000,-312000,-328000,-352000,-376000,-400000,-424000,-460000,-484000,-500000,-540000,-600000,-640000,-700000,-740000,-800000,-840000,-900000,-940000,-1.00E+06,1.00E+06,940000,900000,840000,800000,740000,700000,640000,600000,540000,500000,484000,460000,424000,400000,376000,352000,328000,312000,300000,268000,248000,232000,216000,200000,176000,156000,144000,132000,128000,120000,108000,100000,88000,72000,52000,40000,28000,12000,4000,0 };
//typedef struct //控制结构
//{
//	int turn;
//	int velocity;
//	int brake;
//	int mswitch;
//	int key;
//	int accelerator;
//	int horn;
//	int light;
//}IVMCONTROL;


class CIVExeNew:
	public CVehicle
{
public:
	CIVExeNew(void);
public:
	~CIVExeNew(void);

public:
	//触须
	//double vehicle_r[81] ;//= { -1377.5,-459.167,-196.786,-137.75,-105.962,-76.5278,-62.6136,-55.1,-51.0185,-45.9167,-43.0469,-41.7424,-38.2639,-35.3205,-31.3068,-27.55,-25.5093,-23.75,-22.2177,-20.5597,-18.3667,-17.6603,-16.7988,-15.6534,-14.6543,-13.775,-12.9953,-11.9783,-11.3843,-11.02,-10.2037,-9.18333,-8.60938,-7.87143,-7.44595,-6.8875,-6.55952,-6.12222,-5.8617,-5.6,5.51,5.8617,6.12222,6.55952,6.8875,7.44595,7.87143,8.60938,9.18333,10.2037,11.02,11.3843,11.9783,12.9953,13.775,14.6543,15.6534,16.7988,17.6603,18.3667,20.5597,22.2177,23.75,25.5093,27.55,31.3068,35.3205,38.2639,41.7424,43.0469,45.9167,51.0185,55.1,62.6136,76.5278,105.962,137.75,196.786,459.167,1377.5,100000 };
	//int tentacle_code[81] ;//= { -4000,-12000,-28000,-40000,-52000,-72000,-88000,-100000,-108000,-120000,-128000,-132000,-144000,-156000,-176000,-200000,-216000,-232000,-248000,-268000,-300000,-312000,-328000,-352000,-376000,-400000,-424000,-460000,-484000,-500000,-540000,-600000,-640000,-700000,-740000,-800000,-840000,-900000,-940000,-1.00E+06,1.00E+06,940000,900000,840000,800000,740000,700000,640000,600000,540000,500000,484000,460000,424000,400000,376000,352000,328000,312000,300000,268000,248000,232000,216000,200000,176000,156000,144000,132000,128000,120000,108000,100000,88000,72000,52000,40000,28000,12000,4000,0 };

//GPS信息
public:
	CGetGPSData m_GpsData;//GPS类;
	ComuLcm m_GpsDecision;
//	CIVLocalPath LocalPath;
	//CLoadRNDF RNDF;
//	IVMCONTROL m_ctrl;
	ComuLcm gps;
//控制转向、制动、速度
public:

	// T 方向盘转向值（-450000 ~ 450000）qc
	// V 控制速度值 （0 ~ 40）qc
	// B 刹车位置值（-25500 ~ -42500）qc (-25500为等待位，-42500为刹车到底位)
	// S 换挡（0-P停车 1-R倒档  2-N空挡  3-D行车）
	// K 钥匙（暂时不需要）
	// A 油门（暂时不需要）
	// H 喇叭（暂时不需要）
	// L 灯光（4位二进制 0表示左转 1表示右转 2表示远光 3表示喇叭）0000

	int lastturn;
	CvPoint2D64f last_gps;
	double last_dir;
	DWORD last_t;
	//IVMCONTROL m_control;

	
	//线程
	HANDLE  m_hThreadSend; 
	HANDLE  m_hEvent;
	HANDLE m_MissionEvent;
	DWORD dwSendThreadId;
	void StartSend();
	void EndSend();
	DWORD SendThread();
	static DWORD theSendThread(LPVOID lpParam);

	HANDLE  m_hThreadSendGPS; 
	DWORD dwSendGPSThreadId;
	
	void StartSendGPS();
	void EndSendGPS();
	DWORD SendGPSThread();
	static DWORD theSendGPSThread(LPVOID lpParam);
	
	////////
	lcm_t * lcm_send;
	void send_message(CString buf);
	////////
	/***********************************************************************
	车辆控制状态计算
	************************************************************************/
//	void zGetCtrlParameters( CvPoint2D64f m_gps, double m_direction, CvPoint2D64f m_dstgps, /*double m_dstdir,*/ IVMCONTROL &m_ctrl );

	/*
	已知行车轨迹半径的情况下，搜索触须ID
	输入：r - 半径(m)
	输出：t - 触须ID 
	*/
	int GetTencaleByVr(double r, int &t);

	int seq_num;//起始序号点

////路径规划
public:
	CSocketTransfer send;

	CvMemStorage* storage_road;
	CvSeq* seq_road;
	CvSeq* plan_road;
	CvSeq* mid_road;
	bool sendflag;
	CLoadRNDF RNDFGPS;//;//GPS坐标
	CLoadMDF MDF;
	/*******************************************************
	通过路径轨迹，确定目标点
	********************************************************/
	int zGetAimPoint( CvPoint2D64f m_curgps, double m_direction, CvSeq * m_roadseq, CvPoint2D64f &m_dstgps );
	int zGetArrTime( CvPoint2D64f m_curgps, double m_direction, CvPoint2D64f m_dstgps, double &m_arrtime);
	int zGetLatError( CvPoint2D64f m_curgps, double m_direction, CvPoint2D64f m_dstgps, double &m_laterr);
	int zGetLatSpeed( double m_direction, double &m_latspeed );
	int zGetLatSpeed( CvPoint2D64f m_gps, double m_direction,double &m_latspeed );
	int zGetTurn(CvPoint2D64f m_gps, double m_direction, CvPoint2D64f m_dstgps,int &turn);
	int zGetThrottle(double speed,int &throttle,int &brake);
	int zStop( );
	int zGetAimLen(double speed);
	/******************************************************************************************
	综合控制流程、路径规划及系列目标点生成
	******************************************************************************************/
	//线程
	HANDLE  m_hThreadProc; 
	DWORD dwProcThreadId;
	void StartProc();
	DWORD ProcThread();
	static DWORD theProcThread(LPVOID lpParam);
	bool Path_Error(CvPoint2D64f GPSpoint[],CvSeq* plan_road);

	int state; //直线为1，曲线为0。
	double u_desire;//期望速度
	double aim_length,lastaim_length,lastu;//预瞄距离
	double lastaim_s,lasts;//预瞄距离
	CvPoint2D64f m_end;
	int GetMiddleString(CString &string,const char * firststr, const char * endstr,CString &conResult );

	double laterr_y[3];
	double laterr_u[3];
	int zFilterLatError( double &m_laterr );

	bool way_out;
	//////////////
private:
	CvPoint2D64f rPosition;
	double rDirection;
	
//	CIVKeepLane keeplane;
	CvPoint2D64f MidPoint[200];
	CvPoint2D64f MidGpsPoint[200];
	CvPoint2D64f his_MidPoint[200];
	CvPoint2D64f shift_MidPoint[200];
	//CvPoint2D64f Rndf_MapPoint[200];
	double s_new;
	double s_old;
	ComuLcm rec_map;
public:
	I_Map* vel_Map;
	I_Map* lane_Map;
	I_Map* mid_Map;
	int GetSendGps(CvPoint2D64f m_gps,double m_gpsdir,CvPoint2D64f *MidPoint,CvPoint2D64f (&MidGpsPoint)[200]);
	void ArrayFuZhi(CvPoint2D64f *a,CvPoint2D64f *b);
	void ArrayFuZhi(CvPoint2D64f a,CvPoint2D64f *b);
	void zFitLanesCenter(IplImage *src, IplImage *dst);
	void ProcCenterLane(I_Map *map_src,I_Map *map_dst);
	int GetLaneMap(I_Map *vel_Map,I_Map *lane_Map);
	int KeepLane(CvPoint2D64f *Rndf_MidPoint,CvPoint2D64f (&MidGpsPoint)[200]);
	int GetLanePoint(int n_seg, int n_poi,int &exit_id,CvPoint2D64f WayPoint[],int &n);
	int GetLane(int &n_seg, int &n_poi,int char_id, CvPoint2D64f WayPoint[],int n,CvPoint2D64f (&Point)[200]);
	int GetCrossPoint(CvPoint2D64f cur_gps,double cur_direction,int &next_seg, int &n_poi,CvPoint2D64f WayPoint[]);
	int GetCross(CvPoint2D64f WPoint[], CvPoint2D64f (&Point)[200]);

	int  m_task;
	int n_seg;
	int n_poi;
	int n_exit;

	int PassCheckPoint(CvPoint2D64f m_gps);
	int PassMissionPoint(CvPoint2D64f m_gps,double dir);
	int GetSegmentNumberByGPS( CLoadRNDF RNDF, double wd, double jd, int &n_seg );
	double pointToLine(double x1, double y1, double x2, double y2, double x0, double y0);
	int CountUTurn(I_Map *map,double k,Map_Point &a);
	int GetUTurn(CvPoint2D64f GpsPoint[],CvPoint2D64f WPoint[]);
	int isUTurn(CvPoint2D64f WPoint[]);
	int MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f APoint[],int num,CvPoint2D64f WayPoint[]);
	bool RndfToMap(CvPoint2D64f *Rndf_Point,CvPoint2D64f m_gps,double dir,CvPoint2D64f *Rndf_MapPoint);
	bool SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);//0代表左边换道，1代表右边换道
	bool SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int &ob_y);

	int MoveRight(CvPoint2D64f WayPoint[],CvPoint2D64f RightWayPoint[],double length);
	int MoveLeft(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length);
	int MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f WayPoint[]);

	int on_obs;
	int Stop(CvPoint2D64f m_gps);
	bool AvoidObstacle(CvPoint2D64f cur_point[],CvPoint2D64f left_point[]);
	bool ChangeLeftLane(CvPoint2D64f cur_point[]);
	bool ChangeRightLane(CvPoint2D64f cur_point[]);
	bool GetMidLFroMap(I_Map *OriMap,I_Map *mid_Map);
	int StopLine(double s1);
	int zStop_Odo(double Stop_Lenth);
	int ob_count;
	int isStopLine();
	bool stop_flag;
	bool ChangeLeft_RightLane(CvPoint2D64f cur_point[],int left_right);//1表示左边换道，-1表示右边换道;
	int YanChang(CvPoint2D64f midpoint[],int length,CvPoint2D64f (&shift_point)[200]);
	double VertDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps);
	double ParaDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps);
	int GetMissionNum( );
	int GetMissionPoint(LEAD lead[],int num);
	int GetCross_new(int seq,CvPoint2D64f WayPoint[],CvPoint2D64f (&GpsPoint)[200],int &fx);

	LEAD *LeadPoint;

	bool GetSignal(int (&sign)[4]);

	int pass_count;
	int StopBreak(CvPoint2D64f gps[]);
	int GetUTurn2(CvPoint2D64f GpsPoint[],CvPoint2D64f WPoint[]);
};

