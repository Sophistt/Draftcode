#include "StdAfx.h"
#include "VehicleExe.h"
//#include "ADASISDataReceiver.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
# define I 999999
#define  T 30000
#include<fstream>
using namespace std;
#include <iomanip>

ofstream outproctime("outproctime.txt");
ofstream outpathtime("outpathtime.txt");
ofstream outibeomaptime("outibeomaptime.txt");



ofstream outintersecstate("outintersecstate.txt");

ofstream lightrecord("lightrecord11.txt");

ofstream Ibeomaptime("Ibeomaptime.txt");

ofstream out124("exe_velmap.txt");
ofstream out128("changelae exe.txt");
ofstream outn1("signal.txt");
ofstream outn77("cuowu.txt");
ofstream outn79("acc.txt");
ofstream outn779("nullpoint.txt");
ofstream outn7777("path.txt");
ofstream outn8888("segmentid.txt");

ofstream out_tsmap("ts_map.txt");
ofstream out_vmap("v_map.txt");
ofstream out_dirmap("dir_map.txt");

ofstream controlStates("controlstates.txt");
ofstream outADASISPoint("ADASISPoints.txt");
ofstream outUseAdasPoint("UseAdaspoint.txt");
ofstream outupdatenum("updatenum.txt");
ofstream outcaanshu("number.txt");
ofstream outtimecost("timecost.txt");
My_Timer mytime;
ofstream outtimecostpro("timecostpro.txt");
ofstream outleftchange("leftchangedata.txt");
ofstream outrightchange("rightchangedata.txt");
ofstream outleadpoint("outleadpoint.txt");
ofstream outcurrentpoint("outcurrent.txt");
ofstream outUturnsign("oututurn.txt");
ofstream outlaneline("outlanepts.txt");
ofstream outseqnum("upseqnum.txt");
ofstream outceshi1("kaobian1.txt");
ofstream outceshi2("kaobian2.txt");
ofstream outyuce("yupts.txt");
ofstream outobres("outobres.txt");

ofstream outgps_aimpoint("biandaogps_aimpoint.txt");
ofstream outGPS_Point("biandaoGPS_Point.txt");
ofstream outaimPointBeforFilter("biandaoaimPointBeforFilter.txt");
ofstream outproctimediffer("timeprocdiffer.txt");

ofstream outshigongluduanpath("outshigongluduanpath.txt");

ofstream outshigongluduanpath2("outshigongluduanpath2.txt");

ofstream outshigongluduanpath3("outshigongluduanpath3.txt");

ofstream outsuidaostate("outsuidaostate.txt");

ofstream outqidongstate("outqidongstate.txt");

ofstream outlanechangegpsdata("lanechangegpsdata.txt");

ofstream outApastate("outApastate.txt");

ofstream outRouchestate("outRouchestate.txt");

ofstream outUdpRoadPntstate("outUdpRoadPntstate.txt");
ofstream outdebugstate("outdebugstate.txt");

ofstream outudprcvdata("outudprcvdata.txt");

ofstream outhikangdata("outhikangdata.txt");

/////读取地图时加临界区 切记！！！
//////路口 设置等待障碍物时间
CVehicleExe::CVehicleExe(void)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->m_hEvent=CreateEvent(NULL,FALSE,FALSE,NULL);
	app->m_Ibeo1Event=CreateEvent(NULL,FALSE,FALSE,NULL);
	app->m_Ibeo2Event=CreateEvent(NULL,FALSE,FALSE,NULL);
	seq_num = -1;
	pass_count = 0;
	m_task = IVTASK_LANE;
	approachgoal_flag = false;
	ob_x = 0;
	ob_y = 0;
	ob = 0;
	sendflag = false;
	ob_left = false;
	ob_right = false;
	init_res = false;
	mission_num = 0;
	mission_num2 = 0;
	way_out = true;
	approach_dis = 0;
	ob_speed = 0;
	yulukoucount = 0;

	aim_point = cvPoint2D64f(256,300);
	hisgpsaimpoint = cvPoint2D64f(0,0);
	/*storage_road1 = cvCreateMemStorage(0);
	plann_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road1 );*/
	aim_dis = 0;
	m_pathevent=CreateEvent(NULL,FALSE,FALSE,NULL);

	storage_path = cvCreateMemStorage(0);
	lanechang_flag = true;
	
	global_path = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_path );
	hxdis = 0;
	hxdishis = 0;
	pianyi = cvPoint2D64f(0,0);

	globalstrt = cvPoint2D64f(0,0);

	new_map = new NewMap;
	map_ts = new I_Map;
	map_dan = new I_Map;
	map_v = new I_Map;
	map_dir = new I_Map;
	map_ts1 = new I_Map;
	map_id = new I_Map;
	map_co = new I_Map;
	dy_map=new I_Map;

	map_v_x=new v_Map;
	map_v_y=new v_Map;
	static_Map=new I_Map;
	dynamic_Map=new I_Map;
	dynamicmap_v_x=new v_Map;
	dynamicmap_v_y=new v_Map;
	lukou_Map=new I_Map;
	lukou_v_x=new v_Map;
	lukou_v_y=new v_Map;

	tongji = 0;
	count_tongji=0;
	interob = false;
	interobflag = true;
	m_IbeoMapEvent = CreateEvent(NULL,FALSE,FALSE,NULL);
	xz_flag = false;
	 first_Road = true;
	 first_ApproIntersection = true;
	 first_Intersection = true;
	 able_ChangeLane = true;
	 app->left_right = 0;
	 spdacctemp = 0;
	 spdobaim=0;
	 ////////////////
	 first_state = false;
	 his_state = IVTASK_LANE;
	 /////////////
	max_speed = 75/3.6;
	road_speed = 40/3.6;
	cross_speed1 = 30/3.6;//25/3.6
	cross_speed2 = 10/3.6;
	ob_speed = 15/3.6;
	map3=new I_Map;
	laneturndircnt=0;
	light_res_HIS=-1;
	count_obb1=0;
	count_ob_flag=0;
	lanechangespd=road_speed;
	lukoulockspd=road_speed;
	
	fasterobspdini=road_speed;
	bfasterobacv=0;
	bobfisrstapper=0;
	ob_dis_now=1000;
	ob_dis_pre=1000;
	ob_dis_prepre=1000;
	ob_dis_now_timer=0;
	ob_dis_pre_timer=0;
	ob_dis_prepre_timer=0;
	ob_loss_count=0;
	
	fasterobspdini_lchg=road_speed;
	bfasterobacv_lchg=0;
	bobfisrstapper_lchg=0;
	ob_dis_now_lchg=1000;
	ob_dis_pre_lchg=1000;
	ob_dis_prepre_lchg=1000;
	ob_dis_now_lchg_timer=0;
	ob_dis_pre_lchg_timer=0;
	ob_dis_prepre_lchg_timer=0;
	ob_loss_count_lchg=0;
	
	fasterobspdini_path=road_speed;
	bfasterobacv_path=0;
	bobfisrstapper_path=0;
	ob_dis_now_path=1000;
	ob_dis_pre_path=1000;
	ob_dis_prepre_path=1000;
	ob_dis_now_path_timer=0;
	ob_dis_pre_path_timer=0;
	ob_dis_prepre_path_timer=0;
	ob_loss_count_path=0;

	bholdbrake=0;

	yulukounolanechange=0;
	zhongdiannolanechange=0;
	yulukounolaneassist=0;

	bspecialuturnyulukou=0;

	bspecialbiandaoshu=0;
	specialbiandaoendcnt=1000;

	app->bjianruiuturn=0;
	enterlukou=0;

	app->bqishiluduan=1;

	app->bspecialuturnluduan=0;

	app->bspecialnoleftturnluduan=0;

	app->bleftturnban=0;

	bfeijidongche=0;

	app->bnobiandaoraozhang=0;

	blanesync=true;

	bleftchgallowed=true;
	brightchgallowed=true;

	bxingrenzixingche=0;
	x_right_xingrennum=0;
	x_left_xingrennum=0;

	bspecialzhilukoutingche=false;

	bchelianggenchi=false;
	bspecialraozhangyulukou=false;

	app->kuaisulufulu=0;

	blanechgendly=false;
	blanechgendlycnt=0;

	blaneusedisable=false;

	bMidGpsPointinidataVd=false;

	bhaikanglaneused=false;

	haikanglanecnt=0;

	spdcurrentpnt=-1000;
	spdintersection=-1000;
	bspdlimitused=false;

	halfwidth=1.725;//1.85

	bRturnLightChk=false;
	bLturnLightChk=true;
	bZturnLightChk=true;
}

CVehicleExe::~CVehicleExe(void)
{
	//cvReleaseMemStorage(&storage_road1);
	cvReleaseMemStorage(&storage_path);
	closesocket(SendToContrlSocket);
	WSACleanup();
}
void CVehicleExe::StartProc()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	m_hThreadProc = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theProcThread,
		this, 0, &app->dwProcThreadId);
}

DWORD CVehicleExe::theProcThread(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->ProcThread());
}

void UpDatePath() 
{
	throw std::exception("The method or operation is not implemented.");
}

//综合运行流程
DWORD CVehicleExe::ProcThread()
{

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	init_res = InitExe();
	if(!init_res)
		return 0;
	int gpsmap_road_num = app->gpsmap_road->total;
	if((!app->bGpsMapLoad)||(app->gpsmap_road->total<2))
		return 0;

	/*
	if(!app->AimFlag_ApaPath1)
		AfxMessageBox("泊车路径1未载入!!!");
	if(!app->AimFlag_ApaPath2)
		AfxMessageBox("泊车路径2未载入!!!");
	if(!app->AimFlag_ApaPath3)
		AfxMessageBox("泊车路径3未载入!!!");
		*/

	bool bchusai=0;

	// 设置标志位，具体内容？
	app->drive_curvature_update=max_speed;
	app->limit_speed_update=max_speed;
	app->stop=false;
	app->bnearstop=false;
	app->bspecialraozhangcanshu=false;
	app->bisaiwaiburoad=false;
	bspecialraozhangyulukou=false;


	CvPoint2D64f start_yan[100];
	CvPoint2D64f start_yan_left[100];
	CvPoint2D64f start_yan_right[100];

	// gpsmap_road 是什么？
	CvPoint5D64f_type maproad0ini = *(CvPoint5D64f_type*)cvGetSeqElem(app->gpsmap_road, 0);
	CvPoint5D64f_type maproad1ini = *(CvPoint5D64f_type*)cvGetSeqElem(app->gpsmap_road, 1);

	CvPoint2D64f maproad0;
	CvPoint2D64f maproad1;
	maproad0.x=maproad0ini.x;
	maproad0.y=maproad0ini.y;
	maproad1.x=maproad1ini.x;
	maproad1.y=maproad1ini.y;
	
	CvPoint2D64f maproad0_left;
	CvPoint2D64f maproad1_left;
	CvPoint2D64f maproad0_right;
	CvPoint2D64f maproad1_right;
	maproad0_left.x=maproad0ini.x_left;
	maproad0_left.y=maproad0ini.y_left;
	maproad1_left.x=maproad1ini.x_left;
	maproad1_left.y=maproad1ini.y_left;
	maproad0_right.x=maproad0ini.x_right;
	maproad0_right.y=maproad0ini.y_right;
	maproad1_right.x=maproad1ini.x_right;
	maproad1_right.y=maproad1ini.y_right;

	CvPoint5D64f_type maproad5ini = *(CvPoint5D64f_type*)cvGetSeqElem(app->gpsmap_road, 5);
	CvPoint2D64f maproad5;
	maproad5.x=maproad5ini.x;
	maproad5.y=maproad5ini.y;

	double dirrenwustrt=m_GpsData.GetAngle(maproad5,maproad0);
	double dirstart = m_GpsData.GetAngle(maproad0,maproad1);
	
	// 干了什么？
	for(int i=1;i<100;i++)
	{
		start_yan[i] = m_GpsData.MaptoGPS(maproad0,dirstart,cvPoint2D64f(256,412-(i*6)));
		start_yan_left[i] = m_GpsData.MaptoGPS(maproad0_left,dirstart,cvPoint2D64f(256,412-(i*6)));
		start_yan_right[i] = m_GpsData.MaptoGPS(maproad0_right,dirstart,cvPoint2D64f(256,412-(i*6)));
		CvPoint5D64f_type start_yan_tmp;
		start_yan_tmp.x=start_yan[i].x;
		start_yan_tmp.y=start_yan[i].y;
		start_yan_tmp.z=0;
		start_yan_tmp.laneseq = maproad0ini.laneseq;
		start_yan_tmp.lanenum_ = maproad0ini.lanenum_;
		start_yan_tmp.x_left=start_yan_left[i].x;
		start_yan_tmp.y_left=start_yan_left[i].y;
		start_yan_tmp.x_right=start_yan_right[i].x;
		start_yan_tmp.y_right=start_yan_right[i].y;
		start_yan_tmp.b_left=maproad0ini.b_left;
		start_yan_tmp.b_right=maproad0ini.b_right;
		cvSeqPushFront(app->gpsmap_road, &start_yan_tmp);
	}

	gpsmap_road_num = app->gpsmap_road->total;

	int lastp2num=gpsmap_road_num-5;
	if(lastp2num<0)
		lastp2num=0;

	maproad0ini = *(CvPoint5D64f_type*)cvGetSeqElem(app->gpsmap_road, gpsmap_road_num-1);
	maproad1ini = *(CvPoint5D64f_type*)cvGetSeqElem(app->gpsmap_road, lastp2num);
	globalendpoint.x=maproad0ini.x;
	globalendpoint.y=maproad0ini.y;

	CvPoint5D64f_type maproad11ini = *(CvPoint5D64f_type*)cvGetSeqElem(app->gpsmap_road, lastp2num);
	CvPoint2D64f lastp1,lastp2;
	lastp1.x=maproad0ini.x;
	lastp1.y=maproad0ini.y;
	lastp2.x=maproad11ini.x;
	lastp2.y=maproad11ini.y;

	maproad0.x=maproad0ini.x;
	maproad0.y=maproad0ini.y;
	maproad1.x=maproad1ini.x;
	maproad1.y=maproad1ini.y;

	maproad0_left.x=maproad0ini.x_left;
	maproad0_left.y=maproad0ini.y_left;
	maproad1_left.x=maproad1ini.x_left;
	maproad1_left.y=maproad1ini.y_left;
	maproad0_right.x=maproad0ini.x_right;
	maproad0_right.y=maproad0ini.y_right;
	maproad1_right.x=maproad1ini.x_right;
	maproad1_right.y=maproad1ini.y_right;

	dirstart = m_GpsData.GetAngle(maproad0,maproad1);
	for(int i=1;i<100;i++)
	{
		start_yan[i] = m_GpsData.MaptoGPS(maproad0,dirstart,cvPoint2D64f(256,412-(i*6)));
		start_yan_left[i] = m_GpsData.MaptoGPS(maproad0_left,dirstart,cvPoint2D64f(256,412-(i*6)));
		start_yan_right[i] = m_GpsData.MaptoGPS(maproad0_right,dirstart,cvPoint2D64f(256,412-(i*6)));
		CvPoint5D64f_type start_yan_tmp;
		start_yan_tmp.x=start_yan[i].x;
		start_yan_tmp.y=start_yan[i].y;
		start_yan_tmp.z=0;
		start_yan_tmp.laneseq = maproad0ini.laneseq;
		start_yan_tmp.lanenum_ = maproad0ini.lanenum_;
		start_yan_tmp.x_left=start_yan_left[i].x;
		start_yan_tmp.y_left=start_yan_left[i].y;
		start_yan_tmp.x_right=start_yan_right[i].x;
		start_yan_tmp.y_right=start_yan_right[i].y;
		start_yan_tmp.b_left=maproad0ini.b_left;
		start_yan_tmp.b_right=maproad0ini.b_right;
		//cvSeqPushFront(app->gpsmap_road, &start_yan_tmp);
		cvSeqPush(app->gpsmap_road, &start_yan_tmp);
	}

	gpsmap_road_num = app->gpsmap_road->total;
	m_task = IVTASK_LANE;
	

	for(int i = 0;i<200;i++)
	{
		MidGpsPoint[i] = cvPoint2D64f(app->GPS_Point.x,app->GPS_Point.y);
		OriGpsPoint[i] = cvPoint2D64f(app->GPS_Point.x,app->GPS_Point.y);
	}

	// 需要弄明白每个变量的含义吗？
	int ob_num = 0;	
	int midd_num1 = 0;
	CvPoint2D64f MidPoint[512];
	CvPoint2D64f ccc[4];
	CvPoint2D64f CrossPoint[7];	
	int lane_result = 0;
	int laneob_result = 0;
	CvPoint2D64f WayPoint[200];
	double lane_s = 0;
	realtime_Gps = app->GPS_Point;
	gps_aimpoint = realtime_Gps;
	CvPoint2D64f v_map = cvPoint2D64f(256.0,411.0);
	inter_res = 1;
	int count_move;
	CvPoint2D64f lanepgps;
	double yulukoudis = 100;
	CvPoint2D64f stop_point;
	int lit_count = 0;	
	double laneid = 0;
	double hislaneid = 0;
	int count_lane = 0;
	int  point_num;
	int aaaa;	
	CvPoint2D64f obvegps = cvPoint2D64f(0,0);
	double lanchadis = 0;
	bool appro_ob = false;
	CvPoint2D64f zhongzhuan[200]={0};
	int changedaoshu = 0;
	bool changebz = false;
	int lane_num = 2;
	int inter_yumiao = 40;
	CvPoint2D64f changepoint = app->GPS_Point;
	double changedist;
	double aaa[6];
	bool goalspeed_flag=false;
	bool goalspeed_flag2=false;

	double udir;
	double stoplane_dis;
	int his_id1,his_id2;
	app->detpeople = false;
	int count_people = 0;
	int count_frontpeople = 0;
	bool first_navigate = true;

	bool uturnflag=0;
	bool shigongflag=0;
	bool blrturnflag=0;

	int uturnquitcnt;
	int shigongquitcnt;

	int leftturncnt=0;

	globalstrt = cvPoint2D64f(0,0);
	
	app->qidongstatus=7;

	bool kandeng_flag = true;
	laneturndircnt=0;

	brakeflagchange = false;

	app->continue_spdred = false;
	app->continue_spdredtmp = false;
	app->spdredacc = 0;
	app->spdredacctmp = 0;
	spdacctemp = 0;

	app->continue_brake = false;
	app->continue_braketmp = false;
	double drive_state_tmp;
	double lukouvertdisttmp;

	double speedreducedist_;

	double lukouobdistance;

	int up;
	double left_dis = 10000;
	double right_dis = 10000;
	double middle_dis = 10000;

	CvPoint2D64f rndfbeziertemp[200];
	bool bobserachflag = 1;

	double approinterdist;
	int approintertype;
	double approdistend;
	double approspdend;

	double updettt;

	double proyumiaodist;

	ob_dis_now=1000;
	ob_dis_pre=1000;
	ob_dis_prepre=1000;
	ob_dis_now_timer=0;
	ob_dis_pre_timer=0;
	ob_dis_prepre_timer=0;
	fasterobspdini=road_speed;
	bfasterobacv=0;
	bobfisrstapper=0;
	ob_loss_count=0;
	
	fasterobspdini_lchg=road_speed;
	bfasterobacv_lchg=0;
	bobfisrstapper_lchg=0;
	ob_dis_now_lchg=1000;
	ob_dis_pre_lchg=1000;
	ob_dis_prepre_lchg=1000;
	ob_dis_now_lchg_timer=0;
	ob_dis_pre_lchg_timer=0;
	ob_dis_prepre_lchg_timer=0;
	lanelostcnt=100;
	ob_loss_count_lchg=0;
	
	ob_loss_count_path=0;

	app->blanechanging=false;

	app->proctimelast = 0;
	app->proctimeStamp = 0;

	obleftlosscnt=0;
	obrightlosscnt=0;
	obpianyidir=0;
	width_L=-4;
	width_R=4;

	lukou_fx=1;
	yulukoupoint=cvPoint2D64f(0,0);
	m_task = IVTASK_LANE;
	m_task_history=0;
	countlast=0;

	lukoulockspd=road_speed;

	bholdbrake=0;

	yulukounolanechange=0;
	zhongdiannolanechange=0;
	yulukounolaneassist=0;

	app->braohui=0;

	app->inter_UTURN=0;

	blahuirndf=0;
	blanechangeend=0;
	blanechangeendyulukou=0;

	stopmiddledis=6;
	double broadanglechu;

	bool bspecialuturnpoint;
	bool bspecialshigongluduan;
	bool bspeciallrturn=1;

	app->bchulukoufound=1;

	bspecialuturnyulukou=0;

	app->bjianruiuturn=0;

	bspecialbiandaoshu=0;

	enterlukou=0;

	specialbiandaoendcnt=1000;

	app->bqishiluduan=1;

	app->bspecialuturnluduan=0;

	app->bspecialnoleftturnluduan=0;

	app->bleftturnban=0;

	bfeijidongche=0;

	app->bnobiandaoraozhang=0;

	blanesync=true;

	latpianyi_lanechgstrt=0;
	lonpianyi_lanechgstrt=0;

	bxingrenzixingche=0;
	x_right_xingrennum=0;
	x_left_xingrennum=0;

	for(int i=0;i<15;i++)
	{
		x_right_xingren[i]=0;
		x_left_xingren[i]=0;
	}

	SYSTEMTIME tt;
	GetLocalTime(&tt);
	int timernow=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

	//int timernow=app->systemtimer10ms;

	// 这个是干什么？ lanedriverstate 这个类主要有什么作用？
	for(int i=0;i<5;i++)
	{
		lanedriverstate.y_left_front[i]=-10000;
		lanedriverstate.y_left_rear[i]=-10000;
		lanedriverstate.y_right_front[i]=-10000;
		lanedriverstate.y_right_rear[i]=-10000;
		lanedriverstate.y_middle_front[i]=-10000;
		lanedriverstate.timer_left_front[i]=timernow;
		lanedriverstate.timer_left_rear[i]=timernow;
		lanedriverstate.timer_right_front[i]=timernow;
		lanedriverstate.timer_right_rear[i]=timernow;
		lanedriverstate.timer_middle_front[i]=timernow;
	}
	lanedriverstate.spdrelmidfront=-10000;
	lanedriverstate.spdrelleftfront=-10000;
	lanedriverstate.spdrelleftrear=-10000;
	lanedriverstate.spdrelrightfront=-10000;
	lanedriverstate.spdrelrightrear=-10000;

	bhardraozhang=false;

	lanedriverstate.boblefthis=false;
	lanedriverstate.bobrighthis=false;
	lanedriverstate.boblefthispre=false;
	lanedriverstate.bobrighthispre=false;

	bspecialzhilukoutingche=false;

	pianyifulu.x=0;
	pianyifulu.y=0;
	
	bspeciallukoustop=false;

	double verdistredcstrt;

	blanechgendly=false;
	blanechgendlycnt=0;

	int npianyineadhis=0;
	int npianyineadhisPX=0;

	bMidGpsPointinidataVd=false;

	int pianyinummax;

	bhaikanglaneused=true;

	haikanglanecnt=0;

	spdcurrentpnt=-1000;
	spdintersection=-1000;
	bspdlimitused=false;
	
	blaneusedisable=false;

	halfwidth=1.725;//1.85

	bRturnLightChk=false;
	bLturnLightChk=true;
	bZturnLightChk=true;

	while(1)
	{
		
		int iRet = WaitForSingleObject(m_IbeoMapEvent,100); 
		if(iRet != 0)
			continue;

		app->m_task2ctrl=m_task;
		app->road_speed2ctrl=road_speed;

		DWORD cur_time=GetTickCount();
		if(app->proctimelast)
		{
			outproctimediffer<<cur_time-app->proctimelast<<endl;
			app->proctimeStamp = cur_time-app->proctimelast;
		}
		app->proctimelast = cur_time;
SYSTEMTIME t1;
		GetLocalTime(&t1);
outproctime<<"time1 "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  m_task="<<m_task<<endl;

		// 更新 dymap 和 gps_point
		outtimecostpro<<"1"<<endl;
		app->critical_xinmap.Lock();
		CVehicle::vel_Map = getDyMap();//dy_map;getPerceptionMap();
		app->critical_xinmap.Unlock();
		outtimecostpro<<"2"<<endl;

		app->critical_section.Lock();
		realtime_Gps = app->GPS_Point;
		realtime_Dir = app->GPS_Direction;
		app->critical_section.Unlock();
      
		// 初始化变量
		app->stop = false;
		app->continue_braketmp = false;
		app->continue_spdredtmp = false;
		app->spdredacctmp = 0;
		spdacctemp = 0;
		app->bbanmaxiannoraozhang=0;
		app->drive_people = 50;
		int dis_pp = 1000;
		int count_stop = 0;
		int count_vehicle =0;
		int count_vehicle_time =0;
		MSG adasisData;
		app->range_flag = false;
		whole_range = false;
		app->brake_flag = false;
		brakeflagchange = false;
		app->goal_flag = false;
		app->goal_leftright = false;
		bool yufirst = false;
		int zpt = 0;
		bool stop_flag = false;
		double search_dis = 15;
		CvPoint2D64f spt;

		gpsmap_road_num = app->gpsmap_road->total;
		double dis_end = m_GpsData.GetDistance(globalendpoint.x,globalendpoint.y,app->GPS_Point.x,app->GPS_Point.y);
		double ver_dist_end = VertDist(app->GPS_Point,app->GPS_Direction,globalendpoint);
		int globalnum = app->gpsmap_road->total;

		double lastangle = m_GpsData.GetAngle(lastp1.x,lastp1.y,lastp2.x,lastp2.y);
		bool detaangle = false;
		if(cos(PI*(lastangle - app->GPS_Direction)/180)>cos(50*PI/180))
			detaangle = true;
		
		zhongdiannolanechange=false;
		if(ver_dist_end>0 && ver_dist_end<80 && globalnum<350 && dis_end < 120 && detaangle)
		{
			goalspeed_flag = true;
			if(ver_dist_end<20)
				goalspeed_flag2 =true;

			if(ver_dist_end<60)
				zhongdiannolanechange = true;

			if(ver_dist_end<40)
				approachgoal_flag = false;
		}
		
		if(ver_dist_end<4&& globalnum<310 && dis_end < 30 && detaangle)//5
		{
			Stop();
			Sleep(2000);
			control.setLight(LAMP_OFF);
			return 0;
		}

		app->bnearstop=false;

		drive_state_tmp = road_speed;

		// 判断是不是在车道内
		if(way_out)
		{	
			LaneInit();
			globalstrt = cvPoint2D64f(0,0);
			int ret_value = ReturnPartPoints_Ini(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
			if(ret_value==0)
				return 0;
			GetLanebezier(realtime_Gps,realtime_Dir,pianyi,40);
			lanechang_flag = true;
			able_ChangeLane = true;
			blanesync=true;
			app->left_right = 0;
			ob_tempoint = cvPoint2D64f(realtime_Gps.x,realtime_Gps.y);
			way_out = false;	
			app->drive_state = road_speed;
			app->drive_obstacle = max_speed;
			for(int i=0;i<200;i++)
			{
				rndfbezier_lchgdst[i].x=0;
				rndfbezier_lchgdst[i].y=0;
			}
		}



		app->drive_state = road_speed;
		app->drive_obstacle = max_speed;
		
		drive_state_tmp = road_speed;

		//cross_speed1 = 30/3.6;
		//cross_speed2 = 10/3.6;

		if(cross_speed1>road_speed)
			cross_speed1=road_speed;
		if(cross_speed2>road_speed)
			cross_speed2=road_speed;

		verdistredcstrt=(road_speed*road_speed-10/3.6*10/3.6)/2+20;
		if(verdistredcstrt<30)
			verdistredcstrt=30;
		if(goalspeed_flag && !goalspeed_flag2 && ver_dist_end<verdistredcstrt)
		{
			if(road_speed>10/3.6)
			{
				if(ver_dist_end<20)
					drive_state_tmp = 10/3.6;
				else if(ver_dist_end<verdistredcstrt)
					drive_state_tmp = sqrt(10/3.6*10/3.6+(road_speed*road_speed-10/3.6*10/3.6)*(ver_dist_end-20)/(verdistredcstrt-20));
			}
			if(app->GPS_Speed>10/3.6)
			{
				if(drive_state_tmp>app->GPS_Speed)
					drive_state_tmp = app->GPS_Speed;
			}
			else
				drive_state_tmp = 10/3.6;
			if(app->GPS_Speed>10/3.6 + 2.5/3.6)
				app->continue_braketmp = true;
			if((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(ver_dist_end>=21))
			{
				app->continue_spdredtmp = true;
				spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed)-10/3.6*10/3.6)/(2*(ver_dist_end-20));
				if(spdacctemp<-1.5)
					spdacctemp=-1.5;
			}
			else if(app->GPS_Speed>10/3.6 + 2.5/3.6)
			{
				app->continue_spdredtmp = true;
				spdacctemp = app->spdredacc;
			}
			if(spdacctemp<app->spdredacctmp)
				app->spdredacctmp = spdacctemp;
			if(app->brake_flag == true || brakeflagchange == false)
			{
				app->brake_flag = true;
				brakeflagchange = true;
			}
		}
		if(app->drive_state>drive_state_tmp)
			app->drive_state=drive_state_tmp;

		drive_state_tmp = road_speed;

		if (goalspeed_flag && goalspeed_flag2)
		{
			if(road_speed>5/3.6)
			{
				if(ver_dist_end<10)
					drive_state_tmp = 5/3.6;
				else if(ver_dist_end<20)
					drive_state_tmp = sqrt(5/3.6*5/3.6+(10/3.6*10/3.6-5/3.6*5/3.6)*(ver_dist_end-10)/10);//drive_state_tmp = (ver_dist_end-10)/10*5/3.6+5/3.6;
				else
					drive_state_tmp = 10/3.6;
			}
			if(app->GPS_Speed>5/3.6 + 2.5/3.6)
				app->continue_braketmp = true;
			if(app->GPS_Speed>5/3.6)
			{
				if(drive_state_tmp>app->GPS_Speed)
					drive_state_tmp = app->GPS_Speed;
			}
			else
				drive_state_tmp = 5/3.6;
			if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(ver_dist_end>=11))
			{
				app->continue_spdredtmp = true;
				spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed)-5/3.6*5/3.6)/(2*(ver_dist_end-10));
				if(spdacctemp<-1.5)
					spdacctemp=-1.5;
			}
			else if(app->GPS_Speed>5/3.6 + 2.5/3.6)
			{
				app->continue_spdredtmp = true;
				spdacctemp = app->spdredacc;
			}
			if(spdacctemp<app->spdredacctmp)
				app->spdredacctmp = spdacctemp;
			app->goal_flag = true;
		}
		if(app->drive_state>drive_state_tmp)
			app->drive_state=drive_state_tmp;

		int lanechangenumtmp_;
		CvPoint2D64f rndfbezierforpath[200];
		int ob_hx_left;
		int ob_hx_right;
		int ob_pianyi_num;
		int up_hx;
		int down_hx_left;
		int down_hx_right;
		bool bhxleft;
		bool bhxright;


		if(m_task==IVTASK_LANE)
			app->topDecison = "路上";
		else if(m_task==IVTASK_APPROINTERSECTION)
			app->topDecison = "预路口";
		else if(m_task==IVTASK_INTERSECTION)
			app->topDecison = "路口";

		m_taskpublic=m_task;

		app->bnobiandaoraozhang=0;
		if(m_task==IVTASK_LANE || m_task==IVTASK_INTERSECTION)
			blabianqiansureq=false;

		// 几个不同的Case分别是干什么？
		switch( m_task )
		{
			case IVTASK_LANE:
			case IVTASK_APPROINTERSECTION:

				bspeciallukoustop=false;

				enterlukou=0;
				bspeciallrturn=1;

				app->bjianruiuturn=0;

				uturnquitcnt=0;
				shigongquitcnt=0;

				for(int i=0;i<200;i++)
					CrossPath[i]=rndfbezier[i];

				uturnflag=0;
				shigongflag=0;
				blrturnflag=0;

				yulukounolanechange=false;
				//yulukounolaneassist=0;

				if(m_task==IVTASK_LANE || lukou_fx==8 || lukou_fx==7 || lukou_fx==6)
					fx=1;
				else
					fx=lukou_fx;
				app->inter_small=false;
				app->inter_UTURN=0;
				lukoulockspd=road_speed;

				if(m_task==IVTASK_APPROINTERSECTION)
				{
					yulukoujiansu();
				}
				else
				{
					yulukounolaneassist=0;

					control.ClearzStop(1);
				}
				OnRoadDrive(ob_num,count_move,1);

				if (app->left_right!=0)
				{
					if (app->GPS_Speed < 6/3.6)
					{
						proyumiaodist = 10;
					}
					else if (app->GPS_Speed < 16/3.6)
					{
						proyumiaodist = 13;
					}
					else if (app->GPS_Speed < 22/3.6)
					{
						proyumiaodist = 15;
					}
					else if (app->GPS_Speed < 42/3.6)
					{
						proyumiaodist = 25;
					}
					else if (app->GPS_Speed < 65/3.6)
					{
						proyumiaodist = 40;
					}
					else
					{
						//proyumiaodist = 45;
						proyumiaodist = 40;
					}
				}
				else
				{
					if (app->GPS_Speed < 6/3.6)
					{
						proyumiaodist = 10;
					}
					else if (app->GPS_Speed < 16/3.6)
					{
						proyumiaodist = 13;
					}
					else if (app->GPS_Speed < 22/3.6)
					{
						proyumiaodist = 15;
					}
					else if (app->GPS_Speed < 42/3.6)
					{
						proyumiaodist = 25;
					}
					else if (app->GPS_Speed < 65/3.6)
					{
						proyumiaodist = 40;
					}
					else
					{
						//proyumiaodist = 45;
						proyumiaodist = 40;
					}
				}

				for (int i=0; i<512; i++)
				{
					for (int j=0; j<512; j++)
					{
						vel_Map_path->MapPoint[i][j]=vel_Map->MapPoint[i][j];
					}
				}

				for(int i=0;i<200;i++)
					rndfbezierforpath[i]=rndfbezier[i];

				break;

				case IVTASK_INTERSECTION:

					bspeciallukoustop=false;

					blanechgendly=false;
					blanechgendlycnt=0;

					//timernow=app->systemtimer10ms;
					GetLocalTime(&tt);
					timernow=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));


					for(int i=0;i<5;i++)
					{
						lanedriverstate.y_left_front[i]=-10000;
						lanedriverstate.y_left_rear[i]=-10000;
						lanedriverstate.y_right_front[i]=-10000;
						lanedriverstate.y_right_rear[i]=-10000;
						lanedriverstate.y_middle_front[i]=-10000;
						lanedriverstate.timer_left_front[i]=timernow;
						lanedriverstate.timer_left_rear[i]=timernow;
						lanedriverstate.timer_right_front[i]=timernow;
						lanedriverstate.timer_right_rear[i]=timernow;
						lanedriverstate.timer_middle_front[i]=timernow;
					}
					lanedriverstate.spdrelmidfront=-10000;
					lanedriverstate.spdrelleftfront=-10000;
					lanedriverstate.spdrelleftrear=-10000;
					lanedriverstate.spdrelrightfront=-10000;
					lanedriverstate.spdrelrightrear=-10000;

					lanedriverstate.boblefthis=false;
					lanedriverstate.bobrighthis=false;
					lanedriverstate.boblefthispre=false;
					lanedriverstate.bobrighthispre=false;
					app->left_right=0;
					specialbiandaoendcnt=1000;
					laneturndircnt=0;
					app->laneturndir=0;
					pianyi = cvPoint2D64f(0,0);
					control.ClearzStop(1);
					app->braohui=0;
					blahuirndf=0;
					blanechangeend=0;
					blanechangeendyulukou=0;
					stopmiddledis=6;
					able_ChangeLane = true;
					blanesync=true;
					latpianyi_lanechgstrt=0;
					lonpianyi_lanechgstrt=0;
					for(int i=0;i<200;i++)
					{
						rndfbezier_lchgdst[i].x=0;
						rndfbezier_lchgdst[i].y=0;
					}

					if(lukou_fx==1)
						ReturnPartPoints(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
					else 
						ReturnPartPoints_lukou(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);

					uturnflag=0;
					for(int i=0;i<200;i++)
						CrossPath[i]=rndfbezier[i];

					if(lukou_fx==1)
					{
						if(app->drive_state>cross_speed1)
							app->drive_state=cross_speed1;
						lukoulockspd=cross_speed1;
					}
					else
					{
						if(app->drive_state>cross_speed2)
							app->drive_state=cross_speed2;
						if(lukoulockspd>cross_speed2)
							lukoulockspd=cross_speed2;
						if(app->GPS_Speed<lukoulockspd)
							lukoulockspd=app->GPS_Speed;
						if(lukou_fx==4 || lukou_fx==6)
						{
							if(lukoulockspd<5/3.6)
								lukoulockspd=5/3.6;
						}
						else if(lukou_fx==8)
						{
							if(lukoulockspd<10/3.6)
								lukoulockspd=10/3.6;//8/3.6
						}
						else
						{
							if(lukoulockspd<cross_speed2)
								lukoulockspd=cross_speed2;
						}
						if(app->drive_state>lukoulockspd)
							app->drive_state=lukoulockspd;
						if(lukou_fx==3||lukou_fx==4)
							control.setLight(LEFT_LAMP_ON);
						else if(lukou_fx==2)
							control.setLight(RIGHT_LAMP_ON);
						else
							control.setLight(LAMP_OFF);
					}

					if(lukou_fx==8 || lukou_fx==7 || lukou_fx==6)
						fx=1;
					else
						fx=lukou_fx;
					if(lukou_fx==2||lukou_fx==3||lukou_fx==4||lukou_fx==8||lukou_fx==6)
						app->inter_small=true;
					else
						app->inter_small=false;
					//lukoujiashi();
					if(lukou_fx==1 || lukou_fx==7)
						zhixinglukoujiashi();
					else
					{
						lukoujiashi();
						ob_dis_now=1000;
						ob_dis_pre=1000;
						ob_dis_prepre=1000;
						ob_dis_now_timer=0;
						ob_dis_pre_timer=0;
						ob_dis_prepre_timer=0;
						bfasterobacv=0;
						ob_loss_count=0;
					}

					enterlukou=1;
					fasterobspdini_lchg=road_speed;
					bfasterobacv_lchg=0;
					bobfisrstapper_lchg=0;
					ob_dis_now_lchg=1000;
					ob_dis_pre_lchg=1000;
					ob_dis_prepre_lchg=1000;
					ob_dis_now_lchg_timer=0;
					ob_dis_pre_lchg_timer=0;
					ob_dis_prepre_lchg_timer=0;
					ob_loss_count_lchg=0;

					for (int i=0; i<512; i++)
					{
						for (int j=0; j<512; j++)
						{
							vel_Map_path->MapPoint[i][j]=vel_Map->MapPoint[i][j];
						}
					}
					for(int i=0;i<200;i++)
						rndfbezierforpath[i]=CrossPath[i];

					break;
		}

		if (app->left_right!=0)
		{
			if (app->GPS_Speed < 6/3.6)
			{
				proyumiaodist = 10;
			}
			else if (app->GPS_Speed < 16/3.6)
			{
				proyumiaodist = 13;
			}
			else if (app->GPS_Speed < 22/3.6)
			{
				proyumiaodist = 15;
			}
			else if (app->GPS_Speed < 42/3.6)
			{
				proyumiaodist = 25;
			}
			else if (app->GPS_Speed < 65/3.6)
			{
				proyumiaodist = 40;
			}
			else
			{
				//proyumiaodist = 45;
				proyumiaodist = 40;
			}
		}
		else
		{
			if (app->GPS_Speed < 6/3.6)
			{
				proyumiaodist = 10;
			}
			else if (app->GPS_Speed < 16/3.6)
			{
				proyumiaodist = 13;
			}
			else if (app->GPS_Speed < 22/3.6)
			{
				proyumiaodist = 15;
			}
			else if (app->GPS_Speed < 42/3.6)
			{
				proyumiaodist = 25;
			}
			else if (app->GPS_Speed < 65/3.6)
			{
				proyumiaodist = 40;
			}
			else
			{
				//proyumiaodist = 45;
				proyumiaodist = 40;
			}
		}

		for(int inum=0;inum<3;inum++)
		{
			if(app->GPS_Speed > 6/3.6 && inum==1)
			{
				gps_aimpoint_new[1].x=0;
				gps_aimpoint_new[1].y=0;
				gps_aimdir_new[1]=0;
				gps_aimpoint_new[2].x=0;
				gps_aimpoint_new[2].y=0;
				gps_aimdir_new[2]=0;
				break;
			}
			if(app->GPS_Speed < 6/3.6)
			{
				if(inum==0)
					proyumiaodist=13;
				else if(inum==1)
					proyumiaodist=10;
				else if(inum==2)
					proyumiaodist=8;
			}
			
			{
				bhardraozhang=false;
				gps_aimpoint_new[inum] = passintersectionstate.findUturnAimPoint(realtime_Gps,realtime_Dir,rndfbezierforpath,proyumiaodist,gps_aimdir_new[inum]);
			}
		}

		//aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
		app->continue_brake=app->continue_braketmp;

		app->continue_spdred=app->continue_spdredtmp;
		app->spdredacc=app->spdredacctmp;

		app->drive_state_update=app->drive_state;
		app->drive_curvature_update=app->drive_curvature;
		app->drive_obstacle_update=app->drive_obstacle;
		app->limit_speed_update=app->limit_speed;

		m_task_history=m_task;

		SetEvent(m_pathevent);
		

		GetLocalTime(&t1);
outproctime<<"time2 "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  "<<endl;

	}
	return 0;
}


int CVehicleExe::GetMissionNum( )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	return app->ADAS_points->total;
}
int CVehicleExe::GetMissionNum2( )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	return app->mission_pt.size();
}
int CVehicleExe::GetMissionPoint(LEAD lead[],int num)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	for(int i = 0; i<num;i++)
	{
		lead[i] =  *(LEAD*)cvGetSeqElem(app->ADAS_points,i);
		outleadpoint<<setprecision(11)<<lead[i].id<<" "<<lead[i].lng<<" "<<lead[i].lat<<endl;
	}
	return 1;
}
int CVehicleExe::GetMissionPoint2(LEAD lead[],int num)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	for(int i = 0; i<num;i++)
	{
		lead[i] = app->mission_pt.front();
		app->mission_pt.pop();
	}
	return 1;
}
int CVehicleExe::ReturnPartPointsSeqNum(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err)
{
	int num = gpath->total;
	CvPoint2D64f pt,pt_next;
	double l = 0;
	CvPoint2D64f mpt,mpt_next;
	int count = 0;
	CvPoint2D64f *temp_point;
	for(int i = 0;i<num-1;i++)
	{
		pt = *(CvPoint2D64f*)cvGetSeqElem( gpath, i );
		pt_next =  *(CvPoint2D64f*)cvGetSeqElem( gpath, i+1 );
		l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
		mpt = m_GpsData.APiontConverD(m_gps,pt,dir);
		mpt_next = m_GpsData.APiontConverD(m_gps,pt_next,dir);
		if(l < s && (mpt_next.y<=mpt.y))
		{
			count = i;
			break;
		}
		/////加一个判断 找不到找最近的
	}

	for(int i = 0;i<count;i++)
	{
		cvSeqPopFront( gpath,NULL );
	}

	num = gpath->total;
	if(num<2)
		return 0;
	l=0;
	count = 0;
	for(int i = 0;i<num-1;i++)
	{
		pt = *(CvPoint2D64f*)cvGetSeqElem( gpath, i );
		pt_next =  *(CvPoint2D64f*)cvGetSeqElem( gpath, i+1 );
		l+= m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>100)
		{
			count = i;
			break;
		}//加一个判断
	}
	temp_point = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	for(int i = 0;i<count+2;i++)
	{
		temp_point[i] = *(CvPoint2D64f*)cvGetSeqElem( gpath, i );
	}
	Beziercazhi(temp_point,count+1,rndf);
	for (int i = 0;i<200;i++)
	{
		ori_rndf[i] = rndf[i];
	}
	for (int i = 0;i<200;i++)
	{
		rndf[i].x =  rndf[i].x-err.x;
		rndf[i].y =  rndf[i].y-err.y;
	}
	return count/4+1;
}
int CVehicleExe::ReturnPartPointsSeqNumOnTimer(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200])
{
	int num = gpath->total;
	CvPoint2D64f pt,pt_next;
	double l,dis = 0;
	CvPoint2D64f mpt,mpt_next;
	int count = 0;
	CvPoint2D64f *temp_point;
	bool ab_flag = false;
	for(int i = 0;i<num-1;i++)
	{
		pt = *(CvPoint2D64f*)cvGetSeqElem( gpath, i );
		pt_next =  *(CvPoint2D64f*)cvGetSeqElem( gpath, i+1 );
		l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
		mpt = m_GpsData.APiontConverD(m_gps,pt,dir);
		mpt_next = m_GpsData.APiontConverD(m_gps,pt_next,dir);
		if(l < s && (mpt_next.y<=mpt.y))
		{
			count = i;
			ab_flag = true;
			break;
		}
		/////加一个判断 找不到找最近的
	}

	double min_s = 99999999;
	num = gpath->total;
	if (!ab_flag)
	{
		for (int i=0;i<num-1;i++)
		{
			CvPoint2D64f pt1 = *(CvPoint2D64f*)cvGetSeqElem( gpath, i );
			double L_DIS= m_GpsData.GetDistance(pt1.x,pt1.y,m_gps.x,m_gps.y);
			if(L_DIS < min_s)
			{
				count = i;
				min_s = L_DIS;
			}
		}
	}

	for(int i = 0;i<count;i++)
	{
		cvSeqPopFront( gpath,NULL );
	}
	num = gpath->total;
	if(num<2)
		return 0;
	l=0;
	count = 0;
	for(int i = 0;i<num-1;i++)
	{
		pt = *(CvPoint2D64f*)cvGetSeqElem( gpath, i );
		pt_next =  *(CvPoint2D64f*)cvGetSeqElem( gpath, i+1 );
		l+= m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>100)
		{
			count = i;
			break;
		}//加一个判断
	}
	double temp = 99999999999;
	int count_num=count;
	count=0;
	for(int i = 0;i<count_num+1;i++)
	{
		pt = *(CvPoint2D64f*)cvGetSeqElem( gpath, i );
		dis = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
		if(dis < temp)
		{
			temp = dis;
			count = i;
		}
	}
	return count;
}
//int CVehicleExe::PassMissionPoint(CvPoint2D64f m_gps,double dir)
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	double lukou_jiexian = 0;
//	app->onroad = m_task;
//	if(pass_count>0)
//	{
//		pass_count--;
//		return 2;
//	}
//	LEAD cur_lead = LeadPoint[seq_num-1];
//	if(cur_lead.param1 == 3)
//		app->stop_lane = true;
//	CvPoint2D64f leadpt;
//	leadpt.x=cur_lead.lat;
//	leadpt.y=cur_lead.lng;
//	double dist_vert = VertDist(m_gps,dir,leadpt);
//	double dist = m_GpsData.GetDistance(m_gps.x,m_gps.y,leadpt.x,leadpt.y);
//	//outn1<<"sist_vert "<<dist_vert<<endl;
//	if(cur_lead.param1 == 0||cur_lead.param1 == 1||cur_lead.param1 == 3)
//	{
//		
//		if(dist_vert < 50 &&abs(dist)<50)
//		{	
//			if(m_task == IVTASK_LANE)
//				way_out = true;
//			m_task = IVTASK_APPROINTERSECTION;
//		}
//		else
//			m_task = IVTASK_LANE;
//	}
//	
//	else if(cur_lead.param1 == 2)
//	{
//		m_task = IVTASK_INTERSECTION;
//	}
//	/*if(dist_vert > 10 &&dist_vert < 80 &&abs(dist)<120)
//			m_task = IVTASK_APPROINTERSECTION;*/
//	if(m_task == 5002)
//	{
//	/*	if(app->u_t)
//			lukou_jiexian = -2;
//		else*/
//		lukou_jiexian = 10/*10*/;
//	}
//	else
//		lukou_jiexian = 3;
//	if(cur_lead.param2 == 4)
//		lukou_jiexian = 15;
//	LEAD cur_lead1 = LeadPoint[seq_num];
//	CvPoint2D64f pt;
//	pt.x=cur_lead1.lat;
//	pt.y=cur_lead1.lng;
//	double dir_next = m_GpsData.GetAngle(pt,leadpt);
//	double dirr_err = abs(dir_next-dir);
//	out124<<"seqnum "<<seq_num<<", "<<"distvert "<<dist_vert<<", "<<"dist "<<dist<<endl;
//	if(dist_vert<lukou_jiexian/*10*/&&abs(dist)<30&&(dirr_err<90||abs(360-dirr_err)<90||m_task == IVTASK_APPROINTERSECTION||m_task == IVTASK_LANE))
//	{
//		if(cur_lead.param2==4||cur_lead.param2==5.4||cur_lead.param2==0.4)
//			app->u_t=true;
//		if(cur_lead.param1 == 3)
//			app->stop_lane = true;
//		pass_count=50;
//		outbbb<<"经过的点序号: "<<seq_num<<endl;
//		seq_num++;
//		way_out = true;
//		cur_lead = LeadPoint[seq_num-1];
//		if(cur_lead.param1 == 3)
//			app->stop_lane = true;
//
//		if(cur_lead.param1 == 0||cur_lead.param1 == 1||cur_lead.param1 == 3)
//			m_task = IVTASK_LANE;
//		else if(cur_lead.param1 == 2&&LeadPoint[seq_num-2].param1==1)
//		{
//			m_task = IVTASK_INTERSECTION;
//		}
//		else
//			m_task = IVTASK_LANE;
//		return 1;
//	}
//
//	return 0;
//}
int CVehicleExe::PassMissionPoint(CvPoint2D64f m_gps,double dir)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	//m_task = IVTASK_LANE;
	app->onroad = m_task;
	return 0;
}
int CVehicleExe::PassMissionPointbyRNDF(CvPoint2D64f m_gps,double dir)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	double lukou_jiexian = 0;
	//app->onroad = m_task;

	CvPoint2D64f leadpt,endpt;
	leadpt =  *(CvPoint2D64f*)cvGetSeqElem( global_path, seq_num);

	double dist_vert = VertDist(m_gps,dir,leadpt);
	double dist = m_GpsData.GetDistance(m_gps.x,m_gps.y,leadpt.x,leadpt.y);
	
	solveID(leadpt);
	
	int num = RNDFGPS.GetLaneNum(ID1,ID2);
	endpt = RNDFGPS.GetPoint(ID1,ID2,num);

	double enddist_vert = VertDist(m_gps,dir,endpt);
	double enddist = m_GpsData.GetDistance(m_gps.x,m_gps.y,endpt.x,endpt.y);

	if(ID3!=1)//出口点，第一个点
	{
		if(enddist_vert < 70 &&abs(enddist)<120)
		{	
			if(m_task == IVTASK_LANE)
				way_out = true;
			m_task = IVTASK_APPROINTERSECTION;
		}
		else
			m_task = IVTASK_LANE;
		

	}

	else if(ID3 == 1)//入口点，最后一个点
	{
		m_task = IVTASK_INTERSECTION;
	}


	/*if(dist_vert > 10 &&dist_vert < 80 &&abs(dist)<120)
			m_task = IVTASK_APPROINTERSECTION;*/
	if(m_task == 5002)
	{
	/*	if(app->u_t)
			lukou_jiexian = -2;
		else*/
		lukou_jiexian = 10/*10*/;
	}
	else
		lukou_jiexian = 0/*15*/;

	CvPoint2D64f pt =  *(CvPoint2D64f*)cvGetSeqElem( global_path, seq_num+1);
	double dir_next = m_GpsData.GetAngle(pt,leadpt);
	double dirr_err = abs(dir_next-dir);
	out124<<"seqnum "<<seq_num<<", "<<"distvert "<<dist_vert<<", "<<"dist "<<dist<<endl;
	if(dist_vert<lukou_jiexian/*10*/&&abs(dist)<30&&(dirr_err<30||abs(360-dirr_err)<30||m_task == IVTASK_APPROINTERSECTION||m_task == IVTASK_LANE))
	{
		//if(cur_lead.param2==4||cur_lead.param2==5.4||cur_lead.param2==0.4)
		//	app->u_t=true;
		//if(cur_lead.param1 == 3)
		//	app->stop_lane = true;
		//pass_count=50;

		seq_num++;

		
		leadpt =  *(CvPoint2D64f*)cvGetSeqElem( global_path, seq_num);

		solveID(leadpt);
		if (ID3 == 2 || ID3 == 1)
		{
			way_out = true;
		}
		if(ID3!=1)
			m_task = IVTASK_LANE;
		else
		{
			m_task = IVTASK_INTERSECTION;
		}
	
		return 1;
	}

	return 0;
}

int CVehicleExe::GetSeqNum(CvPoint2D64f gps,double dir) 
{
	//MAPPOINT *p = solvepoint2(gps.x,gps.y,dir);
	MAPPOINTexe *p=solvepoint3(gps.x,gps.y,dir,pathexe);

	if (p!=NULL)
	{
		solveID(p);
		CvPoint2D64f pt1,pt2;
		pt1.x=p->x;
		pt1.y=p->y;
		int n = global_path->total;
		for (int i = 0;i<n;i++)
		{
			pt2= *(CvPoint2D64f*)cvGetSeqElem( global_path, i );
			if (pt2.x==pt1.x&&pt2.y==pt1.y)
				return i;
		}
	}

	return -1;
}
//bool CVehicleExe::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	
//	app->critical_section.Lock();//锁住
//	CvPoint2D64f m_gps = app->GPS_Point;
//	double m_gpsdir = app->GPS_Direction;
//
//	app->critical_section.Unlock();
//
//	CvPoint2D64f Rndf_MapPoint[200]={0};
//	for(int i=0;i<200;i++)
//	{
//
//		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);
//		if(m_gps.x==GPSpoint[i].x&&m_gps.y==GPSpoint[i].y)
//		{
//			Rndf_MapPoint[i].x = 256;
//			Rndf_MapPoint[i].y = 411;
//		}
//	}
//
//	int m_up = 412-up*5;
//	int m_down = 411 -down*5;
//
//	int x = 0;
//	int y = 0;
//	for(int i = 0;i<200;i++)
//	{
//		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 512||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
//			continue;
//		x = Rndf_MapPoint[i].x;
//		y = Rndf_MapPoint[i].y;
//		for(int m=-5;m<5;m++)
//			for(int n=-5;n<5;n++)
//			{
//				/*if(y+m>360&&y+m<430)
//					continue;*/
//				if(y+m>511||y+m<0||x+n>511||x+n<0)
//					continue;
//				if(map->MapPoint[y+m][x+n] == 8)
//				{
//					
//					return true;
//
//				}
//			}
//	}
//	return false;
//}
bool CVehicleExe::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();
	int ob_num = 0;
	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);
		if(m_gps.x==GPSpoint[i].x&&m_gps.y==GPSpoint[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}
	if(Rndf_MapPoint[199].x == 256 && Rndf_MapPoint[199].y == 411)
	{
		return false;
	}
	int m_up = 412-up*5;
	int m_down = 411 -down*5;

	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-10;m<10;m++)
			for(int n=-6;n<6;n++)
			{
				if(y+m>380&&y+m<430)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8||map->MapPoint[y+m][x+n] == 18)
				{
					ob_x = x+n;
					ob_y = y+m;
					vehicleob_x = ob_x;
					vehicleob_y = ob_y;
					ob_num++;
					if(ob_num > 2)
					return true;
			
				}
			}
	}
	return false;
}
int CVehicleExe::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	 up = 80;
//	int up2 = 20;
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();

	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);
		if(m_gps.x==GPSpoint[i].x&&m_gps.y==GPSpoint[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}
	if(Rndf_MapPoint[199].x == 256 && Rndf_MapPoint[199].y == 411)
	{
		return false;
	}
	int m_up = 412-up*5;
	int m_down = 411 -down*5;
	int ori_up = 311;
	int ori_down = 411;
	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)///近处有动态障碍物
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > ori_down||Rndf_MapPoint[i].y < ori_up)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-10;m<10;m++)
			for(int n=-5;n<6;n++)
			{
				/*if(y+m>380&&y+m<430)
					continue;*/
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 28)
				{
					return 1;

				}
			}
	}
	
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-10;m<10;m++)
			for(int n=-5;n<6;n++)
			{
				if(y+m>380&&y+m<430)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8/*||map->MapPoint[y+m][x+n] == 38*/||map->MapPoint[y+m][x+n] == 18)
				{
					return 1;
			
				}
			}
	}
	for(int i = 0;i<200;i++)//zhongduan处有动态障碍物
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 0||Rndf_MapPoint[i].y < 411)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-10;m<10;m++)
			for(int n=-5;n<6;n++)
			{
				/*if(y+m>380&&y+m<430)
				continue;*/
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 28||map->MapPoint[y+m][x+n] == 38)
				{
					return 2;

				}
			}
	}
	return false;
}
int CVehicleExe::SearchLeftRightObstacle()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	MoveLeft(/*OriGpsPoint*/MidGpsPoint,exe_leftpoint,3.7);
	ob_left = SearchObstacle(exe_leftpoint,vel_Map,app->GPS_Speed*2+35,0);
	out128<<"	ob_left "<<ob_left<<endl;
	if(!ob_left)
	{
		app->secondDecison = "左换道";
		app->left_right = 1;

		//urbanroadcontext.SetUrbanRoadState(&changelanestate);
		//changelanestate.ChangeLaneDirve();
		aim_point = cvPoint2D64f(256-3.7*5,250);
		gps_aimpoint = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,aim_point);
		gps_aimdir = 74;
		pathplan.KeepLane(realtime_Gps,realtime_Dir,gps_aimpoint,gps_aimdir,MidGpsPoint,0);
	}
	else
	{
		MoveLeft(/*OriGpsPoint*/MidGpsPoint,exe_rightpoint,-3.7);
		ob_right = SearchObstacle(exe_rightpoint,vel_Map,app->GPS_Speed*2+35,0);
		out128<<"	ob_right "<<ob_right<<endl;
		if(ob_right)
		{	
			app->secondDecison = "遇障停车";
			//app->left_right = 0;
			//aim_point = cvPoint2D64f(ob_x,250);
			////Stop();
			//gps_aimpoint = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,aim_point);
			//gps_aimdir = 74;
			//pathplan.KeepLane(realtime_Gps,realtime_Dir,gps_aimpoint,gps_aimdir,MidGpsPoint);
			//Sleep(100);
		}
		else
		{
			app->secondDecison = "右换道";
			app->left_right = -1;
			aim_point = cvPoint2D64f(256+3.7*5,250);
			gps_aimpoint = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,aim_point);
			gps_aimdir = 74;
			pathplan.KeepLane(realtime_Gps,realtime_Dir,gps_aimpoint,gps_aimdir,MidGpsPoint,0);
		}
	}
	return 0;
}
int CVehicleExe::InitExe()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	m_MissionEvent = CreateEvent(NULL,FALSE,FALSE,NULL);
	PeekMessage(&msgThreadData, NULL, WM_INTERVAL_EXE, WM_INTERVAL_EXE,PM_NOREMOVE);
	setPerceptionNetwork();
	setSignalNetwork();
	app->u_t = false;
	app->bshigongluduanlock=false;
	app->bguihualukou=false;
	app->road_ut = false;
	app->road_goal = false;
	app->update_flag = false;
	app->range_flag = false;
	app->inter_small = false;


	app->drive_state = road_speed;
	app->drive_obstacle = max_speed;
	app->drive_curvature = max_speed;
	way_out = true;
	app->raozhang = false;
	return 1;
}


/*关闭发送数据线程*/
void CVehicleExe::StartCtrl()
{
	m_hThreadCtrl = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theCtrlThread,
		this, 0, &dwCtrlThreadId);
}

/*关闭发送数据线程*/

DWORD CVehicleExe::theCtrlThread(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->CtrlThread());
}

//控制转换线程
DWORD CVehicleExe::CtrlThread()	//控制改
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	MSG   msgThreadData;

	PeekMessage(&msgThreadData, NULL, WM_INTERVAL_EXE, WM_INTERVAL_EXE,PM_NOREMOVE);

	UdpCnt=0;
	lastacc=0;
	spdsentlast=0;

	while(1)
	{	
		if (control.m_endControlThreadFlag)
		{
			AfxEndThread(0);
		}
		//取当前的GPS坐标和方向
		CvPoint2D64f currentPosition = cvPoint2D64f(0, 0);
		double currentDirection = 0;
		double speed = 0;
		double yawrate = 0;

		int iRet = WaitForSingleObject(app->m_hEvent,100);

		app->critical_section.Lock();
		currentPosition = app->GPS_Point;
		currentDirection = app->GPS_Direction /*- 0.5*/;
		speed = app->GPS_Speed;
		yawrate = app->GPS_YawRate;
		app->critical_section.Unlock();

		if ((iRet==0)||(STATUS_TIMEOUT==iRet))
		{
			if(iRet==0)
			{
				app->critical_planningroad.Lock();
				if(planning_road->total > 0)
				{
					cvClearSeq( control.m_roadPoint);
					control.m_roadPoint = cvCloneSeq (planning_road) ;
				}
				app->critical_planningroad.Unlock();
			}
			double du1 = app->drive_state_update;
			double du2 = app->drive_curvature_update;
			double du3 = app->drive_obstacle_update;
			double du4 = app->drive_obstacle2_update;
			double du5 = app->limit_speed_update;
			double desirespd;
			desirespd = min(du1, du2);
			desirespd = min(desirespd, du3);
			desirespd = min(desirespd, du4);
			desirespd = min(desirespd, du5);
			double desacc=0;
			bool desaccVd=false;
			bool bstopflag=false;
			if(((app->continue_spdred || app->continue_brake)&&(lastacc<0 || app->spdredacc<0)) || app->stop || (app->GPS_Speed>desirespd+4/3.6 && desirespd>=0))
			{
				if(app->GPS_Speed>desirespd+4.5/3.6 && desirespd>=0)
				{
					if(app->spdredacc>=0)
					{
						if(lastacc<0)
							app->spdredacc=lastacc;
						else
							app->spdredacc=-1;
					}
				}
				if(app->spdredacc>lastacc)
					desacc=lastacc;
				else
				{
					desacc=app->spdredacc;
					if(desacc<-3)//-4
						desacc=-3;
					lastacc=desacc;
				}
				if(app->stop)
				{
					if(desacc>-1)
					{
						desacc=-1;
						lastacc=desacc;
					}
					bstopflag=true;
				}
				if(desacc<0)
					desaccVd=true;
			}
			else
				lastacc=0;
			
			if(abs(desirespd)<0.0001)
			{
				if(desacc>-1)
				{
					desacc=-1;
					app->spdredacc=-1;
					lastacc=desacc;
					desaccVd=true;
				}
			}

			if(app->stop)
			{
				app->critical_planningroad.Lock();
				cvClearSeq(control.m_roadPoint);
				for(int i = 0; i<200; i++)
				{
					cvSeqPush(control.m_roadPoint, &currentPosition);
				}
				app->critical_planningroad.Unlock();
				desirespd=0;
			}
			double desstr=0;
			bool desstrVd=false;
			int APA=0;

			
			{
				if(spdsentlast!=0 && desirespd>spdsentlast)
					desirespd=desirespd*0.3+spdsentlast*0.7;

				spdsentlast=desirespd;

				control.m_uDesire=desirespd;

				char sendbuf[2000];

				struct
				{
					int cnt;
					bool sstop;
					double desspd;
					double desacc;
					bool desaccVd;
					double desstr;
					bool desstrVd;
					int APA;
					unsigned int light;
					CvPoint2D64f pnts[100];
					int pLevel;
				} udpdatasent;
				udpdatasent.cnt=UdpCnt;
				udpdatasent.sstop=bstopflag;
				udpdatasent.desspd=desirespd;
				udpdatasent.desacc=desacc;
				udpdatasent.desaccVd=desaccVd;
				udpdatasent.desstr=-desstr;
				udpdatasent.desstrVd=desstrVd;
				udpdatasent.APA=APA;
				udpdatasent.light=control.m_controlParam.light;

				/*
				if (app->inter_small)
				{
					m_pidLat.Kp = 35;
					if (fx==3)
					{
						m_pidLat.Kp = 35;
					}
					if (fx==4)
					{
						m_pidLat.Kp = 26;
					}
				}
				else if(app->GPS_Speed>25/3.6)
					m_pidLat.Kp = 35;
				else
					m_pidLat.Kp = 26;
				if(app->bjianruiuturn)
					m_pidLat.Kp = 45;//65//100
				else if(app->inter_UTURN)
					m_pidLat.Kp = 40;//65
					*/
				udpdatasent.pLevel=0;
				if (app->inter_small)
					udpdatasent.pLevel=1;
				else if(app->GPS_Speed>25/3.6)
					udpdatasent.pLevel=0;//1
				else
					udpdatasent.pLevel=0;
				if(app->bjianruiuturn)
					udpdatasent.pLevel=2;
				else if(app->inter_UTURN)
					udpdatasent.pLevel=2;

				bool bbreakflag=0;

				app->critical_planningroad.Lock();
				if(control.m_roadPoint->total<200)
				{
					bbreakflag=true;
					outUdpRoadPntstate<<"wrong!!!!!!!!!!"<<endl;
				}
				else
				{
					for(int i=0;i<100 && 2*i<control.m_roadPoint->total;i++)
					{
						CvPoint3D64f aimPoint_temp = *(CvPoint3D64f*)cvGetSeqElem( control.m_roadPoint, i*2);
						udpdatasent.pnts[i].x=aimPoint_temp.x;
						udpdatasent.pnts[i].y=aimPoint_temp.y;
					}
				}
				app->critical_planningroad.Unlock();
				
				if(bbreakflag==0)
				{

					outUdpRoadPntstate<<"right!! UdpCnt="<<UdpCnt<<endl;

					for(int i=0;i<100;i++)
					{
						outUdpRoadPntstate<<setprecision(18)<<udpdatasent.pnts[i].x<<","<<udpdatasent.pnts[i].y<<",";
					}
					outUdpRoadPntstate<<endl;

					memcpy(sendbuf,(char *)&udpdatasent,sizeof(udpdatasent));
					
					UdpCnt++;
					
					//SendToContrlSocket=socket(AF_INET,SOCK_DGRAM,0);
					sendto(SendToContrlSocket,sendbuf,2000,0,(sockaddr*)&ControlAddrSrv,sizeof(sockaddr));
				}
			}
		}
	}
	return 0;
}
void CVehicleExe::Gettestmap(I_Map &map)
{
	int left = 247;
	int right = 264;
		for(int i = 0;i<512;i++)
		{
			for(int j = 0;j<512;j++)
			{
				map.MapPoint[j][i] = 0;
			}
			
		}
	for(int i = 200; i<300; i++)
		map.MapPoint[i][left]=13;
	for(int i = 200; i<300; i++)
		map.MapPoint[i][right]=12;

	for(int i=-4;i<5;i++)
		for(int j=-4;j<5;j++)
	map.MapPoint[255+i][255+j]=8;

}
bool CVehicleExe::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int &ob_y,int ob_width)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();

	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);
		if(m_gps.x==GPSpoint[i].x&&m_gps.y==GPSpoint[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}
	if(Rndf_MapPoint[199].x == 256 && Rndf_MapPoint[199].y == 411)
	{
		return false;
	}
	int m_up = 412-up*5;
	int m_down = 411 -down*5;

	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=-ob_width;n<ob_width;n++)
			{
				if(y+m>380&&y+m<430)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8)
				{
					ob_x = x+n;
					ob_y = y+m;
					vehicleob_x = ob_x;
					vehicleob_y = ob_y;
					return true;
			
				}
			}
	}
	return false;
}
void CVehicleExe::WaitForSignal()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int sign[4] = {0};
	if(LeadPoint[seq_num-2].param2 > 0)
	{
		
		Stop();//待完善 注意
		Sleep(2000);
		app->Get_Sign = true;
		Sleep(500);
		while(1)
		{	
			//fx 1.直行 3。右转 2。左传 
			//sign 的顺序
			//sign[4] = {0,0,0,0};{sign[0]大饼，sign[1]右，sign[2]直，sign[3]左}
			for(int i =0;i<4;i++)
			{
				sign[i] = 0;
			}
			app->Get_Sign = true;
			GetSignal(sign);
			outn1<<"fx  "<<fx<<"  sign  "<<sign[0]<<sign[1]<<sign[2]<<sign[3]<<endl;
			if(sign[1] == 0 &&sign[2] == 0 &&sign[3] == 0 )
			{	
				if(sign[0] != 2)
					break;
			}
			else if(fx == 1)//直行
			{
			
				if(sign[2] != 2)
				break;
				Sleep(20);
			}
			else if(fx == 2)//左转
			{
			
				if(sign[3] != 2)
				break;
				Sleep(20);
			}
			else if(fx == 3)//右转`
			{
			
				if(sign[1] != 2)
				break;
				Sleep(20);
			}
			else
				break;
		}
	}
}
int CVehicleExe::WaitForSignal1()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int sign[4] = {0};
	//if(LeadPoint[seq_num-2].param2 > 0)
	{
		
		while(1)
		{	
			//fx 1.直行 2。右转 3。左传 
			//sign 的顺序
			//sign[4] = {0,0,0,0};{sign[0]大饼，sign[1]右，sign[2]直，sign[3]左}
			for(int i =0;i<4;i++)
			{
				sign[i] = 0;
			}
			//app->Get_Sign = true;
			GetSignal(sign);
			
			if(fx == 1)//直行
			{
				if(sign[0]>sign[2])
					return sign[0];
				else
					return sign[2];
			}
			else if(fx == 3 || fx == 4)//左转
			{
				if(sign[0]>sign[3])
					return sign[0];
				else
					return sign[3];
			
				//return sign[3];
			}
			else if(fx == 2)//右转`
			{
			
				return sign[1];
			}
			else
				return 0;
		}
	}
}
bool CVehicleExe::GetSignal(int (&sign)[4])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int GREEN[4]={0};
	int RED[4]={0};

	//5/4/3/2/1->STOP/NOTLEFT/NOTRIGHT/LEFT/RIGHT
	int time = 500;
	while(1)
	{
		if(app->Traffic_Sign.size()>=7)
			break;
		Sleep(5);
		time--;
		if(time == 0)
			return false;
	}
	app->critical_light.Lock();
	for(int i = 0; i<7; i++)
	{
		int temp = app->Traffic_Sign.front();
		outn1<<temp<<endl;
		int temp0 = temp;
		for(int j = 0; j<4; j++)
		{
			int temp0 = temp%10;
			if(temp0 == 1)
				GREEN[j]++;
			else if(temp0 == 2)
				RED[j]++;
			temp=temp/10;
		}
		app->Traffic_Sign.pop();
	}
	app->critical_light.Unlock();
	int count = 0;

	for(int j = 0;j<4;j++)
	{
		if(GREEN[j]>=2)
		{
			sign[j]=1;

		}
		if(RED[j]>=2)
		{
			sign[j]=2;
			//sign = j;
		}

	}

	//app->stop_sign = true;
	//sign = 1,2,3 //1左转，2直行，3右转。
	//app->Get_Sign = false;
	app->critical_light.Lock();
	int sign_size = app->Traffic_Sign.size();
	for(int i = 0; i < sign_size;i++)
	{
		app->Traffic_Sign.pop();
	}
	app->critical_light.Unlock();
	return true;


}
void CVehicleExe::SpeedDecision()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	if(vel_Map->MapPoint[0][0] >20 )
		app->speed_status = 21;
	/*else if(vel_Map->MapPoint[0][0] <20&&(vel_Map->MapPoint[0][0] >10))
		app->speed_status = 18;*/
	else
		app->speed_status = 11;
}
void CVehicleExe::SetSpeedInitState(double temp)
{
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		app->drive_state = temp/3.6;
		app->drive_curvature = temp/3.6;
		app->drive_obstacle = temp/3.6;
}
bool CVehicleExe::getAimPoint(int num,CvPoint2D64f (&MidPoint)[512],CvPoint2D64f &Point)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f a;
	double s = 0;
	double x = (MidPoint[num-1].x - 256)/5;
	double y = (MidPoint[num-1].y - 411)/5;
	s = sqrt(x*x+y*y);
	if(s<40)
	{
		Point = MidPoint[num-1];
		return 0;
	}
	//app->critical_planningroad.Lock();
	
	for(int i = 0;i<num;i++)
	{
		s = sqrt(pow(((MidPoint[i].x - 256)*0.2),2)+pow(((MidPoint[i].y - 412)*0.2),2));
		if(s > 40)
		{
			Point = MidPoint[i];
			return 1;
		}
		
	}
	//app->critical_planningroad.Unlock();
}
double CVehicleExe::getAimDir(int num,CvPoint2D64f *MidPoint1,double dir)
{
	double aim_dir = 0;
	double k = 0;
	double angle = 0;
	k = abs(Countk(MidPoint1,num));
	angle = atan(k)*180/3.1415926;
	aim_dir = dir + angle - 90;
	return aim_dir;
}





/********************以下为全局路径规划***********************/


void CVehicleExe::solvewaypointnum()//求rndf文件中所有路点数
{
	waypointnum = 0;
	//astar.rndf=RNDFGPS;

	for(int segnum=0;segnum<RNDFGPS.m_mapInfo.segment_num;segnum++)
	{
		for(int lanenum=0;lanenum<RNDFGPS.m_mapInfo.pSegment[segnum].lane_num;lanenum++)
		{
			waypointnum+=RNDFGPS.m_mapInfo.pSegment[segnum].pLane[lanenum].waypoints_num;
		}
	}
}
MAPPOINT* CVehicleExe::solvepoint(double x,double y)//根据点的GPS坐标求离其最近的路点
{
	//astar.rndf=RNDFGPS;
	double distance;
	double mindis = I;
	MAPPOINT *nearpoint = NULL;
	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)
	{
		for(int j=0;j<RNDFGPS.m_mapInfo.pSegment[i].lane_num;j++)
		{
			for(int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				distance = sqrt(pow((x - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x)),2) + pow((y - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y)),2));
				if(distance<mindis)
				{
					mindis = distance;
					nearpoint = &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
				}
			}
		}
	}

	double julicha=I;
	if(nearpoint!=NULL)
	{
		julicha = m_GpsData.GetDistance(x,y,nearpoint->x,nearpoint->y);
	}
	if(julicha<200)
	{
		return nearpoint;//当求得的最近点与该点距离差在200米以内时，返回求得的最近点，否则返回NULL
	}
	else
	{
		CString str="路网中找不到合适的邻近路点";
		AfxMessageBox(str);
		return NULL;
	}
}

MAPPOINT* CVehicleExe::solvepoint2(double x,double y,double dir)//根据无人车当前GPS坐标及航向求在本车道内离其最近的路点
{
	double distance;
	double direction;//车子与路点连线的方向
	double direction1;//车道的方向
	double angle;//车道方向与车航向的夹角
	double angle1;//车道方向与(车子和路点连线)的方向的夹角
	//int lanepointnum;
	double mindis = I;
	MAPPOINT *nearpoint = NULL;
	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)
	{
		for(int j=0;j<RNDFGPS.m_mapInfo.pSegment[i].lane_num;j++)
		{
			//lanepointnum=RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;
			for(int k=1;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				distance = sqrt(pow((x - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x)),2) + pow((y - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y)),2));	
				direction = m_GpsData.GetAngle(x,y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y);
				direction1 = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y);
				angle=fabs(direction1-dir);
				angle1=fabs(direction1-direction);
				if((distance<mindis)&&(angle<30)&&(angle1<30))//实际上是满足1、车的航向；2、车道的方向；3、车子与点连线的方向这三者相吻合
				{
					mindis = distance;
					nearpoint = &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
				}
			}
		}
	}
	double julicha=I;
	if(nearpoint!=NULL)
	{
		julicha = m_GpsData.GetDistance(x,y,nearpoint->x,nearpoint->y);
	}
	if(julicha<200)
	{
		return nearpoint;//当求得的最近点与该点距离差在200米以内时，返回求得的最近点，否则返回NULL
	}
	else
	{
		CString str="路网中找不到合适的邻近路点";
		AfxMessageBox(str);
		return NULL;
	}
}

MAPPOINTexe* CVehicleExe::solvepoint3(double x,double y,double dir,MAPPOINTexe* path11)//根据无人车当前GPS坐标及航向求在本车道内离其最近的路点
{
	double distance;
	double direction;//车子与路点连线的方向
	double direction1;//车道的方向
	double angle;//车道方向与车航向的夹角
	double angleex;
	double angle1;//车道方向与(车子和路点连线)的方向的夹角
	double angle1ex;
	//int lanepointnum;
	double mindis = I;
	double maxdis = 999999999;
	MAPPOINTexe *nearpoint = NULL;
	MAPPOINTexe* path22;
	MAPPOINTexe* maxpoint;
	CvPoint2D64f pathnextpoint;
	for(path22 = path11->next;path22!=NULL;path22 = path22->next)
	{
		solveID(path22);
		if(ID3==1)
		{
			pathnextpoint = RNDFGPS.GetPoint(ID1,ID2,2);
			direction1 = m_GpsData.GetAngle(pathnextpoint.x,pathnextpoint.y,path22->x,path22->y);
			if (direction1<0)
			{
				direction1 = direction1+360;
			}
		}
		else
		{
			pathnextpoint = RNDFGPS.GetPoint(ID1,ID2,ID3-1);
			direction1 = m_GpsData.GetAngle(path22->x,path22->y,pathnextpoint.x,pathnextpoint.y);
			if (direction1<0)
			{
				direction1 = direction1+360;
			}
		}
				distance = sqrt(pow((x - (path22->x)),2) + pow((y - (path22->y)),2));	
				direction = m_GpsData.GetAngle(path22->x,path22->y,x,y);
				if(direction<0)
				{
					direction = direction + 360;
				}
				//direction1 = m_GpsData.GetAngle(path22->next->x,path22->next->y,path22->x,path22->y);
				if(dir>=360)
				{
					dir = dir-360;
				}
				angle=fabs(direction1-dir);
				angleex = 360 - angle;

				angle1=fabs(direction1-direction);
				angle1ex = 360 - angle1;
				////////
				solveID(path22);
				ID1,ID2,ID3;
				/////////
				if((distance<mindis)&&((angle</*30*/30)||(angleex</*30*/50))&&((angle1<90)||(angle1ex<90)))//实际上是满足1、车的航向；2、车道的方向；3、车子与点连线的方向这三者相吻合
				{
					mindis = distance;
					nearpoint = path22;
				}
	}
	double julicha=I;
	if(nearpoint!=NULL)
	{
		julicha = m_GpsData.GetDistance(x,y,nearpoint->x,nearpoint->y);
	}
	if(julicha<500)
	{
		solveID(nearpoint);
		solvenearpointnum++;
		//outn7777<<"第"<<solvenearpointnum<<"次求最近的点结果为："<<"\t"<<ID1<<"."<<ID2<<"."<<ID3<<endl;
		return nearpoint;//当求得的最近点与该点距离差在200米以内时，返回求得的最近点，否则返回NULL
	}
	else
	{
		for(path22 = path11->next;path22!=NULL;path22 = path22->next)
		{
			distance = sqrt(pow((x - (path22->x)),2) + pow((y - (path22->y)),2));	
			if(distance < maxdis)
			{
				maxdis = distance;
				maxpoint = path22;
			
				//solveID(maxpoint);
			}
		}
		//solvenearpointnum++;
		////outn7777<<"第"<<solvenearpointnum<<"次求最近的点结果为："<<"\t"<<"没有找到合适的邻近点"<<endl;
		//CString str="路网中找不到合适的邻近路点";
		//AfxMessageBox(str);
		//return NULL;
	}
	solveID(maxpoint);
	return maxpoint;
}


void  CVehicleExe::point2point()//生成所有点的连通性
{
	//astar.rndf=RNDFGPS;

	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)//初始化所有路点
	{
		for(int j=0;j<RNDFGPS.m_mapInfo.pSegment[i].lane_num;j++)
		{
			for(int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[0] = NULL;
				RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[1] = NULL;
				RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[2] = NULL;
				RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[3] = NULL;
			}
		}
	}

	for(int num=0;num<RNDFGPS.m_mapInfo.segment_num;num++)
	{
		int waypointNUM;
		int idid,mm,nn,pp;
		if(RNDFGPS.m_mapInfo.pSegment[num].direction_num==1)
		{
			waypointNUM=RNDFGPS.m_mapInfo.pSegment[num].pLane[0].waypoints_num;
			for (int t=0;t<waypointNUM-1;t++)
			{
				RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[t].nearnode[0]=&RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[t+1];
			}

			int j=RNDFGPS.m_mapInfo.pSegment[num].pLane[0].exit_num;

			if(j!=0)
			{  
				for(int k=0;k<j;k++)
				{
					idid = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].exit_Id;
					mm = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].m;
					nn = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].n;
					pp = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].p;

					RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[idid-1].nearnode[k]=&RNDFGPS.m_mapInfo.pSegment[mm-1].pLane[nn-1].pPoint[pp-1];
				}
			}
		}

		else if(RNDFGPS.m_mapInfo.pSegment[num].direction_num==2)
		{
			waypointNUM=RNDFGPS.m_mapInfo.pSegment[num].pLane[0].waypoints_num;
			for (int t=0;t<waypointNUM-1;t++)
			{
				RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[t].nearnode[0]=&RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[t+1];
			}

			int j=RNDFGPS.m_mapInfo.pSegment[num].pLane[0].exit_num;

			if(j!=0)
			{  
				for(int k=0;k<j;k++)
				{
					idid = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].exit_Id;
					mm = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].m;
					nn = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].n;
					pp = RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pExit[k].p;

					RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[idid-1].nearnode[k]=&RNDFGPS.m_mapInfo.pSegment[mm-1].pLane[nn-1].pPoint[pp-1];
				}
			}
			RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[waypointNUM-1].nearnode[3]=&RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pPoint[0];

			waypointNUM=RNDFGPS.m_mapInfo.pSegment[num].pLane[1].waypoints_num;
			for (int t=0;t<waypointNUM-1;t++)
			{
				RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pPoint[t].nearnode[0]=&RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pPoint[t+1];
			}

			j=RNDFGPS.m_mapInfo.pSegment[num].pLane[1].exit_num;

			if(j!=0)
			{  
				for(int k=0;k<j;k++)
				{
					idid = RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pExit[k].exit_Id;
					mm = RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pExit[k].m;
					nn = RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pExit[k].n;
					pp = RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pExit[k].p;

					RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pPoint[idid-1].nearnode[k]=&RNDFGPS.m_mapInfo.pSegment[mm-1].pLane[nn-1].pPoint[pp-1];
				}
			}
			RNDFGPS.m_mapInfo.pSegment[num].pLane[1].pPoint[waypointNUM-1].nearnode[3]=&RNDFGPS.m_mapInfo.pSegment[num].pLane[0].pPoint[0];
		}
	}
	//RNDFGPS.m_mapInfo.pSegment[0].pLane[0].pPoint[0];
	//RNDFGPS.m_mapInfo.pSegment[1].pLane[0].pPoint[0];
}


void CVehicleExe::planning(int m1,int n1,int p1,int m2,int n2,int p2)//m1,n1,p1为起始点的编号，m2,n2,p2为目标点的编号
{
	//astar.rndf=RNDFGPS;

	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)//初始化所有路点
	{
		for(int j=0;j<RNDFGPS.m_mapInfo.pSegment[i].lane_num;j++)
		{
			for(int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].g = I;
				RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].next = NULL;
				RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].par = NULL;
			}
		}
	}

	struct MAPPOINT *node,*best;
	float g_value;
	astar.open=(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT ));  	
	astar.close=(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT )); 
	int existop0,existop1,existop2,existop3,existcl0,existcl1,existcl2,existcl3;

	start_point =&RNDFGPS.m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1];
	end_point = &RNDFGPS.m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1];

	node = start_point;
	astar.open->next = node;
	node->g = 0;


	while((node->x !=end_point->x) || (node->y != end_point->y))
	{
		existop0 = astar.existopen(astar.open,node->nearnode[0]);
		existop1 = astar.existopen(astar.open,node->nearnode[1]);
		existop2 = astar.existopen(astar.open,node->nearnode[2]);
		existop3 = astar.existopen(astar.open,node->nearnode[3]);
		existcl0 = astar.existclose(astar.close,node->nearnode[0]);
		existcl1 = astar.existclose(astar.close,node->nearnode[1]);
		existcl2 = astar.existclose(astar.close,node->nearnode[2]);
		existcl3 = astar.existclose(astar.close,node->nearnode[3]);

		if((node->nearnode[0]!=NULL)&&(existop0 == 0)&&(existcl0 == 0))
		{astar.addopen(node->nearnode[0]);}
		if((node->nearnode[1]!=NULL)&&(existop1 == 0)&&(existcl1 == 0))
		{astar.addopen(node->nearnode[1]);}
		if((node->nearnode[2]!=NULL)&&(existop2 == 0)&&(existcl2 == 0))
		{astar.addopen(node->nearnode[2]);}
		if((node->nearnode[3]!=NULL)&&(existop3 == 0)&&(existcl3 == 0))
		{astar.addopen(node->nearnode[3]);}

		astar.deleteopen(node);
		astar.addclose(node);

		if((astar.open->next==NULL))
		{
			//无法找到最短路径
			/*CString str="无法找到最短路径，请重新设置起始目标点";
			AfxMessageBox(str);*/
			return;
		}
		node->d0 = astar.solvedistance(*node,node->nearnode[0]);
		node->d1 = astar.solvedistance(*node,node->nearnode[1]);
		node->d2 = astar.solvedistance(*node,node->nearnode[2]);
		node->d3 = astar.solvedistance(*node,node->nearnode[3]);

		if((node->nearnode[0]!=NULL) && (!existcl0))
		{
			g_value = node->g + node->d0;
			if ((g_value) < (node->nearnode[0]->g))
			{
				node->nearnode[0]->g = g_value;
				node->nearnode[0]->par = node;
			}
		}
		if((node->nearnode[1]!=NULL) && (!existcl1))
		{
			g_value = node->g + node->d1;
			if ((g_value) < (node->nearnode[1]->g))
			{
				node->nearnode[1]->g = g_value;
				node->nearnode[1]->par = node;
			}
		}
		if((node->nearnode[2]!=NULL) && (!existcl2))
		{
			g_value = node->g + node->d2;
			if ((g_value) < (node->nearnode[2]->g))
			{
				node->nearnode[2]->g = g_value;
				node->nearnode[2]->par = node;
			}
		}
		if((node->nearnode[3]!=NULL) && (!existcl3))
		{
			g_value = node->g + node->d3;
			if ((g_value) < (node->nearnode[3]->g))
			{
				node->nearnode[3]->g = g_value;
				node->nearnode[3]->par = node;
			}
		}


		if((node->nearnode[0]!=NULL) && (!existcl0))
		{
			node->nearnode[0]->h = astar.solveheuristic(node->nearnode[0],*end_point);
		}
		if((node->nearnode[1]!=NULL) && (!existcl1))
		{
			node->nearnode[1]->h = astar.solveheuristic(node->nearnode[1],*end_point);
		}
		if((node->nearnode[2]!=NULL) && (!existcl2))
		{
			node->nearnode[2]->h = astar.solveheuristic(node->nearnode[2],*end_point);
		}
		if((node->nearnode[3]!=NULL) && (!existcl3))
		{
			node->nearnode[3]->h = astar.solveheuristic(node->nearnode[3],*end_point);
		}
		if((node->nearnode[0]!=NULL) && (!existcl0))
		{
			node->nearnode[0]->f = node->nearnode[0]->g + node->nearnode[0]->h;
		}
		if((node->nearnode[1]!=NULL) && (!existcl1))
		{
			node->nearnode[1]->f = node->nearnode[1]->g + node->nearnode[1]->h;
		}
		if((node->nearnode[2]!=NULL) && (!existcl2))
		{
			node->nearnode[2]->f = node->nearnode[2]->g + node->nearnode[2]->h;
		}
		if((node->nearnode[3]!=NULL) && (!existcl3))
		{
			node->nearnode[3]->f = node->nearnode[3]->g + node->nearnode[3]->h;
		}
		astar.Sort();//根据f值大小对open表中所有的点进行排序
		best = astar.SearchBest();//取f值最小的点
		node = best;
	}

	struct MAPPOINT *pathpoint =end_point;
	path =(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT ));

	while(pathpoint != start_point)
	{
		pathpoint->next = path->next;
		path->next = pathpoint;
		pathpoint = pathpoint->par;
	}
	pathpoint->next = path->next;
	path->next = pathpoint;

	pathpointnum = 0;
	struct MAPPOINT *copypath=(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT ));
	copypath->next = path->next;
	for(;copypath->next!=NULL;)
	{
		pathpointnum++;
		copypath = copypath->next;
	}
}


/********************以上为全局路径规划*********************/
double CVehicleExe::Kalman(double observe,double his,double &optimal )
{
	double shiji = 0;
	double show_uncertan = 10;
	double pre_uncertan=0.5;
	double guass=sqrt((optimal*optimal)+pre_uncertan*pre_uncertan);
	double kg=sqrt(guass*guass/(guass*guass+show_uncertan*show_uncertan));
	shiji=his+kg*(observe-his);
	optimal = sqrt((1-kg)*guass*guass);
	return shiji;
}
void CVehicleExe::solveID(MAPPOINT* thepoint)//求路点在路网中的编号
{
	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)
	{
		for (int j=0;j<RNDFGPS.m_mapInfo.pSegment[i].lane_num;j++)
		{
			for (int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				if (thepoint == &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k])
				{
					ID1=i+1;
					ID2=j+1;
					ID3=k+1;
					return;
				}
			}
		}
	}
}
void CVehicleExe::solveID(MAPPOINTexe* thepoint)//求路点在路网中的编号
{
	ID1 = thepoint->ID111;
	ID2 = thepoint->ID222;
	ID3 = thepoint->ID333;
}

void CVehicleExe::solveID(CvPoint2D64f thepoint)//求路点在路网中的编号
{
	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)
	{
		for (int j=0;j<RNDFGPS.m_mapInfo.pSegment[i].lane_num;j++)
		{
			for (int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				if (thepoint.x == RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x&&thepoint.y == RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y)
				{
					ID1=i+1;
					ID2=j+1;
					ID3=k+1;
					return;
				}
			}
		}
	}
}

void CVehicleExe::GetPathString(CvPoint2D64f* path,CString &str,int num,int width,int lane_num)
{
	str = "";
	CString str1,str2,str3;
	for (int i = 0;i<num;i++)
	{	
		str1.Format("%f",path[i].x);
		str2.Format("%f",path[i].y);
		str += str1+','+str2+',';
		
		str3.Format("%d,%d;",width,lane_num);
		str += str3;
	
	}
}
void CVehicleExe::StartPath()
{
	m_hThreadPath = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::thePathThread,
		this, 0, &dwPathThreadId);
}

DWORD CVehicleExe::thePathThread(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->PathThread());
}




//综合运行流程
DWORD CVehicleExe::PathThread()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_dir = app->GPS_Direction;
	int aaaa;

	int ob_num = 0;
	int ob_num2 = 0;

	bool appstop = false;
	int appstop_cnt = 0;
	CvPoint2D64f globalendpoint;
	int n = app->gpsmap_road->total;
	double driveobstacletmp;
	CvPoint2D64f Rndf_MapPoint[200];
	int x;
	int y;
	
	
	// 以下的变量做何用？
	double obabdis;
	double tempobabdis;
	double obdis;
	double tempobdis;
	double obfrontdis;
	double tempdisob;
	int bb;
	double obstablespdaim;
	double speedreducedist_;
	double frontobdist;
	
	fasterobspdini_path=road_speed;
	bfasterobacv_path=0;
	bobfisrstapper_path=0;
	ob_dis_now_path=1000;
	ob_dis_pre_path=1000;
	ob_dis_prepre_path=1000;
	ob_dis_now_path_timer=0;
	ob_dis_pre_path_timer=0;
	ob_dis_prepre_path_timer=0;
	ob_loss_count_path=0;
	bool byulukouzhixing=false;
	
	while(1)
	{

		int iRet = WaitForSingleObject(m_pathevent,150);  // 只有决策部分完成后，PathThread才继续往下执行
		if(iRet != 0)
			continue;
		SYSTEMTIME t1;
		GetLocalTime(&t1);
		outpathtime<<"time1 "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  "<<endl;
		m_gps = app->GPS_Point;
		m_dir = app->GPS_Direction;

		// 当前 gps 坐标与 gps_aimpoint_new 距离小于 2 时，停止车辆
		double goaldis = m_GpsData.GetDistance(m_gps.x,m_gps.y,gps_aimpoint_new[0].x,gps_aimpoint_new[0].y);
		if(goaldis<2)
		{
			Stop();
			continue;
		}

		//限速
		app->drive_obstacle2 = road_speed;

		if(m_GpsData.GetDistance(31.32436573513,121.23512638759,m_gps.x,m_gps.y)<50)  // 31.3243, 121.235这个具体的GPS坐标是干什么的？应该可以删除
		{
			if(app->drive_obstacle2 > 15/3.6)
				app->drive_obstacle2 = 15/3.6;
		}


		switch( m_task )
		{
			case IVTASK_LANE:                            
			
			// 任务： 路上和预路口
			case IVTASK_APPROINTERSECTION:           

				app->bzheng=(abs(gps_aimdir - m_dir) < 6 || abs(360 - abs(gps_aimdir - m_dir)) < 6);

				if(able_ChangeLane)
					speedreducedist_ = app->GPS_Speed*3.6+12;
				else
					speedreducedist_ = (app->GPS_Speed)*(app->GPS_Speed)/8+6;
				
				// 限速
				if(speedreducedist_<15)
					speedreducedist_=15;
				else if(speedreducedist_>80)
					speedreducedist_=80;

				// 限制前车距离8米
				if(app->GPS_Speed*3.6<10)
					frontobdist=8;
				else if(app->GPS_Speed*3.6>60)
					frontobdist=8;
				else
					frontobdist=8;
				if(app->bdirectrunreq)
					frontobdist=8;
				else
					frontobdist=6;
				if(app->left_right!=0)
					frontobdist=6;
				if(bcurveactive)
					frontobdist=6;

				//------------------------------------------------------------------------------------------------------------------------
				// 这里为了比赛做优化？
				if((m_task==IVTASK_APPROINTERSECTION)&&(!(approachgoal_flag||approachyulukou>0 || bspecialraozhangyulukou || (bguzhangcheliang==1) || (!able_ChangeLane))))
					byulukouzhixing=true;
				else
					byulukouzhixing=false;
				
				byulukouzhixing=false;
				
				//if((m_task==IVTASK_APPROINTERSECTION)&&(lukou_fx==3||lukou_fx==4||lukou_fx==8)&&(approachyulukou>0))
				{
					// inum 是什么？
					for(int inum=0;inum<3;inum++)
					{
						pathplan.bpathfound=false;

						pathplan.his_ap_tmp.x=0;
						pathplan.his_ap_tmp.y=0;

						// 难绕障/预路口且...的时候用 keeplanewan，普通情况用 keeplane
						if(((m_task==IVTASK_APPROINTERSECTION) && (lukou_fx==4 || bspecialraozhangyulukou || (bguzhangcheliang==1))) || bhardraozhang)
							aaaa = pathplan.KeepLanewan(m_gps,m_dir, gps_aimpoint_new[inum], gps_aimdir_new[inum], MidGpsPoint, 0);
						else
							aaaa = pathplan.KeepLane(m_gps,m_dir,gps_aimpoint_new[inum],gps_aimdir_new[inum],MidGpsPoint,byulukouzhixing); // byulukouzhixing = false

						// 这个判断的目的是？
						if(app->GPS_Speed > 6/3.6 || pathplan.bpathfound || inum==2 || (gps_aimpoint_new[inum+1].x==0 || gps_aimpoint_new[inum+1].y==0))
						{
							if((pathplan.his_ap_tmp.x!=0)||(pathplan.his_ap_tmp.y!=0))
							{
								pathplan.his_ap.x=pathplan.his_ap_tmp.x;
								pathplan.his_ap.y=pathplan.his_ap_tmp.y;
							}
							gps_aimpoint=gps_aimpoint_new[inum];
							gps_aimdir=gps_aimdir_new[inum];
							aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
							break;
						}
					}
				}

				if(aaaa==-1)
				{
					app->drive_obstacle2 = 0;
					app->continue_spdred = true;
					app->continue_brake = true;
					double s_ob = 0;
					int x = 256;
					int y = 411;
					int path_width = 4;

					bool flag=0;
					for(int m=-1; m>-speedreducedist_*5; m--)
					{
						for(int n=-path_width;n<=path_width;n++)
						{	
							if(y+m>411||y+m<0||x+n>511||x+n<0)
								continue;
							if(vel_Map->MapPoint[y+m][x+n] == 8 || vel_Map->MapPoint[y+m][x+n] == 18 || vel_Map->MapPoint[y+m][x+n] == 28)
							{
								s_ob = abs(m/5.0);
								flag=1;
								break;
							}
						}
						if(flag)
							break;
					}
					if(s_ob>7)
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(s_ob-6));
					else
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
					if(spdacctemp<app->spdredacc)
					{
						if(app->spdredacc<-3)
							spdacctemp=app->spdredacc;
						else if(spdacctemp<-3)
							spdacctemp=-3;
						app->spdredacc = spdacctemp;
					}
			
				}

				// 规划成功
				//if(aaaa==0 || aaaa==3 || aaaa==4)
				{
					// 插值，将200个点插值为1992个点
					int chadiannum=0;
					for(int i=0;i<199;i++)
					{
						for(int kk=0;kk<10;kk++)
						{
							rndfbezierchadian_path[chadiannum].x=(MidGpsPoint[i].x * (10-kk)+ MidGpsPoint[i+1].x * kk) * 0.1;
							rndfbezierchadian_path[chadiannum].y=(MidGpsPoint[i].y * (10-kk)+ MidGpsPoint[i+1].y * kk) * 0.1;
							chadiannum++;
						}
					}
					rndfbezierchadian_path[chadiannum]=MidGpsPoint[199];

					//ob_dis_prepre_path=ob_dis_pre_path;
					//ob_dis_pre_path=ob_dis_now_path;

					// 将1992个点中坐标与当前GPS坐标一致的点设置为栅格图的(256, 411)
					for(int i=0;i<1991;i++)
					{
						Rndf_MapPointchadian_path[i] = m_GpsData.APiontConverD(m_gps,rndfbezierchadian_path[i],m_dir);
						if(m_gps.x == rndfbezierchadian_path[i].x && m_gps.y == rndfbezierchadian_path[i].y)
						{
							Rndf_MapPointchadian_path[i].x = 256;
							Rndf_MapPointchadian_path[i].y = 411;
						}
					}
				
					x = 0;
					y = 0;

					// 中间距离值，为了控制车辆在某点的速度
					obabdis=100000;
					tempobabdis=100000;
					obdis=100000;
					tempobdis=100000;
					obfrontdis=100000;
					tempdisob=100000;

					// 根据 keeplane 返回的不同结果设置障碍物的期望速度
					if(aaaa==0)
						obstablespdaim = 5/3.6;
					else if(aaaa==3)
						obstablespdaim = 12/3.6;
					else if(aaaa==4)
						obstablespdaim = 8/3.6;

					if(obstablespdaim > road_speed)
						obstablespdaim = road_speed;

					// 8，18，28是障碍物标志
					int a=8;
					int b=18;
					int c=28;

					double drivespdtmp = road_speed;

					bool obb_break_flag=0;


					// 是速度控制吗？
					for(int i = 0;i<1991;i++)
					{
						// 筛选，不在 x = (0~511), y=(187~411) 范围内的点都不要
						if(Rndf_MapPointchadian_path[i].x < 0|| Rndf_MapPointchadian_path[i].x > 511 
							||Rndf_MapPointchadian_path[i].y > 411 || Rndf_MapPointchadian_path[i].y < 187)
							continue;
						x = Rndf_MapPointchadian_path[i].x;
						y = Rndf_MapPointchadian_path[i].y;

						// for循环主要目的是？
						for(int m=4;m>=-4;m--)
						{
							for(int n=-4;n<=4;n++)
							{
								if(y+m>411 || y+m<0 || x+n>511 ||x+n<0)
									continue;
								// 8,18,28分别代表什么含义？
								if(vel_Map->MapPoint[y+m][x+n] == 8 || vel_Map->MapPoint[y+m][x+n] == 18 || vel_Map->MapPoint[y+m][x+n] == 28)
								{
									obb_break_flag=1;
									// ob_dis_now_path：障碍物在栅格上的距离
									ob_dis_now_path=412-(y+m);

									SYSTEMTIME tt;
									GetLocalTime(&tt);
									// 拿到这个时间有什么用？
									ob_dis_now_path_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

									// middle_dis: 从栅格转过来的在栅格上的距离
									double middle_dis = abs(412-(y+m))* 0.2; 

									if(ob_dis_pre_path<1000)
									{
										if(abs(ob_dis_now_path-ob_dis_pre_path)*0.2 > 30*(ob_dis_now_path_timer-ob_dis_pre_path_timer)*0.001)
										{
											ob_dis_pre_path=1000;
											ob_dis_prepre_path=1000;
											bfasterobacv_path=0;
										}
									}

									if(ob_dis_prepre_path<1000)
									{
										if(abs(ob_dis_now_path-ob_dis_prepre_path)*0.2>30*(ob_dis_now_path_timer-ob_dis_prepre_path_timer)*0.001)
										{
											ob_dis_pre_path=1000;
											ob_dis_prepre_path=1000;
											bfasterobacv_path=0;
										}
									}

									if(ob_dis_pre_path==1000 && ob_dis_prepre_path==1000)
										bobfisrstapper_path=1;
									else if(bobfisrstapper_path)
									{
										if(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path>=0)
											bobfisrstapper_path=1;
										else if(ob_dis_prepre_path==1000 && ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path>=0)
											bobfisrstapper_path=1;
										else
											bobfisrstapper_path=0;
									}
									else
										bobfisrstapper_path=0;

									if((ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path>=1)||(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path>=1))
									{
										if(bfasterobacv_path==0)
										{
											bfasterobacv_path=1;
											fasterobspdini_path=app->GPS_Speed;
										}
									}

									if(bfasterobacv_path &&((abs(412-(y+m))*0.2)>15) && ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path>=1 && ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path>=3)
									{
										if(fasterobspdini_path<app->GPS_Speed+3.5/3.6)
											fasterobspdini_path=app->GPS_Speed+3.5/3.6;
										if(fasterobspdini_path>road_speed)
											fasterobspdini_path=road_speed;
									}

									double obfrontspd=dynamicmap_v_x->MapPoint[y+m][x+n];

									if(ob_dis_now_path<75)
									{
										if((ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path<0)||(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path<0))
										{
											if(bfasterobacv_path==1)
											{
												bfasterobacv_path=0;
											}
										}
									}
									else
									{
										if((ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path<-1)||(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path<-1))
										{
											if(bfasterobacv_path==1)
											{
												bfasterobacv_path=0;
											}
										}
									}

									if(ob_dis_prepre_path<1000)
									{
										if(ob_dis_now_path_timer>ob_dis_prepre_path_timer)
											obfrontspd=(ob_dis_now_path-ob_dis_prepre_path-1)*0.2/(0.01*(ob_dis_now_path_timer-ob_dis_prepre_path_timer));
										else
											obfrontspd=(ob_dis_now_path-ob_dis_prepre_path-1)*0.2/(0.01);
									}

									obabdis = sqrt(double((256-x-n)*(256-x-n)+(411-y-m)*(411-y-m)))/5;
									if(obabdis<9)
										obabdis=(double(411-y-m))/5.0;
									if((vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b)&&(!(middle_dis>8 && bobfisrstapper_path)))
									{
										if(obabdis<speedreducedist_)
										{
											drivespdtmp = (obabdis-8)/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
											else if(drivespdtmp>road_speed)
												drivespdtmp=road_speed;
											if(obabdis>stopmiddledis&&drivespdtmp<5/3.6)
												drivespdtmp=5/3.6;

											if(app->GPS_Speed>obstablespdaim)
											{
												if(drivespdtmp>app->GPS_Speed)
													drivespdtmp = app->GPS_Speed;
											}
											else if(drivespdtmp>obstablespdaim)
												drivespdtmp = obstablespdaim;
											if(bfasterobacv_path&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini_path-2/3.6)
													drivespdtmp=fasterobspdini_path-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(drivespdtmp<app->drive_obstacle2)
												app->drive_obstacle2=drivespdtmp;
										}
										if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv_path&&middle_dis>8))
											{
												if(((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(obabdis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && obabdis<speedreducedist_))
												{
													app->continue_spdred = true;
													app->continue_brake = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-8));
													if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}

													if(spdacctemp<app->spdredacc && spdacctemp<-2)
													{
														if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
														else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
													}

													if(spdacctemp<app->spdredacc)
														app->spdredacc = spdacctemp;
												}
												else if((app->GPS_Speed>obstablespdaim + 2.5/3.6)||(obabdis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(obabdis<stopmiddledis))))
												{
													app->continue_spdred = true;
													app->continue_brake = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
													if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}
													if(bfasterobacv_path||bobfisrstapper_path)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}

													if(spdacctemp<app->spdredacc && spdacctemp<-2)
													{
														if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
														else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
													}

													if(spdacctemp<app->spdredacc)
														app->spdredacc = spdacctemp;
												}
											}
											else if(bfasterobacv_path&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
												{
													app->continue_spdred = true;
													if(spdacctemp<app->spdredacc)
														app->spdredacc = spdacctemp;
												}
											}
										}
									}
									if((vel_Map->MapPoint[y+m][x+n] == c)&&(!(middle_dis>8 && bobfisrstapper_path)))
									{
										if(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>237)
										{
											if(obabdis>40)
											{
												if(dynamicmap_v_x->MapPoint[y+m][x+n]>0)
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													spdacctemp=0;
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(!(bfasterobacv_path&&middle_dis>15)))
														{
															app->continue_spdred = true;
															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
												}
												else
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n];
														if(drivespdtmp>app->GPS_Speed)
															drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n])*(dynamicmap_v_x->MapPoint[y+m][x+n])/2/(obabdis-36);
													if(spdacctemp<-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-25))
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-25);
													if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12)))
														spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12));
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if(!(bfasterobacv_path&&middle_dis>8))
														{
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(dynamicmap_v_x->MapPoint[y+m][x+n]<-3/3.6)
																	app->continue_brake = true;
																if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
																{
																	double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																	if(spdaccdist>-2)
																		spdaccdist=-2;
																	if(spdacctemp<spdaccdist)
																		spdacctemp=spdaccdist;
																}
																if(spdacctemp<app->spdredacc)
																{
																	if(app->spdredacc<-3)
																		spdacctemp=app->spdredacc;
																	else if(spdacctemp<-3)
																		spdacctemp=-3;
																}
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
														else if(bfasterobacv_path&&middle_dis<15)
														{
															spdacctemp=0;
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
													}
												}
											}
											else if(obabdis>15)
											{
												if(dynamicmap_v_x->MapPoint[y+m][x+n]>10/3.6)
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													spdacctemp=0;
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(!(bfasterobacv_path&&middle_dis>15)))
														{
															app->continue_spdred = true;
															//app->continue_brake = true;
															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
												}
												else
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6;
														if(drivespdtmp>app->GPS_Speed)
															drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													if(obabdis>25)
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-20);
													else if(obabdis>17)
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/((obabdis-15)/2);
													else
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2;
													//spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-20);
													if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12)))
														spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12));
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if(!(bfasterobacv_path&&middle_dis>8))
														{
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(dynamicmap_v_x->MapPoint[y+m][x+n]<7/3.6)
																	app->continue_brake = true;
																if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
																{
																	double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																	if(spdaccdist>-2)
																		spdaccdist=-2;
																	if(spdacctemp<spdaccdist)
																		spdacctemp=spdaccdist;
																}
																if(spdacctemp<app->spdredacc)
																{
																	if(app->spdredacc<-3)
																		spdacctemp=app->spdredacc;
																	else if(spdacctemp<-3)
																		spdacctemp=-3;
																}
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
														else if(bfasterobacv_path&&middle_dis<15)
														{
															spdacctemp=0;
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
													}
												}
											}
											else
											{
												if(obabdis<speedreducedist_)
												{
													drivespdtmp = (obabdis-8)/3.6;
													if(drivespdtmp<0)
														drivespdtmp=0;
													else if(drivespdtmp>road_speed)
														drivespdtmp=road_speed;
													if(obabdis>stopmiddledis&&drivespdtmp<5/3.6)
														drivespdtmp=5/3.6;

													if(app->GPS_Speed>obstablespdaim)
													{
														if(drivespdtmp>app->GPS_Speed)
															drivespdtmp = app->GPS_Speed;
													}
													else if(drivespdtmp>obstablespdaim)
														drivespdtmp = obstablespdaim;
													if(bfasterobacv_path&&middle_dis>8)
													{
														if(drivespdtmp<fasterobspdini_path-2/3.6)
															drivespdtmp=fasterobspdini_path-2/3.6;
														if(drivespdtmp<0)
															drivespdtmp=0;
													}
													if(drivespdtmp<app->drive_obstacle2)
														app->drive_obstacle2=drivespdtmp;
												}
												if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
												{
													if(!(bfasterobacv_path&&middle_dis>8))
													{
														if(((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(obabdis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && obabdis<speedreducedist_))
														{
															app->continue_spdred = true;
															app->continue_brake = true;
															spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-8));
															if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
															{
																double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																if(spdaccdist>-2)
																	spdaccdist=-2;
																if(spdacctemp<spdaccdist)
																	spdacctemp=spdaccdist;
															}
															if(spdacctemp<app->spdredacc)
															{
																if(app->spdredacc<-3)
																	spdacctemp=app->spdredacc;
																else if(spdacctemp<-3)
																	spdacctemp=-3;
															}

															if(spdacctemp<app->spdredacc && spdacctemp<-2)
															{
																if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
																else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
															}

															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
														else if((app->GPS_Speed>obstablespdaim + 2.5/3.6)||(obabdis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(obabdis<stopmiddledis))))
														{
															app->continue_spdred = true;
															app->continue_brake = true;
															spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
															if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
															{
																double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																if(spdaccdist>-2)
																	spdaccdist=-2;
																if(spdacctemp<spdaccdist)
																	spdacctemp=spdaccdist;
															}
															if(spdacctemp<app->spdredacc)
															{
																if(app->spdredacc<-3)
																	spdacctemp=app->spdredacc;
																else if(spdacctemp<-3)
																	spdacctemp=-3;
															}
															if(bfasterobacv_path||bobfisrstapper_path)
															{
																if(spdacctemp<-3)
																	spdacctemp=-3;
															}

															if(spdacctemp<app->spdredacc && spdacctemp<-2)
															{
																if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
																else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
															}

															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
													else if(bfasterobacv_path&&middle_dis<15)
													{
														spdacctemp=0;
														if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
														{
															app->continue_spdred = true;
															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
												}
											}
										}
									}
									break;
								}
							}
							if(obb_break_flag)
								break;
						}
						if(obb_break_flag)
							break;
					}
					if(obb_break_flag==0)
					{
						ob_loss_count_path++;
						if(ob_loss_count_path>=5||ob_dis_now_path==1000)
						{
							ob_dis_now_path=1000;
							ob_dis_pre_path=1000;
							ob_dis_prepre_path=1000;
							ob_dis_now_path_timer=0;
							ob_dis_pre_path_timer=0;
							ob_dis_prepre_path_timer=0;
							bfasterobacv_path=0;
							ob_loss_count_path=0;
						}
					}
					else
					{
						ob_loss_count_path=0;
						ob_dis_prepre_path=ob_dis_pre_path;
						ob_dis_prepre_path_timer=ob_dis_pre_path_timer;
						ob_dis_pre_path=ob_dis_now_path;
						ob_dis_pre_path_timer=ob_dis_now_path_timer;
					}

					x = 256;
					y = 411;
					obfrontdis=pathplan.SearchFrontOb(vel_Map,frontobdist);
					

					if((abs(gps_aimdir - m_dir) < 6 || abs(360 - abs(gps_aimdir - m_dir)) < 6) || able_ChangeLane || app->GPS_Speed<22/3.6)
					{
						if((obfrontdis>0)&&(aaaa==0 || aaaa==3 || aaaa==4))
						{
							if(app->bnearstop||((m_task==IVTASK_APPROINTERSECTION)&&(lukou_fx==3||lukou_fx==4||lukou_fx==8)&&(approachyulukou>0)))
							{
								if(obfrontdis<5)
									driveobstacletmp = 0;
								else
									driveobstacletmp = 5/3.6;
								if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(obfrontdis>=7))
								{
									app->continue_spdred = true;
									spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obfrontdis-6));
								}
								else if(app->GPS_Speed>5/3.6 + 2.5/3.6)
								{
									app->continue_spdred = true;
									spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
								}
								if(spdacctemp<app->spdredacc)
								{
									if(app->spdredacc<-3)
										spdacctemp=app->spdredacc;
									else if(spdacctemp<-3)
										spdacctemp=-3;
									app->spdredacc = spdacctemp;
								}
								if(driveobstacletmp<app->drive_obstacle2)
									app->drive_obstacle2=driveobstacletmp;
								if(app->GPS_Speed>5/3.6 + 2.5/3.6)
									app->continue_brake = true;
							}
							else
							{
								if(obfrontdis<6)
									driveobstacletmp = 0;
								else
									driveobstacletmp = 5/3.6;
								if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(obfrontdis>=7))
								{
									app->continue_spdred = true;
									spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obfrontdis-6));
								}
								else if(app->GPS_Speed>5/3.6 + 2.5/3.6)
								{
									app->continue_spdred = true;
									spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
								}
								if(spdacctemp<app->spdredacc)
								{
									if(app->spdredacc<-3)
										spdacctemp=app->spdredacc;
									else if(spdacctemp<-3)
										spdacctemp=-3;
									app->spdredacc = spdacctemp;
								}
								if(driveobstacletmp<app->drive_obstacle2)
									app->drive_obstacle2=driveobstacletmp;
								if(app->GPS_Speed>5/3.6 + 2.5/3.6)
									app->continue_brake = true;
							}
						}
					}
				}
				break;

			case IVTASK_INTERSECTION:
				speedreducedist_ = app->GPS_Speed*3.6+12;
				if(speedreducedist_<15)
					speedreducedist_=15;
				else if(speedreducedist_>80)
					speedreducedist_=80;
				frontobdist=6;

				{
					for(int inum=0;inum<3;inum++)
					{
						pathplan.bpathfound=false;

						pathplan.his_ap_tmp.x=0;
						pathplan.his_ap_tmp.y=0;

						if(lukou_fx==1)
							aaaa = pathplan.KeepLane_lukou(m_gps,m_dir,gps_aimpoint_new[inum],gps_aimdir_new[inum],MidGpsPoint,lukou_fx);
						else if((m_GpsData.GetDistance(upoint[1].x,upoint[1].y,31.32764871490,121.23440379618)<100) && m_task==IVTASK_INTERSECTION && lukou_fx==4)//if(uturnlukoucnt==2 && m_task==IVTASK_INTERSECTION && lukou_fx==4)
							aaaa = pathplan.KeepLanewan(m_gps,m_dir,gps_aimpoint_new[inum],gps_aimdir_new[inum],MidGpsPoint,0);
						else
							aaaa = pathplan.KeepLane_lukouwan(m_gps,m_dir,gps_aimpoint_new[inum],gps_aimdir_new[inum],MidGpsPoint,lukou_fx);


						if(app->GPS_Speed > 6/3.6 || pathplan.bpathfound || inum==2 || (gps_aimpoint_new[inum+1].x==0 || gps_aimpoint_new[inum+1].y==0))
						{
							if((pathplan.his_ap_tmp.x!=0)||(pathplan.his_ap_tmp.y!=0))
							{
								pathplan.his_ap.x=pathplan.his_ap_tmp.x;
								pathplan.his_ap.y=pathplan.his_ap_tmp.y;
							}
							gps_aimpoint=gps_aimpoint_new[inum];
							gps_aimdir=gps_aimdir_new[inum];
							aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
							break;
						}
					}
				}

				if(aaaa==-1)
				{
					app->drive_obstacle2 = 0;
					app->continue_spdred = true;
					app->continue_brake = true;
					double s_ob = 0;
					int x = 256;
					int y = 411;
					int path_width = 4;
					bool flag=0;
					for(int m=0;m>-speedreducedist_*5;m--)
					{
						for(int n=-path_width;n<=path_width;n++)
						{	
							if(y+m>411||y+m<0||x+n>511||x+n<0)
								continue;
							if(vel_Map->MapPoint[y+m][x+n] == 8 || vel_Map->MapPoint[y+m][x+n] == 18 || vel_Map->MapPoint[y+m][x+n] == 28)
							{
								s_ob = abs(m/5.0);
								flag=1;
								break;
							}
						}
						if(flag)
							break;
					}
					if(s_ob>7)
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(s_ob-6));
					else
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
					if(spdacctemp<app->spdredacc)
					{
						if(app->spdredacc<-3)
							spdacctemp=app->spdredacc;
						else if(spdacctemp<-3)
							spdacctemp=-3;
						app->spdredacc = spdacctemp;
					}
				}
				

				if(lukou_fx==1)
				{
					int chadiannum=0;
					for(int i=0;i<199;i++)
					{
						for(int kk=0;kk<10;kk++)
						{
							rndfbezierchadian_path[chadiannum].x=(MidGpsPoint[i].x*(10-kk)+MidGpsPoint[i+1].x*kk)*0.1;
							rndfbezierchadian_path[chadiannum].y=(MidGpsPoint[i].y*(10-kk)+MidGpsPoint[i+1].y*kk)*0.1;
							chadiannum++;
						}
					}
					rndfbezierchadian_path[chadiannum]=MidGpsPoint[199];

					for(int i=0;i<1991;i++)
					{
						Rndf_MapPointchadian_path[i] = m_GpsData.APiontConverD(m_gps,rndfbezierchadian_path[i],m_dir);
						if(m_gps.x==rndfbezierchadian_path[i].x&&m_gps.y==rndfbezierchadian_path[i].y)
						{
							Rndf_MapPointchadian_path[i].x = 256;
							Rndf_MapPointchadian_path[i].y = 411;
						}
					}
				
					x = 0;
					y = 0;
					obabdis=100000;
					tempobabdis=100000;
					obdis=100000;
					tempobdis=100000;
					obfrontdis=100000;
					tempdisob=100000;

					if(aaaa==0)
						obstablespdaim = 5/3.6;
					else if(aaaa==3)
						obstablespdaim = 12/3.6;
					else if(aaaa==4)
						obstablespdaim = 8/3.6;

					if(obstablespdaim>road_speed)
						obstablespdaim = road_speed;

					int a=8;
					int b=18;
					int c=28;

					double drivespdtmp = road_speed;

					bool obb_break_flag=0;


					for(int i = 0;i<1991;i++)
					{
						if(Rndf_MapPointchadian_path[i].x < 0||Rndf_MapPointchadian_path[i].x > 511||Rndf_MapPointchadian_path[i].y > 411||Rndf_MapPointchadian_path[i].y < 187)
							continue;
						x = Rndf_MapPointchadian_path[i].x;
						y = Rndf_MapPointchadian_path[i].y;

						for(int m=4;m>=-4;m--)
						{
							for(int n=-4;n<=4;n++)
							{
								if(y+m>411||y+m<0||x+n>511||x+n<0)
									continue;
								if(vel_Map->MapPoint[y+m][x+n] == 8 || vel_Map->MapPoint[y+m][x+n] == 18 || vel_Map->MapPoint[y+m][x+n] == 28)
								{
									obb_break_flag=1;
									ob_dis_now_path=412-(y+m);
									//ob_dis_now_path_timer=app->systemtimer10ms;

									SYSTEMTIME tt;
									GetLocalTime(&tt);
									ob_dis_now_path_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

									double middle_dis = abs(412-(y+m))*0.2;

									if(ob_dis_pre_path<1000)
									{
										//if(abs(ob_dis_now_path-ob_dis_pre_path)>30*(ob_loss_count_path+1))
										if(abs(ob_dis_now_path-ob_dis_pre_path)*0.2>30*(ob_dis_now_path_timer-ob_dis_pre_path_timer)*0.001)
										{
											ob_dis_pre_path=1000;
											ob_dis_prepre_path=1000;
											bfasterobacv_path=0;
										}
									}
									if(ob_dis_prepre_path<1000)
									{
										//if(abs(ob_dis_now_path-ob_dis_pre_path)>30*(ob_loss_count_path+1))
										if(abs(ob_dis_now_path-ob_dis_prepre_path)*0.2>30*(ob_dis_now_path_timer-ob_dis_prepre_path_timer)*0.001)
										{
											ob_dis_pre_path=1000;
											ob_dis_prepre_path=1000;
											bfasterobacv_path=0;
										}
									}

									if(ob_dis_pre_path==1000 && ob_dis_prepre_path==1000)
										bobfisrstapper_path=1;
									else if(bobfisrstapper_path)
									{
										if(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path>=0)
											bobfisrstapper_path=1;
										else if(ob_dis_prepre_path==1000 && ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path>=0)
											bobfisrstapper_path=1;
										else
											bobfisrstapper_path=0;
									}
									else
										bobfisrstapper_path=0;

									if((ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path>=1)||(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path>=1))
									{
										if(bfasterobacv_path==0)
										{
											bfasterobacv_path=1;
											fasterobspdini_path=app->GPS_Speed;
										}
									}

									if(bfasterobacv_path &&((abs(412-(y+m))*0.2)>15) && ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path>=1 && ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path>=3)
									{
										if(fasterobspdini_path<app->GPS_Speed+3.5/3.6)
											fasterobspdini_path=app->GPS_Speed+3.5/3.6;
										if(fasterobspdini_path>road_speed)
											fasterobspdini_path=road_speed;
									}

									double obfrontspd=dynamicmap_v_x->MapPoint[y+m][x+n];

									if(ob_dis_now_path<75)
									{
										if((ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path<0)||(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path<0))
										{
											if(bfasterobacv_path==1)
											{
												bfasterobacv_path=0;
											}
										}
									}
									else
									{
										if((ob_dis_pre_path<1000 && ob_dis_now_path-ob_dis_pre_path<-1)||(ob_dis_prepre_path<1000 && ob_dis_now_path-ob_dis_prepre_path<-1))
										{
											if(bfasterobacv_path==1)
											{
												bfasterobacv_path=0;
											}
										}
									}

									if(ob_dis_prepre_path<1000)
									{
										if(ob_dis_now_path_timer>ob_dis_prepre_path_timer)
											obfrontspd=(ob_dis_now_path-ob_dis_prepre_path-1)*0.2/(0.01*(ob_dis_now_path_timer-ob_dis_prepre_path_timer));
										else
											obfrontspd=(ob_dis_now_path-ob_dis_prepre_path-1)*0.2/(0.01);
										/*
										obfrontspd=(ob_dis_now_path-ob_dis_pre_path)*0.2/(0.05*(ob_loss_count_path+1));
										if(obfrontspd>-2*0.2/(0.05*(ob_loss_count_path+1)))
											obfrontspd=-2*0.2/(0.05*(ob_loss_count_path+1));
											*/
									}

									obabdis = sqrt(double((256-x-n)*(256-x-n)+(411-y-m)*(411-y-m)))/5;
									if(obabdis<9)
										obabdis=(double(411-y-m))/5.0;
									if((vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b)&&(!(middle_dis>8 && bobfisrstapper_path)))
									{
										if(obabdis<speedreducedist_)
										{
											drivespdtmp = (obabdis-8)/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
											else if(drivespdtmp>road_speed)
												drivespdtmp=road_speed;
											if(obabdis>6&&drivespdtmp<5/3.6)
												drivespdtmp=5/3.6;

											if(app->GPS_Speed>obstablespdaim)
											{
												if(drivespdtmp>app->GPS_Speed)
													drivespdtmp = app->GPS_Speed;
											}
											else if(drivespdtmp>obstablespdaim)
												drivespdtmp = obstablespdaim;
											if(bfasterobacv_path&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini_path-2/3.6)
													drivespdtmp=fasterobspdini_path-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(drivespdtmp<5/3.6)
												drivespdtmp=5/3.6;
											if(drivespdtmp<app->drive_obstacle2)
												app->drive_obstacle2=drivespdtmp;
										}
										if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv_path&&middle_dis>8))
											{
												if(((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(obabdis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && obabdis<speedreducedist_))
												{
													app->continue_spdred = true;
													app->continue_brake = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-8));
													if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}

													if(spdacctemp<app->spdredacc && spdacctemp<-2)
													{
														if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
														else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
													}

													if(spdacctemp<app->spdredacc)
														app->spdredacc = spdacctemp;
												}
												else if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&((app->GPS_Speed>obstablespdaim + 2.5/3.6)||(obabdis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(obabdis<6)))))
												{
													app->continue_spdred = true;
													app->continue_brake = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
													if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}
													if(bfasterobacv_path||bobfisrstapper_path)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}

													if(spdacctemp<app->spdredacc && spdacctemp<-2)
													{
														if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
														else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
													}

													if(spdacctemp<app->spdredacc)
														app->spdredacc = spdacctemp;
												}
											}
											else if(bfasterobacv_path&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
												{
													app->continue_spdred = true;
													if(spdacctemp<app->spdredacc)
														app->spdredacc = spdacctemp;
												}
											}
										}
									}
									if((vel_Map->MapPoint[y+m][x+n] == c)&&(!(middle_dis>8 && bobfisrstapper_path)))
									{
										if(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>237)
										{
											if(obabdis>40)
											{
												if(dynamicmap_v_x->MapPoint[y+m][x+n]>0)
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(drivespdtmp<5/3.6)
															drivespdtmp=5/3.6;
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													spdacctemp=0;
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(!(bfasterobacv_path&&middle_dis>15)))
														{
															app->continue_spdred = true;
															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
												}
												else
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n];
														if(drivespdtmp>app->GPS_Speed)
															drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(drivespdtmp<5/3.6)
															drivespdtmp=5/3.6;
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n])*(dynamicmap_v_x->MapPoint[y+m][x+n])/2/(obabdis-36);
													if(spdacctemp<-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-25))
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-25);
													if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12)))
														spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12));
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if(!(bfasterobacv_path&&middle_dis>8))
														{
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(dynamicmap_v_x->MapPoint[y+m][x+n]<-3/3.6)
																	app->continue_brake = true;
																if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
																{
																	double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																	if(spdaccdist>-2)
																		spdaccdist=-2;
																	if(spdacctemp<spdaccdist)
																		spdacctemp=spdaccdist;
																}
																if(spdacctemp<app->spdredacc)
																{
																	if(app->spdredacc<-3)
																		spdacctemp=app->spdredacc;
																	else if(spdacctemp<-3)
																		spdacctemp=-3;
																}
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
														else if(bfasterobacv_path&&middle_dis<15)
														{
															spdacctemp=0;
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
													}
												}
											}
											else if(obabdis>15)
											{
												if(dynamicmap_v_x->MapPoint[y+m][x+n]>10/3.6)
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(drivespdtmp<5/3.6)
															drivespdtmp=5/3.6;
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													spdacctemp=0;
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(!(bfasterobacv_path&&middle_dis>15)))
														{
															app->continue_spdred = true;
															//app->continue_brake = true;
															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
												}
												else
												{
													if(obabdis<speedreducedist_)
													{
														drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6;
														if(drivespdtmp>app->GPS_Speed)
															drivespdtmp = app->GPS_Speed;
														if(drivespdtmp<obstablespdaim)
															drivespdtmp=obstablespdaim;
														if(bfasterobacv_path&&middle_dis>8)
														{
															if(drivespdtmp<fasterobspdini_path-2/3.6)
																drivespdtmp=fasterobspdini_path-2/3.6;
															if(drivespdtmp<0)
																drivespdtmp=0;
														}
														if(drivespdtmp<5/3.6)
															drivespdtmp=5/3.6;
														if(app->drive_obstacle2>drivespdtmp)
															app->drive_obstacle2=drivespdtmp;
													}
													if(obabdis>25)
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-20);
													else if(obabdis>17)
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/((obabdis-15)/2);
													else
														spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2;
													//spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(obabdis-20);
													if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12)))
														spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-12));
													if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
													{
														if(!(bfasterobacv_path&&middle_dis>8))
														{
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(dynamicmap_v_x->MapPoint[y+m][x+n]<7/3.6)
																	app->continue_brake = true;
																if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
																{
																	double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																	if(spdaccdist>-2)
																		spdaccdist=-2;
																	if(spdacctemp<spdaccdist)
																		spdacctemp=spdaccdist;
																}
																if(spdacctemp<app->spdredacc)
																{
																	if(app->spdredacc<-3)
																		spdacctemp=app->spdredacc;
																	else if(spdacctemp<-3)
																		spdacctemp=-3;
																}
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
														else if(bfasterobacv_path&&middle_dis<15)
														{
															spdacctemp=0;
															if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
															{
																app->continue_spdred = true;
																if(spdacctemp<app->spdredacc)
																	app->spdredacc = spdacctemp;
															}
														}
													}
												}
											}
											else
											{
												if(obabdis<speedreducedist_)
												{
													drivespdtmp = (obabdis-8)/3.6;
													if(drivespdtmp<0)
														drivespdtmp=0;
													else if(drivespdtmp>road_speed)
														drivespdtmp=road_speed;
													if(obabdis>6&&drivespdtmp<5/3.6)
														drivespdtmp=5/3.6;

													if(app->GPS_Speed>obstablespdaim)
													{
														if(drivespdtmp>app->GPS_Speed)
															drivespdtmp = app->GPS_Speed;
													}
													else if(drivespdtmp>obstablespdaim)
														drivespdtmp = obstablespdaim;
													if(bfasterobacv_path&&middle_dis>8)
													{
														if(drivespdtmp<fasterobspdini_path-2/3.6)
															drivespdtmp=fasterobspdini_path-2/3.6;
														if(drivespdtmp<0)
															drivespdtmp=0;
													}
													if(drivespdtmp<5/3.6)
														drivespdtmp=5/3.6;
													if(drivespdtmp<app->drive_obstacle2)
														app->drive_obstacle2=drivespdtmp;
												}
												if((obabdis<speedreducedist_)||((obabdis<speedreducedist_+11)&&((app->continue_spdred)||(app->continue_brake))))
												{
													if(!(bfasterobacv_path&&middle_dis>8))
													{
														if(((app->GPS_Speed>obstablespdaim + 2.5/3.6)&&(obabdis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && obabdis<speedreducedist_))
														{
															app->continue_spdred = true;
															app->continue_brake = true;
															spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obabdis-8));
															if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
															{
																double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																if(spdaccdist>-2)
																	spdaccdist=-2;
																if(spdacctemp<spdaccdist)
																	spdacctemp=spdaccdist;
															}
															if(spdacctemp<app->spdredacc)
															{
																if(app->spdredacc<-3)
																	spdacctemp=app->spdredacc;
																else if(spdacctemp<-3)
																	spdacctemp=-3;
															}

															if(spdacctemp<app->spdredacc && spdacctemp<-2)
															{
																if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
																else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
															}

															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
														else if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&((app->GPS_Speed>obstablespdaim + 2.5/3.6)||(obabdis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(obabdis<6)))))
														{
															app->continue_spdred = true;
															app->continue_brake = true;
															spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
															if((ob_dis_prepre_path<1000)&&(middle_dis>=9))
															{
																double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
																if(spdaccdist>-2)
																	spdaccdist=-2;
																if(spdacctemp<spdaccdist)
																	spdacctemp=spdaccdist;
															}
															if(spdacctemp<app->spdredacc)
															{
																if(app->spdredacc<-3)
																	spdacctemp=app->spdredacc;
																else if(spdacctemp<-3)
																	spdacctemp=-3;
															}
															if(bfasterobacv_path||bobfisrstapper_path)
															{
																if(spdacctemp<-3)
																	spdacctemp=-3;
															}

															if(spdacctemp<app->spdredacc && spdacctemp<-2)
															{
																if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
																else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
																{
																	if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																		spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
																	if(spdacctemp>app->spdredacc)
																		spdacctemp=app->spdredacc;
																	if(spdacctemp>-2)
																		spdacctemp=-2;
																}
															}

															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
													else if(bfasterobacv_path&&middle_dis<15)
													{
														spdacctemp=0;
														if(app->GPS_Speed>obstablespdaim + 2.5/3.6)
														{
															app->continue_spdred = true;
															if(spdacctemp<app->spdredacc)
																app->spdredacc = spdacctemp;
														}
													}
												}
											}
										}
									}
									break;
								}
							}
							if(obb_break_flag)
								break;
						}
						if(obb_break_flag)
							break;
					}

					// 功能？
					if(obb_break_flag==0)
					{
						ob_loss_count_path++;
						if(ob_loss_count_path>=5||ob_dis_now_path==1000)
						{
							ob_dis_now_path=1000;
							ob_dis_pre_path=1000;
							ob_dis_prepre_path=1000;
							ob_dis_now_path_timer=0;
							ob_dis_pre_path_timer=0;
							ob_dis_prepre_path_timer=0;
							bfasterobacv_path=0;
							ob_loss_count_path=0;
						}
					}
					else
					{
						ob_loss_count_path=0;
						ob_dis_prepre_path=ob_dis_pre_path;
						ob_dis_prepre_path_timer=ob_dis_pre_path_timer;
						ob_dis_pre_path=ob_dis_now_path;
						ob_dis_pre_path_timer=ob_dis_now_path_timer;
					}


					x = 256;
					y = 411;
					obfrontdis=pathplan.SearchFrontOb(vel_Map,frontobdist);

					if((abs(gps_aimdir - m_dir) < 6 || abs(360 - abs(gps_aimdir - m_dir)) < 6) || able_ChangeLane || app->GPS_Speed<22/3.6)
					{
						if((obfrontdis>0)&&(aaaa==0 || aaaa==3 || aaaa==4))
						{
							if(app->bnearstop)
							{
								if(obfrontdis<5)
									driveobstacletmp = 0;
								else
									driveobstacletmp = 5/3.6;
							}
							else
							{
								if(obfrontdis<6)
									driveobstacletmp = 0;
								else
									driveobstacletmp = 5/3.6;
							}
							if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(obfrontdis>=7))
							{
								app->continue_spdred = true;
								spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(obfrontdis-6));
							}
							else if(app->GPS_Speed>5/3.6 + 2.5/3.6)
							{
								app->continue_spdred = true;
								spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
							}
							if(spdacctemp<app->spdredacc)
							{
								if(app->spdredacc<-3)
									spdacctemp=app->spdredacc;
								else if(spdacctemp<-3)
									spdacctemp=-3;
								app->spdredacc = spdacctemp;
							}
							if(driveobstacletmp<app->drive_obstacle2)
								app->drive_obstacle2=driveobstacletmp;
							if(app->GPS_Speed>5/3.6 + 2.5/3.6)
								app->continue_brake = true;
						}
					}
				}
				else
				{
					for(int i=0;i<200;i++)
					{
						Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,MidGpsPoint[i],m_dir);
						if(m_gps.x==MidGpsPoint[i].x&&m_gps.y==MidGpsPoint[i].y)
						{
							Rndf_MapPoint[i].x = 256;
							Rndf_MapPoint[i].y = 411;
						}
					}
					int m_up = 262;
					int m_down = 412;
					
					int x = 0;
					int y = 0;

					int width2 = 5;
					int m;
					int n;
					int a=8;
					int b=18;
					int c=28;
					double mindist=0;
					double drivespdtmp=road_speed;
					bool break_flag=0;
					for(int i=0;i<200;i++)
					{
						if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
							continue;
						x = Rndf_MapPoint[i].x;
						y = Rndf_MapPoint[i].y;
						for(m=5;m>=-5;m--)
						{
							for(n=-5;n<=5;n++)
							{
								if(y+m>=412||y+m<0||x+n>511||x+n<0)
									continue;
								if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b||vel_Map->MapPoint[y+m][x+n] == c)
								{
									mindist = sqrt(double((x+n-256)*(x+n-256)+(y+m-412)*(y+m-412)))/5;
									drivespdtmp = mindist/3.6;
									if(mindist<6)
										drivespdtmp=0;
									if(drivespdtmp>road_speed)
										drivespdtmp=road_speed;
									if(app->drive_obstacle2>drivespdtmp)
										app->drive_obstacle2=drivespdtmp;
									if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(mindist>=6)&&((app->GPS_Speed>drivespdtmp-5/3.6)||(app->GPS_Speed>12.5/3.6)))
									{
										app->continue_spdred = true;
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed)-5/3.6*5/3.6)/(2*(mindist-5));
										app->continue_brake = true;
										if(spdacctemp<-3)
											spdacctemp=-3;
										if(spdacctemp<app->spdredacc)
											app->spdredacc = spdacctemp;
									}
									else if(mindist<6)
									{
										app->continue_spdred = true;
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
										app->continue_brake = true;
										if(spdacctemp<-3)
											spdacctemp=-3;
										if(spdacctemp<app->spdredacc)
											app->spdredacc = spdacctemp;
									}
									break_flag=1;
									break;
								}
							}
							if(break_flag)
								break;
						}
						if(break_flag)
							break;
					}
					fasterobspdini_path=road_speed;
					bfasterobacv_path=0;
					bobfisrstapper_path=0;
					ob_dis_now_path=1000;
					ob_dis_pre_path=1000;
					ob_dis_prepre_path=1000;
					ob_dis_now_path_timer=0;
					ob_dis_pre_path_timer=0;
					ob_dis_prepre_path_timer=0;
					ob_loss_count_path=0;
				}
				break;
		}

		app->drive_obstacle2_update = app->drive_obstacle2;

		if(app->stop)
			continue;

		// 
		app->critical_planningroad.Lock();
		cvClearSeq( CVehicle::planning_road );
		
		for(int i = 0; i<200; i++)
		{
			cvSeqPush( CVehicle::planning_road, &CVehicle::MidGpsPoint[i] );
		}

		if(MidGpsPoint[0].x == MidGpsPoint[199].x && MidGpsPoint[0].y == MidGpsPoint[199].y)
		{
			app->stop = true;
		}
		else
			app->stop = false;

		app->critical_planningroad.Unlock();

		SetEvent(app->m_hEvent);//路径更新后，发送消息
		app->sendflag1 = true;
		GetLocalTime(&t1);
		outpathtime<<"time2 "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  "<<endl;
	}
	return 0 ;
}


int CVehicleExe::GetPathSeq(CvSeq *path,MAPPOINTexe *p)
{
	cvClearSeq(path);
	CvPoint2D64f pt;
	for (;p->next!=NULL;)
	{
		pt.x = p->next->x;
		pt.y = p->next->y;
		cvSeqPush(path,&pt);
		p=p->next;
	}
	return 1;

}


int CVehicleExe::SendtoPerc()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f WayPoint[200];
	CString a;

	if(app->kuaisulufulu==1)
	{
		road_state.send_message("100");
		//app->sendtoper = 0;
		outUturnsign<<"100"<<endl;
		return 0;
	}

	/*
	if((app->bApaActive>=1 && app->bApaActive<11)||(app->PXApaActive>=1 && app->PXApaActive<16))
	{
		road_state.send_message("200");
		//app->sendtoper = 0;
		outUturnsign<<"200"<<endl;
		return 0;
	}
	*/

	if(!approachgoal_flag)
	{
		if(m_task == IVTASK_LANE)
		{
			road_state.send_message("0");
			//app->sendtoper = 0;
			outUturnsign<<"0"<<endl;
			return 0;
		}
		else if(m_task == IVTASK_APPROINTERSECTION)
		{
			if(approachyulukou==1)
			{
				road_state.send_message("8");
				//app->sendtoper =8;
				outUturnsign<<"lushang: 8"<<endl;
				return 0;
			}
			else if(approachyulukou==2)
			{
				road_state.send_message("9");
				//app->sendtoper = 9;
				outUturnsign<<"lushang: 9"<<endl;
				return 0;
			}
			else
			{
				road_state.send_message("2");
				//app->sendtoper =2;
				outUturnsign<<"2"<<endl;
				return 0;
			}
		}
		else
		{
			road_state.send_message("1");
			//app->sendtoper =1;
			outUturnsign<<"1"<<endl;
			return 0;
		}
	}
	else
	{
		road_state.send_message("9");
		//app->sendtoper =9;
		outUturnsign<<"goalpoint: 9"<<endl;
		return 0;
	}
	
	return 0;
	
}

/******************此处为全局规划******************/
//solvewaypointnum();//求整个路网中所有的路点数目waypointnum
//point2point();//设置所有路点的连通性（solvewaypointnum和point2point两个函数都是初始化函数，执行一次就可以了）

///*********函数solvepoint和solvepoint2注解：solvepoint是根据点的经纬度坐标求路网中与其最近的路点；
//solvepoint2是根据点的经纬度坐标和一个方向求路网中与其最近的路点，但是这个路点所在的路段方向必须和
//这个已知方向相符（正负30度以内）。程序中这两个函数都设置了求得的最近路点与已知点之间的最小距离阈值，
//只有求得的最近路点与该点之间的距离小于200米才可
//**********/

////double aaaaa=m_GpsData.GetAngle(31.847095,117.120523,31.847096,117.118473);
////MAPPOINT* t=solvepoint2(31.847095,117.119498,aaaaa+27);
///*测试用*/

//planning(1,1,1,2,1,2);//求1.1.1到2.1.2点的最短路径
//pathpointnum;//全局最短路径经过的路点个数
//path;//全局最短路径存放在该链表中


//planning(2,1,1,2,2,3);//求2.1.1到2.2.3点的最短路径
//pathpointnum;//全局最短路径经过的路点个数
//path;//全局最短路径存放在该链表中

/******************全局规划分界线********************/
void CVehicleExe::GetGlobalPath()
{
	solvewaypointnum();//求整个路网中所有的路点数目waypointnum
	point2point();//
	//planning(1,1,1,3,2,2);
	//planning(startp.m,startp.n,startp.p,endp.m,endp.n,endp.p);
	
	//MAPPOINT *temporary_path=(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT ));
	pathexe =(struct MAPPOINTexe*)calloc(1,sizeof( struct MAPPOINTexe ));
	MAPPOINT *temporary_point;
	MAPPOINTexe *temp_point;
	MAPPOINTexe *temp_point1;
	pathpointnumexe = 0;
	Point3DInt p1 = startp;

	int checknum = MDF.Check_Points.size();
	for (int i = 0; i<checknum;i++)
	{
		int checksn = MDF.Check_Points.front();//
		Point3DInt p2;
		RNDFGPS.GetCheckPointID(checksn,p2.m,p2.n,p2.p);

		planning(p1.m,p1.n,p1.p,p2.m,p2.n,p2.p);
		pathpointnumexe = pathpointnumexe + pathpointnum-1;
		for (temporary_point = path->next;temporary_point->next!=NULL;)
		{
			temp_point = pathexe->par;
			pathexe->par = new MAPPOINTexe;
			solveID(temporary_point);
			pathexe->par->x = temporary_point->x;
			pathexe->par->y = temporary_point->y;
			pathexe->par->ID111 = ID1;
			pathexe->par->ID222 = ID2;
			pathexe->par->ID333 = ID3;
			pathexe->par->par = temp_point;
			temporary_point = temporary_point->next;
		}

		MDF.Check_Points.pop();
		p1=p2;
	}


	/////////////最后一个检测点到终点
	planning(p1.m,p1.n,p1.p,endp.m,endp.n,endp.p);
	pathpointnumexe = pathpointnumexe + pathpointnum;
	for (temporary_point = path->next;temporary_point!=NULL;)
	{
		temp_point = pathexe->par;
		pathexe->par = new MAPPOINTexe;
		solveID(temporary_point);
		pathexe->par->x = temporary_point->x;
		pathexe->par->y = temporary_point->y;
		pathexe->par->ID111 = ID1;
		pathexe->par->ID222 = ID2;
		pathexe->par->ID333 = ID3;
		pathexe->par->par = temp_point;
		temporary_point = temporary_point->next;
	}



	//pathpointnumexe = pathpointnumexe + 1;
	//temp_point = pathexe->par;
	//pathexe->par = new MAPPOINTexe;

	//CvPoint2D64f endpoint;
	//endpoint = RNDFGPS.GetPoint(endp.m,endp.n,endp.p);//取终点
	//solveID(endpoint);//.....
	//pathexe->par->x = endpoint.x;
	//pathexe->par->y = endpoint.y;
	//pathexe->par->ID111 = ID1;
	//pathexe->par->ID222 = ID2;
	//pathexe->par->ID333 = ID3;
	//pathexe->par->par = temp_point;

	for (temp_point1=pathexe->par;temp_point1!=NULL;)
	{
		temp_point = pathexe->next;
		pathexe->next = new MAPPOINTexe;
		pathexe->next->x = temp_point1->x;
		pathexe->next->y = temp_point1->y;
		pathexe->next->ID111 = temp_point1->ID111;
		pathexe->next->ID222 = temp_point1->ID222;
		pathexe->next->ID333 = temp_point1->ID333;
		pathexe->next->next = temp_point;
		temp_point1 = temp_point1->par;
	}


	solvenearpointnum = 0;
}
bool CVehicleExe::SearchObstacleMap(CvPoint2D64f *maplane,int num,I_Map *map,int up,int down)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int m_up = 412-up*5;
	int m_down = 411 -down*5;

	int x = 0;
	int y = 0;
	int Xmin = -6;//-4
	int Xmax = 6;//4
	if(approachgoal_flag||approachyulukou==2)
		Xmax = 8;//8
	if(approachyulukou==1)
		Xmin = -8;//-8

	for(int i = 0;i<num;i++)
	{
		if(maplane[i].x < 0||maplane[i].x > 511||maplane[i].y > m_down||maplane[i].y < m_up)
			continue;
		x = maplane[i].x;
		y = maplane[i].y;
		for(int m=-4;m<=4;m++)
			for(int n=Xmin;n<=Xmax;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if((down==0)&&(y+m>=420))
					continue;
				if(map->MapPoint[y+m][x+n] == 8||map->MapPoint[y+m][x+n] == 18||map->MapPoint[y+m][x+n] == 28)
				{
					if(y+m<362)
						return 2;
					else
						return 1;
				}
			}
	}
	return false;
}

bool CVehicleExe::SearchObstacleMaptmp(CvPoint2D64f *maplane,int num,I_Map *map,int up,int down)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int m_up = 412-up*5;
	int m_down = 411 -down*5;

	int x = 0;
	int y = 0;
	int Xmin = -1;
	int Xmax = 21;//31

	for(int i = 0;i<num;i++)
	{
		if(maplane[i].x < 0||maplane[i].x > 511||maplane[i].y > m_down||maplane[i].y < m_up)
			continue;
		x = maplane[i].x;
		y = maplane[i].y;
		for(int m=-4;m<=4;m++)
			for(int n=Xmin;n<=Xmax;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if((down==0)&&(y+m>=420))
					continue;
				if(map->MapPoint[y+m][x+n] == 8||map->MapPoint[y+m][x+n] == 18||map->MapPoint[y+m][x+n] == 28)
				{
					return true;
				}
			}
	}
	return false;
}

void CVehicleExe::StartLight()
{
	m_hThreadLight = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theLightThread,
		this, 0, &dwLightThreadId);
}

DWORD CVehicleExe::theLightThread(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->LightThread());
}

//综合运行流程
DWORD CVehicleExe::LightThread()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->light_res = 0;

	int redlampcnt=0;
	int realredlamp=0;

	while(1)
	{
		realredlamp =WaitForSignal1();
		if(realredlamp==2)
		{
			redlampcnt=0;
			app->light_res=2;
		}
		else
		{
			redlampcnt++;
			if(redlampcnt<2)
				app->light_res=2;
			else
			{
				redlampcnt=31;
				app->light_res=realredlamp;
			}
		}
		//app->light_res =WaitForSignal1();
		//app->light_res =2;
		if(app->light_res != light_res_HIS)
		{
			light_res_HIS=app->light_res;
			lightrecord<<app->GPS_Point.x<<"     "<<app->GPS_Point.y<<"       "<<light_res_HIS<<endl;
		}
		Sleep(100);
	}
	return 0;
}

int CVehicleExe::GetFangxiang(CvSeq *gp)
{
	CvPoint2D64f Point[4];
	int seq = seq_num;
	int num = RNDFGPS.m_mapInfo.pSegment[ID1-1].pLane[ID2-1].waypoints_num;
	int goal_num = num - ID3;
	Point[0] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq+goal_num-1);
	Point[1] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq+goal_num);
	Point[2] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq+goal_num+1);
	Point[3] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq+goal_num+2);	
	double dir1 = m_GpsData.GetAngle(Point[1],Point[0]);
	double dir2 = m_GpsData.GetAngle(Point[3],Point[2]);
	if(abs(dir1-dir2)<45||abs(360-abs(dir1-dir2))<45  )
	{
		fx = 1;
	
	}
	if(abs(dir1-dir2+270)<45||abs(90-dir1+dir2)<45 /*abs(dir1-dir2-270)<30||abs(90+dir1-dir2)<30*/)
	{
		fx = 2/*2*/;
		
		
	}
	if(abs(dir1-dir2+90)<45||abs(270-dir1+dir2)<45 /*abs(dir1-dir2-90)<30||abs(270+dir1-dir2)<30*/)
	{
		fx = 3/*3*/;
	
		
	}
	if(abs(abs(dir1-dir2)-180)<10/*abs(abs(dir1-dir2-180)<10||abs(180+dir1-dir2))<10*/ /*abs(dir1-dir2-270)<30||abs(90+dir1-dir2)<30*/)
	{
		
		fx = 4;
	}
	return 1;
}

int CVehicleExe::SearchLeftRightObstacleGps(CvPoint2D64f *waypoint)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	bool ob = false;
	int up;
	up=40;
	if(app->GPS_Speed*3.6>30)
		up=45;
	ob = SearchObstacle(waypoint,map_ts,up/*app->GPS_Speed*2+35*/,0);
	
	if (!ob)
	{
		if (ob == 2)
		{
			return 2;
		}
		return 0;
		app->secondDecison = "前道无障";
	}
	else
	{
		MoveLeft(/*OriGpsPoint*/waypoint,exe_leftpoint,3.7);
		ob_left = SearchObstacle(exe_leftpoint,map_ts,up,0);
		out128<<"	ob_left "<<ob_left<<endl;
		if(!ob_left)
		{
			app->secondDecison = "右换道";
			app->left_right = 1;
			lanechang_flag = false;
			app->drive_obstacle = max_speed;
			return -1;
		}
		else
		{
			MoveLeft(/*OriGpsPoint*/waypoint,exe_rightpoint,-3.7);
			ob_right = SearchObstacle(exe_rightpoint,map_ts,up,0);
			
			if(ob_right)
			{	
				app->secondDecison = "遇障停车";
				return 0;
			}
			else
			{
				app->secondDecison = "左换道";
				app->left_right = -1;
				lanechang_flag = false;
				app->drive_obstacle = max_speed;	
				return 1;
			}
		}
	}
	return 0;
}

int CVehicleExe::Calculate2cicurve(int id1,int id2,int id3,CvPoint2D64f (&curve)[200],CvPoint2D64f err)
{
	CvPoint2D64f WayPoint[200];
	CvPoint2D64f CPoint[3];
	int point_num = RNDFGPS.m_mapInfo.pSegment[id1-1].pLane[id2-1].waypoints_num;	

	if(id3 == point_num)
	{
		CPoint[0].x = RNDFGPS.GetPoint(id1,id2,id3-2).x-err.x;
		CPoint[0].y = RNDFGPS.GetPoint(id1,id2,id3-2).y-err.y;
		CPoint[1].x = RNDFGPS.GetPoint(id1,id2,id3-1).x-err.x;
		CPoint[1].y = RNDFGPS.GetPoint(id1,id2,id3-1).y-err.y;
		CPoint[2].x = RNDFGPS.GetPoint(id1,id2,id3).x-err.x;
		CPoint[2].y = RNDFGPS.GetPoint(id1,id2,id3).y-err.y;

	}
	else	
	{
		CPoint[0].x = RNDFGPS.GetPoint(id1,id2,id3-1).x-err.x;
		CPoint[0].y = RNDFGPS.GetPoint(id1,id2,id3-1).y-err.y;
		CPoint[1].x = RNDFGPS.GetPoint(id1,id2,id3).x-err.x;
		CPoint[1].y = RNDFGPS.GetPoint(id1,id2,id3).y-err.y;
		CPoint[2].x = RNDFGPS.GetPoint(id1,id2,id3+1).x-err.x;
		CPoint[2].y = RNDFGPS.GetPoint(id1,id2,id3+1).y-err.y;

	}
	
	lanedriverstate.Get2Curve(CPoint[0],CPoint[1],CPoint[2],curve);
	return 1;

}

int CVehicleExe::GetNearWPoint(int id1,int id2,int id3,CvPoint2D64f m_gps,double m_dir)
{
	CvPoint2D64f WayPoint[200];
	CvPoint2D64f CPoint[3];
	int point_num = RNDFGPS.m_mapInfo.pSegment[id1-1].pLane[id2-1].waypoints_num;
	double temp = 1000000;
	double dist[200] = {0};
	int id = 1;
	for(int i = 0;i<point_num; i++)
	{
		WayPoint[i] = RNDFGPS.GetPoint(id1,id2,i+1);
		dist[i] = m_GpsData.GetDistance(WayPoint[i].x,WayPoint[i].y,m_gps.x,m_gps.y);
		if(dist[i]<temp)
		{
			temp = dist[i];
			id = i+1;
		}
	}
	return id;

}

int CVehicleExe::AppLaneChange(int fx,double dn,CvPoint2D64f *point,CvPoint2D64f aimpoint,double dir,int &change,CvPoint2D64f &pianyi)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	dn += change;
	int lane_num = RNDFGPS.m_mapInfo.pSegment[ID1-1].lane_num/2;
	CvPoint2D64f m_gps= app->GPS_Point;
	double m_dir = app->GPS_Direction;
	CvPoint2D64f oripath[200];
	CvPoint2D64f temp_point[200];
	CvPoint2D64f temp_map[200];
	CvPoint2D64f temp_pianyi;
	ArrayFuZhi(point,temp_point);
	temp_pianyi = pianyi;
	for(int i = 0;i<200;i++)
	{
		temp_map[i] =  m_GpsData.APiontConverD(app->GPS_Point,temp_point[i],app->GPS_Direction);
	}
	
	int up = 20;
	int down = 0;
	CvPoint2D64f upoint,dpoint,mapup,mapdown;
	lanedriverstate.GetPointfromNearGPS(point,200,m_gps,up,upoint);
	lanedriverstate.GetPointfromNearGPS(point,200,m_gps,down,dpoint);
	mapup = m_GpsData.APiontConverD(m_gps,upoint,m_dir);
	mapdown = m_GpsData.APiontConverD(m_gps,dpoint,m_dir);

	
	//lane_num+=1;
	if(fx == 3 && dn >=1/*&& applanechang == false*/)
	{

		if(dn-1>1)
		{
			MoveLeftDir(temp_point,temp_point,-3.7*1,dir,temp_pianyi);
			ArrayFuZhi(temp_point,point);
			pianyi = temp_pianyi;
			change += -1;
			return 1;
		}
		MoveLeftDir(temp_point,temp_point,-3.7*(dn-1-0.2),dir,temp_pianyi);
		//搜它对不对
		int res = SearchObatAppro(20,0,temp_point,app->GPS_Point,app->GPS_Direction,1);
		if(res == 1)//对的
		{
			ArrayFuZhi(temp_point,point);
			pianyi = temp_pianyi;
			if (dn> 1)
				change += int(-dn+1-0.9);

			return 1;
		}
		//不对的
		//ArrayFuZhi(temp_point,point);
		//pianyi = temp_pianyi;
		if(res == 0)
			MoveLeftDir(temp_point,temp_point,3.7,dir,temp_pianyi);
		
		int countob = 0;
		for(int k =0 ; k< 13 ;k++)
		{
			MoveLeftDir(temp_point,temp_point,-0.4,dir,temp_pianyi);
			lanedriverstate.GetPointfromNearGPS(temp_point,200,m_gps,up,upoint);
			lanedriverstate.GetPointfromNearGPS(temp_point,200,m_gps,down,dpoint);
			mapup = m_GpsData.APiontConverD(m_gps,upoint,m_dir);
			mapdown = m_GpsData.APiontConverD(m_gps,dpoint,m_dir);
			for (int i = mapup.y;i < mapdown.y;i++)
			{
				for (int j =0; j<4;j++)
				{
					int y = i ;
					int x = mapup.x + (i-mapup.y)*(mapup.x - mapdown.x)/(mapup.y - mapdown.y);
					if(y< 0||y > 511||x+j > 511||x+j < 0)
						continue;
					if (vel_Map->MapPoint[y][x + j] == 8 || vel_Map->MapPoint[y][x + j] == 18 )
					{
						
						countob++;
						if(countob>25)
						{
							MoveLeftDir(temp_point,temp_point,1,dir,temp_pianyi);
							if (dn> 1)
								change += int(-dn+1-0.9);
							ArrayFuZhi(temp_point,point);
							pianyi = temp_pianyi;
							return 1;
						}

					}
					

				}

			}

		}
		//MoveLeftDir(point,point,3.7*(lane_num-dn+0.2),dir,temp_pianyi);
		if (dn> 1)
		{
			change += int(-dn+1-0.9);
			MoveLeftDir(point,point,-3.7*(dn-1-0.2),dir,temp_pianyi);
		}
		return 1;
	}
	if((fx == 4||fx == 2) && lane_num-dn>=0)
	{
		if(lane_num<=2)
		{
			MoveLeftDir(temp_point,temp_point,3.7*(lane_num-dn),dir,temp_pianyi);
			ArrayFuZhi(temp_point,point);
			pianyi = temp_pianyi;
			if (lane_num - dn > 0)
				change += int(lane_num-dn+0.9);
			return 1;
		}
		else
		{
			
			if(lane_num-dn>1)
			{
				MoveLeftDir(temp_point,temp_point,3.7*1,dir,temp_pianyi);
				ArrayFuZhi(temp_point,point);
				pianyi = temp_pianyi;
				change += 1;
				return 1;
			}		
			MoveLeftDir(temp_point,temp_point,3.7*(lane_num-dn),dir,temp_pianyi);
			
			int res = SearchObatAppro(20,0,temp_point,app->GPS_Point,app->GPS_Direction,-1);
			if(res == 1)//对的
			{
				ArrayFuZhi(temp_point,point);
				pianyi = temp_pianyi;
				if (lane_num - dn > 0)
					change += int(lane_num-dn+0.9);
				return 1;
			}

			if(res == 0)
			{
				MoveLeftDir(temp_point,temp_point,-3.7,dir,temp_pianyi);
				if(lanedriverstate.SearchObstacle(temp_point,vel_Map,8,18,8,20,0,4))
					MoveLeftDir(temp_point,temp_point,-3.7,dir,temp_pianyi);
			}
			int countob = 0;
			for(int k =0 ; k< 13 ;k++)
			{
				MoveLeftDir(temp_point,temp_point,0.6,dir,temp_pianyi);
				lanedriverstate.GetPointfromNearGPS(temp_point,200,m_gps,up,upoint);
				lanedriverstate.GetPointfromNearGPS(temp_point,200,m_gps,down,dpoint);
				mapup = m_GpsData.APiontConverD(m_gps,upoint,m_dir);
				mapdown = m_GpsData.APiontConverD(m_gps,dpoint,m_dir);
				for (int i = mapup.y;i < mapdown.y;i++)
				{
					for (int j =-4; j<0;j++)
					{
						int y = i ;
						int x = mapup.x + (i-mapup.y)*(mapup.x - mapdown.x)/(mapup.y - mapdown.y);
						if(y< 0||y > 511||x+j > 511||x+j < 0)
							continue;
						if (vel_Map->MapPoint[y][x + j] == 8 || vel_Map->MapPoint[y][x + j] == 18 )
						{
							
							countob++;
							if(countob>25)
							{
								MoveLeftDir(temp_point,temp_point,-0.5,dir,temp_pianyi);
								if (lane_num - dn > 0)
									change += int(lane_num-dn+0.9);
								ArrayFuZhi(temp_point,point);
								pianyi = temp_pianyi;
								return 1;
							}

						}
						

					}

				}

			}
			if (lane_num - dn > 0)
			{
				change += int(lane_num-dn+0.9);
				MoveLeftDir(point,point,3.7*(lane_num-dn),dir,temp_pianyi);
			}
			return 1;
		}
	}

	return 0;
}

int CVehicleExe::Setspeed(int drive_state,int drive_obstacle)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->drive_state = drive_state;
	app->drive_obstacle = drive_obstacle;


	return 1;
	
}
int CVehicleExe::Getrndfbezier(CvPoint2D64f &pianyi)
{
	CvPoint2D64f WayPoint[200];
	int point_num = RNDFGPS.m_mapInfo.pSegment[ID1-1].pLane[ID2-1].waypoints_num;	
	for (int i = 1; i<=point_num;i++)
	{
		WayPoint[i-1] = RNDFGPS.GetPoint(ID1,ID2,i);
	}
	if(RNDFGPS.GetLaneShape(ID1,ID2))
		Bezier(WayPoint,point_num-1,rndfbezier);
	else
	{
		if (gps_road!=NULL)
		{
			free(gps_road);
		}
		gps_road = (CvPoint2D64f *)malloc( ((point_num-1)*50)*sizeof(CvPoint2D64f) );
		//GetPartGlobalPath(ID1,ID2,gps_road);
		GetGlobalPathSeq(gps_road,(point_num-1)*50);	
		ReturnPartPoints(CVehicle::global_path,realtime_Gps,realtime_Dir,20,rndfbezier,pianyi);
	}
		//Calculate2cicurve(ID1,ID2,ID3,rndfbezier,pianyi);

	return 1;
}
int CVehicleExe::GetLanebezier(CvPoint2D64f realtime_Gps,double realtime_Dir,CvPoint2D64f &pianyi,double yumiao)
{
	CvPoint2D64f MidPoint[512];
	int midd_num1 =  0;
	//int lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);

	int lane_result;
	lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);

	int laneob_result = SearchObstacleMap(MidPoint,midd_num1,vel_Map,30,0);
	if (lane_result && !laneob_result && abs(MidPoint[int(midd_num1/2)].x - 256)<12 && ((m_task == IVTASK_LANE)||(m_task == IVTASK_APPROINTERSECTION)))
	{
		CvPoint2D64f lanepgps = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,MidPoint[int(midd_num1/2)]);
		//CvPoint2D64f lanepgps = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,MidPoint[0]);
		MoveWay(lanepgps,realtime_Dir,rndfbezier,pianyi);
	}
	else
	{
		MoveWay(realtime_Gps,realtime_Dir,rndfbezier,pianyi);
	}
	return 1;
}

int CVehicleExe::ZhiDaoLane(int &ob_num,int &count_move,CvPoint2D64f &obvegps)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int midd_num1;
	int ob_nixing = 0;
	CvPoint2D64f MidPoint[512];
	int m_down;
	int lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);
	//////如果有车道线，车道线左右5m内没有障碍物，并且目标点与车道线中点之间的垂直距离在2~5m内，修正rndfpath
	if (lanechang_flag)
		app->drive_obstacle = max_speed;
	if (lane_result>0&&lanechang_flag)
	{
		out128<<"有车道"<<endl;
		int laneob_result = SearchObstacleMap(MidPoint,midd_num1,vel_Map, 40,2);
		if (!laneob_result)
		{
			out128<<"没障碍"<<endl;
			CvPoint2D64f lanepgps = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,MidPoint[int(midd_num1/2)]);
			hxdishis = hxdis;
			hxdis = LevelDist(gps_aimpoint,gps_aimdir,lanepgps);
			out128<<"hxdis  "<<hxdis<<endl;
			if(/*abs(hxdis) > 0.3 &&*/ abs(hxdis-hxdishis) < 0.5)
				count_move++;
			else
				count_move = 0;
			if (count_move>1&& lanechang_flag)
			{
				count_move = 0;
				out128<<"修正"<<endl;
				MoveWay(lanepgps,realtime_Dir,rndfbezier);
				xz_flag = true;
				
				app->secondDecison = "修正车道";
			}
		}
	}
	if(app->GPS_Speed*3.6>25)
		m_down = -20;
	else
		m_down = 0;
	gps_aimdir = RNDFGPS.GetLaneDir(ID1,ID2);
	lanechang_dis = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,ob_tempoint.x,ob_tempoint.y);
	//lanechang_dis = lanedriverstate.GetMinDis(rndfbezier,200,realtime_Gps);
	//int num = lanedriverstate.GetMinNum(rndfbezier,200,realtime_Gps);
	//lanechang_dis = LevelDist(rndfbezier[num],gps_aimdir,realtime_Gps);
	//lanechang_dis = LevelDist(realtime_Gps,realtime_Dir,rndfbezier[num]);
	if((abs(gps_aimdir - realtime_Dir) < 7 || abs(360 - abs(gps_aimdir - realtime_Dir)) < 7) && abs(lanechang_dis )>20 &&!lanechang_flag)
		{	
			tongji++;
			if (tongji > 10)
			{
				outn77<<"正了"<<endl;
				app->left_right = 0;
				lanechang_flag = /*false*/true;
				app->drive_obstacle =max_speed;
				control.setLight(LAMP_OFF);
				tongji = 0;
			}
		
			
		}
	if (/*!lanechang_flag&&*/lanedriverstate.SearchObstacle(rndfbezier,map_ts,8,18,38,40,0,0.6))
	{
		
		//if((abs(gps_aimdir - realtime_Dir) < 7 || abs(360 - abs(gps_aimdir - realtime_Dir)) < 7))
		
		//lanechang_flag = true;
		//app->left_right = 0;
		//count_tongji++;
		//if(count_tongji>5)
		{
			//count_tongji=0;
			if (lanedriverstate.SearchObstacle(rndfbezier,map_ts,8,18,38,20,0,0.6))
			{
				lanechang_flag = true;
				app->left_right = 0;
				outn77<<"sudu5"<<endl;
				app->drive_obstacle = obstacle_speed;
				//lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,15,gps_aimpoint);
				lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,40,gps_aimpoint);
				aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
				//return 0;
			}
			
			outn77<<"sudu10"<<endl;
			app->drive_obstacle = obstacle_speed;
			//lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,20,gps_aimpoint);
			//lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,40,gps_aimpoint);
			//aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
			//return 0;
		}
	}
	
	//ob_res = lanedriverstate.SearchObstacle(rndfbezier,map_ts,28,28,28,15,0);
	/////////////////////////////////////////////////////////////////////没管停车
	

	if(lanechang_flag)
	{
		ob_res = lanedriverstate.SearchMoveStaticObstacleGps(rndfbezier);
		outn77<<"  ob_res "<<ob_res<<" "<<lanechang_flag<<", "<<ob_nixing<<endl;
		if(ob_res == 3)
			/*Stop();*/
			app->drive_obstacle = max_speed;
		else if(ob_res == 1||ob_res == -1)
		{
			count_tongji++;
			{
				if(count_tongji > 0)
				{
					ob_tempoint = cvPoint2D64f(realtime_Gps.x,realtime_Gps.y);
					MoveLeft(rndfbezier,rndfbezier,-3.9*ob_res);
					count_tongji = 0;
				}
				//************924**********//
				double light_flag=-3.9*ob_res;
				if(light_flag>0)
					control.setLight(RIGHT_LAMP_ON);
				else
					control.setLight(LEFT_LAMP_ON);
				//************924**********//
				app->secondDecison = "避障";
				app->drive_obstacle = max_speed;
				lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,25,gps_aimpoint);
				aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
				lanechang_flag = false;
				//xz_flag = false;
			}
		}
		else if (ob_res == 4)
		{
			lanechang_flag = true;
			app->drive_obstacle = obstacle_speed/2;
		}
		else if (ob_res == 5)
		{
			
			app->drive_obstacle = obstacle_speed;
			
		}
		else if (ob_res == 6)
		{
			app->drive_obstacle = max_speed/*30*/;
		}
		

	}
	///////////////////////////////////////////////////////////////////
	if(lanechang_flag)
	{

		
			ob_nixing = lanedriverstate.SearchNiXing(rndfbezier);
			
			if(ob_nixing == 1||ob_nixing == -1)
			{
				app->left_right = 1;
				{
					app->drive_obstacle= max_speed;
					ob_tempoint = cvPoint2D64f(realtime_Gps.x,realtime_Gps.y);				
					MoveLeft(rndfbezier,rndfbezier,-3.7*ob_nixing);
					lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,40,gps_aimpoint);
					aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
					lanechang_flag = false;
					xz_flag = false;
				}
			}
		
			
	}////////////
	
	lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,40,gps_aimpoint);
	aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
	if(!lanechang_flag)
	{
		lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,25,gps_aimpoint);
	}
	return 1;
}

int CVehicleExe::WanDaoLane(int &ob_num,int &count_move)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int midd_num1;
	CvPoint2D64f MidPoint[512];
	ID3 = GetNearWPoint(ID1,ID2,ID3,realtime_Gps,realtime_Dir);
	app->secondDecison = "弯道";
	//Calculate2cicurve(ID1,ID2,ID3,rndfbezier,pianyi);

	ReturnPartPoints(CVehicle::global_path,realtime_Gps,realtime_Dir,20,rndfbezier,pianyi);

	lanedriverstate.GetPathDirGPS(rndfbezier,200,realtime_Gps,realtime_Dir,gps_aimdir);
	
	int lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);
	//////如果有车道线，车道线左右5m内没有障碍物，并且目标点与车道线中点之间的垂直距离在2~5m内，修正rndfpath
	if (lane_result)
	{
		out128<<"有车道"<<endl;
		int laneob_result = SearchObstacleMap(MidPoint,midd_num1,vel_Map,40,0);
		if (!laneob_result)
		{
			out128<<"没障碍"<<endl;
			CvPoint2D64f lanepgps = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,MidPoint[int(midd_num1/2)]);
			hxdishis = hxdis;
			hxdis = LevelDist(gps_aimpoint,gps_aimdir,lanepgps);
			out128<<"hxdis  "<<hxdis<<endl;
			if(/*abs(hxdis) > 0.3 &&*/ abs(hxdis-hxdishis) < 0.3)
				count_move++;
			else
				count_move = 0;
			if (count_move>3)
			{
				count_move = 0;
				out128<<"修正"<<endl;
				MoveWay(lanepgps,realtime_Dir,rndfbezier,pianyi);
			//	app->secondDecison = "直道修正车道";
			}
		}
	}

	int num = lanedriverstate.GetMinNum(rndfbezier,200,realtime_Gps);
	lanechang_dis = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,ob_tempoint.x,ob_tempoint.y);
	//
	//lanechang_dis = LevelDist(rndfbezier[num],gps_aimdir,realtime_Gps);
//	lanechang_dis = LevelDist(realtime_Gps,realtime_Dir,rndfbezier[num]);
	if((abs(gps_aimdir - realtime_Dir) < 7 || abs(360 - abs(gps_aimdir - realtime_Dir)) < 7) && abs(lanechang_dis )>10&&!lanechang_flag)
	{
		tongji++;
		if (tongji > 10)
		{
			lanechang_flag = true;
			app->left_right = 0;
			tongji = 0;
		}
	}
	//app->secondDecison = "直道不修正车道";
	if(lanechang_flag)
	{
			
			app->drive_obstacle = max_speed;
			int ob_nixing = lanedriverstate.SearchNiXing(rndfbezier);
			if(ob_nixing == 1||ob_nixing == -1)
			{
				app->left_right = 1;
				{
					ob_tempoint = cvPoint2D64f(realtime_Gps.x,realtime_Gps.y);		
					MoveLeft(rndfbezier,rndfbezier,-3.7*ob_nixing,pianyi);
					lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,40,gps_aimpoint);
					aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
					lanechang_flag = false;
					xz_flag = false;
				}
			}
		
			
	}///
	bool ob_wanmiddle = false;
	bool ob_wanleft = false;
	bool ob_wanright = false;
	CvPoint2D64f middle[200] = {0};
	CvPoint2D64f left[200] = {0};
	CvPoint2D64f right[200] = {0};
	MoveLeft(rndfbezier, left, 3.7);
	MoveLeft(rndfbezier,right,-3.7);
	ob_wanmiddle = lanedriverstate.SearchObstacle(rndfbezier,vel_Map,8,8,8,40,0);
	ob_wanleft = lanedriverstate.SearchObstacle(left,vel_Map,8,8,8,30,0);
	ob_wanright = lanedriverstate.SearchObstacle(right,vel_Map,8,8,8,30,0);
	if (ob_wanmiddle/*||(ob_wanleft&&ob_wanright)*/)
	{
		app->drive_obstacle = obstacle_speed;
	}
	
	lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,40,gps_aimpoint);
	aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
	return 1;
}

int CVehicleExe::LaneInit()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	control.setLight(LAMP_OFF);
	app->on_road = 1;
	app->topDecison = "路上";
	pianyi = cvPoint2D64f(0,0);
	Setspeed(road_speed,road_speed);
	return 1;
}

int CVehicleExe::SearchObatAppro(int up,int down,CvPoint2D64f *temp,CvPoint2D64f m_gps,double dir,int left_right)
{ 
	int count = 0;

	CvPoint2D64f upoint,dpoint,mapup,mapdown;
	lanedriverstate.GetPointfromNearGPS(temp,200,m_gps,up,upoint);
	lanedriverstate.GetPointfromNearGPS(temp,200,m_gps,down,dpoint);
	mapup = m_GpsData.APiontConverD(m_gps,upoint,dir);
	mapdown = m_GpsData.APiontConverD(m_gps,dpoint,dir);

	for (int i = mapup.y;i < mapdown.y;i++)
	{
		for (int j =-6; j<7;j++)
		{
		
			int y = i ;
			int x = mapup.x + (i-mapup.y)*(mapup.x - mapdown.x)/(mapup.y - mapdown.y);
			if(y< 0||y > 511||x+j > 511||x+j < 0)
				continue;
			if (vel_Map->MapPoint[y][x + j] == 8 || vel_Map->MapPoint[y][x + j] == 18 )
			{
				
				count++;
				if(count>25)
					return 0;
				break;

			}

		}

	}

	for (int i = mapup.y;i < mapdown.y;i++)
	{
		////////右转
		if(left_right == 1)
		{
			for (int j =10; j<20;j++)
			{
			
				int y = mapup.y + i;
				int x = mapup.x + i*(mapup.x - mapdown.x)/(mapup.y - mapdown.y);
				if(y< 0||y > 511||x+j > 511||x+j < 0)
					continue;
				if (vel_Map->MapPoint[y][x + j] == 8 || vel_Map->MapPoint[y][x + j] == 18 )
				{		
					count++;
					if(count>25)
						return 1;
					break;
				}

			}
		}
		////////左转
		if(left_right == -1)
		{
			for (int j =-10; j>-20;j--)
			{
			
				int y = mapup.y + i;
				int x = mapup.x + i*(mapup.x - mapdown.x)/(mapup.y - mapdown.y);
				if(y< 0||y > 511||x+j > 511||x+j < 0)
					continue;
				if (vel_Map->MapPoint[y][x + j] == 8 || vel_Map->MapPoint[y][x + j] == 18 )
				{		
					count++;
					if(count>20)
						return 1;
					break;
				}

			}
		}

	}
	return 2;
}

void CVehicleExe::ClearObject()
{
	for (int m1= 0 ;m1<200;m1++)//循环解析x[1]个物体
	{
		ObjectWT1[m1].Object_Id = 0;
		ObjectWT1[m1].Object_Age = 0;
		ObjectWT1[m1].ObjectBox_CenterX = 0;
		ObjectWT1[m1].ObjectBox_CenterY = 0;
		ObjectWT1[m1].ObjectBox_SizeX = 0;
		ObjectWT1[m1].ObjectBox_SizeY = 0;
		ObjectWT1[m1].ObjectBox_orientation = 0;
		ObjectWT1[m1].Relative_VelocityX = 0;
		ObjectWT1[m1].Relative_VelocityY = 0;
		ObjectWT1[m1].iContour_Points = 0;
		ObjectWT1[m1].danger = 0;

		for (int n1=0 ;n1<300;n1++)//循环解析x[1]个物体
		{
			ObjectWT1[m1].iContour_X[n1] = 0;
			ObjectWT1[m1].iContour_Y[n1] = 0;
		}
	}
}

void CVehicleExe::ClearObject2()
{
	for (int m1= 0 ;m1<200;m1++)//循环解析x[1]个物体
	{
		ObjectWT2[m1].Object_Id = 0;
		ObjectWT2[m1].Object_Age = 0;
		ObjectWT2[m1].ObjectBox_CenterX = 0;
		ObjectWT2[m1].ObjectBox_CenterY = 0;
		ObjectWT2[m1].ObjectBox_SizeX = 0;
		ObjectWT2[m1].ObjectBox_SizeY = 0;
		ObjectWT2[m1].ObjectBox_orientation = 0;
		ObjectWT2[m1].Relative_VelocityX = 0;
		ObjectWT2[m1].Relative_VelocityY = 0;
		ObjectWT2[m1].iContour_Points = 0;
		ObjectWT2[m1].danger = 0;

		for (int n1=0 ;n1<300;n1++)//循环解析x[1]个物体
		{
			ObjectWT2[m1].iContour_X[n1] = 0;
			ObjectWT2[m1].iContour_Y[n1] = 0;
		}
	}
}


//void CVehicleExe::StartIbeo()               //开启ibeo的线程
//{
//	if(!VelListenned)
//	{
//		m_hIbeoThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theIbeoThread,
//			this, 0, &dwIbeoThreadId);
//		VelListenned=true;
//	}
//	else
//	{
//		AfxMessageBox("Winsock已在连接中!");
//	}
//	/*m_hThreadProc = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theIbeoThread,
//	this, 0, &dwIbeoThreadId);*/
//}
//
//DWORD CVehicleExe::theIbeoThread(LPVOID lpParam)
//{
//	return (((CVehicleExe*)lpParam)->IbeoThread());
//}
//
//DWORD CVehicleExe::IbeoThread()
//{
//
//	
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	SOCKET TestClient;   
//	WSADATA TNetData;
//	sockaddr_in local;
//
//	CString strIpAddress="192.168.0.2";
//	int wsaret = WSAStartup( MAKEWORD(2,2), &TNetData );
//	if( wsaret != 0 )
//	{
//		return 0;
//	}
//
//	local.sin_family = AF_INET; //Address family
//	local.sin_addr.s_addr = inet_addr( strIpAddress ); //填充服务器的ip地址和端口号
//	local.sin_port = htons((u_short)12002); // 端口号
//
//	TestClient = socket( AF_INET, SOCK_STREAM, 0 );	//建立一个tcp socket
//	if( TestClient == INVALID_SOCKET )
//	{
//		return 0;
//	}
//
//	if(connect( TestClient,(struct sockaddr *)&local, sizeof(local))==SOCKET_ERROR)//接受请求后，实际同客户端socket进行交互的SOCKET client
//	{
//		SuccSocket=false;
//		//AfxMessageBox("SOCKET连接错误!");
//		return 0;
//	}
//	else
//	{
//		SuccSocket=true;
//	} 
//
//	char temp[20000]={0};//temp为带符号显示char
//	unsigned char ubuf[20000]={0};//buf为无符号显示char
//	int x[200]={0};
//	int L1[200]={0};
//	CString S1[30]={""};
//
//	while(1)
//	{
//		if(SuccSocket)//连接成功 开始接收LUX数据
//		{
//Reset1 :memset(temp,0,20000);//出错重新开始接收、赋值。
//			memset(ubuf,0,20000);
//
//			int iLen = recv( TestClient,temp,20000,0);//获取数据
//			if (iLen<0)
//			{ 
//				AfxMessageBox("LUX 连接错误!");
//				return 0;
//			}
//			memcpy((void*)ubuf,(void*)temp,iLen);//拷贝到unsigned char
//
//			//for (int index_i=0; index_i<20000; index_i++)  //将 所有的数据记录下来
//			//{
//			//	ibeo_data<<ubuf[index_i]<<endl;
//			//}
//
//
//			for (int i=0;i<10000;i++)//整个搜索一遍?
//			{
//				if (i>19910) break;//不够一个消息（至少90个字节），退出循环。
//				S1[1].Format("%X",ubuf[i]);//AF
//				S1[2].Format("%X",ubuf[i+1]);//FE
//				S1[3].Format("%X",ubuf[i+2]);//C0
//				S1[4].Format("%X",ubuf[i+3]);//C2
//				S1[5].Format("%X",ubuf[i+14]);//22
//				S1[6].Format("%X",ubuf[i+15]);//21//Winsock获取Object data
//
//				if (S1[1]=="AF" && S1[2]=="FE" && S1[3]=="C0" && S1[4]=="C2" && S1[5]=="22" && S1[6]=="21")//i循环到AF位
//				{
//					x[0]=ubuf[i+11]+ubuf[i+10]*256+ubuf[i+9]*65536+ubuf[i+8]*16777216;//Uint32 本次消息大小
//					if (x[0]>iLen) break;//接收到的字节小于消息长度则抛弃
//					x[1]=ubuf[i+32] + ubuf[i+33]*256;//Object数量
//
//
//					app->critical_winsockread.Lock();     //线程同步
//					if (x[1]>199) x[1]=198;//只接收199个物体
//					iObject1 = x[1];//物体数量
//					ClearObject();//复位结构体
//					//ObjectWT1[200]={0};
//					//memset(ObjectWT1, 0, 200);      //复位结构体  by h 2013 6.24
//
//					for (int j1=0 ;j1<x[1];j1++)//循环解析x[1]个物体
//					{
//						int d3=0;
//						for (int j2=j1;j2>0;j2--)
//						{
//							d3=d3+L1[j2];
//							if (i+34+58*j1+d3*4>19999)
//							{
//								goto Reset1;//出错重新开始接收、赋值。
//							}
//						}
//
//						ObjectWT1[j1].Object_Id = ((unsigned int)ubuf[i+35+58*j1+d3*4] << 8) | ubuf[i+34+58*j1+d3*4];
//						ObjectWT1[j1].Object_Age = ((unsigned int)ubuf[i+37+58*j1+d3*4] << 8) | ubuf[i+36+58*j1+d3*4];
//
//						//ObjectWT1[j1].ObjectBox_CenterX = ((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4];
//						//ObjectWT1[j1].ObjectBox_CenterY = ((int)temp[i+65+58*j1+d3*4] << 8) | ubuf[i+64+58*j1+d3*4];
//
//						//ASSERT(ObjectWT1[j1].Object_Age!=0);
//						ObjectWT1[j1].ObjectBox_CenterX = 256+(((int)temp[i+65+58*j1+d3*4] << 8) | ubuf[i+64+58*j1+d3*4])/20;
//						ObjectWT1[j1].ObjectBox_CenterY = 412+9+(((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4])/20;
//						ObjectWT1[j1].ObjectBox_SizeX = ((unsigned int)ubuf[i+67+58*j1+d3*4] << 8) | ubuf[i+66+58*j1+d3*4];
//						ObjectWT1[j1].ObjectBox_SizeY = ((unsigned int)ubuf[i+69+58*j1+d3*4] << 8) | ubuf[i+68+58*j1+d3*4];
//						int t=(((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4])/20;
//						ASSERT(ObjectWT1[j1].ObjectBox_CenterY>=412);
//						      //(((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4])/20;
//
//						short ObjectBox_orientation_Ori = ((int)temp[i+71+58*j1+d3*4] << 8) | ubuf[i+70+58*j1+d3*4];
//						ObjectWT2[j1].ObjectBox_orientation = ObjectBox_orientation_Ori * 0.01;//度数
//
//						short Relative_VelocityX_Ori = ((int)temp[i+81+58*j1+d3*4] << 8) | ubuf[i+80+58*j1+d3*4];
//						ObjectWT1[j1].Relative_VelocityX = Relative_VelocityX_Ori * 0.036;//千米/时
//						//if (ObjectWT1[j1].Relative_VelocityX < 1.0) ObjectWT1[j1].Relative_VelocityX = 0.0;
//
//						short Relative_VelocityY_Ori = ((int)temp[i+83+58*j1+d3*4] << 8) | ubuf[i+82+58*j1+d3*4];
//						ObjectWT1[j1].Relative_VelocityY = Relative_VelocityY_Ori * 0.036;//千米/时
//						//if (ObjectWT1[j1].Relative_VelocityY < 1.0) ObjectWT1[j1].Relative_VelocityY = 0.0;
//
//
//						ObjectWT1[j1].Classification = ((unsigned int)ubuf[i+85+58*j1+d3*4] << 8) | ubuf[i+84+58*j1+d3*4];
//						ObjectWT1[j1].Classification_Age = ((unsigned int)ubuf[i+87+58*j1+d3*4] << 8) | ubuf[i+86+58*j1+d3*4];
//						ObjectWT1[j1].Classification_Certainty = ((unsigned int)ubuf[i+89+58*j1+d3*4] << 8) | ubuf[i+88+58*j1+d3*4];
//
//
//						ObjectWT1[j1].iContour_Points = ((unsigned int)ubuf[i+91+58*j1+d3*4] << 8) | ubuf[i+90+58*j1+d3*4];
//						L1[j1+1] = ObjectWT2[j1].iContour_Points;//轮廓点数量
//
//						for (int k1=0 ;k1 < L1[j1+1]-1;k1++)//循环解析轮廓点
//						{
//							if (k1 > 299 )
//							{
//								continue;
//							}
//
//							//ObjectWT1[j1].iContour_X[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+2] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+2];
//							//ObjectWT1[j1].iContour_Y[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+4] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+4];
//
//							ObjectWT1[j1].iContour_X[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+4] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+4];
//							ObjectWT1[j1].iContour_Y[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+2] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+2];
//
//						}
//					}//多个物体解析结束
//					app->critical_winsockread.Unlock();
//
//
//				}//if 帧头判断结束
//				break;//退出20000数组的搜索，也可以继续查找。
//			}//20000数组搜寻结束
//		}//连接成功，接收数据
//		else//连接不成功，函数返回
//		{
//			return 0;
//		}
//
//
//
//	}//while(1)循环
//	return 0;
//}

void CVehicleExe::StartUdpSent()
{
	WSADATA TNetData;
	int wsaret = WSAStartup( MAKEWORD(2,2), &TNetData );
	if( wsaret != 0 )
	{
		AfxMessageBox("UDP start up error!!!");
		return;
	}

	if(LOBYTE(TNetData.wVersion)!=2 || HIBYTE(TNetData.wVersion)!=2)
	{
		WSACleanup();
		AfxMessageBox("UDP version error!!!");
		return;
	}

	//SendToContrlSocket=socket(AF_INET,SOCK_DGRAM,0);
	ControlAddrSrv.sin_family = AF_INET;
	ControlAddrSrv.sin_addr.s_addr = inet_addr( "192.168.3.200" );//"192.168.0.200"
	ControlAddrSrv.sin_port = htons((u_short)7665);

	SendToContrlSocket=socket(AF_INET,SOCK_DGRAM,0);

	RcvFromControlAddrSrv.sin_family = AF_INET;
	RcvFromControlAddrSrv.sin_addr.s_addr = inet_addr( "192.168.3.100" );//"192.168.0.20"
	RcvFromControlAddrSrv.sin_port = htons((u_short)8008);

	RcvFromContrlSocket=socket(AF_INET,SOCK_DGRAM,0);
	bind(RcvFromContrlSocket, (SOCKADDR*)&RcvFromControlAddrSrv,sizeof(SOCKADDR));
}

void CVehicleExe::StartUdpRcv()               //开启ibeo的线程
{
	m_hUdpRcvThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theUdpRcvThread,
		this, 0, &dwUdpRcvThreadId);
}

DWORD CVehicleExe::theUdpRcvThread(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->UdpRcvThread());
}

DWORD CVehicleExe::UdpRcvThread()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	while(1)
	{
		int len=sizeof(sockaddr);
		char rcvbuf[2000];
		//recvfrom(SendToContrlSocket,(char *)(&(control.udpdatarcv)),sizeof(control.udpdatarcv),0,(sockaddr*)&ControlAddrSrv,&len);
		recvfrom(RcvFromContrlSocket,rcvbuf,2000,0,(sockaddr*)&RcvFromControlAddrSrv,&len);
		memcpy((char *)(&(control.udpdatarcv)),rcvbuf,sizeof(control.udpdatarcv));

/*
		for(int i=0;i<sizeof(control.udpdatarcv);i++)
			outudprcvdata<<(unsigned int)*(unsigned char*)(&(rcvbuf[i]))<<",";
		outudprcvdata<<endl;
		*/

		outudprcvdata<<control.udpdatarcv.realSpeed<<",  "<<control.udpdatarcv.realSteerAngle<<",  "<<control.udpdatarcv.VehEmergencyStop<<",  "<<control.udpdatarcv.offset_l<<",  "<<control.udpdatarcv.offset_r<<",  "<<control.udpdatarcv.ApaRetInfo<<",  "<<len<<endl;
	}
	return 0;
}

void CVehicleExe::StartIbeo()               //开启ibeo的线程
{
	if(!VelListenned)
	{
		m_hIbeoThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theIbeoThread,
			this, 0, &dwIbeoThreadId);
		VelListenned=true;
	}
	else
	{
		AfxMessageBox("Winsock已在连接中!");
	}
	/*m_hThreadProc = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theIbeoThread,
	this, 0, &dwIbeoThreadId);*/
}

DWORD CVehicleExe::theIbeoThread(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->IbeoThread());
}

DWORD CVehicleExe::IbeoThread()
{

	//AfxMessageBox("hellow");
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	SOCKET TestClient;   
	WSADATA TNetData;
	sockaddr_in local;

	CString strIpAddress="192.168.0.2";
	int wsaret = WSAStartup( MAKEWORD(2,2), &TNetData );
	if( wsaret != 0 )
	{
		return 0;
	}

	local.sin_family = AF_INET; //Address family
	local.sin_addr.s_addr = inet_addr( strIpAddress ); //填充服务器的ip地址和端口号
	local.sin_port = htons((u_short)12002); // 端口号

	TestClient = socket( AF_INET, SOCK_STREAM, 0 );	//建立一个tcp socket
	if( TestClient == INVALID_SOCKET )
	{
		return 0;
	}

	if(connect( TestClient,(struct sockaddr *)&local, sizeof(local))==SOCKET_ERROR)//接受请求后，实际同客户端socket进行交互的SOCKET client
	{
		SuccSocket=false;
		//AfxMessageBox("SOCKET连接错误!");
		return 0;
	}
	else
	{
		SuccSocket=true;
	} 

	char temp[20000]={0};//temp为带符号显示char
	unsigned char ubuf[20000]={0};//buf为无符号显示char
	int x[200]={0};
	int L1[200]={0};
	CString S1[30]={""};

	while(1)
	{
		if(SuccSocket)//连接成功 开始接收LUX数据
		{
Reset1 :memset(temp,0,20000);//出错重新开始接收、赋值。
			memset(ubuf,0,20000);

			int iLen = recv( TestClient,temp,20000,0);//获取数据
			if (iLen<0)
			{ 
				AfxMessageBox("LUX 连接错误!");
				return 0;
			}
			memcpy((void*)ubuf,(void*)temp,iLen);//拷贝到unsigned char

			//for (int index_i=0; index_i<20000; index_i++)  //将 所有的数据记录下来
			//{
			//	ibeo_data<<ubuf[index_i]<<endl;
			//}


			for (int i=0;i<10000;i++)//整个搜索一遍?
			{
				if (i>19910) break;//不够一个消息（至少90个字节），退出循环。
				S1[1].Format("%X",ubuf[i]);//AF
				S1[2].Format("%X",ubuf[i+1]);//FE
				S1[3].Format("%X",ubuf[i+2]);//C0
				S1[4].Format("%X",ubuf[i+3]);//C2
				S1[5].Format("%X",ubuf[i+14]);//22
				S1[6].Format("%X",ubuf[i+15]);//21//Winsock获取Object data

				if (S1[1]=="AF" && S1[2]=="FE" && S1[3]=="C0" && S1[4]=="C2" && S1[5]=="22" && S1[6]=="21")//i循环到AF位
				{
					x[0]=ubuf[i+11]+ubuf[i+10]*256+ubuf[i+9]*65536+ubuf[i+8]*16777216;//Uint32 本次消息大小
					if (x[0]>iLen) break;//接收到的字节小于消息长度则抛弃
					x[1]=ubuf[i+32] + ubuf[i+33]*256;//Object数量


					
					if (x[1]>199) x[1]=198;//只接收199个物体
					iObject1 = x[1];//物体数量
					ClearObject();//复位结构体
					//ObjectWT1[200]={0};
					//memset(ObjectWT1, 0, 200);      //复位结构体  by h 2013 6.24

					for (int j1=0 ;j1<x[1];j1++)//循环解析x[1]个物体
					{
						int d3=0;
						for (int j2=j1;j2>0;j2--)
						{
							d3=d3+L1[j2];
							if (i+34+58*j1+d3*4>19999)
							{
								goto Reset1;//出错重新开始接收、赋值。
							}
						}

						ObjectWT1[j1].Object_Id = ((unsigned int)ubuf[i+35+58*j1+d3*4] << 8) | ubuf[i+34+58*j1+d3*4];
						ObjectWT1[j1].Object_Age = ((unsigned int)ubuf[i+37+58*j1+d3*4] << 8) | ubuf[i+36+58*j1+d3*4];

						//ObjectWT1[j1].ObjectBox_CenterX = ((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4];
						//ObjectWT1[j1].ObjectBox_CenterY = ((int)temp[i+65+58*j1+d3*4] << 8) | ubuf[i+64+58*j1+d3*4];

						ASSERT(ObjectWT1[j1].Object_Age!=0);
						ObjectWT1[j1].ObjectBox_CenterX = 256+(((int)temp[i+65+58*j1+d3*4] << 8) | ubuf[i+64+58*j1+d3*4])/20;
						ObjectWT1[j1].ObjectBox_CenterY = 412+3+((((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4]))/50;
						ObjectWT1[j1].ObjectBox_SizeX = ((unsigned int)ubuf[i+67+58*j1+d3*4] << 8) | ubuf[i+66+58*j1+d3*4];
						ObjectWT1[j1].ObjectBox_SizeY = ((unsigned int)ubuf[i+69+58*j1+d3*4] << 8) | ubuf[i+68+58*j1+d3*4];

						short ObjectBox_orientation_Ori = ((int)temp[i+71+58*j1+d3*4] << 8) | ubuf[i+70+58*j1+d3*4];
						ObjectWT1[j1].ObjectBox_orientation = ObjectBox_orientation_Ori * 0.01;//度数

						short Relative_VelocityX_Ori = ((int)temp[i+81+58*j1+d3*4] << 8) | ubuf[i+80+58*j1+d3*4];
						ObjectWT1[j1].Relative_VelocityX = -Relative_VelocityX_Ori * 0.01;//m/s
						//if (ObjectWT1[j1].Relative_VelocityX < 1.0) ObjectWT1[j1].Relative_VelocityX = 0.0;

						short Relative_VelocityY_Ori = ((int)temp[i+83+58*j1+d3*4] << 8) | ubuf[i+82+58*j1+d3*4];
						ObjectWT1[j1].Relative_VelocityY = -Relative_VelocityY_Ori * 0.01;//m/s
						//if (ObjectWT1[j1].Relative_VelocityY < 1.0) ObjectWT1[j1].Relative_VelocityY = 0.0;


						ObjectWT1[j1].Classification = ((unsigned int)ubuf[i+85+58*j1+d3*4] << 8) | ubuf[i+84+58*j1+d3*4];
						ObjectWT1[j1].Classification_Age = ((unsigned int)ubuf[i+87+58*j1+d3*4] << 8) | ubuf[i+86+58*j1+d3*4];
						ObjectWT1[j1].Classification_Certainty = ((unsigned int)ubuf[i+89+58*j1+d3*4] << 8) | ubuf[i+88+58*j1+d3*4];


						ObjectWT1[j1].iContour_Points = ((unsigned int)ubuf[i+91+58*j1+d3*4] << 8) | ubuf[i+90+58*j1+d3*4];
						L1[j1+1] = ObjectWT1[j1].iContour_Points;//轮廓点数量

						for (int k1=0 ;k1 < L1[j1+1]-1;k1++)//循环解析轮廓点
						{
							if (k1 > 299 )
							{
								continue;
							}

							//ObjectWT1[j1].iContour_X[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+2] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+2];
							//ObjectWT1[j1].iContour_Y[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+4] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+4];

							ObjectWT1[j1].iContour_X[k1] = 256+(((int)temp[i+91+58*j1+d3*4+k1*4+4] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+4])/20;
							ObjectWT1[j1].iContour_Y[k1] = 412+3+((((int)temp[i+91+58*j1+d3*4+k1*4+2] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+2]))/50;

						}
					}//多个物体解析结束
					app->critical_winsockread.Lock();     //线程同步

					iObject1test7=iObject1test6;
					for(int ii=0; ii<iObject1test7; ii++)
					{
						ObjectWT1test7[ii]=ObjectWT1test6[ii];
					}

					iObject1test6=iObject1test5;
					for(int ii=0; ii<iObject1test6; ii++)
					{
						ObjectWT1test6[ii]=ObjectWT1test5[ii];
					}

					iObject1test5=iObject1test4;
					for(int ii=0; ii<iObject1test5; ii++)
					{
						ObjectWT1test5[ii]=ObjectWT1test4[ii];
					}

					iObject1test4=iObject1test3;
					for(int ii=0; ii<iObject1test4; ii++)
					{
						ObjectWT1test4[ii]=ObjectWT1test3[ii];
					}


					iObject1test3=iObject1test2;
					for(int ii=0; ii<iObject1test3; ii++)
					{
						ObjectWT1test3[ii]=ObjectWT1test2[ii];
					}
					

					iObject1test2=iObject1test1;
					for(int ii=0; ii<iObject1test2; ii++)
					{
						ObjectWT1test2[ii]=ObjectWT1test1[ii];
					}
					

					iObject1test1=iObject1;
					for(int ii=0; ii<iObject1test1; ii++)
					{
						ObjectWT1test1[ii]=ObjectWT1[ii];
					}
					app->critical_winsockread.Unlock();
					
					//SetEvent(app->m_Ibeo1Event);
					
					//Sleep(20);


				}//if 帧头判断结束
				break;//退出20000数组的搜索，也可以继续查找。
			}//20000数组搜寻结束
		}//连接成功，接收数据
		else//连接不成功，函数返回
		{
			return 0;
		}



	}//while(1)循环
	return 0;
}





void CVehicleExe::StartIbeo2()               //开启ibeo的线程
{
	if(!VelListenned2)
	{
		m_hIbeoThread2 = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theIbeoThread2,
			this, 0, &dwIbeoThreadId2);
		VelListenned2=true;
	}
	else
	{
		AfxMessageBox("Winsock已在连接中!");
	}
	/*m_hThreadProc = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theIbeoThread,
	this, 0, &dwIbeoThreadId);*/
}

DWORD CVehicleExe::theIbeoThread2(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->IbeoThread2());
}

DWORD CVehicleExe::IbeoThread2()
{

	//AfxMessageBox("hellow");
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	SOCKET TestClient;   
	WSADATA TNetData;
	sockaddr_in local;

	CString strIpAddress="192.168.0.1";
	int wsaret = WSAStartup( MAKEWORD(2,2), &TNetData );
	if( wsaret != 0 )
	{
		return 0;
	}

	local.sin_family = AF_INET; //Address family
	local.sin_addr.s_addr = inet_addr( strIpAddress ); //填充服务器的ip地址和端口号
	local.sin_port = htons((u_short)12002); // 端口号

	TestClient = socket( AF_INET, SOCK_STREAM, 0 );	//建立一个tcp socket
	if( TestClient == INVALID_SOCKET )
	{
		return 0;
	}

	if(connect( TestClient,(struct sockaddr *)&local, sizeof(local))==SOCKET_ERROR)//接受请求后，实际同客户端socket进行交互的SOCKET client
	{
		SuccSocket2=false;
		//AfxMessageBox("SOCKET连接错误!");
		return 0;
	}
	else
	{
		SuccSocket2=true;
	} 

	char temp[20000]={0};//temp为带符号显示char
	unsigned char ubuf[20000]={0};//buf为无符号显示char
	int x[200]={0};
	int L1[200]={0};
	CString S1[30]={""};

	while(1)
	{
		if(SuccSocket2)//连接成功 开始接收LUX数据
		{
Reset1 :memset(temp,0,20000);//出错重新开始接收、赋值。
			memset(ubuf,0,20000);

			int iLen = recv( TestClient,temp,20000,0);//获取数据
			if (iLen<0)
			{ 
				AfxMessageBox("LUX 连接错误!");
				return 0;
			}
			memcpy((void*)ubuf,(void*)temp,iLen);//拷贝到unsigned char

			//for (int index_i=0; index_i<20000; index_i++)  //将 所有的数据记录下来
			//{
			//	ibeo_data<<ubuf[index_i]<<endl;
			//}


			for (int i=0;i<10000;i++)//整个搜索一遍?
			{
				if (i>19910) break;//不够一个消息（至少90个字节），退出循环。
				S1[1].Format("%X",ubuf[i]);//AF
				S1[2].Format("%X",ubuf[i+1]);//FE
				S1[3].Format("%X",ubuf[i+2]);//C0
				S1[4].Format("%X",ubuf[i+3]);//C2
				S1[5].Format("%X",ubuf[i+14]);//22
				S1[6].Format("%X",ubuf[i+15]);//21//Winsock获取Object data

				if (S1[1]=="AF" && S1[2]=="FE" && S1[3]=="C0" && S1[4]=="C2" && S1[5]=="22" && S1[6]=="21")//i循环到AF位
				{
					
					x[0]=ubuf[i+11]+ubuf[i+10]*256+ubuf[i+9]*65536+ubuf[i+8]*16777216;//Uint32 本次消息大小
					if (x[0]>iLen) break;//接收到的字节小于消息长度则抛弃
					x[1]=ubuf[i+32] + ubuf[i+33]*256;//Object数量


					
					if (x[1]>199) x[1]=198;//只接收199个物体
					iObject2 = x[1];//物体数量
					ClearObject2();//复位结构体
					//ObjectWT1[200]={0};
					//memset(ObjectWT1, 0, 200);      //复位结构体  by h 2013 6.24

					for (int j1=0 ;j1<x[1];j1++)//循环解析x[1]个物体
					{
						int d3=0;
						for (int j2=j1;j2>0;j2--)
						{
							d3=d3+L1[j2];
							if (i+34+58*j1+d3*4>19999)
							{
								goto Reset1;//出错重新开始接收、赋值。
							}
						}

						ObjectWT2[j1].Object_Id = ((unsigned int)ubuf[i+35+58*j1+d3*4] << 8) | ubuf[i+34+58*j1+d3*4];
						ObjectWT2[j1].Object_Age = ((unsigned int)ubuf[i+37+58*j1+d3*4] << 8) | ubuf[i+36+58*j1+d3*4];

						//ObjectWT1[j1].ObjectBox_CenterX = ((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4];
						//ObjectWT1[j1].ObjectBox_CenterY = ((int)temp[i+65+58*j1+d3*4] << 8) | ubuf[i+64+58*j1+d3*4];

						ASSERT(ObjectWT2[j1].Object_Age!=0);
						ObjectWT2[j1].ObjectBox_CenterX = 256-(((int)temp[i+65+58*j1+d3*4] << 8) | ubuf[i+64+58*j1+d3*4])/20;
						ObjectWT2[j1].ObjectBox_CenterY = 412-12-(((int)temp[i+63+58*j1+d3*4] << 8) | ubuf[i+62+58*j1+d3*4])/20;
						ASSERT(ObjectWT2[j1].ObjectBox_CenterY<=412);
						ObjectWT2[j1].ObjectBox_SizeX = ((unsigned int)ubuf[i+67+58*j1+d3*4] << 8) | ubuf[i+66+58*j1+d3*4];
						ObjectWT2[j1].ObjectBox_SizeY = ((unsigned int)ubuf[i+69+58*j1+d3*4] << 8) | ubuf[i+68+58*j1+d3*4];

						short ObjectBox_orientation_Ori = ((int)temp[i+71+58*j1+d3*4] << 8) | ubuf[i+70+58*j1+d3*4];
						ObjectWT2[j1].ObjectBox_orientation = ObjectBox_orientation_Ori * 0.01;//度数

						short Relative_VelocityX_Ori = ((int)temp[i+81+58*j1+d3*4] << 8) | ubuf[i+80+58*j1+d3*4];
						ObjectWT2[j1].Relative_VelocityX = Relative_VelocityX_Ori * 0.01;//m/s
						//if (ObjectWT1[j1].Relative_VelocityX < 1.0) ObjectWT1[j1].Relative_VelocityX = 0.0;

						short Relative_VelocityY_Ori = ((int)temp[i+83+58*j1+d3*4] << 8) | ubuf[i+82+58*j1+d3*4];
						ObjectWT2[j1].Relative_VelocityY = Relative_VelocityY_Ori * 0.01;//m/s
						//if (ObjectWT1[j1].Relative_VelocityY < 1.0) ObjectWT1[j1].Relative_VelocityY = 0.0;


						ObjectWT2[j1].Classification = ((unsigned int)ubuf[i+85+58*j1+d3*4] << 8) | ubuf[i+84+58*j1+d3*4];
						ObjectWT2[j1].Classification_Age = ((unsigned int)ubuf[i+87+58*j1+d3*4] << 8) | ubuf[i+86+58*j1+d3*4];
						ObjectWT2[j1].Classification_Certainty = ((unsigned int)ubuf[i+89+58*j1+d3*4] << 8) | ubuf[i+88+58*j1+d3*4];


						ObjectWT2[j1].iContour_Points = ((unsigned int)ubuf[i+91+58*j1+d3*4] << 8) | ubuf[i+90+58*j1+d3*4];
						L1[j1+1] = ObjectWT2[j1].iContour_Points;//轮廓点数量

						for (int k1=0 ;k1 < L1[j1+1]-1;k1++)//循环解析轮廓点
						{
							if (k1 > 299 )
							{
								continue;
							}

							//ObjectWT1[j1].iContour_X[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+2] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+2];
							//ObjectWT1[j1].iContour_Y[k1] = ((int)temp[i+91+58*j1+d3*4+k1*4+4] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+4];

							ObjectWT2[j1].iContour_X[k1] = 256-(((int)temp[i+91+58*j1+d3*4+k1*4+4] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+4])/20;
							ObjectWT2[j1].iContour_Y[k1] = 412-12-(((int)temp[i+91+58*j1+d3*4+k1*4+2] << 8) | ubuf[i+90+58*j1+d3*4+k1*4+2])/20;

						}
					}//多个物体解析结束
					app->critical_winsockread2.Lock();     //线程同步

					iObject2test7=iObject2test6;
					for(int ii=0; ii<iObject2test6; ii++)
					{
						ObjectWT2test7[ii]=ObjectWT2test6[ii];
					}
	
					iObject2test6=iObject2test5;
					for(int ii=0; ii<iObject2test5; ii++)
					{
						ObjectWT2test6[ii]=ObjectWT2test5[ii];
					}

					iObject2test5=iObject2test4;
					for(int ii=0; ii<iObject2test4; ii++)
					{
						ObjectWT2test5[ii]=ObjectWT2test4[ii];
					}

					iObject2test4=iObject2test3;
					for(int ii=0; ii<iObject2test4; ii++)
					{
						ObjectWT2test4[ii]=ObjectWT2test3[ii];
					}

					iObject2test3=iObject2test2;
					for(int ii=0; ii<iObject2test3; ii++)
					{
						ObjectWT2test3[ii]=ObjectWT2test2[ii];
					}
					

					iObject2test2=iObject2test1;
					for(int ii=0; ii<iObject2test2; ii++)
					{
						ObjectWT2test2[ii]=ObjectWT2test1[ii];
					}
					

					iObject2test1=iObject2;
					for(int ii=0; ii<iObject2test1; ii++)
					{
						ObjectWT2test1[ii]=ObjectWT2[ii];
					}
					app->critical_winsockread2.Unlock();
						//SetEvent(app->m_Ibeo2Event);
					//Sleep(20);


				}//if 帧头判断结束
				break;//退出20000数组的搜索，也可以继续查找。
			}//20000数组搜寻结束
		}//连接成功，接收数据
		else//连接不成功，函数返回
		{
			return 0;
		}



	}//while(1)循环
	return 0;
}
//bool CVehicleExe::solvesegment_num1(double x,double y,double dir)//根据无人车当前GPS坐标求其所在车道号
//{
//	double distance;
//	double direction;//车子与路点连线的方向
//	double direction1;//车道的方向
//	double direction2;
//	double angle;//车道方向与车航向的夹角
//	double angle1;//车道方向与(车子和路点连线)的方向的夹角
//	double angle2;
//	double angleex;
//	double angle1ex;
//	double angle2ex;
//	//int lanepointnum;
//	double mindis = I;
//	MAPPOINT *nearpoint = NULL;
//	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)
//	{
//		for(int j=0;j<2;j++)
//		{
//			//lanepointnum=RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;
//			for(int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
//			{
//				distance = sqrt(pow((x - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x)),2) + pow((y - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y)),2));	
//				
//				if (k!=(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num-1))
//				{
//					direction = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y,x,y);
//					direction1 = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k+1].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k+1].y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y);
//					if(direction<0)
//					{
//						direction = direction + 360;
//					}
//					if(direction1<0)
//					{
//						direction1 = direction1 + 360;
//					}
//					if(dir>=360)
//					{
//						dir = dir-360;
//					}
//					angle=fabs(direction1-dir);
//					angleex = 360 - angle;
//
//					angle1=fabs(direction1-direction);
//					angle1ex = 360 - angle1;
//
//					angle2=91;
//					angle2ex = 91;
//				}
//				else if (k==(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num-1))
//				{
//					direction = m_GpsData.GetAngle(x,y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y);
//					direction1 = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].y);
//					
//					if(direction<0)
//					{
//						direction = direction + 360;
//					}
//					direction2 = direction-180;
//					if(direction2<0)
//					{
//						direction2 = direction2 + 360;
//					}
//					if(direction1<0)
//					{
//						direction1 = direction1 + 360;
//					}
//					if(dir>=360)
//					{
//						dir = dir-360;
//					}
//					angle=fabs(direction1-dir);
//					angleex = 360 - angle;
//
//					angle1=fabs(direction1-direction);
//					angle1ex = 360 - angle1;
//
//					angle2=fabs(direction1-direction2);
//					angle2ex = 360 - angle2;
//
//
//				}
//
//				//if(k!=0)
//				{
//					if((distance<mindis)&&((angle<40)||(angleex<40))&&((angle1<90)||(angle1ex<90)||(angle2<90)||(angle2ex<90)))//实际上是满足1、车的航向；2、车道的方向；3、车子与点连线的方向这三者相吻合
//					{
//						mindis = distance;
//						nearpoint = &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
//					}
//				}
//				//else if(k==0)
//				//{
//				//	if((distance<mindis)&&((angle<50)||(angleex<50))&&((angle1<90)||(angle1ex<90)))//实际上是满足1、车的航向；2、车道的方向；3、车子与点连线的方向这三者相吻合
//				//	{
//				//		mindis = distance;
//				//		nearpoint = &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
//				//	}
//				//}
//			}
//		}
//	}
//	double julicha=I;
//	if(nearpoint!=NULL)
//	{
//		julicha = m_GpsData.GetDistance(x,y,nearpoint->x,nearpoint->y);
//	}
//	if(julicha<500)
//	{
//		solveID(nearpoint);
//		return 1;//当求得的最近点与该点距离差在200米以内时，返回求得的最近点，否则返回NULL
//	}
//	else
//	{
//		/*CString str="路网中找不到合适的邻近路点";
//		AfxMessageBox(str);*/
//		return 0;
//	}
//}

bool CVehicleExe::solvesegment_num3(double x,double y,double dir)//根据无人车当前GPS坐标求其所在车道号
{
	double distance;
	double direction;//车子与路点连线的方向
	double direction1;//车道的方向
	double direction2;
	double angle;//车道方向与车航向的夹角
	double angle1;//车道方向与(车子和路点连线)的方向的夹角
	double angle2;
	double angleex;
	double angle1ex;
	double angle2ex;
	//int lanepointnum;
	double mindis = I;
	MAPPOINT *nearpoint = NULL;
	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)
	{
		for(int j=0;j<2;j++)
		{
			//lanepointnum=RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;
			for(int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				distance = sqrt(pow((x - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x)),2) + pow((y - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y)),2));	
				
				if (k!=(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num-1))
				{
					direction = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y,x,y);
					direction1 = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k+1].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k+1].y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y);
					if(direction<0)
					{
						direction = direction + 360;
					}
					direction2 = direction-180;
					if(direction2<0)
					{
						direction2 = direction2 + 360;
					}
					if(direction1<0)
					{
						direction1 = direction1 + 360;
					}
					if(dir>=360)
					{
						dir = dir-360;
					}
					angle=fabs(direction1-dir);
					angleex = 360 - angle;

					angle1=fabs(direction1-direction);
					angle1ex = 360 - angle1;

					angle2=fabs(direction1-direction2);
					angle2ex = 360 - angle2;
				}
				else if (k==(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num-1))
				{
					direction = m_GpsData.GetAngle(x,y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y);
					direction1 = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].y);
					
					if(direction<0)
					{
						direction = direction + 360;
					}
					direction2 = direction-180;
					if(direction2<0)
					{
						direction2 = direction2 + 360;
					}
					if(direction1<0)
					{
						direction1 = direction1 + 360;
					}
					if(dir>=360)
					{
						dir = dir-360;
					}
					angle=fabs(direction1-dir);
					angleex = 360 - angle;

					angle1=fabs(direction1-direction);
					angle1ex = 360 - angle1;

					angle2=fabs(direction1-direction2);
					angle2ex = 360 - angle2;


				}

				//if(k!=0)
				{
					if((distance<mindis)&&((angle<40)||(angleex<40))&&((angle1<90)||(angle1ex<90)||(angle2<90)||(angle2ex<90)))//实际上是满足1、车的航向；2、车道的方向；3、车子与点连线的方向这三者相吻合
					{
						mindis = distance;
						nearpoint = &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
					}
				}
				//else if(k==0)
				//{
				//	if((distance<mindis)&&((angle<50)||(angleex<50))&&((angle1<90)||(angle1ex<90)))//实际上是满足1、车的航向；2、车道的方向；3、车子与点连线的方向这三者相吻合
				//	{
				//		mindis = distance;
				//		nearpoint = &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
				//	}
				//}
			}
		}
	}
	double julicha=I;
	if(nearpoint!=NULL)
	{
		julicha = m_GpsData.GetDistance(x,y,nearpoint->x,nearpoint->y);
	}

	if(julicha<200)
	{
		solveID(nearpoint);
		ID1,ID2,ID3;
		return 1;//当求得的最近点与该点距离差在200米以内时，返回求得的最近点，否则返回NULL
	}
	else
	{
		/*CString str="路网中找不到合适的邻近路点";
		AfxMessageBox(str);*/
		return 0;
	}
}

bool CVehicleExe::solvesegment_num2(double x1,double y1,double x2,double y2)//根据两点求其所在车道号
{
	double distance;
	double direction;
	double direction1;
	double direction2;
	double angle;
	double angle1;
	double angleex;
	double angle1ex;
	//int lanepointnum;
	double mindis = I;
	MAPPOINT *nearpoint = NULL;

	direction2 = m_GpsData.GetAngle(x2,y2,x1,y1);
	if(direction2<0)
	{
		direction2 = direction2 + 360;
	}
	for(int i=0;i<RNDFGPS.m_mapInfo.segment_num;i++)
	{
		for(int j=0;j<RNDFGPS.m_mapInfo.pSegment[i].lane_num;j++)
		{
			//lanepointnum=RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;
			for(int k=0;k<RNDFGPS.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				distance = sqrt(pow((x1 - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x)),2) + pow((y1 - (RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y)),2));	
				direction = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y,x1,y1);
				if (k!=0)
				{
					direction1 = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k-1].y);
				}
				else if (k==0)
				{
					direction1 = m_GpsData.GetAngle(RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k+1].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k+1].y,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x,RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y);
				}


				if(direction<0)
				{
					direction = direction + 360;
				}
				if(direction1<0)
				{
					direction1 = direction1 + 360;
				}
				
				angle=fabs(direction1-direction2);
				angleex = 360 - angle;

				angle1=fabs(direction1-direction);
				angle1ex = 360 - angle1;


				if((distance<mindis)&&((angle<40)||(angleex<40))&&((angle1<90)||(angle1ex<90)))//实际上是满足1、车的航向；2、车道的方向；3、车子与点连线的方向这三者相吻合
				{
					mindis = distance;
					nearpoint = &RNDFGPS.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
				}
			}
		}
	}
	double julicha=I;
	if(nearpoint!=NULL)
	{
		julicha = m_GpsData.GetDistance(x1,y1,nearpoint->x,nearpoint->y);
	}
	if(julicha<500)
	{
		solveID(nearpoint);
		return 1;//当求得的最近点与该点距离差在200米以内时，返回求得的最近点，否则返回NULL
	}
	else
	{
		/*CString str="路网中找不到合适的邻近路点";
		AfxMessageBox(str);*/
		return 0;
	}
}
double CVehicleExe::SearchPointToLine(CvPoint2D64f m_Gps,double realtime_Dir)
{
	CvPoint2D64f WayPoint[200];
	double len = 0;
	int point_num = RNDFGPS.m_mapInfo.pSegment[ID1-1].pLane[ID2-1].waypoints_num;	
	for (int i = 1; i<=point_num;i++)
	{
		WayPoint[i-1] = RNDFGPS.GetPoint(ID1,ID2,i);
	}
	Bezier(WayPoint,point_num-1,temprndf);
	len = lanedriverstate.GetMinDis(temprndf,200,m_Gps);
	return len;
}

int CVehicleExe::GetCross_newexe2(CvPoint2D64f (&IntersectionPath)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f Point[4];
	CvPoint2D64f Cross_Pt[8];
	CvPoint2D64f WayPoint[7];
	CvPoint2D64f upath[4];
	int seq = 0;
	seq = seq_num;

	Point[0].x = LeadPoint[seq-3].lat;
	Point[0].y = LeadPoint[seq-3].lng;
	
	Point[1].x = LeadPoint[seq-2].lat;
	Point[1].y = LeadPoint[seq-2].lng;	
	
	Point[2].x = LeadPoint[seq-1].lat;
	Point[2].y = LeadPoint[seq-1].lng;

	Point[3].x = LeadPoint[seq].lat;
	Point[3].y = LeadPoint[seq].lng;

	
	WayPoint[1] = Point[1];
	WayPoint[0] = Point[0];
	WayPoint[6] = Point[3];
	//
	double dist = m_GpsData.GetDistance( WayPoint[0].x,WayPoint[0].y,WayPoint[1].x, WayPoint[1].y);
	WayPoint[0].x = ((dist-30)*WayPoint[1].x + 30*WayPoint[0].x)/dist;
	WayPoint[0].y = ((dist-30)*WayPoint[1].y + 30*WayPoint[0].y)/dist;	


	WayPoint[5] = Point[2];

	dist = m_GpsData.GetDistance( WayPoint[5].x,WayPoint[5].y,WayPoint[6].x, WayPoint[6].y);
	WayPoint[6].x = ((dist-30)*WayPoint[5].x + 30*WayPoint[6].x)/dist;
	WayPoint[6].y = ((dist-30)*WayPoint[5].y + 30*WayPoint[6].y)/dist;	
	
	upath[0] = WayPoint[0];
	upath[1] = WayPoint[1];
	upath[2] = WayPoint[5];
	upath[3] = WayPoint[6];
	//求两端路的交点
	double x1 = WayPoint[0].x;
	double y1 = WayPoint[0].y;
	double x2 = WayPoint[1].x;
	double y2 = WayPoint[1].y;
	double A1 = y2-y1;
	double B1 = -(x2-x1);
	double C1 = x2*y1-x1*y2;

	x1 = WayPoint[5].x;
	y1 = WayPoint[5].y;
	x2 = WayPoint[6].x;
	y2 = WayPoint[6].y;
	double A2 = y2-y1;
	double B2 = -(x2-x1);
	double C2 =x2*y1-x1*y2;

	WayPoint[3].x = (B1*C2-B2*C1)/(A1*B2-A2*B1);
	WayPoint[3].y = (A1*C2-A2*C1)/(A2*B1-A1*B2);

	double dir1 = m_GpsData.GetAngle(WayPoint[0],WayPoint[1]);
	double dir2 = m_GpsData.GetAngle(WayPoint[5],WayPoint[6]);
	if(/*abs(dir1-dir2)<10||abs(360-abs(dir1-dir2))<10*/  LeadPoint[seq-2].param2 == 1)
	{
		fx = 1;
		
		WayPoint[3].x = (WayPoint[1].x+WayPoint[5].x)/2.0;
		WayPoint[3].y = (WayPoint[1].y+WayPoint[5].y)/2.0;
	}
	if(/*abs(abs(dir1-dir2-270)<30||abs(90+dir1-dir2))<30*/ LeadPoint[seq-2].param2 == 2/*abs(dir1-dir2-270)<30||abs(90+dir1-dir2)<30*/)
	{
		fx = 2/*2*/;
	
		
	}
	if(/*abs(abs(dir1-dir2-90)<30||abs(270+dir1-dir2))<30*/  LeadPoint[seq-2].param2 == 3/*abs(dir1-dir2-90)<30||abs(270+dir1-dir2)<30*/)
	{
		fx = 3/*3*/;
	
	}
	if( LeadPoint[seq-2].param2 == 4)
	{
			Stop();
			Sleep(1000);
			
			uTurn.ReturnUTurnPathRNDF(seq_num,IntersectionPath,upath,1);
			app->stop = false;
			//uTurn.ReturnUTurnPath(seq_num,IntersectionPath);
			return 4;
	}
	
	double dist1 = m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[5].x,WayPoint[5].y);//w3,w5之间距离
	if(dist1>10)
	{
		WayPoint[4].x = (dist1*WayPoint[3].x+0*(WayPoint[5].x-WayPoint[3].x))/dist1;
		WayPoint[4].y = (dist1*WayPoint[3].y+0*(WayPoint[5].y-WayPoint[3].y))/dist1;
	}
	else WayPoint[4] = WayPoint[3];
	double dist2 =  m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[1].x,WayPoint[1].y);//w1,w3之间距离
	if(dist2>10)
	{
		WayPoint[2].x = (dist2*WayPoint[3].x+0*(WayPoint[1].x-WayPoint[3].x))/dist2;
		WayPoint[2].y = (dist2*WayPoint[3].y+0*(WayPoint[1].y-WayPoint[3].y))/dist2;
	}
	else WayPoint[2] = WayPoint[3];
	for(int i = 0;i<3;i++)
	{
		Cross_Pt[i] = WayPoint[i];
	}
	for(int i = 3;i<6;i++)
	{
		Cross_Pt[i] = WayPoint[i+1];
	}
	
	WayPoint[2].x = (WayPoint[1].x+9*WayPoint[3].x)/10;
	WayPoint[2].y = (WayPoint[1].y+9*WayPoint[3].y)/10;
	WayPoint[4].x = (WayPoint[5].x+9*WayPoint[3].x)/10;
	WayPoint[4].y = (WayPoint[5].y+9*WayPoint[3].y)/10;
	//LocalPath.BSpline(Cross_Pt,8,GpsPoint);
	Bezier(WayPoint,6,IntersectionPath);
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();//解锁
	MoveWay(m_gps,m_gpsdir,IntersectionPath);
	return 1;

}
void CVehicleExe::StartIbeoMap()
{
	m_hIbeoMapThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theIbeoMapThread,
		this, 0, &dwIbeoMapThreadId);
}

DWORD CVehicleExe::theIbeoMapThread(LPVOID lpParam)
{
	return (((CVehicleExe*)lpParam)->IbeoMapThread());
}

//综合运行流程
DWORD CVehicleExe::IbeoMapThread()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	double vx_tmp=0;
	double vy_tmp=0;
	int ObjIdNum=0;

	while(1)
	{
		
		int iRet = WaitForSingleObject(app->m_MapEvent,100); 
		if(iRet != 0)
			continue;

		SYSTEMTIME t1;
		GetLocalTime(&t1);
		outibeomaptime<<"time1 "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  "<<endl;

		I_Map *map4=getPerceptionMap();

		for(int ii=0; ii<512; ii++)
			for(int jj=0; jj<512; jj++)
			{
				if(!((ii==411 && jj==0)||(ii==411 && jj==511)||(ii==412 && jj==0)||(ii==412 && jj==511)))
				{
					if(map4->MapPoint[ii][jj]==1)
						map4->MapPoint[ii][jj]=44;
					else if(map4->MapPoint[ii][jj]==2)
						map4->MapPoint[ii][jj]=18;
					else if(map4->MapPoint[ii][jj]==3)
						map4->MapPoint[ii][jj]=8;
					else if(map4->MapPoint[ii][jj]>=10)
					{
						map4->MapPoint[ii][jj]=8;//28
						/*
						if(map4->MapPoint[ii][jj]<128)
							dynamicmap_v_x->MapPoint[ii][jj]=(map4->MapPoint[ii][jj]-10)/3.6-app->GPS_Speed;
						else
							dynamicmap_v_x->MapPoint[ii][jj]=-(map4->MapPoint[ii][jj]-128)/3.6-app->GPS_Speed;
							*/
					}
				}
			}

			for (int i=0; i<512; i++)
			{
				for (int j=0; j<512; j++)
				{
					map3->MapPoint[i][j]=map4->MapPoint[i][j];
				}
			}
		
		app->critical_section.Lock();
		CvPoint2D64f currentpos=app->GPS_Point;
		double currentdir=app->GPS_Direction;
		double currentspeed=app->GPS_Speed;
		app->critical_section.Unlock();

		for (int i=0; i<512; i++)
		{
			for (int j=0; j<512; j++)
			{
				dynamic_Map->MapPoint[i][j]=0;
				map_v_x->MapPoint[i][j]=0;
				map_v_y->MapPoint[i][j]=0;
			}
		}
		
		
		int iObjectnum;//物体数量
		ObjectWT tempObjectWT[2000];//预先定义400个物体
		app->critical_winsockread.Lock();

		bool break_flag=0;

		for(int i=0;i<iObject1test1;i++)
		{
			vx_tmp=ObjectWT1test1[i].Relative_VelocityX;
			vy_tmp=ObjectWT1test1[i].Relative_VelocityY;
			ObjIdNum=1;
			break_flag=0;
			if(abs(vy_tmp)<0.2)
			{
				for(int k=0;k<iObject1test2;k++)
				{
					if(ObjectWT1test2[k].Object_Id==ObjectWT1test1[i].Object_Id)
					{
						vy_tmp=vy_tmp+ObjectWT1test2[k].Relative_VelocityY;
						ObjIdNum++;
						break_flag=1;
						break;
					}
				}
				if(break_flag)
				{
					break_flag=0;
					for(int k=0;k<iObject1test3;k++)
					{
						if(ObjectWT1test3[k].Object_Id==ObjectWT1test1[i].Object_Id)
						{
							vy_tmp=vy_tmp+ObjectWT1test3[k].Relative_VelocityY;
							ObjIdNum++;
							break_flag=1;
							break;
						}
					}
				}
				if(break_flag)
				{
					break_flag=0;
					for(int k=0;k<iObject1test4;k++)
					{
						if(ObjectWT1test4[k].Object_Id==ObjectWT1test1[i].Object_Id)
						{
							vy_tmp=vy_tmp+ObjectWT1test4[k].Relative_VelocityY;
							ObjIdNum++;
							break_flag=1;
							break;
						}
					}
				}
				vy_tmp=vy_tmp/ObjIdNum;
			}
			ObjIdNum=1;
			break_flag=0;
			if((abs(vx_tmp+currentspeed)<0.2)||(abs(vx_tmp)<0.2))
			{
			}
			for(int j=0;j<ObjectWT1test1[i].iContour_Points;j++)
			{
				if((ObjectWT1test1[i].iContour_Y[j]>=0)&&(ObjectWT1test1[i].iContour_Y[j]<512)&&(ObjectWT1test1[i].iContour_X[j]>=0)&&(ObjectWT1test1[i].iContour_X[j]<512))
				{
					dynamic_Map->MapPoint[ObjectWT1test1[i].iContour_Y[j]][ObjectWT1test1[i].iContour_X[j]]=28;
					map_v_x->MapPoint[ObjectWT1test1[i].iContour_Y[j]][ObjectWT1test1[i].iContour_X[j]]=vx_tmp;
					map_v_y->MapPoint[ObjectWT1test1[i].iContour_Y[j]][ObjectWT1test1[i].iContour_X[j]]=vy_tmp;
					if(m_task==IVTASK_INTERSECTION)
					{
						//if((ObjectWT1test1[i].iContour_Y[j]>=372)&&(ObjectWT1test1[i].iContour_Y[j]<412))
						if((ObjectWT1test1[i].iContour_Y[j]>=332)&&(ObjectWT1test1[i].iContour_Y[j]<432)&&(ObjectWT1test1[i].iContour_X[j]>=252)&&(ObjectWT1test1[i].iContour_X[j]<=260))
							map3->MapPoint[ObjectWT1test1[i].iContour_Y[j]][ObjectWT1test1[i].iContour_X[j]]=8;
					}
				}
			}
			if((ObjectWT1test1[i].ObjectBox_CenterY>=0)&&(ObjectWT1test1[i].ObjectBox_CenterY<512)&&(ObjectWT1test1[i].ObjectBox_CenterX>=0)&&(ObjectWT1test1[i].ObjectBox_CenterX<512))
			{
				dynamic_Map->MapPoint[ObjectWT1test1[i].ObjectBox_CenterY][ObjectWT1test1[i].ObjectBox_CenterX]=28;
				map_v_x->MapPoint[ObjectWT1test1[i].ObjectBox_CenterY][ObjectWT1test1[i].ObjectBox_CenterX]=vx_tmp;
				map_v_y->MapPoint[ObjectWT1test1[i].ObjectBox_CenterY][ObjectWT1test1[i].ObjectBox_CenterX]=vy_tmp;
				if(m_task==IVTASK_INTERSECTION)
				{
					//if((ObjectWT1test1[i].ObjectBox_CenterY>=372)&&(ObjectWT1test1[i].ObjectBox_CenterY<412))
					if((ObjectWT1test1[i].ObjectBox_CenterY>=332)&&(ObjectWT1test1[i].ObjectBox_CenterY<432)&&(ObjectWT1test1[i].ObjectBox_CenterX>=252)&&(ObjectWT1test1[i].ObjectBox_CenterX<=260))
						map3->MapPoint[ObjectWT1test1[i].ObjectBox_CenterY][ObjectWT1test1[i].ObjectBox_CenterX]=8;
				}
			}
		}
		
		
		int obsum=0;

		
		frontnum=iObject1test1;
	
		for(int i=0; i<frontnum; i++)
		{
			tempObjectWT[i]=ObjectWT1test1[i];
		}
		obsum+=frontnum;

		/*
		frontnum=iObject1test4;
	
		for(int i=0; i<frontnum; i++)
		{
			tempObjectWT[i]=ObjectWT1test4[i];
		}
		obsum+=frontnum;

		frontnum=iObject1test3;
		for (int i=0; i<frontnum;i++ )
		{
			tempObjectWT[obsum+i]=ObjectWT1test3[i];
		}
		obsum+=frontnum;

		frontnum=iObject1test2;
		for (int i=0; i<frontnum;i++ )
		{
			tempObjectWT[obsum+i]=ObjectWT1test2[i];
		}
		obsum+=frontnum;
		*/
		app->critical_winsockread.Unlock();

		

		app->critical_winsockread2.Lock();

		for(int i=0;i<iObject2test1;i++)
		{
			vx_tmp=ObjectWT2test1[i].Relative_VelocityX;
			vy_tmp=ObjectWT2test1[i].Relative_VelocityY;
			ObjIdNum=1;
			break_flag=0;
			if(abs(vy_tmp)<0.2)
			{
				for(int k=0;k<iObject2test2;k++)
				{
					if(ObjectWT2test2[k].Object_Id==ObjectWT2test1[i].Object_Id)
					{
						vy_tmp=vy_tmp+ObjectWT2test2[k].Relative_VelocityY;
						ObjIdNum++;
						break_flag=1;
						break;
					}
				}
				if(break_flag)
				{
					break_flag=0;
					for(int k=0;k<iObject2test3;k++)
					{
						if(ObjectWT2test3[k].Object_Id==ObjectWT2test1[i].Object_Id)
						{
							vy_tmp=vy_tmp+ObjectWT2test3[k].Relative_VelocityY;
							ObjIdNum++;
							break_flag=1;
							break;
						}
					}
				}
				if(break_flag)
				{
					break_flag=0;
					for(int k=0;k<iObject2test4;k++)
					{
						if(ObjectWT2test4[k].Object_Id==ObjectWT2test1[i].Object_Id)
						{
							vy_tmp=vy_tmp+ObjectWT2test4[k].Relative_VelocityY;
							ObjIdNum++;
							break_flag=1;
							break;
						}
					}
				}
				vy_tmp=vy_tmp/ObjIdNum;
			}
			ObjIdNum=1;
			break_flag=0;
			if((abs(vx_tmp+currentspeed)<0.2)||(abs(vx_tmp)<0.2))
			{
			}
			for(int j=0;j<ObjectWT2test1[i].iContour_Points;j++)
			{
				if((ObjectWT2test1[i].iContour_Y[j]>=0)&&(ObjectWT2test1[i].iContour_Y[j]<512)&&(ObjectWT2test1[i].iContour_X[j]>=0)&&(ObjectWT2test1[i].iContour_X[j]<512))
				{
					dynamic_Map->MapPoint[ObjectWT2test1[i].iContour_Y[j]][ObjectWT2test1[i].iContour_X[j]]=28;
					map_v_x->MapPoint[ObjectWT2test1[i].iContour_Y[j]][ObjectWT2test1[i].iContour_X[j]]=vx_tmp;
					map_v_y->MapPoint[ObjectWT2test1[i].iContour_Y[j]][ObjectWT2test1[i].iContour_X[j]]=vy_tmp;
					if(m_task==IVTASK_INTERSECTION)
					{
						//if((ObjectWT2test1[i].iContour_Y[j]>=372)&&(ObjectWT2test1[i].iContour_Y[j]<412))
						if((ObjectWT2test1[i].iContour_Y[j]>=332)&&(ObjectWT2test1[i].iContour_Y[j]<432)&&(ObjectWT2test1[i].iContour_X[j]>=252)&&(ObjectWT2test1[i].iContour_X[j]<=260))
							map3->MapPoint[ObjectWT2test1[i].iContour_Y[j]][ObjectWT2test1[i].iContour_X[j]]=8;
					}
				}
			}
			if((ObjectWT2test1[i].ObjectBox_CenterY>=0)&&(ObjectWT2test1[i].ObjectBox_CenterY<512)&&(ObjectWT2test1[i].ObjectBox_CenterX>=0)&&(ObjectWT2test1[i].ObjectBox_CenterX<512))
			{
				dynamic_Map->MapPoint[ObjectWT2test1[i].ObjectBox_CenterY][ObjectWT2test1[i].ObjectBox_CenterX]=28;
				map_v_x->MapPoint[ObjectWT2test1[i].ObjectBox_CenterY][ObjectWT2test1[i].ObjectBox_CenterX]=vx_tmp;
				map_v_y->MapPoint[ObjectWT2test1[i].ObjectBox_CenterY][ObjectWT2test1[i].ObjectBox_CenterX]=vy_tmp;
				if(m_task==IVTASK_INTERSECTION)
				{
					//if((ObjectWT2test1[i].ObjectBox_CenterY>=372)&&(ObjectWT2test1[i].ObjectBox_CenterY<412))
					if((ObjectWT2test1[i].ObjectBox_CenterY>=332)&&(ObjectWT2test1[i].ObjectBox_CenterY<432)&&(ObjectWT2test1[i].ObjectBox_CenterX>=252)&&(ObjectWT2test1[i].ObjectBox_CenterX<=260))
						map3->MapPoint[ObjectWT2test1[i].ObjectBox_CenterY][ObjectWT2test1[i].ObjectBox_CenterX]=8;
				}
			}
		}

		trailnum=iObject2test1;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test1[i];
		}
		obsum+=trailnum;

		/*
		trailnum=iObject2test2;
		ASSERT(trailnum<200);
		for (int i=0; i<trailnum; i++)
		{
			//ObjectWTtrail[i]=ObjectWT2[i];
			tempObjectWT[i+obsum]=ObjectWT2test2[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test1;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test1[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test3;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test3[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test4;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test4[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test5;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test5[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test6;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test6[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test7;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test7[i];
		}
		obsum+=trailnum;
		*/
		
		app->critical_winsockread2.Unlock();

		iObjectnum=obsum;

	
		app->critical_dyob.Lock();
		fuser.SetVehicle(currentdir,currentpos,map3);

		for (int i=0; i<512; i++)
		{
			for (int j=0; j<512; j++)
			{
				fuser.current_map_v_x->MapPoint[i][j]=0;
				fuser.current_map_v_y->MapPoint[i][j]=0;
			}
		}

		for(int i=0;i<5000;i++)
		{
			fuser.cluster.v_x_clusterId[i]=0;
			fuser.cluster.v_y_clusterId[i]=0;
		}

		//fuser.UpDatelist(map3,tempObjectWT,iObjectnum);
		fuser.cluster.obmap=new ObMap;
		fuser.cluster.Init_history(map3,tempObjectWT,iObjectnum);

		for (int i=0; i<512; i++)
		{
			for (int j=0; j<512; j++)
			{
				if(fuser.cluster.obmap->obstacle[i][j].clusterId>-1)
				{
					fuser.current_map->MapPoint[i][j] = 28;
					fuser.current_map_v_x->MapPoint[i][j]=fuser.cluster.v_x_clusterId[fuser.cluster.obmap->obstacle[i][j].clusterId];
					fuser.current_map_v_y->MapPoint[i][j]=fuser.cluster.v_y_clusterId[fuser.cluster.obmap->obstacle[i][j].clusterId];
					map3->MapPoint[i][j]=fuser.current_map->MapPoint[i][j];
				}
			}
		}
		delete fuser.cluster.obmap;

		for(int i=0;i<5000;i++)
		{
			fuser.cluster.v_x_clusterId[i]=0;
			fuser.cluster.v_y_clusterId[i]=0;
		}
		fuser.cluster.obmap=new ObMap;

		obsum=0;

		app->critical_winsockread.Lock();

		frontnum=iObject1test2;
		for (int i=0; i<frontnum;i++ )
		{
			tempObjectWT[obsum+i]=ObjectWT1test2[i];
		}
		obsum+=frontnum;

		frontnum=iObject1test3;
		for (int i=0; i<frontnum;i++ )
		{
			tempObjectWT[obsum+i]=ObjectWT1test3[i];
		}
		obsum+=frontnum;

		frontnum=iObject1test4;
		for(int i=0; i<frontnum; i++)
		{
			tempObjectWT[obsum+i]=ObjectWT1test4[i];
		}
		obsum+=frontnum;

		frontnum=iObject1test5;
		for(int i=0; i<frontnum; i++)
		{
			tempObjectWT[obsum+i]=ObjectWT1test5[i];
		}
		obsum+=frontnum;

		frontnum=iObject1test6;
		for(int i=0; i<frontnum; i++)
		{
			tempObjectWT[obsum+i]=ObjectWT1test6[i];
		}
		obsum+=frontnum;

		frontnum=iObject1test7;
		for(int i=0; i<frontnum; i++)
		{
			tempObjectWT[obsum+i]=ObjectWT1test7[i];
		}
		obsum+=frontnum;

		app->critical_winsockread.Unlock();

		app->critical_winsockread2.Lock();

		trailnum=iObject2test2;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test2[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test3;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test3[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test4;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test4[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test5;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test5[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test6;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test6[i];
		}
		obsum+=trailnum;

		trailnum=iObject2test7;
		for (int i=0; i<trailnum; i++)
		{
			tempObjectWT[i+obsum]=ObjectWT2test7[i];
		}
		obsum+=trailnum;

		app->critical_winsockread2.Unlock();

		iObjectnum=obsum;


		fuser.cluster.Init_history(map3,tempObjectWT,iObjectnum);

		for (int i=0; i<512; i++)
		{
			for (int j=0; j<512; j++)
			{
				if(fuser.cluster.obmap->obstacle[i][j].clusterId>-1)
				{
					fuser.current_map->MapPoint[i][j] = 28;
					fuser.current_map_v_x->MapPoint[i][j]=fuser.cluster.v_x_clusterId[fuser.cluster.obmap->obstacle[i][j].clusterId];
					fuser.current_map_v_y->MapPoint[i][j]=fuser.cluster.v_y_clusterId[fuser.cluster.obmap->obstacle[i][j].clusterId];
					map3->MapPoint[i][j]=fuser.current_map->MapPoint[i][j];
				}
				if(fuser.current_map->MapPoint[i][j]==28 && fuser.current_map_v_x->MapPoint[i][j]==-10000 && fuser.current_map_v_y->MapPoint[i][j]==-10000)
				{
					fuser.current_map->MapPoint[i][j] = 8;
					fuser.current_map_v_x->MapPoint[i][j]=0;
					fuser.current_map_v_y->MapPoint[i][j]=0;
				}
				if(fuser.current_map->MapPoint[i][j]==28 && (sqrt(((fuser.current_map_v_x->MapPoint[i][j]+currentspeed)*(fuser.current_map_v_x->MapPoint[i][j]+currentspeed))+((fuser.current_map_v_y->MapPoint[i][j])*(fuser.current_map_v_y->MapPoint[i][j])))<1.5))
				{
					fuser.current_map->MapPoint[i][j] = 8;
					fuser.current_map_v_x->MapPoint[i][j]=0;
					fuser.current_map_v_y->MapPoint[i][j]=0;
				}
			}
		}
		delete fuser.cluster.obmap;



		/*
		if (m_task==IVTASK_INTERSECTION)
		{
			fuser.dylist.clear();
		}
		fuser.UpDateUselist();
		*/
		app->critical_dyob.Unlock();

		DWORD cur_time=GetTickCount();
		if(app->percepIbeoMaptimelast)
		{
			Ibeomaptime<<cur_time-app->percepIbeoMaptimelast<<endl;
			app->IbeoMaptimeStamp = cur_time-app->percepIbeoMaptimelast;
		}
		app->percepIbeoMaptimelast = cur_time;

		/*
		for (int i=0; i<512; i++)
		{
			for (int j=228; j<284; j++)
			{
				if((i>=(415+(abs(j-256)/tan(50*PI/180))))||(i<=(395-(abs(j-256)/tan(50*PI/180)))))
				{
					if((dynamic_Map->MapPoint[i][j])==28)
					{
						if(sqrt(((map_v_x->MapPoint[i][j]+currentspeed)*(map_v_x->MapPoint[i][j]+currentspeed))+((map_v_y->MapPoint[i][j])*(map_v_y->MapPoint[i][j])))>=1.5)
						{
							fuser.current_map->MapPoint[i][j] = 28;
							fuser.current_map_v_x->MapPoint[i][j]=map_v_x->MapPoint[i][j];
							fuser.current_map_v_y->MapPoint[i][j]=map_v_y->MapPoint[i][j];
						}
						else
						{
							fuser.current_map->MapPoint[i][j] = 8;
							fuser.current_map_v_x->MapPoint[i][j]=0;
							fuser.current_map_v_y->MapPoint[i][j]=0;
						}
					}
					//else if(((fuser.current_map->MapPoint[i][j])==8)&&j>244&&j<268&&((i>=(415+(abs(j-256)/tan(30*PI/180))))||(i<=(395-(abs(j-256)/tan(30*PI/180))))))
						//fuser.current_map->MapPoint[i][j] = 0;
				}
			}
		}
		*/

		for (int ii=0; ii<512; ii++)
		{
			for (int jj=0; jj<512; jj++)
			{
				if(abs(fuser.current_map_v_x->MapPoint[ii][jj])<0.5)
					fuser.current_map_v_x->MapPoint[ii][jj]=0;
				if(abs(fuser.current_map_v_y->MapPoint[ii][jj])<0.5)
					fuser.current_map_v_y->MapPoint[ii][jj]=0;
				dynamic_Map->MapPoint[ii][jj]=fuser.current_map->MapPoint[ii][jj];
				map_v_x->MapPoint[ii][jj]=fuser.current_map_v_x->MapPoint[ii][jj];
				map_v_y->MapPoint[ii][jj]=fuser.current_map_v_y->MapPoint[ii][jj];
			}
		}

		/*
		app->critical_lukoumap.Lock();
		for(int ii=0; ii<512; ii++)
		{
			for(int jj=0; jj<512; jj++)
			{
				lukou_Map->MapPoint[ii][jj]=dynamic_Map->MapPoint[ii][jj];
				lukou_v_x->MapPoint[ii][jj]=map_v_x->MapPoint[ii][jj];
				lukou_v_y->MapPoint[ii][jj]=map_v_y->MapPoint[ii][jj];
			}
		}
		for(int ii=0; ii<512; ii++)
		{
			for(int jj=0; jj<512; jj++)
			{
				if(dynamic_Map->MapPoint[ii][jj]==28)
				{
					lukou_v_x->MapPoint[ii][jj]=map_v_x->MapPoint[ii][jj]+currentspeed;
					lukou_v_y->MapPoint[ii][jj]=map_v_y->MapPoint[ii][jj];
					float lukou_v=sqrt(pow(lukou_v_x->MapPoint[ii][jj],2)+pow(lukou_v_y->MapPoint[ii][jj],2));
					double lukou_angle;
					if(lukou_v_x->MapPoint[ii][jj]==0)
					{
						if(lukou_v_y->MapPoint[ii][jj]>0)
							lukou_angle=-3.1415926535897932/2;
						else if(lukou_v_y->MapPoint[ii][jj]<0)
							lukou_angle=3.1415926535897932/2;
						else
							lukou_angle=0;
					}
					else
						lukou_angle=atan((-lukou_v_y->MapPoint[ii][jj])/(lukou_v_x->MapPoint[ii][jj]));
					if (lukou_v_x->MapPoint[ii][jj]<0)
						lukou_angle=lukou_angle+3.1415926535897932;
					lukou_angle = lukou_angle * 180 / 3.1415926535897932 + currentdir;
					CvPoint2D64f dyobpos=m_GpsData.MaptoGPS(currentpos,currentdir,cvPoint2D64f(jj,ii));
					CvPoint2D64f dypreobpos[200];
					bool flagpre=0;
					int kk=0;
					for(kk=1; kk<200&&(kk*0.2<=lukou_v); kk++)
					{
						flagpre=1;
						dypreobpos[kk]=m_GpsData.MaptoGPS(dyobpos,lukou_angle,cvPoint2D64f(256,412-kk));
						dypreobpos[kk] = m_GpsData.APiontConverD(currentpos,dypreobpos[kk],currentdir);
						if((abs(dypreobpos[kk].x-256)<=5)&&(dypreobpos[kk].y<=421)&&(dypreobpos[kk].y>=397))
						{
							flagpre=0;
							break;
						}
					}
					if(flagpre)
					{
						for(int mm=1;mm<kk;mm++)
						{
							if((dypreobpos[mm].y>=0)&&(dypreobpos[mm].y<512)&&(dypreobpos[mm].x>=0)&&(dypreobpos[mm].x<512))
							{
								lukou_Map->MapPoint[(int)(dypreobpos[mm].y)][(int)(dypreobpos[mm].x)]=dynamic_Map->MapPoint[ii][jj];
								lukou_v_x->MapPoint[(int)(dypreobpos[mm].y)][(int)(dypreobpos[mm].x)]=map_v_x->MapPoint[ii][jj];
								lukou_v_y->MapPoint[(int)(dypreobpos[mm].y)][(int)(dypreobpos[mm].x)]=map_v_y->MapPoint[ii][jj];
							}
						}
					}
					lukou_v_x->MapPoint[ii][jj]=map_v_x->MapPoint[ii][jj];
				}
			}
		}
		app->critical_lukoumap.Unlock();
		*/

		app->critical_xinmap.Lock();
			for(int ii=0; ii<512; ii++)
				for(int jj=0; jj<512; jj++)
				{
					//dy_map->MapPoint[ii][jj]=fuser.current_map->MapPoint[ii][jj];
					dy_map->MapPoint[ii][jj]=dynamic_Map->MapPoint[ii][jj];
					dynamicmap_v_x->MapPoint[ii][jj]=map_v_x->MapPoint[ii][jj];
					dynamicmap_v_y->MapPoint[ii][jj]=map_v_y->MapPoint[ii][jj];
				}
			
		app->critical_xinmap.Unlock();

		SetEvent(m_IbeoMapEvent);


		GetLocalTime(&t1);
		outibeomaptime<<"time2 "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  "<<endl;

			/*
		app->critical_xinmap.Lock();
			for(int ii=0; ii<512; ii++)
				for(int jj=0; jj<512; jj++)
				{
					dy_map->MapPoint[ii][jj]=map4->MapPoint[ii][jj];
					dynamicmap_v_x->MapPoint[ii][jj]=-app->GPS_Speed;
					dynamicmap_v_y->MapPoint[ii][jj]=0;
					if(!((ii==411 && jj==0)||(ii==411 && jj==511)||(ii==412 && jj==0)||(ii==412 && jj==511)))
					{
						if(map4->MapPoint[ii][jj]==1)
							dy_map->MapPoint[ii][jj]=44;
						else if(map4->MapPoint[ii][jj]==2)
							dy_map->MapPoint[ii][jj]=18;
						else if(map4->MapPoint[ii][jj]==3)
							dy_map->MapPoint[ii][jj]=8;
						else if(map4->MapPoint[ii][jj]>=10)
						{
							dy_map->MapPoint[ii][jj]=28;
							if(map4->MapPoint[ii][jj]<128)
								dynamicmap_v_x->MapPoint[ii][jj]=(map4->MapPoint[ii][jj]-10)/3.6-app->GPS_Speed;
							else
								dynamicmap_v_x->MapPoint[ii][jj]=-(map4->MapPoint[ii][jj]-128)/3.6-app->GPS_Speed;
						}
					}
				}
		app->critical_xinmap.Unlock();
	
		SetEvent(m_IbeoMapEvent);
		
		*/
		
		
	}
	return 0;
}
//int CVehicleExe::AppLaneChange(int fx,int dn,CvPoint2D64f *point,CvPoint2D64f aimpoint,int dir,int &change)
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	dn += change;
//	int lane_num = RNDFGPS.m_mapInfo.pSegment[ID1-1].lane_num/2;
//	CvPoint2D64f aa,bb;
//	CvPoint2D64f oripath[200];
//	CvPoint2D64f temp_point[200];
//	ArrayFuZhi(point,temp_point);
//	aa = m_GpsData.APiontConverD(app->GPS_Point,aimpoint,app->GPS_Direction);
//	bb = aa;
//	if(fx == 3 && dn > 0/*&& applanechang == false*/)
//	{
//		if(dn-1>2)
//			dn=3;
//		MoveLeftDir(temp_point,temp_point,-3.7*(dn-2),dir);
//		
//		ArrayFuZhi(temp_point,point);
//		change = -dn+1;
//		//applanechang = true;
//		return 1;
//	}
//
//	if((fx == 4||fx == 2) && lane_num-dn>-1 /*&& applanechang == false*/)
//	{
//		
//	
//		MoveLeftDir(temp_point,temp_point,3.7*(lane_num-dn),dir);
//	
//		
//	
//		
//		/*if(dn-1>2)
//				dn=3;*/
//		MoveLeftDir(temp_point,temp_point,-3,dir);
//		for(int k =0 ; k< 32;k++)
//		{
//			MoveLeftDir(temp_point,temp_point,0.4,dir);
//			lanedriverstate.GetPointfromNearGPS(point,200,app->GPS_Point,40,gps_aimpoint);
//			bb = m_GpsData.APiontConverD(app->GPS_Point,gps_aimpoint,app->GPS_Direction);
//			//int ccc = int(412-bb.y );
//			for (int i = 0;i<150;i++)
//			{
//				for (int j =0; j<10;j++)
//				{
//					if (vel_Map->MapPoint[int(bb.y)+i][int(bb.x) - j] == 8 || vel_Map->MapPoint[int(bb.y)+i][int(bb.x) - j] == 18 )
//					{
//						//MoveLeftDir(point,point,-0.2,dir);
//						ArrayFuZhi(temp_point,point);
//						change = lane_num-dn;
//						return 1;
//
//					}
//
//
//				}
//
//			}
//
//		}
//		change = lane_num-dn;
//		return 1;
//	}
//	return 0;
//}
//lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);
//if(!lane_result)
//{
//	road_state.send_messagerndf(a);
//	way_out = true;
//	continue;
//}
//
//getAimPoint(200,MidPoint,aim_point);
//gps_aimpoint = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,aim_point);
///*	gps_aimdir = getAimDir(midd_num1, MidPoint,realtime_Dir);*/
//hisaimpoint = aim_point;
//hisgpsaimpoint = gps_aimpoint;
///*hisgpsaimddir = gps_aimdir;*/

	
				//show_midpoint = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
								
		      //SpeedDecision();
				
				//ob_speed=SearchObstacle(MidGpsPoint,vel_Map,80/*30*/,2,ob_x,ob_y,20);
				//if(ob_speed)
				//{
				//	app->secondDecison = "可能会绕障";
				//	app->drive_obstacle = 33;
				//}
				/////3.5
				//ob=SearchObstacle(search_obGpsPoint,vel_Map,app->GPS_Speed*2+30,0,ob_x,ob_y);
				//if(ob)
				//	ob_num++;
				//else
				//	ob_num = 0;
				/*if(ob_num>0&&MidGpsPoint[199].x!=first_gps.x&&MidGpsPoint[199].y!=first_gps.y)
				{					
					app->drive_obstacle = 15;
					SearchLeftRightObstacle();
					break;
				} */

				//app->raozhang  = false;
				
/*if(cos(rad(gps_aimdir - realtime_Dir))> cos(rad(5))&&lanechang_flag)
	{
		ob_res = SearchLeftRightObstacleGps(rndfbezier);
		if(!ob_res)ob_num=0;
		else
			ob_num++;
	}
	else	
	{
		ob_res = 0;
		ob_num = 0;
	}

	if (ob_num > 3)
	{
		MoveLeft(rndfbezier,rndfbezier,-3.7*ob_res);
		obvegps = cvPoint2D64f(realtime_Gps.x,realtime_Gps.y);
		lanedriverstate.GetPointfromNearGPS(rndfbezier,200,realtime_Gps,40,gps_aimpoint);
		aim_point = m_GpsData.APiontConverD(realtime_Gps,gps_aimpoint,realtime_Dir);
		ob_num = 0;
	}*/
int CVehicleExe::GetFangxiang1()//方案二
{
	CvPoint2D64f waypoint[4];
	int seq = seq_num;
	waypoint[0].x = LeadPoint[seq-2].lat;
	waypoint[0].y = LeadPoint[seq-2].lng;
	
	waypoint[1].x = LeadPoint[seq-1].lat;
	waypoint[1].y = LeadPoint[seq-1].lng;	
	
	waypoint[2].x = LeadPoint[seq].lat;
	waypoint[2].y = LeadPoint[seq].lng;

	waypoint[3].x = LeadPoint[seq+1].lat;
	waypoint[3].y = LeadPoint[seq+1].lng;
	double dir1 = m_GpsData.GetAngle(waypoint[1],waypoint[0]);
	double dir2 = m_GpsData.GetAngle(waypoint[3],waypoint[2]);
	if(abs(dir1-dir2)<45||abs(360-abs(dir1-dir2))<45  )
	{
		fx = 1;
	
	}
	if(abs(dir1-dir2+270)<45||abs(90-dir1+dir2)<45 /*abs(dir1-dir2-270)<30||abs(90+dir1-dir2)<30*/)
	{
		fx = 2/*2*/;
		
		
	}
	if(abs(dir1-dir2+90)<45||abs(270-dir1+dir2)<45 /*abs(dir1-dir2-90)<30||abs(270+dir1-dir2)<30*/)
	{
		fx = 3/*3*/;
	
		
	}
	if(abs(abs(dir1-dir2)-180)<10/*abs(abs(dir1-dir2-180)<10||abs(180+dir1-dir2))<10*/ /*abs(dir1-dir2-270)<30||abs(90+dir1-dir2)<30*/)
	{
		
		fx = 4;
	}
	return 1;
}

bool  CVehicleExe::JudgeBetweenTwoDis(CvPoint2D64f goal,double up, double down,CvPoint2D64f m_gps,double dir)
{
	double dist_vert = VertDist(m_gps,dir,goal);
	if (dist_vert < up && dist_vert >down)
	{
		return true;
	}
	else
		return false;
}
double CVehicleExe::SearchFrontVehicle(I_Map *map,double up,double width)//正前方range米内有无障碍
{

	int x = 256;
	int y = 411;
	for(int m=0;m>-up*5;m--)
		for(int n=-width*5;n<=width*5;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 88)
			{
				return abs(m/5.0);

			}
		}

		return 0;

}
double CVehicleExe::SearchFrontPeople(I_Map *map,double up,double width)//正前方range米内有无障碍
{

	int x = 256;
	int y = 411;
	for(int m=0;m>-up*5;m--)
		for(int n=-width*5;n<=width*5;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 58)
			{
				return abs(m/5.0);

			}
		}

		return 0;

}

double CVehicleExe::SearchYulukouOb(I_Map *map,double range)
{
	int x = 256;
	int y = 411;
	for(int m=0;m>-range*5;m--)
		for(int n=-4;n<=4;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8)
			{
				return abs(m/5.0);

			}
		}
		return 0;
}

int CVehicleExe::GetPartGlobalPath(CvPoint2D64f *curve)
{
	lanedriverstate.GetBSpline1(LeadPoint,mission_num,curve);
	return 1;
}
int CVehicleExe::GetGlobalPathSeq(CvPoint2D64f *path,int num)
{
	CvPoint2D64f start_yan[100];
	CvPoint2D64f end_yan[100];

	double dir1 = m_GpsData.GetAngle(path[0],path[1]);
	double dir2 = m_GpsData.GetAngle(path[num-1],path[num-2]);

	for(int i=0;i<100;i++)
	{
		start_yan[i] = m_GpsData.MaptoGPS(path[0],dir1,cvPoint2D64f(256,412-(i*6)));
		end_yan[i] = m_GpsData.MaptoGPS(path[num-1],dir2,cvPoint2D64f(256,412-(i*6)));
	}


	cvClearSeq( CVehicle::global_path );
	for(int i = 0; i<100; i++)
	{
		cvSeqPush( CVehicle::global_path, &start_yan[99-i] );
	}

	for(int i = 0; i<num; i++)
	{
		cvSeqPush( CVehicle::global_path, &path[i] );
	}

	for(int i = 0; i<100; i++)
	{
		cvSeqPush( CVehicle::global_path, &end_yan[i] );
	}

	for(int i = 0; i<num+200; i++)
	{
		CvPoint2D64f pt = *(CvPoint2D64f*)cvGetSeqElem(CVehicle::global_path, i );
		outn77<<setprecision(11)<<pt.x<<" ,  "<<pt.y<<endl;
	}
	return 1;

}
int CVehicleExe::PickUpIntersectionPoints(int *c, int &b)
{
	int j = 0;
	for(int i = mission_num-1; i>0;i--)
	{
		if(LeadPoint[i].param1 == 3&&LeadPoint[i-1].param1 == 3)
		{
			c[j] = i+1;
			i=i-1;
			j++;
		}
	}
	b = j;
	return 0;
}
//int CVehicleExe::PickUpIntersectionPoints(int *c, int &b)
//{
//	int j = 0;
//	for(int i = 0; i<mission_num;i++)
//	{
//		if(LeadPoint[i].param1 == 3&&LeadPoint[i+1].param1 == 3)
//		{
//			c[j] = i+1+1;
//			i=i+1;
//			j++;
//		}
//	}
//	b = j;
//	return 0;
//}
void CVehicleExe::UpDatePath()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->update_flag = false;
	//if(app->ADAS_points->total!=0)
	app->critical_adasdata.Lock();
	int adas_num = app->ADAS_points->total;
	outupdatenum<<"2"<<endl;

	LEAD p1;
	for(int i=1;i<=adas_num;i++)
	{
		p1 = *(LEAD*)cvGetSeqElem(app->ADAS_points,i-1);
		if (p1.param1 == 1)
		{
			globalendpoint.x = p1.lat;
			globalendpoint.y = p1.lng;
		}
		else
		{
			//LEAD p2 =  *(LEAD*)cvGetSeqElem(app->ADAS_points,adas_num-1);
			globalendpoint.x = 0;
			globalendpoint.y = 0;
		}
		outADASISPoint<<setprecision(11)<<p1.id<<"	"<<p1.lng<<"	"<<p1.lat<<"	"<<p1.param1<<"  "<<p1.param2<<"  "<<p1.param3<<endl;
	}

	ReturnNeedPoints(app->ADAS_points,app->GPS_Point,app->GPS_Direction,1000);
	mission_num = GetMissionNum();
	LeadPoint = (LEAD *)malloc( (mission_num)*sizeof(LEAD) );
	GetMissionPoint(LeadPoint,mission_num);
	for (int i = 0;i<500;i++)
	{
		InterSecPoints[i] = 0;
	}
	PickUpIntersectionPoints(InterSecPoints,InterSecNum);
	app->critical_adasdata.Unlock();
	gps_road = (CvPoint2D64f *)malloc( ((mission_num-1)*50)*sizeof(CvPoint2D64f) );
	GetPartGlobalPath(gps_road);
	GetGlobalPathSeq(gps_road,(mission_num-1)*50);
	free(gps_road);
	int numseq = ReturnPartPointsSeqNumOnTimer(CVehicle::global_path,app->GPS_Point,app->GPS_Direction,20,rndfbezier);
	if((mission_num-1)*50+100-global_path->total+numseq<0)
		seq_num=1;
	else
		seq_num = (int)((mission_num-1)*50+100-global_path->total+numseq)/50+2;
	//seq_num = (int)(numseq)/50;
	outseqnum<<"seqnum: "<<seq_num<<endl;
	app->update_flag = true;
	//app->critical_adasisupdate.Lock();

	//app->critical_adasisupdate.Unlock();

}

int CVehicleExe::ReturnPartPoints(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int num = gpath->total;
	CvPoint2D64f pt,pt_next;
	double l = 0;
	CvPoint2D64f mpt,mpt_next;
	int count = 0;
	CvPoint2D64f *temp_point;
	int min_s = 99999999;
	CvPoint5D64f_type pt_3D;
	CvPoint5D64f_type pt_3D_temp;
	double l_last=0;
	for (int i=countlast;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
		if(l < min_s)
		{
			count = i;
			min_s = l;
		}
		if(l>300 && i>countlast)
			break;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l_last=l_last+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);

		{
			if(l_last>15 && i>countlast)
				break;
		}
	}

	CvPoint2D64f pt_count;
	pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, count);
	pt_count.x=pt_3D.x;
	pt_count.y=pt_3D.y;

	spdcurrentpnt=pt_3D.spdiniset;

	blaneusedisable=!(pt_3D.laneexist);

	for (int i=countlast;i<count;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(pt_3D.z>0 && pt_3D.z<9)
			lukou_fx=pt_3D.z;
	}

	l=0;
	bool flag_check_=0;
	for (int i=count;i<num;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		if(pt_3D.z>0 && pt_3D.z<=9)
		{
			if(pt_3D.z<9)
			{
				app->bchulukoufound=0;
				lukou_fx=pt_3D.z;
				yulukoupoint=pt;
				if(((!(((lukou_fx == 1 && bZturnLightChk) || (lukou_fx == 3 && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) &&(app->light_res == 2)))||bspecialzhilukoutingche)&&(l<5))
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(lukou_fx==6)
						app->bshigongluduanlock=true;
					if(lukou_fx==2 || lukou_fx==3)
						app->bguihualukou==true;
					approachyulukou=0;
					if(upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				else
				{
					if(m_task!=IVTASK_APPROINTERSECTION)
					{
						upoint[0].x=0;
						upoint[0].y=0;
					}
					upoint[1]=pt;
					m_task = IVTASK_APPROINTERSECTION;

					upointyulukou[1]=pt;
					upointyulukou[0]=pt;
					upointyulukou[2]=pt;
					upointyulukou[3]=pt;
					double lyulukoutmp=0;
					int iyulukou;
					for(iyulukou=i-1;iyulukou>=0;iyulukou--)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						ptlukou_3Dtmpnxt = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou+1);
						lyulukoutmp=lyulukoutmp+m_GpsData.GetDistance(ptlukou_3Dtmp.x,ptlukou_3Dtmp.y,ptlukou_3Dtmpnxt.x,ptlukou_3Dtmpnxt.y);
						if(lyulukoutmp>10)
						{
							upointyulukou[0].x=ptlukou_3Dtmp.x;
							upointyulukou[0].y=ptlukou_3Dtmp.y;
							break;
						}
					}
					lyulukoutmp=0;
					for(iyulukou=i+1;iyulukou<num;iyulukou++)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						if(ptlukou_3Dtmp.z==9)
						{
							spdintersection=ptlukou_3Dtmp.spdiniset;

							upointyulukou[2].x=ptlukou_3Dtmp.x;
							upointyulukou[2].y=ptlukou_3Dtmp.y;
							upointyulukou[3]=upointyulukou[2];
							break;
						}
					}
					for(iyulukou=iyulukou+1;iyulukou<num;iyulukou++)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						ptlukou_3Dtmpnxt = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou-1);
						lyulukoutmp=lyulukoutmp+m_GpsData.GetDistance(ptlukou_3Dtmp.x,ptlukou_3Dtmp.y,ptlukou_3Dtmpnxt.x,ptlukou_3Dtmpnxt.y);
						if(lyulukoutmp>10)
						{
							upointyulukou[3].x=ptlukou_3Dtmp.x;
							upointyulukou[3].y=ptlukou_3Dtmp.y;
							break;
						}
					}

					double labianlen=40;//40
					/*
					if(pt_3D.z==4 && m_GpsData.GetDistance(pt.x,pt.y,globalstrt.x,globalstrt.y)<200)
						labianlen=80;
						*/
					double angleerr=0;
					double lengtherr=0;
					if(i>0)
					{
						int j=i-1;
						CvPoint5D64f_type pt_3D_tmp;
						CvPoint2D64f pt_tmp;
						for (j=i-1;j>=0;j--)
						{
							pt_3D_tmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pt_tmp.x=pt_3D_tmp.x;
							pt_tmp.y=pt_3D_tmp.y;
							if(m_GpsData.GetDistance(pt.x,pt.y,pt_tmp.x,pt_tmp.y)>10)
								break;
						}
						if(j<0)
							j=0;
						pt_3D_tmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
						pt_tmp.x=pt_3D_tmp.x;
						pt_tmp.y=pt_3D_tmp.y;
						angleerr=m_GpsData.GetAngle(m_gps,pt_tmp)-m_GpsData.GetAngle(pt,pt_tmp);
						lengtherr=abs((m_GpsData.GetDistance(m_gps.x,m_gps.y,pt_tmp.x,pt_tmp.y))*(sin(angleerr*PI/180)));
					}
					if(lengtherr>11.1)
						labianlen=80;
					else if(lengtherr>7.4)
						labianlen=65;
					else if(lengtherr>3.7)
						labianlen=50;
					if((l<labianlen)||(l<80 && approachyulukou>0))
					{
						if(pt_3D.z==3 || pt_3D.z==4 || pt_3D.z==2)
							blabianqiansureq=true;
						else
							blabianqiansureq=false;
					}
					else
						blabianqiansureq=false;
					if(((l<labianlen)||(l<80 && approachyulukou>0)) && app->GPS_Speed<20/3.6)
					{
						if(pt_3D.z==3 || pt_3D.z==4)
							approachyulukou=0;
						else if(pt_3D.z==2)
							approachyulukou=0;
						else
							approachyulukou=0;
					}
					if(l<10&&upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				/*
				if(l<5&&lukou_fx==4)
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				*/
			}
			else
			{
				spdintersection=pt_3D.spdiniset;
				if(l < 60)
					app->bchulukoufound=1;
				if(!(((((lukou_fx == 1 && bZturnLightChk) || (lukou_fx == 3 && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) &&app->light_res == 2)||bspecialzhilukoutingche) && m_task == IVTASK_APPROINTERSECTION))
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(lukou_fx==6)
						app->bshigongluduanlock=true;
					if(lukou_fx==2 || lukou_fx==3)
						app->bguihualukou==true;
					approachyulukou=0;
				}
			}
			flag_check_=1;
			break;
		}
		if(i+1<num)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			pt_next.x=pt_3D.x;
			pt_next.y=pt_3D.y;
			l = l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		}
		if((l > 280)||((app->bchulukoufound==1)&&(l>80)))
		{
			m_task = IVTASK_LANE;
			flag_check_=1;
			approachyulukou=0;
			break;
		}
	}
	if(flag_check_==0)
	{
		m_task = IVTASK_LANE;
		approachyulukou=0;
	}

	flag_check_=0;
	l=0;
	for (int i=count;i<num;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		if(pt_3D.z>0&&pt_3D.z<9)
		{
			flag_check_=1;
			app->lukoulanechangereq=1;
			break;
		}
		if(i+1<num)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			pt_next.x=pt_3D.x;
			pt_next.y=pt_3D.y;
			l = l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		}
		if(l > 120)
		{
			flag_check_=2;
			app->lukoulanechangereq=0;
			break;
		}
	}
	if(flag_check_==0)
		app->lukoulanechangereq=0;

	for(int i=countlast+1;i<=count;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
		if(pt_3D.z==30)
		{
			blanesync=1;
			break;
		}
	}

	l=0;
	int countstrt=0;
	for(int i=count-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l=l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>s)
		{
			countstrt=i;
			break;
		}
	}

	countlast=count-countstrt;

	for(int i = 0;i<countstrt;i++)
	{
		cvSeqPopFront( gpath,NULL );
	}

	num = gpath->total;
	if(num<2)
		return 0;
	l=0;
	count = num-2;
	double sfront=100;
	if(m_GpsData.GetDistance(31.32436573513,121.23512638759,m_gps.x,m_gps.y)<50)
		sfront=50;
	for(int i = 0;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l+= m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>sfront+s)
		{
			count = i;
			break;
		}
		else if(i>countlast-1 && i<num-2 && m_task==IVTASK_APPROINTERSECTION)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+2);
			if(pt_3D.z>0 && pt_3D.z<9)
			{
				count = i;
				break;
			}
			else
			{
				pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
				if(pt_3D.z>0 && pt_3D.z<9)
				{
					count = i;
					break;
				}
				else
				{
					pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
					if(pt_3D.z>0 && pt_3D.z<9)
					{
						count = i;
						break;
					}
				}
			}
		}
	}
	bool bleftsidelaneflag=0;
	bool brightsidelaneflag=0;
	double latpianyi = 0;
	double lngpianyi = 0;
	temp_point = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	CvPoint2D64f *temp_point_left = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	CvPoint2D64f *temp_point_right = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	for(int i = countlast;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(i==countlast)
		{
			bleftchgallowed=pt_3D.b_left;
			brightchgallowed=pt_3D.b_right;
		}
		if(i>countlast)
		{
			if(pt_3D.z==30)
			{
				pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i-1);
				latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
				lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
			}
		}
		if(latpianyi==0 && lngpianyi==0)
		{
			if(pt_3D.laneseq==1)
			{
				app->brightsidelane=true;
				brightsidelaneflag=true;
			}
			if(pt_3D.laneseq==pt_3D.lanenum_)
			{
				app->bleftsidelane=true;
				bleftsidelaneflag=true;
			}
		}
		temp_point[i].x=pt_3D.x+latpianyi;
		temp_point[i].y=pt_3D.y+lngpianyi;

		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );

		temp_point_left[i].x=pt_3D.x_left-pt_3D.x+temp_point[i].x;
		temp_point_left[i].y=pt_3D.y_left-pt_3D.y+temp_point[i].y;

		temp_point_right[i].x=pt_3D.x_right-pt_3D.x+temp_point[i].x;
		temp_point_right[i].y=pt_3D.y_right-pt_3D.y+temp_point[i].y;
	}
	if(bleftsidelaneflag==0)
		app->bleftsidelane=false;
	if(brightsidelaneflag==0)
		app->brightsidelane=false;
	latpianyi = 0;
	lngpianyi = 0;
	for(int i = countlast-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(pt_3D.z==29)
		{
			pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
			lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
		}
		temp_point[i].x=pt_3D.x+latpianyi;
		temp_point[i].y=pt_3D.y+lngpianyi;

		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );

		temp_point_left[i].x=pt_3D.x_left-pt_3D.x+temp_point[i].x;
		temp_point_left[i].y=pt_3D.y_left-pt_3D.y+temp_point[i].y;

		temp_point_right[i].x=pt_3D.x_right-pt_3D.x+temp_point[i].x;
		temp_point_right[i].y=pt_3D.y_right-pt_3D.y+temp_point[i].y;
	}
	/*
	for(int i = 0;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		temp_point[i].x=pt_3D.x;
		temp_point[i].y=pt_3D.y;
	}
	*/
	Beziercazhi(temp_point,count+1,rndf);
	for (int i = 0;i<200;i++)
	{
		rndf[i].x =  rndf[i].x-err.x;
		rndf[i].y =  rndf[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(rndf[199],rndf[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=rndf[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(rndf[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			rndf[i].x=inter_yanchang[i].x;
			rndf[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point);

	Beziercazhi(temp_point_left,count+1,app->rndfbezier_left);
	for (int i = 0;i<200;i++)
	{
		app->rndfbezier_left[i].x =  app->rndfbezier_left[i].x-err.x;
		app->rndfbezier_left[i].y =  app->rndfbezier_left[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(app->rndfbezier_left[199],app->rndfbezier_left[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=app->rndfbezier_left[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(app->rndfbezier_left[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			app->rndfbezier_left[i].x=inter_yanchang[i].x;
			app->rndfbezier_left[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point_left);

	Beziercazhi(temp_point_right,count+1,app->rndfbezier_right);
	for (int i = 0;i<200;i++)
	{
		app->rndfbezier_right[i].x =  app->rndfbezier_right[i].x-err.x;
		app->rndfbezier_right[i].y =  app->rndfbezier_right[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(app->rndfbezier_right[199],app->rndfbezier_right[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=app->rndfbezier_right[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(app->rndfbezier_right[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			app->rndfbezier_right[i].x=inter_yanchang[i].x;
			app->rndfbezier_right[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point_right);

	return 1;
}

int CVehicleExe::ReturnPartPoints_lanechg(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int num = gpath->total;
	CvPoint2D64f pt,pt_next;
	double l = 0;
	CvPoint2D64f mpt,mpt_next;
	int count = 0;
	CvPoint2D64f *temp_point;
	int min_s = 99999999;
	CvPoint5D64f_type pt_3D;
	CvPoint5D64f_type pt_3D_temp;
	double l_last=0;
	for (int i=countlast;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
		if(l < min_s)
		{
			count = i;
			min_s = l;
		}
		if(l>300 && i>countlast)
			break;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l_last=l_last+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);

		{
			if(l_last>15 && i>countlast)
				break;
		}
	}

	CvPoint2D64f pt_count;
	pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, count);
	pt_count.x=pt_3D.x;
	pt_count.y=pt_3D.y;

	spdcurrentpnt=pt_3D.spdiniset;

	blaneusedisable=!(pt_3D.laneexist);

	for (int i=countlast;i<count;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(pt_3D.z>0 && pt_3D.z<9)
			lukou_fx=pt_3D.z;
	}

	l=0;
	bool flag_check_=0;
	for (int i=count;i<num;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		if(pt_3D.z>0 && pt_3D.z<=9)
		{
			if(pt_3D.z<9)
			{
				app->bchulukoufound=0;
				lukou_fx=pt_3D.z;
				yulukoupoint=pt;
				if(((!(((lukou_fx == 1 && bZturnLightChk) || (lukou_fx == 3 && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) &&(app->light_res == 2)))||bspecialzhilukoutingche)&&(l<5))
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(lukou_fx==6)
						app->bshigongluduanlock=true;
					if(lukou_fx==2 || lukou_fx==3)
						app->bguihualukou==true;
					approachyulukou=0;
					if(upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				else
				{
					if(m_task!=IVTASK_APPROINTERSECTION)
					{
						upoint[0].x=0;
						upoint[0].y=0;
					}
					upoint[1]=pt;
					m_task = IVTASK_APPROINTERSECTION;

					upointyulukou[1]=pt;
					upointyulukou[0]=pt;
					upointyulukou[2]=pt;
					upointyulukou[3]=pt;
					double lyulukoutmp=0;
					int iyulukou;
					for(iyulukou=i-1;iyulukou>=0;iyulukou--)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						ptlukou_3Dtmpnxt = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou+1);
						lyulukoutmp=lyulukoutmp+m_GpsData.GetDistance(ptlukou_3Dtmp.x,ptlukou_3Dtmp.y,ptlukou_3Dtmpnxt.x,ptlukou_3Dtmpnxt.y);
						if(lyulukoutmp>10)
						{
							upointyulukou[0].x=ptlukou_3Dtmp.x;
							upointyulukou[0].y=ptlukou_3Dtmp.y;
							break;
						}
					}
					lyulukoutmp=0;
					for(iyulukou=i+1;iyulukou<num;iyulukou++)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						if(ptlukou_3Dtmp.z==9)
						{
							spdintersection=ptlukou_3Dtmp.spdiniset;

							upointyulukou[2].x=ptlukou_3Dtmp.x;
							upointyulukou[2].y=ptlukou_3Dtmp.y;
							upointyulukou[3]=upointyulukou[2];
							break;
						}
					}
					for(iyulukou=iyulukou+1;iyulukou<num;iyulukou++)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						ptlukou_3Dtmpnxt = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou-1);
						lyulukoutmp=lyulukoutmp+m_GpsData.GetDistance(ptlukou_3Dtmp.x,ptlukou_3Dtmp.y,ptlukou_3Dtmpnxt.x,ptlukou_3Dtmpnxt.y);
						if(lyulukoutmp>10)
						{
							upointyulukou[3].x=ptlukou_3Dtmp.x;
							upointyulukou[3].y=ptlukou_3Dtmp.y;
							break;
						}
					}

					double labianlen=40;//40
					/*
					if(pt_3D.z==4 && m_GpsData.GetDistance(pt.x,pt.y,globalstrt.x,globalstrt.y)<200)
						labianlen=80;
						*/
					double angleerr=0;
					double lengtherr=0;
					if(i>0)
					{
						int j=i-1;
						CvPoint5D64f_type pt_3D_tmp;
						CvPoint2D64f pt_tmp;
						for (j=i-1;j>=0;j--)
						{
							pt_3D_tmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pt_tmp.x=pt_3D_tmp.x;
							pt_tmp.y=pt_3D_tmp.y;
							if(m_GpsData.GetDistance(pt.x,pt.y,pt_tmp.x,pt_tmp.y)>10)
								break;
						}
						if(j<0)
							j=0;
						pt_3D_tmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
						pt_tmp.x=pt_3D_tmp.x;
						pt_tmp.y=pt_3D_tmp.y;
						angleerr=m_GpsData.GetAngle(m_gps,pt_tmp)-m_GpsData.GetAngle(pt,pt_tmp);
						lengtherr=abs((m_GpsData.GetDistance(m_gps.x,m_gps.y,pt_tmp.x,pt_tmp.y))*(sin(angleerr*PI/180)));
					}
					if(lengtherr>11.1)
						labianlen=80;
					else if(lengtherr>7.4)
						labianlen=65;
					else if(lengtherr>3.7)
						labianlen=50;
					if((l<labianlen)||(l<80 && approachyulukou>0))
					{
						if(pt_3D.z==3 || pt_3D.z==4 || pt_3D.z==2)
							blabianqiansureq=true;
						else
							blabianqiansureq=false;
					}
					else
						blabianqiansureq=false;
					if(((l<labianlen)||(l<80 && approachyulukou>0)) && app->GPS_Speed<20/3.6)
					{
						if(pt_3D.z==3 || pt_3D.z==4)
							approachyulukou=0;
						else if(pt_3D.z==2)
							approachyulukou=0;
						else
							approachyulukou=0;
					}
					if(l<10&&upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				/*
				if(l<5&&lukou_fx==4)
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				*/
			}
			else
			{
				spdintersection=pt_3D.spdiniset;
				if(l < 60)
					app->bchulukoufound=1;
				if(!(((((lukou_fx == 1 && bZturnLightChk) || (lukou_fx == 3 && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) &&app->light_res == 2)||bspecialzhilukoutingche) && m_task == IVTASK_APPROINTERSECTION))
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(lukou_fx==6)
						app->bshigongluduanlock=true;
					if(lukou_fx==2 || lukou_fx==3)
						app->bguihualukou==true;
					approachyulukou=0;
				}
			}
			flag_check_=1;
			break;
		}
		if(i+1<num)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			pt_next.x=pt_3D.x;
			pt_next.y=pt_3D.y;
			l = l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		}
		if((l > 280)||((app->bchulukoufound==1)&&(l>80)))
		{
			m_task = IVTASK_LANE;
			flag_check_=1;
			approachyulukou=0;
			break;
		}
	}
	if(flag_check_==0)
	{
		m_task = IVTASK_LANE;
		approachyulukou=0;
	}

	flag_check_=0;
	l=0;
	for (int i=count;i<num;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		if(pt_3D.z>0&&pt_3D.z<9)
		{
			flag_check_=1;
			app->lukoulanechangereq=1;
			break;
		}
		if(i+1<num)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			pt_next.x=pt_3D.x;
			pt_next.y=pt_3D.y;
			l = l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		}
		if(l > 120)
		{
			flag_check_=2;
			app->lukoulanechangereq=0;
			break;
		}
	}
	if(flag_check_==0)
		app->lukoulanechangereq=0;

	for(int i=countlast+1;i<=count;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
		if(pt_3D.z==30)
		{
			pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i-1);
			latpianyi_lanechgstrt=latpianyi_lanechgstrt+pt_3D_temp.x-pt_3D.x;
			lonpianyi_lanechgstrt=lonpianyi_lanechgstrt+pt_3D_temp.y-pt_3D.y;
		}
	}

	l=0;
	int countstrt=0;
	for(int i=count-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l=l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>s)
		{
			countstrt=i;
			break;
		}
	}

	countlast=count-countstrt;

	for(int i = 0;i<countstrt;i++)
	{
		cvSeqPopFront( gpath,NULL );
	}

	num = gpath->total;
	if(num<2)
		return 0;
	l=0;
	count = num-2;
	double sfront=100;
	if(m_GpsData.GetDistance(31.32436573513,121.23512638759,m_gps.x,m_gps.y)<50)
		sfront=50;
	for(int i = 0;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l+= m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>sfront+s)
		{
			count = i;
			break;
		}
		else if(i>countlast-1 && i<num-2 && m_task==IVTASK_APPROINTERSECTION)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+2);
			if(pt_3D.z>0 && pt_3D.z<9)
			{
				count = i;
				break;
			}
			else
			{
				pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
				if(pt_3D.z>0 && pt_3D.z<9)
				{
					count = i;
					break;
				}
				else
				{
					pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
					if(pt_3D.z>0 && pt_3D.z<9)
					{
						count = i;
						break;
					}
				}
			}
		}
	}
	bool bleftsidelaneflag=0;
	bool brightsidelaneflag=0;
	double latpianyi = 0;
	double lngpianyi = 0;
	temp_point = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	CvPoint2D64f *temp_point_dst = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	CvPoint2D64f *temp_point_left = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	CvPoint2D64f *temp_point_right = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	double latpianyi_dst = 0;
	double lngpianyi_dst = 0;
	int lanchangecntdst=lanechangenumtmp;
	if(lanchangecntdst>app->lanechangenum)
		lanchangecntdst=app->lanechangenum;

	for(int i = countlast;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(ob_res == -1)
		{
			pt_3D.x=pt_3D.x+(pt_3D.x_left-pt_3D.x)*lanechangecount/7.0;
			pt_3D.y=pt_3D.y+(pt_3D.y_left-pt_3D.y)*lanechangecount/7.0;
		}
		else if(ob_res == 1)
		{
			pt_3D.x=pt_3D.x+(pt_3D.x_right-pt_3D.x)*lanechangecount/7.0;
			pt_3D.y=pt_3D.y+(pt_3D.y_right-pt_3D.y)*lanechangecount/7.0;
		}
		if(i>countlast)
		{
			if(pt_3D.z==30)
			{
				pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i-1);
				if(ob_res == -1)
				{
					pt_3D_temp.x=pt_3D_temp.x+(pt_3D_temp.x_left-pt_3D_temp.x)*lanechangecount/7.0;
					pt_3D_temp.y=pt_3D_temp.y+(pt_3D_temp.y_left-pt_3D_temp.y)*lanechangecount/7.0;
				}
				else if(ob_res == 1)
				{
					pt_3D_temp.x=pt_3D_temp.x+(pt_3D_temp.x_right-pt_3D_temp.x)*lanechangecount/7.0;
					pt_3D_temp.y=pt_3D_temp.y+(pt_3D_temp.y_right-pt_3D_temp.y)*lanechangecount/7.0;
				}
				latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
				lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
				if(ob_res == -1)
				{
					latpianyi_dst=latpianyi_dst+pt_3D_temp.x_left-pt_3D.x_left;
					lngpianyi_dst=lngpianyi_dst+pt_3D_temp.y_left-pt_3D.y_left;
				}
				else if(ob_res == 1)
				{
					latpianyi_dst=latpianyi_dst+pt_3D_temp.x_right-pt_3D.x_right;
					lngpianyi_dst=lngpianyi_dst+pt_3D_temp.y_right-pt_3D.y_right;
				}
			}
		}
		if(latpianyi==0 && lngpianyi==0)
		{
			if(pt_3D.laneseq==1)
			{
				app->brightsidelane=true;
				brightsidelaneflag=true;
			}
			if(pt_3D.laneseq==pt_3D.lanenum_)
			{
				app->bleftsidelane=true;
				bleftsidelaneflag=true;
			}
		}
		temp_point[i].x=pt_3D.x+latpianyi+latpianyi_lanechgstrt;
		temp_point[i].y=pt_3D.y+lngpianyi+lonpianyi_lanechgstrt;

		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );

		/*
		if(ob_res == -1)
		{
			temp_point_dst[i].x=pt_3D.x_left+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y_left+lngpianyi_dst+lonpianyi_lanechgstrt;
		}
		else if(ob_res == 1)
		{
			temp_point_dst[i].x=pt_3D.x_right+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y_right+lngpianyi_dst+lonpianyi_lanechgstrt;
		}
		*/

		if(ob_res == -1)
		{
			temp_point_dst[i].x=pt_3D.x+(pt_3D.x_left-pt_3D.x)*lanchangecntdst/7.0+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y+(pt_3D.y_left-pt_3D.y)*lanchangecntdst/7.0+lngpianyi_dst+lonpianyi_lanechgstrt;
		}
		else if(ob_res == 1)
		{
			temp_point_dst[i].x=pt_3D.x+(pt_3D.x_right-pt_3D.x)*lanchangecntdst/7.0+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y+(pt_3D.y_right-pt_3D.y)*lanchangecntdst/7.0+lngpianyi_dst+lonpianyi_lanechgstrt;
		}

		temp_point_left[i].x=pt_3D.x_left-pt_3D.x+temp_point[i].x;
		temp_point_left[i].y=pt_3D.y_left-pt_3D.y+temp_point[i].y;

		temp_point_right[i].x=pt_3D.x_right-pt_3D.x+temp_point[i].x;
		temp_point_right[i].y=pt_3D.y_right-pt_3D.y+temp_point[i].y;
	}
	if(bleftsidelaneflag==0)
		app->bleftsidelane=false;
	if(brightsidelaneflag==0)
		app->brightsidelane=false;
	latpianyi = 0;
	lngpianyi = 0;
	latpianyi_dst = 0;
	lngpianyi_dst = 0;
	for(int i = countlast-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(ob_res == -1)
		{
			pt_3D.x=pt_3D.x+(pt_3D.x_left-pt_3D.x)*lanechangecount/7.0;
			pt_3D.y=pt_3D.y+(pt_3D.y_left-pt_3D.y)*lanechangecount/7.0;
		}
		else if(ob_res == 1)
		{
			pt_3D.x=pt_3D.x+(pt_3D.x_right-pt_3D.x)*lanechangecount/7.0;
			pt_3D.y=pt_3D.y+(pt_3D.y_right-pt_3D.y)*lanechangecount/7.0;
		}
		if(pt_3D.z==29)
		{
			pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			if(ob_res == -1)
			{
				pt_3D_temp.x=pt_3D_temp.x+(pt_3D_temp.x_left-pt_3D_temp.x)*lanechangecount/7.0;
				pt_3D_temp.y=pt_3D_temp.y+(pt_3D_temp.y_left-pt_3D_temp.y)*lanechangecount/7.0;
			}
			else if(ob_res == 1)
			{
				pt_3D_temp.x=pt_3D_temp.x+(pt_3D_temp.x_right-pt_3D_temp.x)*lanechangecount/7.0;
				pt_3D_temp.y=pt_3D_temp.y+(pt_3D_temp.y_right-pt_3D_temp.y)*lanechangecount/7.0;
			}
			latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
			lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
			if(ob_res == -1)
			{
				latpianyi_dst=latpianyi_dst+pt_3D_temp.x_left-pt_3D.x_left;
				lngpianyi_dst=lngpianyi_dst+pt_3D_temp.y_left-pt_3D.y_left;
			}
			else if(ob_res == 1)
			{
				latpianyi_dst=latpianyi_dst+pt_3D_temp.x_right-pt_3D.x_right;
				lngpianyi_dst=lngpianyi_dst+pt_3D_temp.y_right-pt_3D.y_right;
			}
		}
		temp_point[i].x=pt_3D.x+latpianyi+latpianyi_lanechgstrt;
		temp_point[i].y=pt_3D.y+lngpianyi+lonpianyi_lanechgstrt;

		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );

		/*
		if(ob_res == -1)
		{
			temp_point_dst[i].x=pt_3D.x_left+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y_left+lngpianyi_dst+lonpianyi_lanechgstrt;
		}
		else if(ob_res == 1)
		{
			temp_point_dst[i].x=pt_3D.x_right+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y_right+lngpianyi_dst+lonpianyi_lanechgstrt;
		}
		*/

		if(ob_res == -1)
		{
			temp_point_dst[i].x=pt_3D.x+(pt_3D.x_left-pt_3D.x)*lanchangecntdst/7.0+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y+(pt_3D.y_left-pt_3D.y)*lanchangecntdst/7.0+lngpianyi_dst+lonpianyi_lanechgstrt;
		}
		else if(ob_res == 1)
		{
			temp_point_dst[i].x=pt_3D.x+(pt_3D.x_right-pt_3D.x)*lanchangecntdst/7.0+latpianyi_dst+latpianyi_lanechgstrt;
			temp_point_dst[i].y=pt_3D.y+(pt_3D.y_right-pt_3D.y)*lanchangecntdst/7.0+lngpianyi_dst+lonpianyi_lanechgstrt;
		}

		temp_point_left[i].x=pt_3D.x_left-pt_3D.x+temp_point[i].x;
		temp_point_left[i].y=pt_3D.y_left-pt_3D.y+temp_point[i].y;

		temp_point_right[i].x=pt_3D.x_right-pt_3D.x+temp_point[i].x;
		temp_point_right[i].y=pt_3D.y_right-pt_3D.y+temp_point[i].y;
	}
	/*
	for(int i = 0;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		temp_point[i].x=pt_3D.x;
		temp_point[i].y=pt_3D.y;
	}
	*/
	Beziercazhi(temp_point,count+1,rndf);
	for (int i = 0;i<200;i++)
	{
		rndf[i].x =  rndf[i].x-err.x;
		rndf[i].y =  rndf[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(rndf[199],rndf[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=rndf[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(rndf[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			rndf[i].x=inter_yanchang[i].x;
			rndf[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point);

	Beziercazhi(temp_point_dst,count+1,rndfbezier_lchgdst);
	for (int i = 0;i<200;i++)
	{
		rndfbezier_lchgdst[i].x =  rndfbezier_lchgdst[i].x-err.x;
		rndfbezier_lchgdst[i].y =  rndfbezier_lchgdst[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(rndfbezier_lchgdst[199],rndfbezier_lchgdst[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=rndfbezier_lchgdst[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(rndfbezier_lchgdst[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			rndfbezier_lchgdst[i].x=inter_yanchang[i].x;
			rndfbezier_lchgdst[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point_dst);

	Beziercazhi(temp_point_left,count+1,app->rndfbezier_left);
	for (int i = 0;i<200;i++)
	{
		app->rndfbezier_left[i].x =  app->rndfbezier_left[i].x-err.x;
		app->rndfbezier_left[i].y =  app->rndfbezier_left[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(app->rndfbezier_left[199],app->rndfbezier_left[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=app->rndfbezier_left[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(app->rndfbezier_left[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			app->rndfbezier_left[i].x=inter_yanchang[i].x;
			app->rndfbezier_left[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point_left);

	Beziercazhi(temp_point_right,count+1,app->rndfbezier_right);
	for (int i = 0;i<200;i++)
	{
		app->rndfbezier_right[i].x =  app->rndfbezier_right[i].x-err.x;
		app->rndfbezier_right[i].y =  app->rndfbezier_right[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(app->rndfbezier_right[199],app->rndfbezier_right[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=app->rndfbezier_right[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(app->rndfbezier_right[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			app->rndfbezier_right[i].x=inter_yanchang[i].x;
			app->rndfbezier_right[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point_right);

	return 1;
}

int CVehicleExe::ReturnPartPoints_lukou(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int num = gpath->total;
	CvPoint2D64f pt,pt_next;
	double l = 0;
	CvPoint2D64f mpt,mpt_next;
	int count = 0;
	CvPoint2D64f *temp_point;
	int min_s = 99999999;
	CvPoint5D64f_type pt_3D;
	CvPoint5D64f_type pt_3D_temp;
	double l_last=0;
	for (int i=countlast;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
		if(l < min_s)
		{
			count = i;
			min_s = l;
		}
		if(l>300 && i>countlast)
			break;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l_last=l_last+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);

		{
			if(l_last>15 && i>countlast)
				break;
		}
	}

	CvPoint2D64f pt_count;
	pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, count);
	pt_count.x=pt_3D.x;
	pt_count.y=pt_3D.y;

	spdcurrentpnt=pt_3D.spdiniset;

	blaneusedisable=!(pt_3D.laneexist);

	for (int i=countlast;i<count;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(pt_3D.z>0 && pt_3D.z<9)
			lukou_fx=pt_3D.z;
	}

	l=0;
	bool flag_check_=0;
	for (int i=count;i<num;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		if(pt_3D.z>0 && pt_3D.z<=9)
		{
			if(pt_3D.z<9)
			{
				app->bchulukoufound=0;
				lukou_fx=pt_3D.z;
				yulukoupoint=pt;
				if((!((((lukou_fx == 1 && bZturnLightChk) || (lukou_fx == 3 && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) &&(app->light_res == 2))||bspecialzhilukoutingche))&&(l<5))
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(lukou_fx==6)
						app->bshigongluduanlock=true;
					if(lukou_fx==2 || lukou_fx==3)
						app->bguihualukou==true;
					approachyulukou=0;
					if(upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				else
				{
					if(m_task!=IVTASK_APPROINTERSECTION)
					{
						upoint[0].x=0;
						upoint[0].y=0;
					}
					upoint[1]=pt;
					m_task = IVTASK_APPROINTERSECTION;

					upointyulukou[1]=pt;
					upointyulukou[0]=pt;
					upointyulukou[2]=pt;
					upointyulukou[3]=pt;
					double lyulukoutmp=0;
					int iyulukou;
					for(iyulukou=i-1;iyulukou>=0;iyulukou--)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						ptlukou_3Dtmpnxt = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou+1);
						lyulukoutmp=lyulukoutmp+m_GpsData.GetDistance(ptlukou_3Dtmp.x,ptlukou_3Dtmp.y,ptlukou_3Dtmpnxt.x,ptlukou_3Dtmpnxt.y);
						if(lyulukoutmp>10)
						{
							upointyulukou[0].x=ptlukou_3Dtmp.x;
							upointyulukou[0].y=ptlukou_3Dtmp.y;
							break;
						}
					}
					lyulukoutmp=0;
					for(iyulukou=i+1;iyulukou<num;iyulukou++)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						if(ptlukou_3Dtmp.z==9)
						{
							spdintersection=ptlukou_3Dtmp.spdiniset;

							upointyulukou[2].x=ptlukou_3Dtmp.x;
							upointyulukou[2].y=ptlukou_3Dtmp.y;
							upointyulukou[3]=upointyulukou[2];
							break;
						}
					}
					for(iyulukou=iyulukou+1;iyulukou<num;iyulukou++)
					{
						CvPoint5D64f_type ptlukou_3Dtmp;
						ptlukou_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou);
						CvPoint5D64f_type ptlukou_3Dtmpnxt;
						ptlukou_3Dtmpnxt = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, iyulukou-1);
						lyulukoutmp=lyulukoutmp+m_GpsData.GetDistance(ptlukou_3Dtmp.x,ptlukou_3Dtmp.y,ptlukou_3Dtmpnxt.x,ptlukou_3Dtmpnxt.y);
						if(lyulukoutmp>10)
						{
							upointyulukou[3].x=ptlukou_3Dtmp.x;
							upointyulukou[3].y=ptlukou_3Dtmp.y;
							break;
						}
					}
					double labianlen=40;//40
					/*
					if(pt_3D.z==4 && m_GpsData.GetDistance(pt.x,pt.y,globalstrt.x,globalstrt.y)<200)
						labianlen=80;
						*/
					double angleerr=0;
					double lengtherr=0;
					if(i>0)
					{
						int j=i-1;
						CvPoint5D64f_type pt_3D_tmp;
						CvPoint2D64f pt_tmp;
						for (j=i-1;j>=0;j--)
						{
							pt_3D_tmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pt_tmp.x=pt_3D_tmp.x;
							pt_tmp.y=pt_3D_tmp.y;
							if(m_GpsData.GetDistance(pt.x,pt.y,pt_tmp.x,pt_tmp.y)>10)
								break;
						}
						if(j<0)
							j=0;
						pt_3D_tmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
						pt_tmp.x=pt_3D_tmp.x;
						pt_tmp.y=pt_3D_tmp.y;
						angleerr=m_GpsData.GetAngle(m_gps,pt_tmp)-m_GpsData.GetAngle(pt,pt_tmp);
						lengtherr=abs((m_GpsData.GetDistance(m_gps.x,m_gps.y,pt_tmp.x,pt_tmp.y))*(sin(angleerr*PI/180)));
					}
					if(lengtherr>11.1)
						labianlen=80;
					else if(lengtherr>7.4)
						labianlen=65;
					else if(lengtherr>3.7)
						labianlen=50;
					if((l<labianlen)||(l<80 && approachyulukou>0))
					{
						if(pt_3D.z==3 || pt_3D.z==4 || pt_3D.z==2)
							blabianqiansureq=true;
						else
							blabianqiansureq=false;
					}
					else
						blabianqiansureq=false;
					if(((l<labianlen)||(l<80 && approachyulukou>0)) && app->GPS_Speed<20/3.6)
					{
						if(pt_3D.z==3 || pt_3D.z==4)
							approachyulukou=0;
						else if(pt_3D.z==2)
							approachyulukou=0;
						else
							approachyulukou=0;
					}
					if(l<10&&upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				/*
				if(l<5&&lukou_fx==4)
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;

								spdintersection=pt_3Dtmp.spdiniset;

								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				*/
			}
			else
			{
				spdintersection=pt_3D.spdiniset;

				if(l < 60)
					app->bchulukoufound=1;
				if(!(((((lukou_fx == 1 && bZturnLightChk) || (lukou_fx == 3 && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) && app->light_res == 2)||bspecialzhilukoutingche) && m_task == IVTASK_APPROINTERSECTION))
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(lukou_fx==6)
						app->bshigongluduanlock=true;
					if(lukou_fx==2 || lukou_fx==3)
						app->bguihualukou==true;
					approachyulukou=0;
				}
			}
			flag_check_=1;
			break;
		}
		if(i+1<num)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			pt_next.x=pt_3D.x;
			pt_next.y=pt_3D.y;
			l = l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		}
		if((l > 280)||((app->bchulukoufound==1)&&(l>80)))
		{
			m_task = IVTASK_LANE;
			flag_check_=1;
			approachyulukou=0;
			break;
		}
	}
	if(flag_check_==0)
	{
		m_task = IVTASK_LANE;
		approachyulukou=0;
	}

	flag_check_=0;
	l=0;
	for (int i=count;i<num;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		if(pt_3D.z>0&&pt_3D.z<9)
		{
			flag_check_=1;
			app->lukoulanechangereq=1;
			break;
		}
		if(i+1<num)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			pt_next.x=pt_3D.x;
			pt_next.y=pt_3D.y;
			l = l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		}
		if(l > 120)
		{
			flag_check_=2;
			app->lukoulanechangereq=0;
			break;
		}
	}
	if(flag_check_==0)
		app->lukoulanechangereq=0;
	/*
	if(flag_check_==0)
		app->lukoulanechangereq=2;
		*/

	l=0;
	int countstrt=0;
	for(int i=count-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l=l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>s)
		{
			countstrt=i;
			break;
		}
	}

	countlast=count-countstrt;

	for(int i = 0;i<countstrt;i++)
	{
		cvSeqPopFront( gpath,NULL );
	}

	num = gpath->total;
	if(num<2)
		return 0;
	l=0;
	count = num-2;
	double sfront=100;
	if(m_GpsData.GetDistance(31.32436573513,121.23512638759,m_gps.x,m_gps.y)<50)
		sfront=50;
	for(int i = 0;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l+= m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>sfront+s)
		{
			count = i;
			break;
		}
		else if(i>countlast-1 && i<num-2 && m_task==IVTASK_APPROINTERSECTION)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+2);
			if(pt_3D.z>0 && pt_3D.z<9)
			{
				count = i;
				break;
			}
			else
			{
				pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
				if(pt_3D.z>0 && pt_3D.z<9)
				{
					count = i;
					break;
				}
				else
				{
					pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
					if(pt_3D.z>0 && pt_3D.z<9)
					{
						count = i;
						break;
					}
				}
			}
		}
	}
	bool bleftsidelaneflag=0;
	bool brightsidelaneflag=0;
	double latpianyi = 0;
	double lngpianyi = 0;
	temp_point = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	CvPoint2D64f *temp_point_left = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	CvPoint2D64f *temp_point_right = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	for(int i = countlast;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(i>countlast)
		{
			if(pt_3D.z==30)
			{
				pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i-1);
				latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
				lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
			}
		}
		if(latpianyi==0 && lngpianyi==0)
		{
			if(pt_3D.laneseq==1)
			{
				app->brightsidelane=true;
				brightsidelaneflag=true;
			}
			if(pt_3D.laneseq==pt_3D.lanenum_)
			{
				app->bleftsidelane=true;
				bleftsidelaneflag=true;
			}
		}
		temp_point[i].x=pt_3D.x+latpianyi;
		temp_point[i].y=pt_3D.y+lngpianyi;

		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );

		temp_point_left[i].x=pt_3D.x_left-pt_3D.x+temp_point[i].x;
		temp_point_left[i].y=pt_3D.y_left-pt_3D.y+temp_point[i].y;

		temp_point_right[i].x=pt_3D.x_right-pt_3D.x+temp_point[i].x;
		temp_point_right[i].y=pt_3D.y_right-pt_3D.y+temp_point[i].y;
	}
	if(bleftsidelaneflag==0)
		app->bleftsidelane=false;
	if(brightsidelaneflag==0)
		app->brightsidelane=false;
	latpianyi = 0;
	lngpianyi = 0;
	for(int i = countlast-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(pt_3D.z==29)
		{
			pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
			lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
		}
		temp_point[i].x=pt_3D.x+latpianyi;
		temp_point[i].y=pt_3D.y+lngpianyi;

		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );

		temp_point_left[i].x=pt_3D.x_left-pt_3D.x+temp_point[i].x;
		temp_point_left[i].y=pt_3D.y_left-pt_3D.y+temp_point[i].y;

		temp_point_right[i].x=pt_3D.x_right-pt_3D.x+temp_point[i].x;
		temp_point_right[i].y=pt_3D.y_right-pt_3D.y+temp_point[i].y;
	}
	/*
	for(int i = 0;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		temp_point[i].x=pt_3D.x;
		temp_point[i].y=pt_3D.y;
	}
	*/
	Bezierlukou(temp_point,count+1,rndf);
	for (int i = 0;i<200;i++)
	{
		rndf[i].x =  rndf[i].x-err.x;
		rndf[i].y =  rndf[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(rndf[199],rndf[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=rndf[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(rndf[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			rndf[i].x=inter_yanchang[i].x;
			rndf[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point);

	Beziercazhi(temp_point_left,count+1,app->rndfbezier_left);
	for (int i = 0;i<200;i++)
	{
		app->rndfbezier_left[i].x =  app->rndfbezier_left[i].x-err.x;
		app->rndfbezier_left[i].y =  app->rndfbezier_left[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(app->rndfbezier_left[199],app->rndfbezier_left[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=app->rndfbezier_left[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(app->rndfbezier_left[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			app->rndfbezier_left[i].x=inter_yanchang[i].x;
			app->rndfbezier_left[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point_left);

	Beziercazhi(temp_point_right,count+1,app->rndfbezier_right);
	for (int i = 0;i<200;i++)
	{
		app->rndfbezier_right[i].x =  app->rndfbezier_right[i].x-err.x;
		app->rndfbezier_right[i].y =  app->rndfbezier_right[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(app->rndfbezier_right[199],app->rndfbezier_right[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=app->rndfbezier_right[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(app->rndfbezier_right[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			app->rndfbezier_right[i].x=inter_yanchang[i].x;
			app->rndfbezier_right[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point_right);

	return 1;
}

int CVehicleExe::ReturnPartPoints_Ini(CvSeq *gpath,CvPoint2D64f m_gps,double dir,double s, CvPoint2D64f (&rndf)[200],CvPoint2D64f err)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int num = gpath->total;
	CvPoint2D64f pt,pt_next;
	double l = 0;
	CvPoint2D64f mpt,mpt_next;
	int count = 0;
	CvPoint2D64f *temp_point;
	int min_s = 99999999;
	CvPoint5D64f_type pt_3D;
	CvPoint5D64f_type pt_3D_temp;
	double l_ini=0;
	for (int i=0;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
		pt_3D_temp=pt_3D;
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
		if(pt_3D.z==29)
		{
			for(i=i+1;i<num-1;i++)
			{
				pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
				pt.x=pt_3D.x;
				pt.y=pt_3D.y;
				l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
				if(pt_3D.z==30 || i==num-2)
				{
					break;
				}
			}
		}
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		double roadangle = m_GpsData.GetAngle(pt_next.x,pt_next.y,pt.x,pt.y);
		if((l < min_s)&&(cos(PI*(roadangle - dir)/180)>cos(50*PI/180)))
		{
			count = i;
			min_s = l;
		}
		l_ini=l_ini+m_GpsData.GetDistance(pt_3D_temp.x,pt_3D_temp.y,pt_next.x,pt_next.y);
		if(l_ini>250)
			break;
	}

	pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, count);
	pt.x=pt_3D.x;
	pt.y=pt_3D.y;
	l = m_GpsData.GetDistance(pt.x,pt.y,m_gps.x,m_gps.y);
	if(globalstrt.x==0 && globalstrt.y==0)
	{
		globalstrt.x=pt.x;
		globalstrt.y=pt.y;
	}
	if(l>50)
		return 0;

	for (int i=0;i<count;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(pt_3D.z>0 && pt_3D.z<9)
		{
			lukou_fx=pt_3D.z;
			upoint[1].x=pt_3D.x;
			upoint[1].y=pt_3D.y;
			if(i-5>=0)
			{
				pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i-5);
				upoint[0].x=pt_3D.x;
				upoint[0].y=pt_3D.y;
			}
			else if(i>0)
			{
				pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, 0);
				upoint[0].x=pt_3D.x;
				upoint[0].y=pt_3D.y;
			}
		}
	}

	CvPoint2D64f pt_count;
	pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, count);
	pt_count.x=pt_3D.x;
	pt_count.y=pt_3D.y;
	l=0;
	for (int i=count;i<num;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		if(pt_3D.z>0 && pt_3D.z<=9)
		{
			if(pt_3D.z<9)
			{
				app->bchulukoufound=0;
				lukou_fx=pt_3D.z;
				yulukoupoint=pt;
				if(m_task!=IVTASK_APPROINTERSECTION)
				{
					upoint[0].x=0;
					upoint[0].y=0;
				}
				upoint[1]=pt;
				m_task = IVTASK_APPROINTERSECTION;
				if(l>=10)
				{
					upoint[0].x=0;
					upoint[0].y=0;
				}
				if(l<10&&upoint[0].x==0&&upoint[0].y==0)
				{
					upoint[0]=pt_count;
					upoint[1]=pt;
					upoint[2]=pt;
					upoint[3]=pt;
					for(int j=i+1;j<num;j++)
					{
						CvPoint5D64f_type pt_3Dtmp;
						CvPoint2D64f pttmp;
						pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
						pttmp.x=pt_3Dtmp.x;
						pttmp.y=pt_3Dtmp.y;
						if(pt_3Dtmp.z==9)
						{
							upoint[2]=pttmp;
							if(j+5<num)
							{
								pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
								pttmp.x=pt_3Dtmp.x;
								pttmp.y=pt_3Dtmp.y;
								upoint[3]=pttmp;
							}
							else
							{
								pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
								pttmp.x=pt_3Dtmp.x;
								pttmp.y=pt_3Dtmp.y;
								upoint[3]=pttmp;
							}
							break;
						}
					}
				}
				/*
				if(l<5&&lukou_fx==4)
				{
					m_task = IVTASK_INTERSECTION;
					if(lukou_fx==4)
						app->u_t=true;
					if(upoint[0].x==0&&upoint[0].y==0)
					{
						upoint[0]=pt_count;
						upoint[1]=pt;
						upoint[2]=pt;
						upoint[3]=pt;
						for(int j=i+1;j<num;j++)
						{
							CvPoint5D64f_type pt_3Dtmp;
							CvPoint2D64f pttmp;
							pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j);
							pttmp.x=pt_3Dtmp.x;
							pttmp.y=pt_3Dtmp.y;
							if(pt_3Dtmp.z==9)
							{
								upoint[2]=pttmp;
								if(j+5<num)
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, j+5);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								else
								{
									pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
									pttmp.x=pt_3Dtmp.x;
									pttmp.y=pt_3Dtmp.y;
									upoint[3]=pttmp;
								}
								break;
							}
						}
					}
				}
				*/
			}
			else
			{
				if(l < 60)
					app->bchulukoufound=1;
				if(!(((((lukou_fx == 1 && bZturnLightChk) || (lukou_fx == 3 && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) && app->light_res == 2)||bspecialzhilukoutingche) && m_task == IVTASK_APPROINTERSECTION))
				{
					m_task = IVTASK_INTERSECTION;

					CvPoint5D64f_type pt_3Dtmp;
					upoint[2]=pt;
					if(i+5<num)
					{
						pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+5);
						upoint[3].x=pt_3Dtmp.x;
						upoint[3].y=pt_3Dtmp.y;
					}
					else if(i<num-1)
					{
						pt_3Dtmp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, num-1);
						upoint[3].x=pt_3Dtmp.x;
						upoint[3].y=pt_3Dtmp.y;
					}

					if(lukou_fx==4)
						app->u_t=true;
					if(lukou_fx==6)
						app->bshigongluduanlock=true;
					if(lukou_fx==2 || lukou_fx==3)
						app->bguihualukou==true;
				}
			}
			break;
		}
		if(i+1<num)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			pt_next.x=pt_3D.x;
			pt_next.y=pt_3D.y;
			l = l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		}
		if(l > 80)
		{
			m_task = IVTASK_LANE;
			break;
		}
	}

	if(m_task==IVTASK_LANE)
	{
		upoint[0].x=0;
		upoint[0].y=0;
		upoint[1].x=0;
		upoint[1].y=0;
	}

	l=0;
	int countstrt=0;
	for(int i=count-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l=l+m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>s)
		{
			countstrt=i;
			break;
		}
	}

	countlast=count-countstrt;

	for(int i = 0;i<countstrt;i++)
	{
		cvSeqPopFront( gpath,NULL );
	}

	num = gpath->total;
	if(num<2)
		return 0;
	l=0;
	count = num-2;
	for(int i = 0;i<num-1;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		pt.x=pt_3D.x;
		pt.y=pt_3D.y;
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
		pt_next.x=pt_3D.x;
		pt_next.y=pt_3D.y;
		l+= m_GpsData.GetDistance(pt.x,pt.y,pt_next.x,pt_next.y);
		if(l>100+s)
		{
			count = i;
			break;
		}
		else if(i>countlast-1 && i<num-2 && m_task==IVTASK_APPROINTERSECTION)
		{
			pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+2);
			if(pt_3D.z>0 && pt_3D.z<9)
			{
				count = i;
				break;
			}
			else
			{
				pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
				if(pt_3D.z>0 && pt_3D.z<9)
				{
					count = i;
					break;
				}
				else
				{
					pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i);
					if(pt_3D.z>0 && pt_3D.z<9)
					{
						count = i;
						break;
					}
				}
			}
		}
	}
	bool bleftsidelaneflag=0;
	bool brightsidelaneflag=0;
	double latpianyi = 0;
	double lngpianyi = 0;
	temp_point = (CvPoint2D64f *)malloc( (count+2)*sizeof(CvPoint2D64f) );
	for(int i = countlast;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(i>countlast)
		{
			if(pt_3D.z==30)
			{
				pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i-1);
				latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
				lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
			}
		}
		if(latpianyi==0 && lngpianyi==0)
		{
			if(pt_3D.laneseq==1)
			{
				app->brightsidelane=true;
				brightsidelaneflag=true;
			}
			if(pt_3D.laneseq==pt_3D.lanenum_)
			{
				app->bleftsidelane=true;
				bleftsidelaneflag=true;
			}
		}
		temp_point[i].x=pt_3D.x+latpianyi;
		temp_point[i].y=pt_3D.y+lngpianyi;
	}
	if(bleftsidelaneflag==0)
		app->bleftsidelane=false;
	if(brightsidelaneflag==0)
		app->brightsidelane=false;
	latpianyi = 0;
	lngpianyi = 0;
	for(int i = countlast-1;i>=0;i--)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		if(pt_3D.z==29)
		{
			pt_3D_temp = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i+1);
			latpianyi=latpianyi+pt_3D_temp.x-pt_3D.x;
			lngpianyi=lngpianyi+pt_3D_temp.y-pt_3D.y;
		}
		temp_point[i].x=pt_3D.x+latpianyi;
		temp_point[i].y=pt_3D.y+lngpianyi;
	}
	/*
	for(int i = 0;i<count+2;i++)
	{
		pt_3D = *(CvPoint5D64f_type*)cvGetSeqElem(gpath, i );
		temp_point[i].x=pt_3D.x;
		temp_point[i].y=pt_3D.y;
	}
	*/
	Beziercazhi(temp_point,count+1,rndf);
	for (int i = 0;i<200;i++)
	{
		rndf[i].x =  rndf[i].x-err.x;
		rndf[i].y =  rndf[i].y-err.y;
	}
	if(m_task == IVTASK_APPROINTERSECTION)
	{
		double yan_dir =  m_GpsData.GetAngle(rndf[199],rndf[195]);
		CvPoint2D64f inter_yanchang[200];
		for (int i = 0;i<100;i++)
		{
			inter_yanchang[i]=rndf[2*i];
		}
		for(int i=100;i<200;i++)
		{
			inter_yanchang[i]=m_GpsData.MaptoGPS(rndf[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
		}
		for (int i = 0;i<200;i++)
		{
			rndf[i].x=inter_yanchang[i].x;
			rndf[i].y=inter_yanchang[i].y;
		}
	}
	free(temp_point);
	return 1;
}

int CVehicleExe::Yulukou_yanchang(CvPoint2D64f (&rndf)[200],double length)
{					
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	LEAD current_lead1,current_lead2;
		current_lead1 = LeadPoint[seq_num-1];
		current_lead2 = LeadPoint[seq_num-2];
		CvPoint2D64f leadpt1,leadpt2;
		leadpt1.x = current_lead1.lat;leadpt1.y = current_lead1.lng;
		leadpt2.x = current_lead2.lat;leadpt2.y = current_lead2.lng;
		double angle1 = m_GpsData.GetAngle(leadpt1.x,leadpt1.y,leadpt2.x,leadpt2.y);
		CvPoint2D64f aaa;
		double l = 0;
		CvPoint2D64f yuyanchang[200];
		int xiabiao = 0;
		double s = 9999999999;
		for(int i = 0;i<200;i++)
		{
			l = m_GpsData.GetDistance(app->GPS_Point.x,app->GPS_Point.y,rndf[i].x,rndf[i].y);
			if(l < s)
			{
				xiabiao = i;
				s = l;
			}
		}
		for (int i=0;i<200;i++)
		{
			//rndf[i] = m_GpsData.MaptoGPS(rndf[xiabiao],angle1,cvPoint2D64f(256,412-(i*length*5)/200));
			rndf[i] = m_GpsData.MaptoGPS(rndf[xiabiao],angle1,cvPoint2D64f(256,511-(i*length*5)/200));
		}

		//***************************************
		//for (int i = 0;i<200;i++)
		//{
		//	yuyanchang[i].x = 0;
		//	yuyanchang[i].y = 0;
		//}

		//for (int i = 0;i<100;i++)
		//{
		//	yuyanchang[i]=rndf[2*i];
		//}
		//for(int i=100;i<200;i++)
		//{
		//	yuyanchang[i]=m_GpsData.MaptoGPS(rndf[199],angle1,cvPoint2D64f(256,412-(i-100)*2));
		//}

		//for (int i = 0;i<200;i++)
		//{
		//	rndf[i].x = yuyanchang[i].x;
		//	rndf[i].y = yuyanchang[i].y;
		//}
		return 1;

}

void CVehicleExe::ReturnNeedPoints(CvSeq *adaspoint, CvPoint2D64f m_gps, double dir, double s)
{
	int num = adaspoint->total;
	outUseAdasPoint<<"num of points:"<<num<<endl;
	CvPoint2D64f p1,p1_next,p1next_next;
	double angle1=0,angle2=0;
	double dis = 0.0;
	CvPoint2D64f mp1,mp1_next;
	int count = 0;

	LEAD gpsp1,gpsp1_next;

	for (int i=0;i<num-1;i++)
	{
		gpsp1 = *(LEAD*)cvGetSeqElem( adaspoint, i);
		gpsp1_next = *(LEAD*)cvGetSeqElem( adaspoint, i+1);
		p1.x = gpsp1.lat;
		p1.y = gpsp1.lng;
		p1_next.x = gpsp1_next.lat;
		p1_next.y = gpsp1_next.lng;
		angle1 = m_GpsData.GetAngle(p1_next.x, p1_next.y, p1.x, p1.y);
		dis = m_GpsData.GetDistance(p1.x,p1.y,m_gps.x,m_gps.y);
		mp1 = m_GpsData.APiontConverD(m_gps,p1,dir);
		mp1_next = m_GpsData.APiontConverD(m_gps,p1_next,dir);
		if(dis < s && (mp1_next.y<=mp1.y))
		{
			//if (gpsp1.param1 == 3 && gpsp1_next.param1 == 2)
			//{
			//	count = i+1;
			//	break;
			//}
			count = i;
			break;
		}
	}
	for(int i = 0;i<count;i++)
	{
		cvSeqPopFront( adaspoint,NULL );
	}
    //***************//
	//LEAD gpsp1,gpsp1_next,gpsp1next_next;
	//for (int i=1;i<num-1;i++)
	//{
	//	gpsp1 = *(LEAD*)cvGetSeqElem( adaspoint, i-1);
	//	gpsp1_next = *(LEAD*)cvGetSeqElem( adaspoint, i);
	//	gpsp1next_next = *(LEAD*)cvGetSeqElem( adaspoint, i+1);
	//	p1.x = gpsp1.lat;
	//	p1.y = gpsp1.lng;
	//	p1_next.x = gpsp1_next.lat;
	//	p1_next.y = gpsp1_next.lng;
	//	p1next_next.x = gpsp1next_next.lat;
	//	p1next_next.y = gpsp1next_next.lng;

	//	angle1 = m_GpsData.GetAngle(p1_next.x, p1_next.y, p1.x, p1.y);
	//	angle2 = m_GpsData.GetAngle(p1next_next.x, p1next_next.y, p1_next.x, p1_next.y);

	//	dis = m_GpsData.GetDistance(p1.x,p1.y,m_gps.x,m_gps.y);

	//	mp1 = m_GpsData.APiontConverD(m_gps,p1,dir);
	//	mp1_next = m_GpsData.APiontConverD(m_gps,p1_next,dir);
	//	if(dis < s && (mp1_next.y<=mp1.y))
	//	{
	//		count = i;
	//		break;
	//	}
	//}
	//for(int i = 0;i<count;i++)
	//{
	//	cvSeqPopFront( adaspoint,NULL );
	//}
	//***************//


	LEAD p;
	int num2 = adaspoint->total;
	outUseAdasPoint<<"num of using points:"<<num2<<endl;
	for(int i=1;i<=num2;i++)
	{
		p = *(LEAD*)cvGetSeqElem(adaspoint,i-1);
		outUseAdasPoint<<setprecision(11)<<p.id<<"	"<<p.lng<<"	"<<p.lat<<"	"<<p.param1<<endl;
	}
}
//*******添加ADASIS数据处理线程*******//
//void CVehicleExe::StartADASIS()
//{
//	m_hThreadADASIS = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CVehicleExe::theADASISThread,
//		this, 0, &dwADASISThreadId);
//}
//
//DWORD CVehicleExe::theADASISThread(LPVOID lpParam)
//{
//	return (((CVehicleExe*)lpParam)->ADASISThread());
//}

//DWORD CVehicleExe::ADASISThread()
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	ADASIS_Receiver.OpenADASISCAN();
//	Sleep(1000);
//	LEAD lead_ADASpoint;
//
//	NODE<RNPoint> *p = ADASIS_Receiver.m_rnPts.front;  
//	int size_p = ADASIS_Receiver.m_rnPts.Size();
//
//	int i = 1;
//	while (p != ADASIS_Receiver.m_rnPts.rear)  
//	{  
//
//		lead_ADASpoint.id = i;
//		lead_ADASpoint.height = p->data.height;
//		lead_ADASpoint.lat = p->data.lat;
//		lead_ADASpoint.lng = p->data.lng;
//		lead_ADASpoint.param1 = p->data.type;
//		app->Lead_pt.push(lead_ADASpoint);
//		i++;
//		p = p->next;
//
//	}
//
//	int point_size = app->Lead_pt.size();
//	for (int i=0;i<point_size;i++)
//	{
//		LEAD temp;
//		temp = app->Lead_pt.front();
//		app->Lead_pt.pop();
//		outADASISPoint<<temp.id<<"	"<<temp.lat<<"	"<<temp.lng<<"	"<<temp.height<<"	"<<temp.param1<<endl;
//	}
//	return 0;
//
//}
//*******添加ADASIS数据处理线程*******//

void CVehicleExe::lukoujiashi()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_Gps_ = app->GPS_Point;
	double m_Dir_ = app->GPS_Direction;
	app->critical_section.Unlock();
	
	double rulukouangle = m_GpsData.GetAngle(upoint[1].x,upoint[1].y,upoint[0].x,upoint[0].y);

	bool brightjiaocalukoustop=0;

	CvPoint2D64f pianyiini=pianyi;

	//if(lukou_fx==1 || (app->bbanmaxiannoraozhang && (lukou_fx==2)))
	if(lukou_fx==1)
	{
		MoveWay(m_Gps_,m_Dir_,CrossPath,pianyi);

		double dx = pianyi.x - pianyiini.x;
		double dy = pianyi.y - pianyiini.y;
		for(int i=0;i<200;i++)
		{
			app->rndfbezier_left[i].x -= dx;
			app->rndfbezier_left[i].y -= dy;

			app->rndfbezier_right[i].x -= dx;
			app->rndfbezier_right[i].y -= dy;
		}
	}
	int i_pianyitmp=0;
	int npiamyimin=0;
	double mindis_pianyi = 100000;
	double dis_tmp=0;

	int nrulanelast=199;

	for(i_pianyitmp=0;i_pianyitmp<200;i_pianyitmp++)
	{
		dis_tmp = m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,CrossPath[i_pianyitmp].x,CrossPath[i_pianyitmp].y);
		if(dis_tmp < mindis_pianyi)
		{
			mindis_pianyi= dis_tmp;
			npiamyimin=i_pianyitmp;
		}
	}

	for(int i=npiamyimin;i<195;i++)
	{
		double pntangle = m_GpsData.GetAngle(CrossPath[i+5].x,CrossPath[i+5].y,CrossPath[i].x,CrossPath[i].y);
		if((cos(PI*(pntangle - rulukouangle)/180)<cos(30*PI/180)))
		{
			nrulanelast=i+5;
			break;
		}
		else
			nrulanelast=i+5;
	}

	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_Gps_,CrossPath[i],m_Dir_);
		if(m_Gps_.x==CrossPath[i].x&&m_Gps_.y==CrossPath[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}
	int m_up = 262;
	int m_down = 412;
	
	int x = 0;
	int y = 0;

	int width2 = 5;
	int m;
	int n;
	int a=8;
	int b=18;
	int c=28;
	double mindist=0;
	double drivespdtmp=road_speed;
	bool break_flag=0;
	for(int i=npiamyimin;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(m=5;m>=-5;m--)
		{
			for(n=-5;n<=5;n++)
			{
				if(y+m>=412||y+m<0||x+n>511||x+n<0)
					continue;
				if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b||vel_Map->MapPoint[y+m][x+n] == c)
				{
					mindist = sqrt(double((x+n-256)*(x+n-256)+(y+m-412)*(y+m-412)))/5;
					drivespdtmp = (mindist-3)/3.6;
					if(drivespdtmp<5/3.6)
						drivespdtmp=5/3.6;
					else if(drivespdtmp>road_speed)
						drivespdtmp=road_speed;
					if(app->drive_obstacle>drivespdtmp)
						app->drive_obstacle=drivespdtmp;
					if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(mindist>=4)&&((app->GPS_Speed>drivespdtmp-2.5/3.6)||(app->GPS_Speed>12.5/3.6)))
					{
						app->continue_spdredtmp = true;
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed)-5/3.6*5/3.6)/(2*(mindist-3));
						app->continue_braketmp = true;
						if(spdacctemp<-3)
							spdacctemp=-3;
						if(spdacctemp<app->spdredacctmp)
							app->spdredacctmp = spdacctemp;
					}
					else if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(mindist<4))
					{
						app->continue_spdredtmp = true;
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed)-5/3.6*5/3.6)/2;
						app->continue_braketmp = true;
						if(spdacctemp<-3)
							spdacctemp=-3;
						if(spdacctemp<app->spdredacctmp)
							app->spdredacctmp = spdacctemp;
					}
					break_flag=1;
					break;
				}
			}
			if(break_flag)
				break;
		}
		if(break_flag)
			break;
	}
	double lanenochangedist;
	double lanenochangedist_;
	double middle_dis;

	lanenochangedist=6;
	double enddis=5;

	//middle_dis=lanedriverstate.SearchFrontOb(vel_Map,lanenochangedist);
	middle_dis=pathplan.SearchFrontOb(vel_Map,lanenochangedist);
	if(middle_dis>0)
	{
		if(middle_dis>enddis)
			drivespdtmp=5/3.6;
		else
			drivespdtmp=0;

		if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(middle_dis>=5))
		{
			app->continue_spdredtmp = true;
			app->continue_braketmp = true;
			spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-4));
		}
		else if((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis<enddis))
		{
			app->continue_spdredtmp = true;
			app->continue_braketmp = true;
			spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
		}

		if(spdacctemp<app->spdredacc)
		{
			if(app->spdredacc<-3)
				spdacctemp=app->spdredacc;
			else if(spdacctemp<-3)
				spdacctemp=-3;
		}
		if(spdacctemp<app->spdredacctmp)
			app->spdredacctmp = spdacctemp;

		if(app->drive_obstacle>drivespdtmp)
			app->drive_obstacle=drivespdtmp;
	}

	if(!((app->bbanmaxiannoraozhang)&&(app->bGpsGanrao==0)))
	{
		app->obb_left = false;
		app->obb_right = false;
	}
	else
	{
		app->obb_left=true;
		app->obb_right=true;
	}
}

void CVehicleExe::zhixinglukoujiashi()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_Gps_ = app->GPS_Point;
	double m_Dir_ = app->GPS_Direction;
	app->critical_section.Unlock();

	CvPoint2D64f pianyiini=pianyi;

	//if(enterlukou==0 && uturnlukoucnt<2)
	//if(enterlukou==0)
	{
		MoveWay(m_Gps_,m_Dir_,CrossPath,pianyi);

		double dx = pianyi.x - pianyiini.x;
		double dy = pianyi.y - pianyiini.y;
		for(int i=0;i<200;i++)
		{
			app->rndfbezier_left[i].x -= dx;
			app->rndfbezier_left[i].y -= dy;

			app->rndfbezier_right[i].x -= dx;
			app->rndfbezier_right[i].y -= dy;
		}
	}

	int i_pianyitmp=0;
	int npiamyimin=0;
	double mindis_pianyi = 100000;
	double dis_tmp=0;
	for(i_pianyitmp=0;i_pianyitmp<200;i_pianyitmp++)
	{
		dis_tmp = m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,CrossPath[i_pianyitmp].x,CrossPath[i_pianyitmp].y);
		if(dis_tmp < mindis_pianyi)
		{
			mindis_pianyi= dis_tmp;
			npiamyimin=i_pianyitmp;
		}
	}
	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_Gps_,CrossPath[i],m_Dir_);
		if(m_Gps_.x==CrossPath[i].x&&m_Gps_.y==CrossPath[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}
	int m_up = 262;
	int m_down = 412;
	
	int x = 0;
	int y = 0;

	int width2 = 5;
	int m;
	int n;
	int a=8;
	int b=18;
	int c=28;
	double mindist=0;
	double drivespdtmp=road_speed;
	bool break_flag=0;

	double lanenochangedist;
	double lanenochangedist_;
	double middle_dis;

	bool bholdbrake=0;

	middle_dis=0;
	drivespdtmp=road_speed;

	double obfrontspd=0;

	double updett = app->GPS_Speed*3.6+12;

	if(updett<15)
		updett=15;
	else if(updett>80)
		updett=80;

	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();

	//double middle_dis_front=lanedriverstate.SearchFrontOb(vel_Map,6);
	double middle_dis_front=pathplan.SearchFrontOb(vel_Map,6);
	if(middle_dis_front==0)
		middle_dis_front=100;
	
	double len = 0;
	double min_len = 99999;
	n = 0;
	for(int i = 0;i<200;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,CrossPath[i].x,CrossPath[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	if(n==0)
		n=1;

	int chadiannum=0;
	for(int i=0;i<199;i++)
	{
		for(int kk=0;kk<10;kk++)
		{
			rndfbezierchadian[chadiannum].x=(CrossPath[i].x*(10-kk)+CrossPath[i+1].x*kk)*0.1;
			rndfbezierchadian[chadiannum].y=(CrossPath[i].y*(10-kk)+CrossPath[i+1].y*kk)*0.1;
			chadiannum++;
		}
	}
	rndfbezierchadian[chadiannum]=CrossPath[199];

	n=(n-1)*10;

	//ob_dis_prepre=ob_dis_pre;
	//ob_dis_pre=ob_dis_now;

	for(int i=0;i<1991;i++)
	{
		Rndf_MapPointchadian[i] = m_GpsData.APiontConverD(m_gps,rndfbezierchadian[i],m_gpsdir);
		if(m_gps.x==rndfbezierchadian[i].x&&m_gps.y==rndfbezierchadian[i].y)
		{
			Rndf_MapPointchadian[i].x = 256;
			Rndf_MapPointchadian[i].y = 411;
		}
	}
	m_up = 12;
	m_down = 412;

	a=8;
	b=18;
	c=28;
	
	x = 0;
	y = 0;

	width2 = 4;
	if (app->range_flag)
	{
		width2 = 4;
	}
	bool obb_break_flag=0;
	for(int i = n;i<1991;i++)
	{
		if(Rndf_MapPointchadian[i].x < 0||Rndf_MapPointchadian[i].x > 511||Rndf_MapPointchadian[i].y > m_down||Rndf_MapPointchadian[i].y < m_up)
			continue;
		x = Rndf_MapPointchadian[i].x;
		y = Rndf_MapPointchadian[i].y;
		for(int m=4;m>=-4;m--)
		{
			for(int n=width_L;n<=width_R;n++)
			{
				if(y+m>=412)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b||vel_Map->MapPoint[y+m][x+n] == c)
				{
					ob_dis_now=412-(y+m);
					//ob_dis_now_timer=app->systemtimer10ms;

					SYSTEMTIME tt;
					GetLocalTime(&tt);
					ob_dis_now_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

					obb_break_flag=1;

					if(ob_dis_pre<1000)
					{
						//if(abs(ob_dis_now-ob_dis_pre)>30*(ob_loss_count+1))
						if(abs(ob_dis_now-ob_dis_pre)*0.2>30*(ob_dis_now_timer-ob_dis_pre_timer)*0.001)
						{
							ob_dis_pre=1000;
							ob_dis_prepre=1000;
							bfasterobacv=0;
						}
					}
					if(ob_dis_prepre<1000)
					{
						//if(abs(ob_dis_now-ob_dis_pre)>30*(ob_loss_count+1))
						if(abs(ob_dis_now-ob_dis_prepre)*0.2>30*(ob_dis_now_timer-ob_dis_prepre_timer)*0.001)
						{
							ob_dis_pre=1000;
							ob_dis_prepre=1000;
							bfasterobacv=0;
						}
					}

					if(ob_dis_pre==1000 && ob_dis_prepre==1000)
						bobfisrstapper=1;
					else if(bobfisrstapper)
					{
						if(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=0)
							bobfisrstapper=1;
						else if(ob_dis_prepre==1000 && ob_dis_pre<1000 && ob_dis_now-ob_dis_pre>=0)
							bobfisrstapper=1;
						else
							bobfisrstapper=0;
					}
					else
						bobfisrstapper=0;

					if((ob_dis_pre<1000 && ob_dis_now-ob_dis_pre>=1)||(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=1))
					{
						if(bfasterobacv==0)
						{
							bfasterobacv=1;
							fasterobspdini=app->GPS_Speed;
						}
					}

					bholdbrake=0;
					/*
					if(ob_dis_pre<1000 && ob_dis_now>41)
					{
						if(ob_dis_now-ob_dis_pre>=1)
							bholdbrake=1;
						else if((ob_dis_now-ob_dis_pre)-(ob_dis_pre-ob_dis_prepre)>2)
						{
							if((ob_dis_now-ob_dis_pre)-(ob_dis_pre-ob_dis_prepre)>-(ob_dis_now-ob_dis_pre)*(1-(ob_dis_now-ob_dis_pre))/(2*(ob_dis_now-40))+1)
								bholdbrake=1;
						}
					}
					*/

					if(bfasterobacv &&((abs(412-(y+m))*0.2)>15) && ob_dis_pre<1000 && ob_dis_now-ob_dis_pre>=1 && ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=3)
					{
						if(fasterobspdini<app->GPS_Speed+3.5/3.6)
							fasterobspdini=app->GPS_Speed+3.5/3.6;
						if(fasterobspdini>road_speed)
							fasterobspdini=road_speed;
					}

					obfrontspd=dynamicmap_v_x->MapPoint[y+m][x+n];

					if(ob_dis_now<75)
					{
						if((ob_dis_pre<1000 && ob_dis_now-ob_dis_pre<0)||(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<0))
						{
							if(bfasterobacv==1)
							{
								bfasterobacv=0;
							}
						}
					}
					else
					{
						if((ob_dis_pre<1000 && ob_dis_now-ob_dis_pre<-1)||(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<-1))
						{
							if(bfasterobacv==1)
							{
								bfasterobacv=0;
							}
						}
					}

					if(ob_dis_prepre<1000)
					{
						if(ob_dis_now_timer>ob_dis_prepre_timer)
							obfrontspd=(ob_dis_now-ob_dis_prepre-1)*0.2/(0.001*(ob_dis_now_timer-ob_dis_prepre_timer));
						else
							obfrontspd=(ob_dis_now-ob_dis_prepre-1)*0.2/(0.001);
						/*
						obfrontspd=(ob_dis_now-ob_dis_pre)*0.2/(0.05*(ob_loss_count+1));
						if(obfrontspd>-2*0.2/(0.05*(ob_loss_count+1)))
							obfrontspd=-2*0.2/(0.05*(ob_loss_count+1));
							*/
					}

					middle_dis = abs(412-(y+m))*0.2;
					if(middle_dis>=updett && able_ChangeLane)
					{
						drivespdtmp=road_speed;
						if(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<=0)
						{
							drivespdtmp = (middle_dis-12)/3.6;
							if(middle_dis<=updett+11)
							{
								if(drivespdtmp<app->GPS_Speed)
								{
									drivespdtmp=app->GPS_Speed-2.5/3.6;
									if(drivespdtmp<(middle_dis-12)/3.6)
										drivespdtmp=(middle_dis-12)/3.6;
								}
							}
							else
							{
								if(drivespdtmp<=app->GPS_Speed)
									drivespdtmp=app->GPS_Speed;
							}
						}
						else if(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<=2)
						{
							drivespdtmp = (middle_dis-12)/3.6;
							if(middle_dis<=updett+11)
								drivespdtmp=app->GPS_Speed;
							else
							{
								if(drivespdtmp<=app->GPS_Speed)
									drivespdtmp=app->GPS_Speed;
							}
						}
						if(bfasterobacv&&ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=0)
						{
							if(drivespdtmp<fasterobspdini-2/3.6)
								drivespdtmp=fasterobspdini-2/3.6;
						}
						if(drivespdtmp<=10/3.6)
							drivespdtmp=10/3.6;
						if(drivespdtmp>road_speed)
							drivespdtmp=road_speed;
						if(app->drive_obstacle>drivespdtmp)
							app->drive_obstacle=drivespdtmp;
					}

					drivespdtmp=road_speed;

					if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b)
					{
						middle_dis = abs(412-(y+m))*0.2;
						if(middle_dis<updett)
						{
							drivespdtmp = (middle_dis-8)/3.6;
							if(drivespdtmp<0)
								drivespdtmp=0;
							else if(drivespdtmp>road_speed)
								drivespdtmp=road_speed;
							if(middle_dis_front>6&&drivespdtmp<5/3.6)
								drivespdtmp=5/3.6;

							if(drivespdtmp>app->GPS_Speed)
							{
								if(app->GPS_Speed>10/3.6)
									drivespdtmp = app->GPS_Speed;
								else if(middle_dis>18)
									drivespdtmp = 10/3.6;
							}
							/*
							if(middle_dis>8 && bobfisrstapper)
							{
								if(drivespdtmp<app->GPS_Speed-1/3.6)
									drivespdtmp=app->GPS_Speed-1/3.6;
								if(drivespdtmp<0)
									drivespdtmp=0;
							}
							*/
							if(bfasterobacv&&middle_dis>8)
							{
								if(drivespdtmp<fasterobspdini-2/3.6)
									drivespdtmp=fasterobspdini-2/3.6;
								if(drivespdtmp<0)
									drivespdtmp=0;
							}
							if(drivespdtmp<5/3.6)
								drivespdtmp=5/3.6;
							if(app->drive_obstacle>drivespdtmp)
								app->drive_obstacle=drivespdtmp;
						}
						if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
						{
							if(!(bfasterobacv&&middle_dis>8))
							{
								if(((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(middle_dis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && middle_dis<updett))
								{
									app->continue_spdredtmp = true;
									spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-8));
									if((ob_dis_prepre<1000)&&(middle_dis>=9))
									{
										double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
										if(spdaccdist>-2)
											spdaccdist=-2;
										if(spdacctemp<spdaccdist)
											spdacctemp=spdaccdist;
									}
									if(middle_dis>8 && bobfisrstapper)
									{
										if(spdacctemp<-3)
											spdacctemp=-3;
									}
									else
										app->continue_braketmp = true;
									if(bholdbrake)
									{
										if(spdacctemp<app->spdredacc)
										{
											if(app->spdredacc<-3)
												spdacctemp=app->spdredacc;
											else if(spdacctemp<-3)
												spdacctemp=-3;
										}
									}
									if(spdacctemp<app->spdredacctmp)
										app->spdredacctmp = spdacctemp;
								}
								else if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&((app->GPS_Speed>10/3.6 + 2.5/3.6)||(middle_dis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis_front<6)))))
								{
									app->continue_spdredtmp = true;
									spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
									if((ob_dis_prepre<1000)&&(middle_dis>=9))
									{
										double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
										if(spdaccdist>-2)
											spdaccdist=-2;
										if(spdacctemp<spdaccdist)
											spdacctemp=spdaccdist;
									}
									if(middle_dis>8 && bobfisrstapper)
									{
										if(spdacctemp<-3)
											spdacctemp=-3;
									}
									else
										app->continue_braketmp = true;
									if(bholdbrake)
									{
										if(spdacctemp<app->spdredacc)
										{
											if(app->spdredacc<-3)
												spdacctemp=app->spdredacc;
											else if(spdacctemp<-3)
												spdacctemp=-3;
										}
									}
									if(bfasterobacv||bobfisrstapper)
									{
										if(spdacctemp<-3)
											spdacctemp=-3;
									}

									if(spdacctemp<app->spdredacc && spdacctemp<-2)
									{
										if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
										{
											if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
												spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
											if(spdacctemp>app->spdredacc)
												spdacctemp=app->spdredacc;
											if(spdacctemp>-2)
												spdacctemp=-2;
										}
										else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
										{
											if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
												spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
											if(spdacctemp>app->spdredacc)
												spdacctemp=app->spdredacc;
											if(spdacctemp>-2)
												spdacctemp=-2;
										}
									}

									if(spdacctemp<app->spdredacctmp)
										app->spdredacctmp = spdacctemp;
								}
							}
							else if(bfasterobacv&&middle_dis<15)
							{
								spdacctemp=0;
								if(app->GPS_Speed>10/3.6 + 2.5/3.6)
								{
									app->continue_spdredtmp = true;
									if(spdacctemp<app->spdredacctmp)
										app->spdredacctmp = spdacctemp;
								}
							}
						}
					}
					if(vel_Map->MapPoint[y+m][x+n] == c)
					{
						if(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>237)
						{
							middle_dis = abs(412-(y+m))*0.2;
							if(middle_dis>40)
							{
								if(dynamicmap_v_x->MapPoint[y+m][x+n]>0)
								{
									if(middle_dis<updett)
									{
										drivespdtmp = app->GPS_Speed;
										if(drivespdtmp<10/3.6)
											drivespdtmp=10/3.6;
										if(bfasterobacv&&middle_dis>8)
										{
											if(drivespdtmp<fasterobspdini-2/3.6)
												drivespdtmp=fasterobspdini-2/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										if(drivespdtmp<5/3.6)
											drivespdtmp=5/3.6;
										if(app->drive_obstacle>drivespdtmp)
											app->drive_obstacle=drivespdtmp;
									}
									spdacctemp=0;
									if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
									{
										if((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(!(bfasterobacv&&middle_dis>15)))
										{
											app->continue_spdredtmp = true;
											if(spdacctemp<app->spdredacctmp)
												app->spdredacctmp = spdacctemp;
										}
									}
								}
								else
								{
									if(middle_dis<updett)
									{
										drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n];
										if(drivespdtmp>app->GPS_Speed)
											drivespdtmp = app->GPS_Speed;
										if(drivespdtmp<10/3.6)
											drivespdtmp=10/3.6;
										/*
										if(middle_dis>8 && bobfisrstapper)
										{
											if(drivespdtmp<app->GPS_Speed-1/3.6)
												drivespdtmp=app->GPS_Speed-1/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										*/
										if(bfasterobacv&&middle_dis>8)
										{
											if(drivespdtmp<fasterobspdini-2/3.6)
												drivespdtmp=fasterobspdini-2/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										if(drivespdtmp<5/3.6)
											drivespdtmp=5/3.6;
										if(app->drive_obstacle>drivespdtmp)
											app->drive_obstacle=drivespdtmp;
									}
									spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n])*(dynamicmap_v_x->MapPoint[y+m][x+n])/2/(middle_dis-36);
									if(spdacctemp<-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-25))
										spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-25);
									if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12)))
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12));
									if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
									{
										if(!(bfasterobacv&&middle_dis>8))
										{
											if(app->GPS_Speed>10/3.6 + 2.5/3.6)
											{
												app->continue_spdredtmp = true;
												if((ob_dis_prepre<1000)&&(middle_dis>=9))
												{
													double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
													if(spdaccdist>-2)
														spdaccdist=-2;
													if(spdacctemp<spdaccdist)
														spdacctemp=spdaccdist;
												}
												if(middle_dis>8 && bobfisrstapper)
												{
													if(spdacctemp<-3)
														spdacctemp=-3;
												}
												else if(dynamicmap_v_x->MapPoint[y+m][x+n]<-3/3.6)
													app->continue_braketmp = true;
												if(bholdbrake)
												{
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}
												}
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
										else if(bfasterobacv&&middle_dis<15)
										{
											spdacctemp=0;
											if(app->GPS_Speed>10/3.6 + 2.5/3.6)
											{
												app->continue_spdredtmp = true;
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
									}
								}
							}
							else if(middle_dis>15)
							{
								if(dynamicmap_v_x->MapPoint[y+m][x+n]>10/3.6)
								{
									if(middle_dis<updett)
									{
										drivespdtmp = app->GPS_Speed;
										if(drivespdtmp<10/3.6)
											drivespdtmp=10/3.6;
										if(bfasterobacv&&middle_dis>8)
										{
											if(drivespdtmp<fasterobspdini-2/3.6)
												drivespdtmp=fasterobspdini-2/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										if(drivespdtmp<5/3.6)
											drivespdtmp=5/3.6;
										if(app->drive_obstacle>drivespdtmp)
											app->drive_obstacle=drivespdtmp;
									}
									spdacctemp=0;
									if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
									{
										if((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(!(bfasterobacv&&middle_dis>15)))
										{
											app->continue_spdredtmp = true;
											if(spdacctemp<app->spdredacctmp)
												app->spdredacctmp = spdacctemp;
										}
									}
								}
								else
								{
									if(middle_dis<updett)
									{
										drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6;
										if(drivespdtmp>app->GPS_Speed)
											drivespdtmp = app->GPS_Speed;
										if(drivespdtmp<10/3.6)
											drivespdtmp=10/3.6;
										/*
										if(middle_dis>8 && bobfisrstapper)
										{
											if(drivespdtmp<app->GPS_Speed-1/3.6)
												drivespdtmp=app->GPS_Speed-1/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										*/
										if(bfasterobacv&&middle_dis>8)
										{
											if(drivespdtmp<fasterobspdini-2/3.6)
												drivespdtmp=fasterobspdini-2/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										if(drivespdtmp<5/3.6)
											drivespdtmp=5/3.6;
										if(app->drive_obstacle>drivespdtmp)
											app->drive_obstacle=drivespdtmp;
									}
									if(middle_dis>25)
										spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-20);
									else if(middle_dis>17)
										spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/((middle_dis-15)/2);
									else
										spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2;
									if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12)))
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12));
									if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
									{
										if(!(bfasterobacv&&middle_dis>8))
										{
											if(app->GPS_Speed>10/3.6 + 2.5/3.6)
											{
												app->continue_spdredtmp = true;
												if((ob_dis_prepre<1000)&&(middle_dis>=9))
												{
													double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
													if(spdaccdist>-2)
														spdaccdist=-2;
													if(spdacctemp<spdaccdist)
														spdacctemp=spdaccdist;
												}
												if(middle_dis>8 && bobfisrstapper)
												{
													if(spdacctemp<-3)
														spdacctemp=-3;
												}
												else if(dynamicmap_v_x->MapPoint[y+m][x+n]<7/3.6)
													app->continue_braketmp = true;
												if(bholdbrake)
												{
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}
												}
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
										else if(bfasterobacv&&middle_dis<15)
										{
											spdacctemp=0;
											if(app->GPS_Speed>10/3.6 + 2.5/3.6)
											{
												app->continue_spdredtmp = true;
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
									}
								}
							}
							else
								{
									if(middle_dis<updett)
									{
										drivespdtmp = (middle_dis-8)/3.6;
										if(drivespdtmp<0)
											drivespdtmp=0;
										else if(drivespdtmp>road_speed)
											drivespdtmp=road_speed;
										if(middle_dis_front>6&&drivespdtmp<5/3.6)
											drivespdtmp=5/3.6;

										if(drivespdtmp>app->GPS_Speed)
										{
											if(app->GPS_Speed>10/3.6)
												drivespdtmp = app->GPS_Speed;
											else if(middle_dis>18)
												drivespdtmp = 10/3.6;
										}
										/*
										if(middle_dis>8 && bobfisrstapper)
										{
											if(drivespdtmp<app->GPS_Speed-1/3.6)
												drivespdtmp=app->GPS_Speed-1/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										*/
										if(bfasterobacv&&middle_dis>8)
										{
											if(drivespdtmp<fasterobspdini-2/3.6)
												drivespdtmp=fasterobspdini-2/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
										}
										if(drivespdtmp<5/3.6)
											drivespdtmp=5/3.6;
										if(app->drive_obstacle>drivespdtmp)
											app->drive_obstacle=drivespdtmp;
									}
									if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
									{
										if(!(bfasterobacv&&middle_dis>8))
										{
											if(((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(middle_dis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && middle_dis<updett))
											{
												app->continue_spdredtmp = true;
												spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-8));
												if((ob_dis_prepre<1000)&&(middle_dis>=9))
												{
													double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
													if(spdaccdist>-2)
														spdaccdist=-2;
													if(spdacctemp<spdaccdist)
														spdacctemp=spdaccdist;
												}
												if(middle_dis>8 && bobfisrstapper)
												{
													if(spdacctemp<-3)
														spdacctemp=-3;
												}
												else
													app->continue_braketmp = true;
												if(bholdbrake)
												{
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}
												}
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
											else if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&((app->GPS_Speed>10/3.6 + 2.5/3.6)||(middle_dis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis_front<6)))))
											{
												app->continue_spdredtmp = true;
												spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
												if((ob_dis_prepre<1000)&&(middle_dis>=9))
												{
													double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
													if(spdaccdist>-2)
														spdaccdist=-2;
													if(spdacctemp<spdaccdist)
														spdacctemp=spdaccdist;
												}
												if(middle_dis>8 && bobfisrstapper)
												{
													if(spdacctemp<-3)
														spdacctemp=-3;
												}
												else
													app->continue_braketmp = true;
												if(bholdbrake)
												{
													if(spdacctemp<app->spdredacc)
													{
														if(app->spdredacc<-3)
															spdacctemp=app->spdredacc;
														else if(spdacctemp<-3)
															spdacctemp=-3;
													}
												}
												if(bfasterobacv||bobfisrstapper)
												{
													if(spdacctemp<-3)
														spdacctemp=-3;
												}

												if(spdacctemp<app->spdredacc && spdacctemp<-2)
												{
													if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
													{
														if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
															spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
														if(spdacctemp>app->spdredacc)
															spdacctemp=app->spdredacc;
														if(spdacctemp>-2)
															spdacctemp=-2;
													}
													else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
													{
														if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
															spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
														if(spdacctemp>app->spdredacc)
															spdacctemp=app->spdredacc;
														if(spdacctemp>-2)
															spdacctemp=-2;
													}
												}

												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
										else if(bfasterobacv&&middle_dis<15)
										{
											spdacctemp=0;
											if(app->GPS_Speed>10/3.6 + 2.5/3.6)
											{
												app->continue_spdredtmp = true;
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
									}
								}
						}
					}
					break;
				}
			}
			if(obb_break_flag)
				break;
		}
		if(obb_break_flag)
			break;
	}
	if(obb_break_flag==0)
	{
		ob_loss_count++;
		if(ob_loss_count>=5||ob_dis_now==1000)
		{
			ob_dis_now=1000;
			ob_dis_pre=1000;
			ob_dis_prepre=1000;
			//ob_dis_now_timer=0;
			//ob_dis_pre_timer=0;
			//ob_dis_prepre_timer=0;

			SYSTEMTIME tt;
			GetLocalTime(&tt);
			ob_dis_now_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

			ob_dis_pre_timer=ob_dis_now_timer;
			ob_dis_prepre_timer=ob_dis_now_timer;

			bfasterobacv=0;
			ob_loss_count=0;
		}
	}
	else
	{
		ob_loss_count=0;
		ob_dis_prepre=ob_dis_pre;
		ob_dis_prepre_timer=ob_dis_pre_timer;
		ob_dis_pre=ob_dis_now;
		ob_dis_pre_timer=ob_dis_now_timer;
	}

	if(app->GPS_Speed*3.6<10)
		lanenochangedist_=10;
	else if(app->GPS_Speed*3.6>60)
		lanenochangedist_=15;
	else
		lanenochangedist_=9+(app->GPS_Speed*3.6)/10;
	if(lanedriverstate.SearchObstacle1_frontnear(CrossPath,vel_Map,8,18,28,lanenochangedist_,0))
		lanenochangedist=8;
	else
		lanenochangedist=6;

	//middle_dis=lanedriverstate.SearchFrontOb(vel_Map,lanenochangedist);
	middle_dis=pathplan.SearchFrontOb(vel_Map,lanenochangedist);
	if(middle_dis>0)
	{
		double stopdiat=6;
		if(app->bnearstop)
			stopdiat=5;
		if(middle_dis>stopdiat)
			drivespdtmp=5/3.6;
		else
			drivespdtmp=0;

		if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(middle_dis>=6))
		{
			app->continue_spdredtmp = true;
			app->continue_braketmp = true;
			spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-5));
		}
		else if((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis<stopdiat))
		{
			app->continue_spdredtmp = true;
			app->continue_braketmp = true;
			spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
		}

		if(spdacctemp<app->spdredacc)
		{
			if(app->spdredacc<-3)
				spdacctemp=app->spdredacc;
			else if(spdacctemp<-3)
				spdacctemp=-3;
		}
		if(spdacctemp<app->spdredacctmp)
			app->spdredacctmp = spdacctemp;

		if(app->drive_obstacle>drivespdtmp)
			app->drive_obstacle=drivespdtmp;
	}

	double left_dis = 10000;
	double right_dis = 10000;
	int up=4;
	//MoveLeft(CrossPath,exe_leftpoint,3.7);
	for(int i=0;i<200;i++)
	{
		exe_leftpoint[i].x=app->rndfbezier_left[i].x;
		exe_leftpoint[i].y=app->rndfbezier_left[i].y;
	}
	ob_left = lanedriverstate.SearchObstacle111(exe_leftpoint,vel_Map,8,18,28,up,-12,left_dis);
	//MoveLeft(CrossPath,exe_rightpoint,-3.7);
	for(int i=0;i<200;i++)
	{
		exe_rightpoint[i].x=app->rndfbezier_right[i].x;
		exe_rightpoint[i].y=app->rndfbezier_right[i].y;
	}
	ob_right = lanedriverstate.SearchObstacle111(exe_rightpoint,vel_Map,8,18,28,up,-12,right_dis);
	CvPoint2D64f rndfbeziertemp[200];
	for(int i=0;i<200;i++)
		rndfbeziertemp[i]=CrossPath[i];
	bool bobserachflag = 1;
	if(!ob_left)
	{
		for(int i=7;i>=5;i--)
		{
			MoveLeft(CrossPath,exe_leftpoint,0.53*i);
			if(!lanedriverstate.SearchObstacle222(exe_leftpoint,vel_Map,8,18,28,up,-12,left_dis))
			{
				bobserachflag = 0;
				break;
			}
		}
		ob_left = bobserachflag;
	}
	bobserachflag = 1;
	if(!ob_right)
	{
		for(int i=7;i>=5;i--)
		{
			MoveLeft(CrossPath,exe_rightpoint,-0.53*i);
			if(!lanedriverstate.SearchObstacle222(exe_rightpoint,vel_Map,8,18,28,up,-12,right_dis))
			{
				bobserachflag = 0;
				break;
			}
		}
		ob_right = bobserachflag;
	}

	ob_left=false;
	ob_right=false;

	if(app->bGpsGanrao)
	{
		app->obb_left = false;
		app->obb_right = false;
	}
	else if(!((app->bbanmaxiannoraozhang)&&(app->bGpsGanrao==0)))
	{
		app->obb_left = ob_left;
		app->obb_right = ob_right;
	}
	else
	{
		app->obb_left=true;
		app->obb_right=true;
	}
}

void CVehicleExe::yulukoujiansu()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();
	CvPoint2D64f m_Gps_ = app->GPS_Point;
	double m_Dir_ = app->GPS_Direction;
	app->critical_section.Unlock();

	double vertdistlukou=VertDist(app->GPS_Point,app->GPS_Direction,yulukoupoint);

	if(vertdistlukou<20)//30
		yulukounolaneassist=1;
	else
		yulukounolaneassist=0;

	if(vertdistlukou<40)//60
		yulukounolanechange=true;

	double lukoudstspd=cross_speed1;
	if(lukou_fx==1)
	{
		lukoudstspd=cross_speed1;
	}
	else if(vertdistlukou>20)
	{
		if(cross_speed2>15/3.6)
			lukoudstspd=cross_speed2;
		else
			lukoudstspd=15/3.6;
	}
	else if(lukou_fx==2||lukou_fx==3||lukou_fx==4||lukou_fx==8||lukou_fx==7||lukou_fx==6)
		lukoudstspd=cross_speed2;
	if(((lukou_fx == 1 && bZturnLightChk) || ((lukou_fx == 3 || lukou_fx == 4) && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) && (app->light_res == 2))
	{
		lukoudstspd=cross_speed2;
		if(lukoudstspd>10/3.6)
			lukoudstspd=10/3.6;
	}

	double drivespdtemp = road_speed;

	double verdistredcstrt=(road_speed*road_speed-lukoudstspd*lukoudstspd)/2+20;
	if(verdistredcstrt<30)
		verdistredcstrt=30;

	if(vertdistlukou<verdistredcstrt)
	{
		if(drivespdtemp>lukoudstspd)
		{
			if(vertdistlukou<20)
				drivespdtemp = lukoudstspd;
			else if(vertdistlukou<verdistredcstrt)
				drivespdtemp = sqrt(lukoudstspd*lukoudstspd+(road_speed*road_speed-lukoudstspd*lukoudstspd)*(vertdistlukou-20)/(verdistredcstrt-20));
		}
		if(app->GPS_Speed>lukoudstspd)
		{
			if(drivespdtemp>app->GPS_Speed)
				drivespdtemp = app->GPS_Speed;
		}
		else
			drivespdtemp = lukoudstspd;
		if(app->GPS_Speed>lukoudstspd + 2.5/3.6)
			app->continue_braketmp = true;
		spdacctemp=0;
		if((app->GPS_Speed>lukoudstspd + 2.5/3.6)&&(vertdistlukou>=21))
		{
			app->continue_spdredtmp = true;
			spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed)-lukoudstspd*lukoudstspd)/(2*(vertdistlukou-20));
			if(spdacctemp<-1.5)
				spdacctemp=-1.5;
		}
		else if(app->GPS_Speed>lukoudstspd + 2.5/3.6)
		{
			app->continue_spdredtmp = true;
			spdacctemp = app->spdredacc;
		}
		if(spdacctemp<app->spdredacctmp)
			app->spdredacctmp = spdacctemp;
		if(app->brake_flag == true || brakeflagchange == false)
		{
			app->brake_flag = true;
			brakeflagchange = true;
		}
		if(app->drive_state>drivespdtemp)
			app->drive_state=drivespdtemp;
	}

	if(((((lukou_fx == 1 && bZturnLightChk) || ((lukou_fx == 3) && bLturnLightChk) || (lukou_fx == 2 && bRturnLightChk)) && app->light_res == 2)) && vertdistlukou<20)
	{
		drivespdtemp = lukoudstspd;
		if(vertdistlukou<4)
		{
			drivespdtemp=0;
		}
		else if(vertdistlukou>5)
			drivespdtemp = sqrt(5/3.6*5/3.6+(lukoudstspd*lukoudstspd-5/3.6*5/3.6)*(vertdistlukou-5)/15);
		else
			drivespdtemp = 5/3.6;
		if(app->GPS_Speed>5/3.6)
		{
			if(drivespdtemp>app->GPS_Speed)
				drivespdtemp = app->GPS_Speed;
		}
		else if(drivespdtemp>5/3.6)
			drivespdtemp = 5/3.6;
		if(app->GPS_Speed>5/3.6 + 2.5/3.6)
			app->continue_braketmp = true;
		spdacctemp=0;
		if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(vertdistlukou>=5))
		{
			app->continue_braketmp = true;
			app->continue_spdredtmp = true;
			spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed)-5/3.6*5/3.6)/(2*(vertdistlukou-4));
		}
		else if(app->GPS_Speed>5/3.6 + 2.5/3.6)
		{
			app->continue_braketmp = true;
			app->continue_spdredtmp = true;
			spdacctemp = app->spdredacc;
		}
		if(spdacctemp<app->spdredacctmp)
			app->spdredacctmp = spdacctemp;

		if(app->drive_state>drivespdtemp)
			app->drive_state=drivespdtemp;
	}
}

int CVehicleExe::OnRoadDrive(int &ob_num,int &count_move,bool is_avoidobstacle)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->bleftturnban=0;
	int midd_num1=0;
	CvPoint2D64f MidPoint[512];

	bool Lane_able =false;
	bcurveactive=0;

	int n = 0;
	double len = 0;
	double min_len = 99999;

	for(int i=0;i<200;i++)
	{
		len = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,rndfbezier[i].x,rndfbezier[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}

	bool rndf_vangle1 = false;
	double dist_dir_next = 0;
	int n_dir_next = n+1;
	if(n_dir_next>199)
		n_dir_next=199;

	for(int j=n+2;j<200;j++)
	{
		dist_dir_next=dist_dir_next+m_GpsData.GetDistance(rndfbezier[j-1].x,rndfbezier[j-1].y,rndfbezier[j].x,rndfbezier[j].y);
		if(dist_dir_next>5)
		{
			n_dir_next=j;
			break;
		}
	}
	if(dist_dir_next>5)
	{
		double anglerndf1 = m_GpsData.GetAngle(rndfbezier[n_dir_next].x,rndfbezier[n_dir_next].y,rndfbezier[n].x,rndfbezier[n].y);
		if(cos(PI*(anglerndf1 - app->GPS_Direction)/180)>cos(25*PI/180))
			rndf_vangle1 = true;
	}

	lanechang_dis = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,ob_tempoint.x,ob_tempoint.y);

	double lanechangeoverdist=app->GPS_Speed*3.6*0.9;
	if(lanechangeoverdist<18)
		lanechangeoverdist=18;

	CTime t1 = CTime::GetCurrentTime();
	CString strTime1 = t1.Format("%Y-%m-%d %H:%M:%S");
	if((abs(gps_aimdir - realtime_Dir) < 7 || abs(360 - abs(gps_aimdir - realtime_Dir)) < 7) && abs(lanechang_dis )>lanechangeoverdist &&!able_ChangeLane)
	{	
		tongji++;
		if (tongji > 10)
		{
			app->left_right = 0;
			able_ChangeLane = true;
			blanesync = true;
			//app->drive_obstacle = max_speed;
			control.setLight(LAMP_OFF);
			outleftchange<<"换道结束"<<strTime1<<endl;
			tongji = 0;
			if(app->braohui)
				raohuistart=cvPoint2D64f(realtime_Gps.x,realtime_Gps.y);
		}
	}

	//double lanemovedist;
	//int lanechangenumtmp;
	if(app->GPS_Speed*3.6<=25)
	{
		lanemovedist=0.5;
		lanechangenumtmp=6;
	}
	else if(app->GPS_Speed*3.6<=45)
	{
		lanemovedist=1;
		//lanechangenumtmp=7;
		lanechangenumtmp=6;
	}
	else if(app->GPS_Speed*3.6<=62)
	{
		lanemovedist=1+(app->GPS_Speed*3.6-40)/10;
		lanechangenumtmp=6;
	}
	else
	{
		lanemovedist=3.2;
		lanechangenumtmp=6;
	}

	if(able_ChangeLane&&(app->braohui))
	{
		if(m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,raohuistart.x,raohuistart.y)>80)
			app->braohui=0;
	}


	if(bhardraozhang)
		blanesync = true;

	if(able_ChangeLane)
	{
		for(int i=0;i<200;i++)
		{
			rndfbezier_lchgdst[i].x=0;
			rndfbezier_lchgdst[i].y=0;
		}
		latpianyi_lanechgstrt=0;
		lonpianyi_lanechgstrt=0;
		app->left_right = 0;
	}

	CvPoint2D64f pianyitmp;
	pianyitmp.x=0;
	pianyitmp.y=0;
	CvPoint2D64f rndfbezierlanechangeinitial[200];

	app->drive_obstacle = road_speed;

	//int lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);
	int lane_result;
	lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);

	int laner_sultold=lane_result;

	bool bnormallane=false;

	if(!(approachgoal_flag||approachyulukou>0))
	{
		if(lane_result)
		{
			if(midd_num1>=25)
			{
				double x_boottom= MidPoint[0].x;
				for(int i=1;i<25;i++)
				{
					if(abs(MidPoint[i].x-x_boottom)>2)
					{
						lane_result=0;
						break;
					}
				}
			}
			else
				lane_result=0;
		}
	}

	if(blaneusedisable)
		lane_result=0;

	if(!able_ChangeLane)
	{
		if(lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)
		{
			if(ob_res == -1)
				app->leftturnreq_proc=false;
			if(ob_res == 1)
				app->rightturnreq_proc=false;
		}
		if(lanechangespd<12/3.6)
		{
			lanechangecount=lanechangenumtmp;
			if(lanechangecount>app->lanechangenum)
				lanechangecount=app->lanechangenum;
		}
		//else if((lanechang_dis-lanechangedistmp>lanemovedist || lanechangedistmp==0) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)
		else if((lanechang_dis-lanechangedistmp>lanemovedist) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)
		{
			//MoveLeft(rndfbezier,rndfbezier,-0.53*ob_res,pianyi);
			lanechangecount++;
			lanechangedistmp=lanechang_dis;
		}
		ReturnPartPoints_lanechg(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
	}
	else
	{
		ReturnPartPoints(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezierlanechangeinitial,pianyitmp);

		for (int j = 0; j<200;j++)
		{
			rndfbezierlanechangeinitialdisp[j].x = rndfbezierlanechangeinitial[j].x;
			rndfbezierlanechangeinitialdisp[j].y = rndfbezierlanechangeinitial[j].y;
		}

		double lengthdiserr=0;

		if(pianyi.x==0 && pianyi.y==0)
			blanesync=true;

		if ((able_ChangeLane&&(yulukounolaneassist==0))||approachgoal_flag||approachyulukou>0||blanesync)
		{
			able_ChangeLane=true;

			int laneob_result = SearchObstacleMap(MidPoint,midd_num1,vel_Map,30,0);

			if(laneob_result==2)
			{
				if(bnormallane)
					laneob_result=false;
				else
					laneob_result=true;
			}

			int numtemp = int(midd_num1/2);
			//int numtemp = 0;
			bool bmidlanewrong=0;

			if (abs(MidPoint[numtemp].x - 256)>11)
			{
				bmidlanewrong = 1;
			}
			if(approachgoal_flag||approachyulukou>0)
			{
				bmidlanewrong=0;

				CvPoint2D64f lanepgps = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,MidPoint[numtemp]);
				CvPoint2D64f rndfbezierlanechangetest[200];
				for (int j = 0; j<200;j++)
				{
					rndfbezierlanechangetest[j].x = rndfbezierlanechangeinitial[j].x;
					rndfbezierlanechangetest[j].y = rndfbezierlanechangeinitial[j].y;
				}
				MoveWay(lanepgps,realtime_Dir,rndfbezierlanechangetest);
				if(lane_result&&!laneob_result)
				{
					double ob_dis = 0;
					int approach_dir=0;
					if(approachgoal_flag||approachyulukou==2)
					{
						approach_dir=2;
					}
					else if(approachyulukou==1)
					{
						approach_dir=1;
					}
					int ob_flag = lanedriverstate.SearchObstacle2(rndfbezierlanechangetest,vel_Map,8,18,28,40,-25,ob_dis,approach_dir);
					if(ob_flag==2)
					{
						app->drive_obstacle=0;
						app->continue_spdredtmp = true;
						app->continue_braketmp = true;
						if(-3<app->spdredacctmp)
							app->spdredacctmp = -3;
						bmidlanewrong = 1;
					}
				}
			}
			if((lane_result&&!laneob_result && !bmidlanewrong && rndf_vangle1)||blanesync)
			{
				double lengtherrmid=0;
				double lengtherrend=0;
				CvPoint2D64f lanepgps = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,MidPoint[numtemp]);
				pianyi = cvPoint2D64f(0,0);
				//ReturnPartPoints(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
				CvPoint2D64f rndfbezierlanechange[200];
				CvPoint2D64f rndfbezierlanechangetmp[200];
				bool blaneok=lane_result&&!laneob_result && !bmidlanewrong && rndf_vangle1;
				//ReturnPartPoints(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezierlanechange,pianyi);
				for (int j = 0; j<200;j++)
				{
					rndfbezierlanechange[j].x = rndfbezierlanechangeinitial[j].x;
					rndfbezierlanechange[j].y = rndfbezierlanechangeinitial[j].y;
				}
				for (int j = 0; j<200;j++)
				{
					rndfbezierlanechangetmp[j].x = rndfbezierlanechange[j].x;
					rndfbezierlanechangetmp[j].y = rndfbezierlanechange[j].y;
				}
				if(blaneok && (!(m_task==IVTASK_APPROINTERSECTION && VertDist(app->GPS_Point,app->GPS_Direction,yulukoupoint)<50)))
				{
					CvPoint2D64f rndfbezierlanechangetest[200];
					for (int j = 0; j<200;j++)
					{
						rndfbezierlanechangetest[j].x = rndfbezierlanechangeinitial[j].x;
						rndfbezierlanechangetest[j].y = rndfbezierlanechangeinitial[j].y;
					}
					MoveWay(lanepgps,realtime_Dir,rndfbezierlanechangetest);
					if(lanedriverstate.SearchObstacle_laneyichang(rndfbezierlanechangetest,vel_Map) && (!bnormallane))
						blaneok=0;
				}

				
				for (int j = 0; j<200;j++)
				{
					rndfbezier[j].x = rndfbezierlanechangetmp[j].x;
					rndfbezier[j].y = rndfbezierlanechangetmp[j].y;
				}

				{
					if(blaneok &&(yulukounolaneassist==0))
					{
						CvPoint2D64f pianyiini=pianyi;

						MoveWay(lanepgps,realtime_Dir,rndfbezier,pianyi);
						lanelostcnt=0;
						Lane_able = true;

						double dx = pianyi.x - pianyiini.x;
						double dy = pianyi.y - pianyiini.y;
						for(int i=0;i<200;i++)
						{
							app->rndfbezier_left[i].x -= dx;
							app->rndfbezier_left[i].y -= dy;

							app->rndfbezier_right[i].x -= dx;
							app->rndfbezier_right[i].y -= dy;
						}
					}
					else
					{
						CvPoint2D64f pianyiini=pianyi;

						lanelostcnt=10;
						MoveWay(realtime_Gps,realtime_Dir,rndfbezier,pianyi);

						double dx = pianyi.x - pianyiini.x;
						double dy = pianyi.y - pianyiini.y;
						for(int i=0;i<200;i++)
						{
							app->rndfbezier_left[i].x -= dx;
							app->rndfbezier_left[i].y -= dy;

							app->rndfbezier_right[i].x -= dx;
							app->rndfbezier_right[i].y -= dy;
						}
					}
				}
			}	
			else
			{
				{
					lanelostcnt++;
					if(lanelostcnt>=10)
					{
						lanelostcnt=10;
						pianyi = cvPoint2D64f(0,0);

						CvPoint2D64f pianyiini=pianyi;

						for (int j = 0; j<200;j++)
						{
							rndfbezier[j].x = rndfbezierlanechangeinitial[j].x;
							rndfbezier[j].y = rndfbezierlanechangeinitial[j].y;
						}
						//ReturnPartPoints(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
						MoveWay(realtime_Gps,realtime_Dir,rndfbezier,pianyi);

						double dx = pianyi.x - pianyiini.x;
						double dy = pianyi.y - pianyiini.y;
						for(int i=0;i<200;i++)
						{
							app->rndfbezier_left[i].x -= dx;
							app->rndfbezier_left[i].y -= dy;

							app->rndfbezier_right[i].x -= dx;
							app->rndfbezier_right[i].y -= dy;
						}
					}
					else
					{
						for (int i = 0;i<200;i++)
						{
							rndfbezier[i].x =  rndfbezierlanechangeinitial[i].x-pianyi.x;
							rndfbezier[i].y =  rndfbezierlanechangeinitial[i].y-pianyi.y;

							app->rndfbezier_left[i].x =  app->rndfbezier_left[i].x-pianyi.x;
							app->rndfbezier_left[i].y =  app->rndfbezier_left[i].y-pianyi.y;

							app->rndfbezier_right[i].x =  app->rndfbezier_right[i].x-pianyi.x;
							app->rndfbezier_right[i].y =  app->rndfbezier_right[i].y-pianyi.y;
						}
						//ReturnPartPoints(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
					}
				}
			}
		}
		else
		{
			for (int i = 0;i<200;i++)
			{
				rndfbezier[i].x =  rndfbezierlanechangeinitial[i].x-pianyi.x;
				rndfbezier[i].y =  rndfbezierlanechangeinitial[i].y-pianyi.y;

				app->rndfbezier_left[i].x =  app->rndfbezier_left[i].x-pianyi.x;
				app->rndfbezier_left[i].y =  app->rndfbezier_left[i].y-pianyi.y;

				app->rndfbezier_right[i].x =  app->rndfbezier_right[i].x-pianyi.x;
				app->rndfbezier_right[i].y =  app->rndfbezier_right[i].y-pianyi.y;
			}
			//ReturnPartPoints(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
			lanelostcnt=0;
		}
	}
	blanesync=false;

	if((!able_ChangeLane) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)
		app->blanechanging=true;
	else if(!able_ChangeLane)
	{
		if((!(lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum))&&(abs(gps_aimdir - realtime_Dir) < 7 || abs(360 - abs(gps_aimdir - realtime_Dir)) < 7) && abs(lanechang_dis )>lanechangeoverdist)
			app->blanechanging=false;
	}
	else
		app->blanechanging=false;

	CvPoint2D64f Lanepoint = MidPoint[int(midd_num1/2)];
	if(Lane_able)
	{
		if(Lanepoint.x - 256< -10)
			control.setLight(LEFT_LAMP_ON);
		else if(Lanepoint.x -256 >10)
			control.setLight(RIGHT_LAMP_ON);
		else
			control.setLight(LAMP_OFF);
	}
	else if(able_ChangeLane)
		control.setLight(LAMP_OFF);

	double middle_dis=0;
	double drivespdtmp = road_speed;

	double obfrontspd=0;

	int up;
	up=app->GPS_Speed*3.6+23;
	if(up<35)
		up=35;
	else if(up>80)
		up=80;

	bool obb;

	//app->drive_obstacle = road_speed;

	double speedreducedist;
	speedreducedist=up-15;
	if(speedreducedist<20)
		speedreducedist=20;

	if(able_ChangeLane)
		lanechangespd=road_speed;
	else
	{
		if(lanechangespd>app->GPS_Speed)
			lanechangespd=app->GPS_Speed;
		if(lanechangespd<10/3.6)
			lanechangespd=10/3.6;
	}

	double directlinereqlen=up+lanemovedist*7;
	if(directlinereqlen<app->GPS_Speed*3.6*0.9+app->GPS_Speed*3)
		directlinereqlen=app->GPS_Speed*3.6*0.9+app->GPS_Speed*3;

	int n_last=n;
	int n_first=n;
	double ll=0;
	for(n_last=n;n_last<199;n_last++)
	{
		ll=ll+m_GpsData.GetDistance(rndfbezier[n_last].x,rndfbezier[n_last].y,rndfbezier[n_last+1].x,rndfbezier[n_last+1].y);
		if(ll>directlinereqlen)
		{
			n_last++;
			break;
		}
	}
	ll=0;
	for(n_first=n;n_first>0;n_first--)
	{
		ll=ll+m_GpsData.GetDistance(rndfbezier[n_first].x,rndfbezier[n_first].y,rndfbezier[n_first-1].x,rndfbezier[n_first-1].y);
		if(ll>5)
		{
			n_first--;
			break;
		}
	}
	double angle1n = m_GpsData.GetAngle(rndfbezier[n_last].x,rndfbezier[n_last].y,rndfbezier[n_first].x,rndfbezier[n_first].y);
	bool flaganglechack=0;
	for(int iii=n_first;iii+3<=n_last;iii++)
	{
		double angletmp=m_GpsData.GetAngle(rndfbezier[iii+3].x,rndfbezier[iii+3].y,rndfbezier[iii].x,rndfbezier[iii].y);
		if (abs(angle1n-angletmp)>60&&abs(360-abs(angle1n-angletmp)>60))//3//30
		{
			flaganglechack=1;
			break;
		}
	}

	bforcedirectrun=0;

	if(able_ChangeLane||(!((!able_ChangeLane) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)))
	{
		int up_pianyinott=app->GPS_Speed*3.6+40;
		double middledistmp=0;
		int ob__ = lanedriverstate.SearchObstacle1_vehlanechange(rndfbezier,vel_Map,8,18,28,up_pianyinott,0,middledistmp);
		if(ob__==5)
			stopmiddledis=6;
		else
			stopmiddledis=6;
	}
	else
		stopmiddledis=6;

	if(((m_task==IVTASK_LANE)||((m_task==IVTASK_APPROINTERSECTION)&&(!(approachgoal_flag||approachyulukou>0)))) && able_ChangeLane)
		stopmiddledis=8;
		
	stopmiddledis=6;

	//int timernow=app->systemtimer10ms;

	SYSTEMTIME tt;
	GetLocalTime(&tt);
	int timernow=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));


	if(able_ChangeLane||(!((!able_ChangeLane) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)))//if(able_ChangeLane)
	{
		for(int i=3;i>=0;i--)
		{
			lanedriverstate.y_left_front[i+1]=lanedriverstate.y_left_front[i];
			lanedriverstate.y_left_rear[i+1]=lanedriverstate.y_left_rear[i];
			lanedriverstate.y_right_front[i+1]=lanedriverstate.y_right_front[i];
			lanedriverstate.y_right_rear[i+1]=lanedriverstate.y_right_rear[i];
			lanedriverstate.y_middle_front[i+1]=lanedriverstate.y_middle_front[i];
			lanedriverstate.timer_left_front[i+1]=lanedriverstate.timer_left_front[i];
			lanedriverstate.timer_left_rear[i+1]=lanedriverstate.timer_left_rear[i];
			lanedriverstate.timer_right_front[i+1]=lanedriverstate.timer_right_front[i];
			lanedriverstate.timer_right_rear[i+1]=lanedriverstate.timer_right_rear[i];
			lanedriverstate.timer_middle_front[i+1]=lanedriverstate.timer_middle_front[i];
		}

		double ymr=-10000;
		lanedriverstate.y_left_front[0]=-10000;
		lanedriverstate.y_left_rear[0]=-10000;
		lanedriverstate.y_right_front[0]=-10000;
		lanedriverstate.y_right_rear[0]=-10000;
		lanedriverstate.y_middle_front[0]=-10000;
		lanedriverstate.timer_left_front[0]=timernow;
		lanedriverstate.timer_left_rear[0]=timernow;
		lanedriverstate.timer_right_front[0]=timernow;
		lanedriverstate.timer_right_rear[0]=timernow;
		lanedriverstate.timer_middle_front[0]=timernow;

		double spdrelmidfronthis=lanedriverstate.spdrelmidfront;
		double spdrelleftfronthis=lanedriverstate.spdrelleftfront;
		double spdrelleftrearhis=lanedriverstate.spdrelleftrear;
		double spdrelrightfronthis=lanedriverstate.spdrelrightfront;
		double spdrelrightrearhis=lanedriverstate.spdrelrightrear;

		lanedriverstate.spdrelmidfront=-10000;
		lanedriverstate.spdrelleftfront=-10000;
		lanedriverstate.spdrelleftrear=-10000;
		lanedriverstate.spdrelrightfront=-10000;
		lanedriverstate.spdrelrightrear=-10000;

		lanedriverstate.SearchObstacle1_lchgconcheck(rndfbezier,vel_Map,lanedriverstate.y_middle_front[0],ymr);
		lanedriverstate.SearchObstacle1_lchgconcheck(app->rndfbezier_left,vel_Map,lanedriverstate.y_left_front[0],lanedriverstate.y_left_rear[0]);
		lanedriverstate.SearchObstacle1_lchgconcheck(app->rndfbezier_right,vel_Map,lanedriverstate.y_right_front[0],lanedriverstate.y_right_rear[0]);

		if(lanedriverstate.y_middle_front[0]>=0)
		{
			for(int i=3;i<5;i++)
			{
				if(lanedriverstate.y_middle_front[i]>=0)
				{
					double spdtmp;
					if(lanedriverstate.timer_middle_front[0]>lanedriverstate.timer_middle_front[i])
						spdtmp=(lanedriverstate.y_middle_front[0]-lanedriverstate.y_middle_front[i])/((lanedriverstate.timer_middle_front[0]-lanedriverstate.timer_middle_front[i])*0.001);
					else
						spdtmp=(lanedriverstate.y_middle_front[0]-lanedriverstate.y_middle_front[i])/0.001;

					if(spdtmp>33 || spdtmp<-42)
					{
						for(int j=i;j<5;j++)
						{
							lanedriverstate.y_middle_front[j]=-10000;
							lanedriverstate.timer_middle_front[j]=timernow;
						}
						break;
					}
					else
					{
						if(lanedriverstate.spdrelmidfront==-10000)
							lanedriverstate.spdrelmidfront=spdtmp;
						else
						{
							if(spdtmp<lanedriverstate.spdrelmidfront)
								lanedriverstate.spdrelmidfront=spdtmp;
						}
					}
				}
			}
		}
		else if(lanedriverstate.y_middle_front[1]<0 && lanedriverstate.y_middle_front[2]<0)
		{
			for(int i=0;i<5;i++)
			{
				lanedriverstate.y_middle_front[i]=-10000;
				lanedriverstate.timer_middle_front[i]=timernow;
			}
		}

		if(lanedriverstate.spdrelmidfront>-1000 && spdrelmidfronthis>-1000 && lanedriverstate.spdrelmidfront>spdrelmidfronthis)
			lanedriverstate.spdrelmidfront=lanedriverstate.spdrelmidfront*0.5 + spdrelmidfronthis*0.5;

		if(lanedriverstate.y_left_front[0]>=0)
		{
			for(int i=3;i<5;i++)
			{
				if(lanedriverstate.y_left_front[i]>=0)
				{
					double spdtmp;
					if(lanedriverstate.timer_left_front[0]>lanedriverstate.timer_left_front[i])
						spdtmp=(lanedriverstate.y_left_front[0]-lanedriverstate.y_left_front[i])/((lanedriverstate.timer_left_front[0]-lanedriverstate.timer_left_front[i])*0.001);
					else
						spdtmp=(lanedriverstate.y_left_front[0]-lanedriverstate.y_left_front[i])/0.001;
					if(spdtmp>33 || spdtmp<-42)
					{
						for(int j=i;j<5;j++)
						{
							lanedriverstate.y_left_front[j]=-10000;
							lanedriverstate.timer_left_front[j]=timernow;
						}
						break;
					}
					else
					{
						if(lanedriverstate.spdrelleftfront==-10000)
							lanedriverstate.spdrelleftfront=spdtmp;
						else
						{
							if(spdtmp<lanedriverstate.spdrelleftfront)
								lanedriverstate.spdrelleftfront=spdtmp;
						}
					}
				}
			}
		}
		else if(lanedriverstate.y_left_front[1]<0 && lanedriverstate.y_left_front[2]<0)
		{
			for(int i=0;i<5;i++)
			{
				lanedriverstate.y_left_front[i]=-10000;
				lanedriverstate.timer_left_front[i]=timernow;
			}
		}

		if(lanedriverstate.spdrelleftfront>-1000 && spdrelleftfronthis>-1000 && lanedriverstate.spdrelleftfront>spdrelleftfronthis)
			lanedriverstate.spdrelleftfront=lanedriverstate.spdrelleftfront*0.5 + spdrelleftfronthis*0.5;

		if(lanedriverstate.y_left_rear[0]>=0)
		{
			for(int i=3;i<5;i++)
			{
				if(lanedriverstate.y_left_rear[i]>=0)
				{
					double spdtmp;
					if(lanedriverstate.timer_left_rear[0]>lanedriverstate.timer_left_rear[i])
						spdtmp=(lanedriverstate.y_left_rear[0]-lanedriverstate.y_left_rear[i])/((lanedriverstate.timer_left_rear[0]-lanedriverstate.timer_left_rear[i])*0.001);
					else
						spdtmp=(lanedriverstate.y_left_rear[0]-lanedriverstate.y_left_rear[i])/0.001;
					if(spdtmp>42 || spdtmp<-33)
					{
						for(int j=i;j<5;j++)
						{
							lanedriverstate.y_left_rear[j]=-10000;
							lanedriverstate.timer_left_rear[j]=timernow;
						}
						break;
					}
					else
					{
						if(lanedriverstate.spdrelleftrear==-10000)
							lanedriverstate.spdrelleftrear=-spdtmp;
						else
						{
							if(-spdtmp>lanedriverstate.spdrelleftrear)
								lanedriverstate.spdrelleftrear=-spdtmp;
						}
					}
				}
			}
		}
		else if(lanedriverstate.y_left_rear[1]<0 && lanedriverstate.y_left_rear[2]<0)
		{
			for(int i=0;i<5;i++)
			{
				lanedriverstate.y_left_rear[i]=-10000;
				lanedriverstate.timer_left_rear[i]=timernow;
			}
		}

		if(lanedriverstate.spdrelleftrear>-1000 && spdrelleftrearhis>-1000 && lanedriverstate.spdrelleftrear<spdrelleftrearhis)
			lanedriverstate.spdrelleftrear=lanedriverstate.spdrelleftrear*0.5 + spdrelleftrearhis*0.5;

		if(lanedriverstate.y_right_front[0]>=0)
		{
			for(int i=3;i<5;i++)
			{
				if(lanedriverstate.y_right_front[i]>=0)
				{
					double spdtmp;
					if(lanedriverstate.timer_right_front[0]>lanedriverstate.timer_right_front[i])
						spdtmp=(lanedriverstate.y_right_front[0]-lanedriverstate.y_right_front[i])/((lanedriverstate.timer_right_front[0]-lanedriverstate.timer_right_front[i])*0.001);
					else
						spdtmp=(lanedriverstate.y_right_front[0]-lanedriverstate.y_right_front[i])/0.001;
					if(spdtmp>33 || spdtmp<-42)
					{
						for(int j=i;j<5;j++)
						{
							lanedriverstate.y_right_front[j]=-10000;
							lanedriverstate.timer_right_front[j]=timernow;
						}
						break;
					}
					else
					{
						if(lanedriverstate.spdrelrightfront==-10000)
							lanedriverstate.spdrelrightfront=spdtmp;
						else
						{
							if(spdtmp<lanedriverstate.spdrelrightfront)
								lanedriverstate.spdrelrightfront=spdtmp;
						}
					}
				}
			}
		}
		else if(lanedriverstate.y_right_front[1]<0 && lanedriverstate.y_right_front[2]<0)
		{
			for(int i=0;i<5;i++)
			{
				lanedriverstate.y_right_front[i]=-10000;
				lanedriverstate.timer_right_front[i]=timernow;
			}
		}

		if(lanedriverstate.spdrelrightfront>-1000 && spdrelrightfronthis>-1000 && lanedriverstate.spdrelrightfront>spdrelrightfronthis)
			lanedriverstate.spdrelrightfront=lanedriverstate.spdrelrightfront*0.5 + spdrelrightfronthis*0.5;

		if(lanedriverstate.y_right_rear[0]>=0)
		{
			for(int i=3;i<5;i++)
			{
				if(lanedriverstate.y_right_rear[i]>=0)
				{
					double spdtmp;
					if(lanedriverstate.timer_right_rear[0]>lanedriverstate.timer_right_rear[i])
						spdtmp=(lanedriverstate.y_right_rear[0]-lanedriverstate.y_right_rear[i])/((lanedriverstate.timer_right_rear[0]-lanedriverstate.timer_right_rear[i])*0.001);
					else
						spdtmp=(lanedriverstate.y_right_rear[0]-lanedriverstate.y_right_rear[i])/0.001;
					if(spdtmp>42 || spdtmp<-33)
					{
						for(int j=i;j<5;j++)
						{
							lanedriverstate.y_right_rear[j]=-10000;
							lanedriverstate.timer_right_rear[j]=timernow;
						}
						break;
					}
					else
					{
						if(lanedriverstate.spdrelrightrear==-10000)
							lanedriverstate.spdrelrightrear=-spdtmp;
						else
						{
							if(-spdtmp>lanedriverstate.spdrelrightrear)
								lanedriverstate.spdrelrightrear=-spdtmp;
						}
					}
				}
			}
		}
		else if(lanedriverstate.y_right_rear[1]<0 && lanedriverstate.y_right_rear[2]<0)
		{
			for(int i=0;i<5;i++)
			{
				lanedriverstate.y_right_rear[i]=-10000;
				lanedriverstate.timer_right_rear[i]=timernow;
			}
		}

		if(lanedriverstate.spdrelrightrear>-1000 && spdrelrightrearhis>-1000 && lanedriverstate.spdrelrightrear<spdrelrightrearhis)
			lanedriverstate.spdrelrightrear=lanedriverstate.spdrelrightrear*0.5 + spdrelrightrearhis*0.5;

	}
	else
	{
		for(int i=0;i<5;i++)
		{
			lanedriverstate.y_left_front[i]=-10000;
			lanedriverstate.y_left_rear[i]=-10000;
			lanedriverstate.y_right_front[i]=-10000;
			lanedriverstate.y_right_rear[i]=-10000;
			lanedriverstate.y_middle_front[i]=-10000;
			lanedriverstate.timer_left_front[i]=timernow;
			lanedriverstate.timer_left_rear[i]=timernow;
			lanedriverstate.timer_right_front[i]=timernow;
			lanedriverstate.timer_right_rear[i]=timernow;
			lanedriverstate.timer_middle_front[i]=timernow;
		}
		lanedriverstate.spdrelmidfront=-10000;
		lanedriverstate.spdrelleftfront=-10000;
		lanedriverstate.spdrelleftrear=-10000;
		lanedriverstate.spdrelrightfront=-10000;
		lanedriverstate.spdrelrightrear=-10000;
	}

	if(able_ChangeLane)
		blanechginiobbrake=false;

	if(able_ChangeLane)
		lchghxdist=10000;

	bobwaitlanechg=false;

	bool blanechangerouchedis=false;

	double lenerr;
	if(able_ChangeLane&&(flaganglechack==0)&&(app->lanekeepreq==false)&&(app->lanekeepreqhspdapp==false)&&(!(approachgoal_flag||approachyulukou>0))&&(!(yulukounolanechange)))
	{
		{
			CvPoint2D64f pianyipointtmp111=realtime_Gps;
			double mindis_pianyi=100000;
			double dis_tmp=0;
			int i_pianyitmp=0;
			for(i_pianyitmp=0;i_pianyitmp<200;i_pianyitmp++)
			{
				dis_tmp = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,rndfbezier[i_pianyitmp].x,rndfbezier[i_pianyitmp].y);
				if(dis_tmp < mindis_pianyi)
				{
					mindis_pianyi= dis_tmp;
					pianyipointtmp111=rndfbezier[i_pianyitmp];
				}
			}
			int npiamyimin=0;
			mindis_pianyi = 100000;
			dis_tmp=0;
			CvPoint2D64f pianyipoint1;
			CvPoint2D64f pianyipoint2;
			double angleerr;
			double lengtherr;
			for(i_pianyitmp=0;i_pianyitmp<200;i_pianyitmp++)
			{
				dis_tmp = m_GpsData.GetDistance(pianyipointtmp111.x,pianyipointtmp111.y,rndfbezierlanechangeinitial[i_pianyitmp].x,rndfbezierlanechangeinitial[i_pianyitmp].y);
				if(dis_tmp < mindis_pianyi)
				{
					mindis_pianyi= dis_tmp;
					npiamyimin=i_pianyitmp;
				}
			}
			int n_last=npiamyimin;
			int n_first=npiamyimin;
			double ll=0;
			for(n_last=npiamyimin;n_last<199;n_last++)
			{
				ll=ll+m_GpsData.GetDistance(rndfbezierlanechangeinitial[n_last].x,rndfbezierlanechangeinitial[n_last].y,rndfbezierlanechangeinitial[n_last+1].x,rndfbezierlanechangeinitial[n_last+1].y);
				if(ll>1)
				{
					n_last++;
					break;
				}
			}
			ll=0;
			for(n_first=npiamyimin;n_first>0;n_first--)
			{
				ll=ll+m_GpsData.GetDistance(rndfbezierlanechangeinitial[n_first].x,rndfbezierlanechangeinitial[n_first].y,rndfbezierlanechangeinitial[n_first-1].x,rndfbezierlanechangeinitial[n_first-1].y);
				if(ll>1)
				{
					n_first--;
					break;
				}
			}

			pianyipoint1=rndfbezierlanechangeinitial[n_first];
			pianyipoint2=rndfbezierlanechangeinitial[n_last];

			angleerr=m_GpsData.GetAngle(pianyipointtmp111,pianyipoint1)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
			lengtherr=(m_GpsData.GetDistance(pianyipointtmp111.x,pianyipointtmp111.y,pianyipoint1.x,pianyipoint1.y))*(sin(angleerr*PI/180));
			if(app->lukoulanechangereq)
			{
				if(lengtherr>2.5)
				{
					if(laneturndircnt<0)
						laneturndircnt=0;
					else
						laneturndircnt++;
					if(laneturndircnt>3)
						app->laneturndir=1;
				}
				else if(lengtherr<-2.5)
				{
					if(laneturndircnt>0)
						laneturndircnt=0;
					else
						laneturndircnt--;
					if(laneturndircnt<-3)
						app->laneturndir=2;
				}
				else
					laneturndircnt=0;
			}
			else
			{
				if(lengtherr>2.5)//3
				{
					if(laneturndircnt<0)
						laneturndircnt=0;
					else
						laneturndircnt++;
					if(laneturndircnt>3)
						app->laneturndir=1;
				}
				else if(lengtherr<-2.5)//-3
				{
					if(laneturndircnt>0)
						laneturndircnt=0;
					else
						laneturndircnt--;
					if(laneturndircnt<-3)
						app->laneturndir=2;
				}
				else
					laneturndircnt=0;
			}
			lenerr=lengtherr;
		}

		lanedriverstate.bwaitlchg=false;

		bobwaitlanechg=false;

		ob_res = lanedriverstate.SearchMoveStaticObstacleGps1(rndfbezier);

		if((lanedriverstate.bobfrontlchgreqdisp)&&(!(ob_res==1 || ob_res == -1)))
		{
			bobwaitlanechg=true;
			if(lanedriverstate.bobleftlchgnotalooweddisp && lanedriverstate.bobrightlchgnotalooweddisp)
				blanechangerouchedis=true;
		}
		else
			bobwaitlanechg=false;

		ob_tempoint = cvPoint2D64f(realtime_Gps.x,realtime_Gps.y);
		lanechangedistmp=0;
		app->secondDecison = "车道保持";
		if( ob_res==1 || ob_res == -1)
		{
			double threshold;
			if(app->GPS_Speed<15/3.6)
				threshold=20;
			else if(app->GPS_Speed>45/3.6)
				threshold=40;
			else
				threshold=20+(app->GPS_Speed*3.6-15)/(45-15)*(40-20);

			threshold=threshold+5;//12

			if(lanedriverstate.y_middle_front[0]>-1000 && lanedriverstate.y_middle_front[0]<threshold)
				blanechginiobbrake=true;

			{
				int i_pianyitmp=0;
				int npiamyimin=0;
				double mindis_pianyi = 100000;
				double dis_tmp=0;
				CvPoint2D64f pianyipoint1;
				CvPoint2D64f pianyipoint2;
				double angleerr;
				double lengtherr;
				for(i_pianyitmp=0;i_pianyitmp<200;i_pianyitmp++)
				{
					dis_tmp = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,rndfbezier[i_pianyitmp].x,rndfbezier[i_pianyitmp].y);
					if(dis_tmp < mindis_pianyi)
					{
						mindis_pianyi= dis_tmp;
						npiamyimin=i_pianyitmp;
					}
				}
				int t=199;
				double len = 0;
				for(int i = npiamyimin;i<199;i++)
				{
					len += m_GpsData.GetDistance(rndfbezier[i].x,rndfbezier[i].y,rndfbezier[i+1].x,rndfbezier[i+1].y);
					if(len > threshold)
					{
						t = i+1;
						break;
					}
				}
				lanechginiendpnt=rndfbezier[t];
			}


			//MoveLeft(rndfbezier,rndfbezier,-0.53*ob_res,pianyi);
			lanechangecount=1;//lanechangecount=0;
			able_ChangeLane = false;
			ReturnPartPoints_lanechg(app->gpsmap_road,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);

			lanechangespd=app->GPS_Speed;
			if(lanechangespd<10/3.6)
				lanechangespd=10/3.6;
			if(ob_res == -1)
				app->leftturnreq_proc=false;
			if(ob_res == 1)
				app->rightturnreq_proc=false;
			app->laneturndir = 0;
			if(ob_res == -1)
			{
				app->secondDecison = "左换道";
				app->left_right = -1;
			    control.setLight(LEFT_LAMP_ON);
				outleftchange<<app->secondDecison<<"	"<<strTime1<<endl;
			}
			if(ob_res == 1)
			{
				app->secondDecison = "右换道";
				app->left_right = 1;
			    control.setLight(RIGHT_LAMP_ON);
				outrightchange<<app->secondDecison<<"	"<<strTime1<<endl;
			}
		}
		//if (ob_res == 2 || ob_res == 4 || ob_res == 5)
		if(ob_res == 5 || ob_res==10)
		{
			if(ob_res==10)
			{
				app->continue_spdredtmp = true;
				app->continue_braketmp = true;
				if(-2<app->spdredacctmp)
					app->spdredacctmp = -2;
				app->drive_obstacle=0;
				bspeciallukoustop=true;
			}

			if(lanedriverstate.bwaitlchg)
			{
				if(app->drive_state>10/3.6)
					app->drive_state=10/3.6;
				if(app->GPS_Speed>12/3.6)
				{
					app->continue_spdredtmp = true;
					app->continue_braketmp = true;
					if(app->spdredacctmp>-1.5)
						app->spdredacctmp=-1.5;
				}
			}

			double lanenochangedist;
			if(app->GPS_Speed*3.6<10)
				lanenochangedist=8;
			else if(app->GPS_Speed*3.6>60)
				lanenochangedist=8;
			else
				lanenochangedist=8;

			double lanenochangedist_;
			if(app->GPS_Speed*3.6<10)
				lanenochangedist_=10;
			else if(app->GPS_Speed*3.6>60)
				lanenochangedist_=15;
			else
				lanenochangedist_=9+(app->GPS_Speed*3.6)/10;
			if(lanedriverstate.SearchObstacle1_frontnear(rndfbezier,vel_Map,8,18,28,lanenochangedist_,0))
				lanenochangedist=8;
			else
				lanenochangedist=6;

			if(bcurveactive)
				lanenochangedist=6;

			//middle_dis=lanedriverstate.SearchFrontOb(vel_Map,lanenochangedist);
			middle_dis=pathplan.SearchFrontOb(vel_Map,lanenochangedist);
			if(middle_dis>0)
			{
				if(app->bnearstop||((m_task==IVTASK_APPROINTERSECTION)&&(lukou_fx==3||lukou_fx==4||lukou_fx==8)&&(approachyulukou>0)))
				{
					if(middle_dis>stopmiddledis)//5
						drivespdtmp=5/3.6;
					else
						drivespdtmp=0;

					if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(middle_dis>=7))
					{
						app->continue_spdredtmp = true;
						app->continue_braketmp = true;
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
					}
					else if((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis<5))
					{
						app->continue_spdredtmp = true;
						app->continue_braketmp = true;
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
					}

					if(spdacctemp<app->spdredacc)
					{
						if(app->spdredacc<-2)
							spdacctemp=app->spdredacc;
						else if(spdacctemp<-2)
							spdacctemp=-2;
					}
					if(spdacctemp<app->spdredacctmp)
						app->spdredacctmp = spdacctemp;

					if(app->drive_obstacle>drivespdtmp)
						app->drive_obstacle=drivespdtmp;
				}
				else
				{
					if(middle_dis>stopmiddledis)
						drivespdtmp=5/3.6;
					else
						drivespdtmp=0;

					if((app->GPS_Speed>5/3.6 + 2.5/3.6)&&(middle_dis>=7))
					{
						app->continue_spdredtmp = true;
						app->continue_braketmp = true;
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
					}
					else if((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis<6))//7
					{
						app->continue_spdredtmp = true;
						app->continue_braketmp = true;
						spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
					}

					if(spdacctemp<app->spdredacc)
					{
						if(app->spdredacc<-2)
							spdacctemp=app->spdredacc;
						else if(spdacctemp<-2)
							spdacctemp=-2;
					}
					if(spdacctemp<app->spdredacctmp)
						app->spdredacctmp = spdacctemp;

					if(app->drive_obstacle>drivespdtmp)
						app->drive_obstacle=drivespdtmp;
				}
			}
		}
	}
	else
	{
		bobwaitlanechg=0;

		laneturndircnt=0;
		lanedriverstate.boblefthis=false;
		lanedriverstate.bobrighthis=false;
		lanedriverstate.boblefthispre=false;
		lanedriverstate.bobrighthispre=false;

		lanedriverstate.bobleftlchgnotalooweddisp=100;
		lanedriverstate.bobrightlchgnotalooweddisp=100;
		lanedriverstate.bobfrontlchgnotalooweddisp=100;
		lanedriverstate.bobfrontlchgreqdisp=100;

		if(able_ChangeLane&&(!(approachgoal_flag||approachyulukou>0)))
		{
			bspeciallukoustop=true;
		}
	}

	app->bdirectrunreq=false;

	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();

	//double middle_dis_front=lanedriverstate.SearchFrontOb(vel_Map,6);
	double middle_dis_front=pathplan.SearchFrontOb(vel_Map,6);
	if(middle_dis_front==0)
		middle_dis_front=100;

	/*
	CvPoint2D64f rndfbezier_new[200];
	for(int i = 0;i<200;i++)
	{
		rndfbezier_new[i].x=rndfbezier[i].x;
		rndfbezier_new[i].y=rndfbezier[i].y;
	}
	if(able_ChangeLane&&(!(approachgoal_flag||approachyulukou>0)))
	{
		len = 0;
		min_len = 99999;
		n = 0;
		for(int i = 0;i<200;i++)
		{
			len = m_GpsData.GetDistance(m_gps.x,m_gps.y,rndfbezier[i].x,rndfbezier[i].y);
			if(len < min_len)
			{
				min_len = len;
				n = i;
			}
		}
		if(n==0)
			n=1;

		int chadiannum=0;
		for(int i=0;i<199;i++)
		{
			for(int kk=0;kk<10;kk++)
			{
				rndfbezierchadian[chadiannum].x=(rndfbezier[i].x*(10-kk)+rndfbezier[i+1].x*kk)*0.1;
				rndfbezierchadian[chadiannum].y=(rndfbezier[i].y*(10-kk)+rndfbezier[i+1].y*kk)*0.1;
				chadiannum++;
			}
		}
		rndfbezierchadian[chadiannum]=rndfbezier[199];

		n=(n-1)*10;

		for(int i=0;i<1991;i++)
		{
			Rndf_MapPointchadian[i] = m_GpsData.APiontConverD(m_gps,rndfbezierchadian[i],m_gpsdir);
			if(m_gps.x==rndfbezierchadian[i].x&&m_gps.y==rndfbezierchadian[i].y)
			{
				Rndf_MapPointchadian[i].x = 256;
				Rndf_MapPointchadian[i].y = 411;
			}
		}

		int n_obleft=10000;
		int n_obright=10000;
		int n_obleft_his=10000;
		int n_obright_his=10000;
		int x_his=10000;

		for(int i = n;i<1991;i++)
		{
			if(Rndf_MapPointchadian[i].x < 0||Rndf_MapPointchadian[i].x > 511||Rndf_MapPointchadian[i].y > 412||Rndf_MapPointchadian[i].y < 12)
				continue;
			x = Rndf_MapPointchadian[i].x;
			y = Rndf_MapPointchadian[i].y;
			m=0;
			int n_obleft_last=10000;
			int n_obright_last=10000;

			for(int n=-10;n<=10;n++)
			{
				for(int m=5;m>=-5;m--)
				{
					if(y+m>=412||y+m<12||x+n>511||x+n<0)
						continue;
					if(vel_Map->MapPoint[y+m][x+n] == 8||vel_Map->MapPoint[y+m][x+n] == 18||vel_Map->MapPoint[y+m][x+n] == 28)
					{
						if(n_obleft_last==10000)
							n_obleft_last=n;
						else if(n-n_obleft_last<5)
							n_obleft_last=n;
						else
							break;
					}
				}
				if((n_obleft_last!=10000)&&(n-n_obleft_last>=5))
					break;
			}
		}
	}
	*/

	if(able_ChangeLane)
	{
		double lanenochangedist;
		if(app->GPS_Speed*3.6<10)
			lanenochangedist=10;
		else if(app->GPS_Speed*3.6>60)
			lanenochangedist=15;
		else
			lanenochangedist=9+(app->GPS_Speed*3.6)/10;

		/*
		if(lanedriverstate.SearchFrontOb(vel_Map,lanenochangedist)>0)
			app->bdirectrunreq=true;
			*/
		if(lanedriverstate.SearchObstacle1_frontnear(rndfbezier,vel_Map,8,18,28,lanenochangedist,0))
			app->bdirectrunreq=true;
	}

	middle_dis=0;
	drivespdtmp=road_speed;

	bholdbrake=0;

	double updett;
	if(able_ChangeLane||(!((!able_ChangeLane) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)))
		updett = app->GPS_Speed*3.6+12;
	else
	{
		int obresnum=7-lanechangecount;

		/*
		if(lanechangenumtmp<app->lanechangenum)
			obresnum=lanechangenumtmp-lanechangecount;
		else
			obresnum=app->lanechangenum-lanechangecount;
			*/

		updett=(lanemovedist+2)*(7-lanechangecount)+3;//updett = (app->GPS_Speed)*(app->GPS_Speed)/8+6;//need to be modified according to the real lane autochange distance.
		if(updett>(app->GPS_Speed)*(app->GPS_Speed)/8+6)
			updett = (app->GPS_Speed)*(app->GPS_Speed)/8+6;
		//updett = app->GPS_Speed*3.6-10;
		if(!blanechginiobbrake)
			updett=5;
	}
	if(updett<15)
		updett=15;
	else if(updett>80)
		updett=80;

	CvPoint2D64f rndfbeziertmp[200];
	for(int i=0;i<200;i++)
		rndfbeziertmp[i]=rndfbezier[i];

	/*
	if((!able_ChangeLane) && (lanechangecount>=lanechangenumtmp || lanechangecount>=app->lanechangenum))
	{
		for(int i=0;i<200;i++)
			rndfbeziertmp[i]=rndfbezier_lchgdst[i];
	}
	*/
	
	len = 0;
	min_len = 99999;
	n = 0;
	for(int i = 0;i<200;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,rndfbeziertmp[i].x,rndfbeziertmp[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	if(n==0)
		n=1;

	int chadiannum=0;
	for(int i=0;i<199;i++)
	{
		for(int kk=0;kk<10;kk++)
		{
			rndfbezierchadian[chadiannum].x=(rndfbeziertmp[i].x*(10-kk)+rndfbeziertmp[i+1].x*kk)*0.1;
			rndfbezierchadian[chadiannum].y=(rndfbeziertmp[i].y*(10-kk)+rndfbeziertmp[i+1].y*kk)*0.1;
			chadiannum++;
		}
	}
	rndfbezierchadian[chadiannum]=rndfbeziertmp[199];

	n=(n-1)*10;

	//ob_dis_prepre=ob_dis_pre;
	//ob_dis_pre=ob_dis_now;

	for(int i=0;i<1991;i++)
	{
		Rndf_MapPointchadian[i] = m_GpsData.APiontConverD(m_gps,rndfbezierchadian[i],m_gpsdir);
		if(m_gps.x==rndfbezierchadian[i].x&&m_gps.y==rndfbezierchadian[i].y)
		{
			Rndf_MapPointchadian[i].x = 256;
			Rndf_MapPointchadian[i].y = 411;
		}
	}
	int m_up = 12;
	int m_down = 412;

	int a=8;
	int b=18;
	int c=28;
	
	int x = 0;
	int y = 0;

	int width2 = 4;
	if (app->range_flag)
	{
		width2 = 4;
	}
	bool obb_break_flag=0;

	if(able_ChangeLane||(!((!able_ChangeLane) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum)))
	{
		for(int i = n;i<1991;i++)
		{
			if(Rndf_MapPointchadian[i].x < 0||Rndf_MapPointchadian[i].x > 511||Rndf_MapPointchadian[i].y > m_down||Rndf_MapPointchadian[i].y < m_up)
				continue;
			x = Rndf_MapPointchadian[i].x;
			y = Rndf_MapPointchadian[i].y;
			for(int m=4;m>=-4;m--)
			{
				for(int n=width_L;n<=width_R;n++)
				{
					if(y+m>=412)
						continue;
					if(y+m>511||y+m<0||x+n>511||x+n<0)
						continue;
					if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b||vel_Map->MapPoint[y+m][x+n] == c)
					{
						ob_dis_now=412-(y+m);
						//ob_dis_now_timer=app->systemtimer10ms;

						SYSTEMTIME tt;
						GetLocalTime(&tt);
						ob_dis_now_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));


						obb_break_flag=1;

						if(ob_dis_pre<1000)
						{
							//if(abs(ob_dis_now-ob_dis_pre)>30*(ob_loss_count+1))
							if(abs(ob_dis_now-ob_dis_pre)*0.2>30*(ob_dis_now_timer-ob_dis_pre_timer)*0.001)
							{
								ob_dis_pre=1000;
								ob_dis_prepre=1000;
								bfasterobacv=0;
							}
						}
						if(ob_dis_prepre<1000)
						{
							//if(abs(ob_dis_now-ob_dis_pre)>30*(ob_loss_count+1))
							if(abs(ob_dis_now-ob_dis_prepre)*0.2>30*(ob_dis_now_timer-ob_dis_prepre_timer)*0.001)
							{
								ob_dis_pre=1000;
								ob_dis_prepre=1000;
								bfasterobacv=0;
							}
						}

						if(ob_dis_pre==1000 && ob_dis_prepre==1000)
							bobfisrstapper=1;
						else if(bobfisrstapper)
						{
							if(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=0)
								bobfisrstapper=1;
							else if(ob_dis_prepre==1000 && ob_dis_pre<1000 && ob_dis_now-ob_dis_pre>=0)
								bobfisrstapper=1;
							else
								bobfisrstapper=0;
						}
						else
							bobfisrstapper=0;

						bobfisrstapper=0;

						if((ob_dis_pre<1000 && ob_dis_now-ob_dis_pre>=1)||(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=1))
						{
							if(bfasterobacv==0)
							{
								bfasterobacv=1;
								fasterobspdini=app->GPS_Speed;
							}
						}

						bholdbrake=0;
						/*
						if(ob_dis_pre<1000 && ob_dis_now>41)
						{
							if(ob_dis_now-ob_dis_pre>=1)
								bholdbrake=1;
							else if((ob_dis_now-ob_dis_pre)-(ob_dis_pre-ob_dis_prepre)>2)
							{
								if((ob_dis_now-ob_dis_pre)-(ob_dis_pre-ob_dis_prepre)>-(ob_dis_now-ob_dis_pre)*(1-(ob_dis_now-ob_dis_pre))/(2*(ob_dis_now-40))+1)
									bholdbrake=1;
							}
						}
						*/

						if(bfasterobacv &&((abs(412-(y+m))*0.2)>15) && ob_dis_pre<1000 && ob_dis_now-ob_dis_pre>=1 && ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=3)
						{
							if(fasterobspdini<app->GPS_Speed+3.5/3.6)
								fasterobspdini=app->GPS_Speed+3.5/3.6;
							if(fasterobspdini>road_speed)
								fasterobspdini=road_speed;
						}

						obfrontspd=dynamicmap_v_x->MapPoint[y+m][x+n];

						if(ob_dis_now<75)
						{
							if((ob_dis_pre<1000 && ob_dis_now-ob_dis_pre<0)||(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<0))
							{
								if(bfasterobacv==1)
								{
									bfasterobacv=0;
								}
							}
						}
						else
						{
							if((ob_dis_pre<1000 && ob_dis_now-ob_dis_pre<-1)||(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<-1))
							{
								if(bfasterobacv==1)
								{
									bfasterobacv=0;
								}
							}
						}
						if(ob_dis_prepre<1000)
						{
							if(ob_dis_now_timer>ob_dis_prepre_timer)
								obfrontspd=(ob_dis_now-ob_dis_prepre-1)*0.2/(0.001*(ob_dis_now_timer-ob_dis_prepre_timer));
							else
								obfrontspd=(ob_dis_now-ob_dis_prepre-1)*0.2/(0.001);
							/*
							obfrontspd=(ob_dis_now-ob_dis_pre)*0.2/(0.05*(ob_loss_count+1));
							if(obfrontspd>-2*0.2/(0.05*(ob_loss_count+1)))
								obfrontspd=-2*0.2/(0.05*(ob_loss_count+1));
								*/
						}

						middle_dis = abs(412-(y+m))*0.2;
						if(middle_dis>=updett && able_ChangeLane)
						{
							drivespdtmp=road_speed;
							if(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<=0)
							{
								drivespdtmp = (middle_dis-12)/3.6;
								if(middle_dis<=updett+11)
								{
									if(drivespdtmp<app->GPS_Speed)
									{
										drivespdtmp=app->GPS_Speed-2.5/3.6;
										if(drivespdtmp<(middle_dis-12)/3.6)
											drivespdtmp=(middle_dis-12)/3.6;
									}
								}
								else
								{
									if(drivespdtmp<=app->GPS_Speed)
										drivespdtmp=app->GPS_Speed;
								}
							}
							else if(ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre<=2)
							{
								drivespdtmp = (middle_dis-12)/3.6;
								if(middle_dis<=updett+11)
									drivespdtmp=app->GPS_Speed;
								else
								{
									if(drivespdtmp<=app->GPS_Speed)
										drivespdtmp=app->GPS_Speed;
								}
							}
							if(bfasterobacv&&ob_dis_prepre<1000 && ob_dis_now-ob_dis_prepre>=0)
							{
								if(drivespdtmp<fasterobspdini-2/3.6)
									drivespdtmp=fasterobspdini-2/3.6;
							}
							if(drivespdtmp<=10/3.6)
								drivespdtmp=10/3.6;
							if(drivespdtmp>road_speed)
								drivespdtmp=road_speed;
							if(app->drive_obstacle>drivespdtmp)
								app->drive_obstacle=drivespdtmp;
						}

						drivespdtmp=road_speed;

						if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b)
						{
							middle_dis = abs(412-(y+m))*0.2;
							if(middle_dis<updett)
							{
								drivespdtmp = (middle_dis-8)/3.6;
								if(middle_dis_front>stopmiddledis&&drivespdtmp<5/3.6)
									drivespdtmp=5/3.6;
								if(drivespdtmp<0)
									drivespdtmp=0;
								else if(drivespdtmp>road_speed)
									drivespdtmp=road_speed;

								if(drivespdtmp>app->GPS_Speed)
								{
									if(app->GPS_Speed>10/3.6)
										drivespdtmp = app->GPS_Speed;
									else if(middle_dis>18)
										drivespdtmp = 10/3.6;
								}
								/*
								if(middle_dis>8 && bobfisrstapper)
								{
									if(drivespdtmp<app->GPS_Speed-1/3.6)
										drivespdtmp=app->GPS_Speed-1/3.6;
									if(drivespdtmp<0)
										drivespdtmp=0;
								}
								*/
								if(bobwaitlanechg && middle_dis<12)
								{
									drivespdtmp=0;
									if(blanechangerouchedis)
										bspeciallukoustop=true;
								}

								if(bfasterobacv&&middle_dis>8)
								{
									if(drivespdtmp<fasterobspdini-2/3.6)
										drivespdtmp=fasterobspdini-2/3.6;
									if(drivespdtmp<0)
										drivespdtmp=0;
								}
								if(app->drive_obstacle>drivespdtmp)
									app->drive_obstacle=drivespdtmp;
							}
							if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
							{
								if(!(bfasterobacv&&middle_dis>8))
								{
									if(((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(middle_dis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && middle_dis<updett))
									{
										app->continue_spdredtmp = true;
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-8));

										if(bobwaitlanechg && middle_dis>12)
										{
											spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-11));
										}

										if((ob_dis_prepre<1000)&&(middle_dis>=9))
										{
											double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));

											if(bobwaitlanechg && middle_dis>12)
											{
												spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-11));
											}

											if(spdaccdist>-2)
												spdaccdist=-2;
											if(spdacctemp<spdaccdist)
												spdacctemp=spdaccdist;
										}
										if(middle_dis>8 && bobfisrstapper)
										{
											if(spdacctemp<-3)
												spdacctemp=-3;
										}
										else
											app->continue_braketmp = true;
										if(bholdbrake)
										{
											if(spdacctemp<app->spdredacc)
											{
												if(app->spdredacc<-3)
													spdacctemp=app->spdredacc;
												else if(spdacctemp<-3)
													spdacctemp=-3;
											}
										}
										if(spdacctemp<app->spdredacctmp)
											app->spdredacctmp = spdacctemp;
									}
									else if((app->GPS_Speed>10/3.6 + 2.5/3.6)||(middle_dis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis_front<stopmiddledis))))
									{
										app->continue_spdredtmp = true;
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
										if((ob_dis_prepre<1000)&&(middle_dis>=9))
										{
											double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));

											if(bobwaitlanechg && middle_dis>12)
											{
												spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-11));
											}

											if(spdaccdist>-2)
												spdaccdist=-2;
											if(spdacctemp<spdaccdist)
												spdacctemp=spdaccdist;
										}
										if(middle_dis>8 && bobfisrstapper)
										{
											if(spdacctemp<-3)
												spdacctemp=-3;
										}
										else
											app->continue_braketmp = true;
										if(bholdbrake)
										{
											if(spdacctemp<app->spdredacc)
											{
												if(app->spdredacc<-3)
													spdacctemp=app->spdredacc;
												else if(spdacctemp<-3)
													spdacctemp=-3;
											}
										}
										if(bfasterobacv||bobfisrstapper)
										{
											if(spdacctemp<-3)
												spdacctemp=-3;
										}

										if(spdacctemp<app->spdredacc && spdacctemp<-2 && (!bobwaitlanechg))
										{
											if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
											{
												if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
													spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
												if(spdacctemp>app->spdredacc)
													spdacctemp=app->spdredacc;
												if(spdacctemp>-2)
													spdacctemp=-2;
											}
											else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
											{
												if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
													spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
												if(spdacctemp>app->spdredacc)
													spdacctemp=app->spdredacc;
												if(spdacctemp>-2)
													spdacctemp=-2;
											}
										}

										if(spdacctemp<app->spdredacctmp)
											app->spdredacctmp = spdacctemp;
									}
								}
								else if(bfasterobacv&&middle_dis<15)
								{
									spdacctemp=0;
									if(app->GPS_Speed>10/3.6 + 2.5/3.6)
									{
										app->continue_spdredtmp = true;
										if(spdacctemp<app->spdredacctmp)
											app->spdredacctmp = spdacctemp;
									}
								}
							}
						}
						if(vel_Map->MapPoint[y+m][x+n] == c)
						{
							if(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>237)
							{
								middle_dis = abs(412-(y+m))*0.2;
								if(middle_dis>40)
								{
									if(dynamicmap_v_x->MapPoint[y+m][x+n]>0)
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											if(bfasterobacv&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini-2/3.6)
													drivespdtmp=fasterobspdini-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										spdacctemp=0;
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(!(bfasterobacv&&middle_dis>15)))
											{
												app->continue_spdredtmp = true;
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
									}
									else
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n];
											if(drivespdtmp>app->GPS_Speed)
												drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											/*
											if(middle_dis>8 && bobfisrstapper)
											{
												if(drivespdtmp<app->GPS_Speed-1/3.6)
													drivespdtmp=app->GPS_Speed-1/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											*/
											if(bfasterobacv&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini-2/3.6)
													drivespdtmp=fasterobspdini-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n])*(dynamicmap_v_x->MapPoint[y+m][x+n])/2/(middle_dis-36);
										if(spdacctemp<-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-25))
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-25);
										if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12)))
											spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12));
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv&&middle_dis>8))
											{
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if((ob_dis_prepre<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else if(dynamicmap_v_x->MapPoint[y+m][x+n]<-3/3.6)
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
											else if(bfasterobacv&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
										}
									}
								}
								else if(middle_dis>15)
								{
									if(dynamicmap_v_x->MapPoint[y+m][x+n]>10/3.6)
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											if(bfasterobacv&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini-2/3.6)
													drivespdtmp=fasterobspdini-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										spdacctemp=0;
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(!(bfasterobacv&&middle_dis>15)))
											{
												app->continue_spdredtmp = true;
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
									}
									else
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6;
											if(drivespdtmp>app->GPS_Speed)
												drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											/*
											if(middle_dis>8 && bobfisrstapper)
											{
												if(drivespdtmp<app->GPS_Speed-1/3.6)
													drivespdtmp=app->GPS_Speed-1/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											*/
											if(bfasterobacv&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini-2/3.6)
													drivespdtmp=fasterobspdini-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										if(middle_dis>25)
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-20);
										else if(middle_dis>17)
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/((middle_dis-15)/2);
										else
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2;
										if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12)))
											spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12));
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv&&middle_dis>8))
											{
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if((ob_dis_prepre<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else if(dynamicmap_v_x->MapPoint[y+m][x+n]<7/3.6)
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
											else if(bfasterobacv&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
										}
									}
								}
								else
									{
										if(middle_dis<updett)
										{
											drivespdtmp = (middle_dis-8)/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
											else if(drivespdtmp>road_speed)
												drivespdtmp=road_speed;
											if(middle_dis_front>stopmiddledis&&drivespdtmp<5/3.6)
												drivespdtmp=5/3.6;

											if(drivespdtmp>app->GPS_Speed)
											{
												if(app->GPS_Speed>10/3.6)
													drivespdtmp = app->GPS_Speed;
												else if(middle_dis>18)
													drivespdtmp = 10/3.6;
											}
											/*
											if(middle_dis>8 && bobfisrstapper)
											{
												if(drivespdtmp<app->GPS_Speed-1/3.6)
													drivespdtmp=app->GPS_Speed-1/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											*/

											if(bobwaitlanechg && middle_dis<12)
											{
												drivespdtmp=0;
												if(blanechangerouchedis)
													bspeciallukoustop=true;
											}

											if(bfasterobacv&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini-2/3.6)
													drivespdtmp=fasterobspdini-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv&&middle_dis>8))
											{
												if(((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(middle_dis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && middle_dis<updett))
												{
													app->continue_spdredtmp = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-8));

													if(bobwaitlanechg && middle_dis>12)
													{
														spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-11));
													}

													if((ob_dis_prepre<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));

														if(bobwaitlanechg && middle_dis>12)
														{
															spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-11));
														}

														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
												else if((app->GPS_Speed>10/3.6 + 2.5/3.6)||(middle_dis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis_front<stopmiddledis))))
												{
													app->continue_spdredtmp = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
													if((ob_dis_prepre<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));

														if(bobwaitlanechg && middle_dis>12)
														{
															spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-11));
														}

														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(bfasterobacv||bobfisrstapper)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}

													if(spdacctemp<app->spdredacc && spdacctemp<-2 && (!bobwaitlanechg))
													{
														if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
														else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
													}

													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
											else if(bfasterobacv&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
										}
									}
							}
						}
						break;
					}
				}
				if(obb_break_flag)
					break;
			}
			if(obb_break_flag)
				break;
		}
		if(obb_break_flag==0)
		{
			ob_loss_count++;
			if(ob_loss_count>=5||ob_dis_now==1000)
			{
				ob_dis_now=1000;
				ob_dis_pre=1000;
				ob_dis_prepre=1000;
				//ob_dis_now_timer=app->systemtimer10ms;
				//ob_dis_pre_timer=app->systemtimer10ms;
				//ob_dis_prepre_timer=app->systemtimer10ms;

				SYSTEMTIME tt;
				GetLocalTime(&tt);
				ob_dis_now_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

				ob_dis_pre_timer=ob_dis_now_timer;
				ob_dis_prepre_timer=ob_dis_now_timer;

				bfasterobacv=0;
				ob_loss_count=0;
			}
		}
		else
		{
			ob_dis_prepre=ob_dis_pre;
			ob_dis_prepre_timer=ob_dis_pre_timer;
			ob_dis_pre=ob_dis_now;
			ob_dis_pre_timer=ob_dis_now_timer;
			ob_loss_count=0;
		}
	}
	else
	{
		fasterobspdini=road_speed;
		bfasterobacv=0;
		bobfisrstapper=0;
		ob_dis_now=1000;
		ob_dis_pre=1000;
		ob_dis_prepre=1000;
		//ob_dis_now_timer=app->systemtimer10ms;
		//ob_dis_pre_timer=app->systemtimer10ms;
		//ob_dis_prepre_timer=app->systemtimer10ms;

		SYSTEMTIME tt;
		GetLocalTime(&tt);
		ob_dis_now_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

		ob_dis_pre_timer=ob_dis_now_timer;
		ob_dis_prepre_timer=ob_dis_now_timer;
		ob_loss_count=0;
	}

	bholdbrake=0;

	//if((!able_ChangeLane) && lanechangecount<lanechangenumtmp && lanechangecount<app->lanechangenum && rndfbezier_lchgdst[0].x!=0 && rndfbezier_lchgdst[0].y!=0)
	if((!able_ChangeLane) && rndfbezier_lchgdst[0].x!=0 && rndfbezier_lchgdst[0].y!=0)
	{
		updett = app->GPS_Speed*3.6+12;
		if(updett<15)
			updett=15;
		else if(updett>80)
			updett=80;

		int obresnum=0;
		if(lanechangenumtmp<app->lanechangenum)
			obresnum=lanechangenumtmp-lanechangecount;
		else
			obresnum=app->lanechangenum-lanechangecount;

		CvPoint2D64f rndfbeziertemp[200];
		for(int i=0;i<200;i++)
			rndfbeziertemp[i]=rndfbezier_lchgdst[i];

		//MoveLeft(rndfbezier,rndfbeziertemp,-0.53*ob_res*obresnum);

		app->critical_section.Lock();
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;
		app->critical_section.Unlock();

		len = 0;
		min_len = 99999;
		n = 0;
		for(int i = 0;i<200;i++)
		{
			len = m_GpsData.GetDistance(m_gps.x,m_gps.y,rndfbeziertemp[i].x,rndfbeziertemp[i].y);
			if(len < min_len)
			{
				min_len = len;
				n = i;
			}
		}
		if(n==0)
			n=1;

		chadiannum=0;
		for(int i=0;i<199;i++)
		{
			for(int kk=0;kk<10;kk++)
			{
				rndfbezierchadian[chadiannum].x=(rndfbeziertemp[i].x*(10-kk)+rndfbeziertemp[i+1].x*kk)*0.1;
				rndfbezierchadian[chadiannum].y=(rndfbeziertemp[i].y*(10-kk)+rndfbeziertemp[i+1].y*kk)*0.1;
				chadiannum++;
			}
		}
		rndfbezierchadian[chadiannum]=rndfbeziertemp[199];

		n=(n-1)*10;

		//ob_dis_prepre_lchg=ob_dis_pre_lchg;
		//ob_dis_pre_lchg=ob_dis_now_lchg;

		CvPoint2D64f Rndf_Maplanechginiendpnt = m_GpsData.APiontConverD(m_gps,lanechginiendpnt,m_gpsdir);

		int i_pianyitmp=0;
		int npiamyimin=0;
		double mindis_pianyi = 100000;
		double dis_tmp=0;
		CvPoint2D64f pianyipoint1;
		CvPoint2D64f pianyipoint2;
		double angleerr;
		double lengtherr;
		for(i_pianyitmp=0;i_pianyitmp<200;i_pianyitmp++)
		{
			dis_tmp = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,rndfbeziertemp[i_pianyitmp].x,rndfbeziertemp[i_pianyitmp].y);
			if(dis_tmp < mindis_pianyi)
			{
				mindis_pianyi= dis_tmp;
				npiamyimin=i_pianyitmp;
			}
		}
		int n_last=npiamyimin;
		int n_first=npiamyimin;
		double ll=0;
		for(n_last=npiamyimin;n_last<199;n_last++)
		{
			ll=ll+m_GpsData.GetDistance(rndfbeziertemp[n_last].x,rndfbeziertemp[n_last].y,rndfbeziertemp[n_last+1].x,rndfbeziertemp[n_last+1].y);
			if(ll>1)
			{
				n_last++;
				break;
			}
		}
		ll=0;
		for(n_first=npiamyimin;n_first>0;n_first--)
		{
			ll=ll+m_GpsData.GetDistance(rndfbeziertemp[n_first].x,rndfbeziertemp[n_first].y,rndfbeziertemp[n_first-1].x,rndfbeziertemp[n_first-1].y);
			if(ll>1)
			{
				n_first--;
				break;
			}
		}

		pianyipoint1=rndfbeziertemp[n_first];
		pianyipoint2=rndfbeziertemp[n_last];

		angleerr=m_GpsData.GetAngle(realtime_Gps,pianyipoint1)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
		lengtherr=(m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,pianyipoint1.x,pianyipoint1.y))*(sin(angleerr*PI/180));

		int nleft=width_L;
		int nright=width_R;

		if(ob_res == -1 && lengtherr>0)
		{
			if(nright<(int)((lengtherr+1)/0.2))
				nright=(int)((lengtherr+1)/0.2);
		}
		if(ob_res == 1 && lengtherr<0)
		{
			if(nleft>(int)((lengtherr-1)/0.2))
				nleft=(int)((lengtherr-1)/0.2);
		}

		for(int i=0;i<1991;i++)
		{
			Rndf_MapPointchadian[i] = m_GpsData.APiontConverD(m_gps,rndfbezierchadian[i],m_gpsdir);
			if(m_gps.x==rndfbezierchadian[i].x&&m_gps.y==rndfbezierchadian[i].y)
			{
				Rndf_MapPointchadian[i].x = 256;
				Rndf_MapPointchadian[i].y = 411;
			}
		}
		int m_up = 12;
		int m_down = 412;

		int a=8;
		int b=18;
		int c=28;
		
		int x = 0;
		int y = 0;

		int width2 = 4;
		if (app->range_flag)
		{
			width2 = 4;
		}
		bool obb_break_flag=0;
		for(int i = n;i<1991;i++)
		{
			if(Rndf_MapPointchadian[i].x < 0||Rndf_MapPointchadian[i].x > 511||Rndf_MapPointchadian[i].y > m_down||Rndf_MapPointchadian[i].y < m_up)
				continue;
			x = Rndf_MapPointchadian[i].x;
			y = Rndf_MapPointchadian[i].y;
			for(int m=4;m>=-4;m--)
			{
				for(int n=nleft;n<=nright;n++)
				{
					if(y+m<Rndf_Maplanechginiendpnt.y && (n<width_L || n>width_R))
						continue;
					if(y+m>=412)
						continue;
					if(y+m>511||y+m<0||x+n>511||x+n<0)
						continue;
					if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b||vel_Map->MapPoint[y+m][x+n] == c)
					{
						ob_dis_now_lchg=412-(y+m);
						//ob_dis_now_lchg_timer=app->systemtimer10ms;

						SYSTEMTIME tt;
						GetLocalTime(&tt);
						ob_dis_now_lchg_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

						obb_break_flag=1;

						if(ob_dis_pre_lchg<1000)
						{
							//if(abs(ob_dis_now_lchg-ob_dis_pre_lchg)>30*(ob_loss_count_lchg+1))
							if(abs(ob_dis_now_lchg-ob_dis_pre_lchg)*0.2>30*(ob_dis_now_lchg_timer-ob_dis_pre_lchg_timer)*0.001)
							{
								ob_dis_pre_lchg=1000;
								ob_dis_prepre_lchg=1000;
								bfasterobacv_lchg=0;
							}
						}

						if(ob_dis_prepre_lchg<1000)
						{
							//if(abs(ob_dis_now_lchg-ob_dis_pre_lchg)>30*(ob_loss_count_lchg+1))
							if(abs(ob_dis_now_lchg-ob_dis_prepre_lchg)*0.2>30*(ob_dis_now_lchg_timer-ob_dis_prepre_lchg_timer)*0.001)
							{
								ob_dis_pre_lchg=1000;
								ob_dis_prepre_lchg=1000;
								bfasterobacv_lchg=0;
							}
						}

						if(ob_dis_pre_lchg==1000 && ob_dis_prepre_lchg==1000)
							bobfisrstapper_lchg=1;
						else if(bobfisrstapper_lchg)
						{
							if(ob_dis_prepre_lchg<1000 && ob_dis_now_lchg-ob_dis_prepre_lchg>=0)
								bobfisrstapper_lchg=1;
							else if(ob_dis_prepre_lchg==1000 && ob_dis_pre_lchg<1000 && ob_dis_now_lchg-ob_dis_pre_lchg>=0)
								bobfisrstapper_lchg=1;
							else
								bobfisrstapper_lchg=0;
						}
						else
							bobfisrstapper_lchg=0;

						if((ob_dis_pre_lchg<1000 && ob_dis_now_lchg-ob_dis_pre_lchg>=1)||(ob_dis_prepre_lchg<1000 && ob_dis_now_lchg-ob_dis_prepre_lchg>=1))
						{
							if(bfasterobacv_lchg==0)
							{
								bfasterobacv_lchg=1;
								fasterobspdini_lchg=app->GPS_Speed;
							}
						}

						bholdbrake=0;
						/*
						if(ob_dis_pre_lchg<1000 && ob_dis_now_lchg>41)
						{
							if(ob_dis_now_lchg-ob_dis_pre_lchg>=1)
								bholdbrake=1;
							else if((ob_dis_now_lchg-ob_dis_pre_lchg)-(ob_dis_pre_lchg-ob_dis_prepre_lchg)>2)
							{
								if((ob_dis_now_lchg-ob_dis_pre_lchg)-(ob_dis_pre_lchg-ob_dis_prepre_lchg)>-(ob_dis_now_lchg-ob_dis_pre_lchg)*(1-(ob_dis_now_lchg-ob_dis_pre_lchg))/(2*(ob_dis_now_lchg-40))+1)
									bholdbrake=1;
							}
						}
						*/

						if(bfasterobacv_lchg &&((abs(412-(y+m))*0.2)>15) && ob_dis_pre_lchg<1000 && ob_dis_now_lchg-ob_dis_pre_lchg>=1 && ob_dis_prepre_lchg<1000 && ob_dis_now_lchg-ob_dis_prepre_lchg>=3)
						{
							if(fasterobspdini_lchg<app->GPS_Speed+3.5/3.6)
								fasterobspdini_lchg=app->GPS_Speed+3.5/3.6;
							if(fasterobspdini_lchg>road_speed)
								fasterobspdini_lchg=road_speed;
						}

						obfrontspd=dynamicmap_v_x->MapPoint[y+m][x+n];

						if(ob_dis_now_lchg<75)
						{
							if((ob_dis_pre_lchg<1000 && ob_dis_now_lchg-ob_dis_pre_lchg<0)||(ob_dis_prepre_lchg<1000 && ob_dis_now_lchg-ob_dis_prepre_lchg<0))
							{
								if(bfasterobacv_lchg==1)
								{
									bfasterobacv_lchg=0;
								}
							}
						}
						else
						{
							if((ob_dis_pre_lchg<1000 && ob_dis_now_lchg-ob_dis_pre_lchg<-1)||(ob_dis_prepre_lchg<1000 && ob_dis_now_lchg-ob_dis_prepre_lchg<-1))
							{
								if(bfasterobacv_lchg==1)
								{
									bfasterobacv_lchg=0;
								}
							}
						}
						if(ob_dis_prepre_lchg<1000)
						{
							if(ob_dis_now_lchg_timer>ob_dis_prepre_lchg_timer)
								obfrontspd=(ob_dis_now_lchg-ob_dis_prepre_lchg-1)*0.2/(0.001*(ob_dis_now_lchg_timer-ob_dis_prepre_lchg_timer));
							else
								obfrontspd=(ob_dis_now_lchg-ob_dis_prepre_lchg-1)*0.2/(0.001);
							/*
							obfrontspd=(ob_dis_now_lchg-ob_dis_pre_lchg)*0.2/(0.05*(ob_loss_count_lchg+1));
							if(obfrontspd>-2*0.2/(0.05*(ob_loss_count_lchg+1)))
								obfrontspd=-2*0.2/(0.05*(ob_loss_count_lchg+1));
								*/
						}

						if(vel_Map->MapPoint[y+m][x+n] == a||vel_Map->MapPoint[y+m][x+n] == b)
						{
							middle_dis = abs(412-(y+m))*0.2;
							if(middle_dis<updett)
							{
								drivespdtmp = (middle_dis-8)/3.6;
								if(drivespdtmp<0)
									drivespdtmp=0;
								else if(drivespdtmp>road_speed)
									drivespdtmp=road_speed;
								if(middle_dis_front>stopmiddledis&&drivespdtmp<5/3.6)
									drivespdtmp=5/3.6;

								if(drivespdtmp>app->GPS_Speed)
								{
									if(app->GPS_Speed>10/3.6)
										drivespdtmp = app->GPS_Speed;
									else if(middle_dis>18)
										drivespdtmp = 10/3.6;
								}
								/*
								if(middle_dis>8 && bobfisrstapper_lchg)
								{
									if(drivespdtmp<app->GPS_Speed-1/3.6)
										drivespdtmp=app->GPS_Speed-1/3.6;
									if(drivespdtmp<0)
										drivespdtmp=0;
								}
								*/
								if(bfasterobacv_lchg&&middle_dis>8)
								{
									if(drivespdtmp<fasterobspdini_lchg-2/3.6)
										drivespdtmp=fasterobspdini_lchg-2/3.6;
									if(drivespdtmp<0)
										drivespdtmp=0;
								}
								if(app->drive_obstacle>drivespdtmp)
									app->drive_obstacle=drivespdtmp;
							}
							if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
							{
								if(!(bfasterobacv_lchg&&middle_dis>8))
								{
									if(((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(middle_dis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && middle_dis<updett))
									{
										app->continue_spdredtmp = true;
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-8));
										if((ob_dis_prepre_lchg<1000)&&(middle_dis>=9))
										{
											double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
											if(spdaccdist>-2)
												spdaccdist=-2;
											if(spdacctemp<spdaccdist)
												spdacctemp=spdaccdist;
										}
										if(middle_dis>8 && bobfisrstapper_lchg)
										{
											if(spdacctemp<-3)
												spdacctemp=-3;
										}
										else
											app->continue_braketmp = true;
										if(bholdbrake)
										{
											if(spdacctemp<app->spdredacc)
											{
												if(app->spdredacc<-3)
													spdacctemp=app->spdredacc;
												else if(spdacctemp<-3)
													spdacctemp=-3;
											}
										}
										if(spdacctemp<app->spdredacctmp)
											app->spdredacctmp = spdacctemp;
									}
									else if((app->GPS_Speed>10/3.6 + 2.5/3.6)||(middle_dis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis_front<stopmiddledis))))
									{
										app->continue_spdredtmp = true;
										spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
										if((ob_dis_prepre_lchg<1000)&&(middle_dis>=9))
										{
											double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
											if(spdaccdist>-2)
												spdaccdist=-2;
											if(spdacctemp<spdaccdist)
												spdacctemp=spdaccdist;
										}
										if(middle_dis>8 && bobfisrstapper_lchg)
										{
											if(spdacctemp<-3)
												spdacctemp=-3;
										}
										else
											app->continue_braketmp = true;
										if(bholdbrake)
										{
											if(spdacctemp<app->spdredacc)
											{
												if(app->spdredacc<-3)
													spdacctemp=app->spdredacc;
												else if(spdacctemp<-3)
													spdacctemp=-3;
											}
										}
										if(bfasterobacv_lchg||bobfisrstapper_lchg)
										{
											if(spdacctemp<-3)
												spdacctemp=-3;
										}

										if(spdacctemp<app->spdredacc && spdacctemp<-2)
										{
											if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
											{
												if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
													spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
												if(spdacctemp>app->spdredacc)
													spdacctemp=app->spdredacc;
												if(spdacctemp>-2)
													spdacctemp=-2;
											}
											else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
											{
												if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
													spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
												if(spdacctemp>app->spdredacc)
													spdacctemp=app->spdredacc;
												if(spdacctemp>-2)
													spdacctemp=-2;
											}
										}

										if(spdacctemp<app->spdredacctmp)
											app->spdredacctmp = spdacctemp;
									}
								}
								else if(bfasterobacv_lchg&&middle_dis<15)
								{
									spdacctemp=0;
									if(app->GPS_Speed>10/3.6 + 2.5/3.6)
									{
										app->continue_spdredtmp = true;
										if(spdacctemp<app->spdredacctmp)
											app->spdredacctmp = spdacctemp;
									}
								}
							}
						}
						if(vel_Map->MapPoint[y+m][x+n] == c)
						{
							if(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>237)
							{
								middle_dis = abs(412-(y+m))*0.2;
								if(middle_dis>40)
								{
									if(dynamicmap_v_x->MapPoint[y+m][x+n]>0)
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											if(bfasterobacv_lchg&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini_lchg-2/3.6)
													drivespdtmp=fasterobspdini_lchg-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										spdacctemp=0;
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(!(bfasterobacv_lchg&&middle_dis>15)))
											{
												app->continue_spdredtmp = true;
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
									}
									else
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n];
											if(drivespdtmp>app->GPS_Speed)
												drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											/*
											if(middle_dis>8 && bobfisrstapper_lchg)
											{
												if(drivespdtmp<app->GPS_Speed-1/3.6)
													drivespdtmp=app->GPS_Speed-1/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											*/
											if(bfasterobacv_lchg&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini_lchg-2/3.6)
													drivespdtmp=fasterobspdini_lchg-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n])*(dynamicmap_v_x->MapPoint[y+m][x+n])/2/(middle_dis-36);
										if(spdacctemp<-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-25))
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-25);
										if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12)))
											spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12));
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv_lchg&&middle_dis>8))
											{
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if((ob_dis_prepre_lchg<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper_lchg)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else if(dynamicmap_v_x->MapPoint[y+m][x+n]<-3/3.6)
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
											else if(bfasterobacv_lchg&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
										}
									}
								}
								else if(middle_dis>15)
								{
									if(dynamicmap_v_x->MapPoint[y+m][x+n]>10/3.6)
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											if(bfasterobacv_lchg&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini_lchg-2/3.6)
													drivespdtmp=fasterobspdini_lchg-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										spdacctemp=0;
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(!(bfasterobacv_lchg&&middle_dis>15)))
											{
												app->continue_spdredtmp = true;
												if(spdacctemp<app->spdredacctmp)
													app->spdredacctmp = spdacctemp;
											}
										}
									}
									else
									{
										if(middle_dis<updett)
										{
											drivespdtmp = app->GPS_Speed+dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6;
											if(drivespdtmp>app->GPS_Speed)
												drivespdtmp = app->GPS_Speed;
											if(drivespdtmp<10/3.6)
												drivespdtmp=10/3.6;
											/*
											if(middle_dis>8 && bobfisrstapper_lchg)
											{
												if(drivespdtmp<app->GPS_Speed-1/3.6)
													drivespdtmp=app->GPS_Speed-1/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											*/
											if(bfasterobacv_lchg&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini_lchg-2/3.6)
													drivespdtmp=fasterobspdini_lchg-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										if(middle_dis>25)
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-20);
										else if(middle_dis>17)
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/((middle_dis-15)/2);
										else
											spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2;
										//spdacctemp=-(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)*(dynamicmap_v_x->MapPoint[y+m][x+n]-10/3.6)/2/(middle_dis-20);
										if(spdacctemp < -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12)))
											spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-12));
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv_lchg&&middle_dis>8))
											{
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if((ob_dis_prepre_lchg<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper_lchg)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else if(dynamicmap_v_x->MapPoint[y+m][x+n]<7/3.6)
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
											else if(bfasterobacv_lchg&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
										}
									}
								}
								else
									{
										if(middle_dis<updett)
										{
											drivespdtmp = (middle_dis-8)/3.6;
											if(drivespdtmp<0)
												drivespdtmp=0;
											else if(drivespdtmp>road_speed)
												drivespdtmp=road_speed;
											if(middle_dis_front>stopmiddledis&&drivespdtmp<5/3.6)
												drivespdtmp=5/3.6;

											if(drivespdtmp>app->GPS_Speed)
											{
												if(app->GPS_Speed>10/3.6)
													drivespdtmp = app->GPS_Speed;
												else if(middle_dis>18)
													drivespdtmp = 10/3.6;
											}
											/*
											if(middle_dis>8 && bobfisrstapper_lchg)
											{
												if(drivespdtmp<app->GPS_Speed-1/3.6)
													drivespdtmp=app->GPS_Speed-1/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											*/
											if(bfasterobacv_lchg&&middle_dis>8)
											{
												if(drivespdtmp<fasterobspdini_lchg-2/3.6)
													drivespdtmp=fasterobspdini_lchg-2/3.6;
												if(drivespdtmp<0)
													drivespdtmp=0;
											}
											if(app->drive_obstacle>drivespdtmp)
												app->drive_obstacle=drivespdtmp;
										}
										if((middle_dis<updett)||((middle_dis<updett+11)&&((app->continue_spdred)||(app->continue_brake))))
										{
											if(!(bfasterobacv_lchg&&middle_dis>8))
											{
												if(((app->GPS_Speed>10/3.6 + 2.5/3.6)&&(middle_dis>=9))||(app->GPS_Speed>5/3.6 + 2.5/3.6 && middle_dis<updett))
												{
													app->continue_spdredtmp = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-8));
													if((ob_dis_prepre_lchg<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper_lchg)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
												else if((app->GPS_Speed>10/3.6 + 2.5/3.6)||(middle_dis<9&&((app->GPS_Speed>5/3.6 + 2.5/3.6)||(middle_dis_front<stopmiddledis))))
												{
													app->continue_spdredtmp = true;
													spdacctemp = -((app->GPS_Speed)*(app->GPS_Speed))/2;
													if((ob_dis_prepre_lchg<1000)&&(middle_dis>=9))
													{
														double spdaccdist = -(obfrontspd*obfrontspd)/(2*(middle_dis-8));
														if(spdaccdist>-2)
															spdaccdist=-2;
														if(spdacctemp<spdaccdist)
															spdacctemp=spdaccdist;
													}
													if(middle_dis>8 && bobfisrstapper_lchg)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}
													else
														app->continue_braketmp = true;
													if(bholdbrake)
													{
														if(spdacctemp<app->spdredacc)
														{
															if(app->spdredacc<-3)
																spdacctemp=app->spdredacc;
															else if(spdacctemp<-3)
																spdacctemp=-3;
														}
													}
													if(bfasterobacv_lchg||bobfisrstapper_lchg)
													{
														if(spdacctemp<-3)
															spdacctemp=-3;
													}

													if(spdacctemp<app->spdredacc && spdacctemp<-2)
													{
														if(app->GPS_Speed<5/3.6 + 2.5/3.6 && middle_dis>7)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-6));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
														else if(app->GPS_Speed<10/3.6 + 2.5/3.6 && middle_dis>8)
														{
															if(spdacctemp<-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7)))
																spdacctemp=-((app->GPS_Speed)*(app->GPS_Speed))/(2*(middle_dis-7));
															if(spdacctemp>app->spdredacc)
																spdacctemp=app->spdredacc;
															if(spdacctemp>-2)
																spdacctemp=-2;
														}
													}

													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
											else if(bfasterobacv_lchg&&middle_dis<15)
											{
												spdacctemp=0;
												if(app->GPS_Speed>10/3.6 + 2.5/3.6)
												{
													app->continue_spdredtmp = true;
													if(spdacctemp<app->spdredacctmp)
														app->spdredacctmp = spdacctemp;
												}
											}
										}
									}
							}
						}
						break;
					}
				}
				if(obb_break_flag)
					break;
			}
			if(obb_break_flag)
				break;
		}
		if(obb_break_flag==0)
		{
			ob_loss_count_lchg++;
			if(ob_loss_count_lchg>=5||ob_dis_now_lchg==1000)
			{
				ob_dis_now_lchg=1000;
				ob_dis_pre_lchg=1000;
				ob_dis_prepre_lchg=1000;
				//ob_dis_now_lchg_timer=app->systemtimer10ms;
				//ob_dis_pre_lchg_timer=app->systemtimer10ms;
				//ob_dis_prepre_lchg_timer=app->systemtimer10ms;

				SYSTEMTIME tt;
				GetLocalTime(&tt);
				ob_dis_now_lchg_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

				ob_dis_pre_lchg_timer=ob_dis_now_lchg_timer;
				ob_dis_prepre_lchg_timer=ob_dis_now_lchg_timer;

				bfasterobacv_lchg=0;
				ob_loss_count_lchg=0;
			}
		}
		else
		{
			ob_loss_count_lchg=0;
			ob_dis_prepre_lchg=ob_dis_pre_lchg;
			ob_dis_prepre_lchg_timer=ob_dis_pre_lchg_timer;
			ob_dis_pre_lchg=ob_dis_now_lchg;
			ob_dis_pre_lchg_timer=ob_dis_now_lchg_timer;
		}
	}
	else
	{
		fasterobspdini_lchg=road_speed;
		bfasterobacv_lchg=0;
		bobfisrstapper_lchg=0;
		ob_dis_now_lchg=1000;
		ob_dis_pre_lchg=1000;
		ob_dis_prepre_lchg=1000;
		//ob_dis_now_lchg_timer=app->systemtimer10ms;
		//ob_dis_pre_lchg_timer=app->systemtimer10ms;
		//ob_dis_prepre_lchg_timer=app->systemtimer10ms;

		SYSTEMTIME tt;
		GetLocalTime(&tt);
		ob_dis_now_lchg_timer=(((unsigned int)(tt.wHour))*3600+((unsigned int)(tt.wMinute))*60+(unsigned int)(tt.wSecond))*1000+((unsigned int)(tt.wMilliseconds));

		ob_dis_pre_lchg_timer=ob_dis_now_lchg_timer;
		ob_dis_prepre_lchg_timer=ob_dis_now_lchg_timer;

		ob_loss_count_lchg=0;
	}

	if(approachgoal_flag||approachyulukou>0)
	{
		double ob_dis = 0;
		int approach_dir=0;
		if(approachgoal_flag||approachyulukou==2)
		{
			control.setLight(RIGHT_LAMP_ON);
			approach_dir=2;
		}
		else if(approachyulukou==1)
		{
			control.setLight(LEFT_LAMP_ON);
			approach_dir=1;
		}
		else
			control.setLight(LAMP_OFF);
		int ob_flag = lanedriverstate.SearchObstacle2(rndfbezier,vel_Map,8,18,28,40,-25,ob_dis,approach_dir);
		if(ob_flag==2)
		{
			app->drive_obstacle=0;
			app->continue_spdredtmp = true;
			app->continue_braketmp = true;
			if(-3<app->spdredacctmp)
				app->spdredacctmp = -3;

			bspeciallukoustop=true;
		}
	}

	double left_dis = 10000;
	double right_dis = 10000;
	up=(int)(app->GPS_Speed*3.6);
	if(up<15)
		up=15;
	else if(up>80)
		up=80;
	//MoveLeft(rndfbezier,exe_leftpoint,3.7);
	for(int i=0;i<200;i++)
	{
		exe_leftpoint[i].x=app->rndfbezier_left[i].x;
		exe_leftpoint[i].y=app->rndfbezier_left[i].y;
	}
	ob_left = lanedriverstate.SearchObstacle111(exe_leftpoint,vel_Map,8,18,28,up,-25,left_dis);
	//MoveLeft(rndfbezier,exe_rightpoint,-3.7);
	for(int i=0;i<200;i++)
	{
		exe_rightpoint[i].x=app->rndfbezier_right[i].x;
		exe_rightpoint[i].y=app->rndfbezier_right[i].y;
	}
	ob_right = lanedriverstate.SearchObstacle111(exe_rightpoint,vel_Map,8,18,28,up,-25,right_dis);
	CvPoint2D64f rndfbeziertemp[200];
	for(int i=0;i<200;i++)
		rndfbeziertemp[i]=rndfbezier[i];
	bool bobserachflag = 1;
	if(!ob_left)
	{
		for(int i=7;i>=5;i--)
		{
			MoveLeft(rndfbezier,exe_leftpoint,0.53*i);
			if(!lanedriverstate.SearchObstacle222(exe_leftpoint,vel_Map,8,18,28,up,-25,left_dis))
			{
				bobserachflag = 0;
				break;
			}
		}
		ob_left = bobserachflag;
	}
	/*
	if(!ob_left)
	{
		MoveWay(realtime_Gps,realtime_Dir,rndfbeziertemp);
		MoveLeft(rndfbeziertemp,exe_leftpoint,3.7);
		ob_left = lanedriverstate.SearchObstacle111(exe_leftpoint,vel_Map,8,18,28,up,-25,left_dis);
		bobserachflag = 1;
		if(!ob_left)
		{
			for(int i=7;i>=5;i--)
			{
				MoveLeft(rndfbeziertemp,exe_leftpoint,0.53*i);
				if(!lanedriverstate.SearchObstacle222(exe_leftpoint,vel_Map,8,18,28,up,-25,left_dis))
				{
					bobserachflag = 0;
					break;
				}
			}
			ob_left = bobserachflag;
		}
	}
	*/

	bobserachflag = 1;
	if(!ob_right)
	{
		for(int i=7;i>=5;i--)
		{
			MoveLeft(rndfbezier,exe_rightpoint,-0.53*i);
			if(!lanedriverstate.SearchObstacle222(exe_rightpoint,vel_Map,8,18,28,up,-25,right_dis))
			{
				bobserachflag = 0;
				break;
			}
		}
		ob_right = bobserachflag;
	}
	/*
	if(!ob_right)
	{
		MoveWay(realtime_Gps,realtime_Dir,rndfbeziertemp);
		MoveLeft(rndfbeziertemp,exe_rightpoint,-3.7);
		ob_right = lanedriverstate.SearchObstacle111(exe_rightpoint,vel_Map,8,18,28,up,-25,right_dis);
		bobserachflag = 1;
		if(!ob_right)
		{
			for(int i=7;i>=5;i--)
			{
				MoveLeft(rndfbeziertemp,exe_rightpoint,-0.53*i);
				if(!lanedriverstate.SearchObstacle222(exe_rightpoint,vel_Map,8,18,28,up,-25,right_dis))
				{
					bobserachflag = 0;
					break;
				}
			}
			ob_right = bobserachflag;
		}
	}
	*/

	ob_left=false;
	ob_right=false;

	if(app->PercepMap->MapPoint[412][0]==255 || app->PercepMap->MapPoint[411][0]==255)
		ob_left = true;
	if(app->PercepMap->MapPoint[412][511]==255 || app->PercepMap->MapPoint[411][511]==255)
		ob_right = true;

	//if(able_ChangeLane||((!((!able_ChangeLane) && lanechangecount<lanechangenumtmp_ && lanechangecount<app->lanechangenum))&&(lchghxdist<1000)&&(((ob_res==-1)&&(lchghxdist<0.5))||((ob_res==1)&&(lchghxdist>-0.5)))))
	if(able_ChangeLane)
	{
		if((lanedriverstate.y_left_rear[0]<8)||(lanedriverstate.y_left_rear[0]<=50 && (lanedriverstate.spdrelleftrear>5/3.6))||(lanedriverstate.y_left_rear[0]<=20 && (lanedriverstate.spdrelleftrear>0))||(lanedriverstate.y_left_rear[0]<=15 && app->GPS_Speed>10/3.6 && (lanedriverstate.spdrelleftrear>-5/3.6)))
			ob_left=true;
		/*
		if(lanedriverstate.y_left_rear[0]<=50 && (lanedriverstate.spdrelleftrear<-1000))
			ob_left=true;
			*/
		if((lanedriverstate.y_right_rear[0]<8)||(lanedriverstate.y_right_rear[0]<=50 && (lanedriverstate.spdrelrightrear>5/3.6))||(lanedriverstate.y_right_rear[0]<=20 && (lanedriverstate.spdrelrightrear>0))||(lanedriverstate.y_right_rear[0]<=15 && app->GPS_Speed>10/3.6 && (lanedriverstate.spdrelrightrear>-5/3.6)))
			ob_right=true;
		/*
		if(lanedriverstate.y_right_rear[0]<=50 && (lanedriverstate.spdrelrightrear<-1000))
			ob_right=true;
			*/
	}

	if((!able_ChangeLane) && lanechangecount<(lanechangenumtmp-1) && lanechangecount<(app->lanechangenum-1))
	{
		if(ob_res==-1)
		{
			if(app->bGpsGanrao)
				ob_right = false;
			else
				ob_right = true;
		}
		if(ob_res==1)
		{
			if(app->bGpsGanrao)
				ob_left = false;
			else
				ob_left = true;
		}
	}

	if(app->bGpsGanrao)
	{
		app->obb_left = false;
		app->obb_right = false;
	}
	else
	{
		app->obb_left = ob_left;
		app->obb_right = ob_right;
	}

	drivespdtmp=road_speed;

	if(app->drive_obstacle>lanechangespd)
		app->drive_obstacle=lanechangespd;

	return 1;
}

int CVehicleExe::ApproIntersectionDrive(int &ob_num,int &count_move,bool is_avoidobstacle)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int midd_num1=0;
	CvPoint2D64f MidPoint[512];

	pianyi.x = 0;
	pianyi.y = 0;
	//**************//
	//{	
		//ReturnPartPoints(CVehicle::global_path,realtime_Gps,realtime_Dir,80,rndfbezier,pianyi);
	//}
	//**************//
	int lane_result = lanedriverstate.LaneDrive(midd_num1,MidPoint);
	int laneob_result = SearchObstacleMap(MidPoint,midd_num1,vel_Map,30,0);
	//////如果有车道线，车道线左右5m内没有障碍物，并且目标点与车道线中点之间的垂直距离在2~5m内，修正rndfpath
	if (lane_result && !laneob_result && !is_avoidobstacle)
	{
		int numtemp = int(midd_num1/2);
		if (abs(MidPoint[numtemp].x - 256)>35)
		{
			return 1;
		}
		CvPoint2D64f lanepgps = m_GpsData.MaptoGPS(realtime_Gps,realtime_Dir,MidPoint[int(midd_num1/2)]/**/);
		MoveWay(lanepgps,realtime_Dir,rndfbezier,pianyi);
		//for (int i=0;i<200;i++)
		//{
		//	prebezier[i].x = rndfbezier[i].x;
		//	prebezier[i].y = rndfbezier[i].y;
		//}
	}
	//else
	//{
	//	for (int i=0;i<200;i++)
	//	{
	//		rndfbezier[i].x = prebezier[i].x;
	//		rndfbezier[i].y = prebezier[i].y;
	//	}
	//}
	return 1;
}

int CVehicleExe::JudgeIntersectionFx(CvPoint2D64f &stop_point)
{
	if (seq_num>=mission_num)
	{
		stop_point.x = LeadPoint[mission_num - 1].lat;
		stop_point.y = LeadPoint[mission_num - 1].lng;
		fx = LeadPoint[mission_num - 1].param2;
		if(LeadPoint[mission_num - 1].param2 == 5)
			fx = 3;
		if(LeadPoint[mission_num - 1].param2 == 3 ||LeadPoint[mission_num - 1].param2 == 2)
			fx = 2;
		return 0;
	}		
	
	if (LeadPoint[seq_num - 1].param1 == 3)
	{
		stop_point.x = LeadPoint[seq_num - 1].lat;
		stop_point.y = LeadPoint[seq_num - 1].lng;
		fx = LeadPoint[seq_num-1].param2;
		if(LeadPoint[seq_num-1].param2 == 5)
			fx = 3;
		if(LeadPoint[seq_num-1].param2 == 3 || LeadPoint[seq_num-1].param2 == 2)
			fx = 2;
	}
	else if (LeadPoint[seq_num].param1 == 3)
	{
		stop_point.x = LeadPoint[seq_num].lat;
		stop_point.y = LeadPoint[seq_num].lng;
		fx = LeadPoint[seq_num].param2;
		if(LeadPoint[seq_num].param2 == 5)
			fx = 3;
		if(LeadPoint[seq_num].param2 == 3 || LeadPoint[seq_num].param2 == 2)
			fx = 2;
	}
	return 0;
}

int CVehicleExe::RangeLUkouFx()
{
	fx = LeadPoint[seq_num-2].param2;
	if(LeadPoint[seq_num-2].param2 == 5)
		fx = 3;
	if(LeadPoint[seq_num-2].param2 == 3 ||LeadPoint[seq_num-2].param2 == 2)
		fx = 2;
	return 0;
}
bool CVehicleExe::SearchUTurnObstacleMoveWay1_rightsidespecial(CvPoint2D64f Rndf_MapPoint[],I_Map *map)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	int x = 0;
	int y = 0;
	for(int i = 0;i<150;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 412||Rndf_MapPoint[i].y < 262)
					continue;
		x = Rndf_MapPoint[i].x ;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=4;n<30;n++)//10//12
			{
				if(y+m>412||y+m<262||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					return true;
				}
			}
	}
	return false;
}