
#pragma once
#include "IVDecision.h"
#include <string>
#include <sstream>
#include <math.h>



using namespace std;


typedef struct 
{
	int m;
	int n;
	int p;
}CONNECT;

typedef struct MAPPOINT
{
	double x;
	double y;
	int point_Id;
	int con_num;
	CONNECT connect[3];
	float g,h,f;
	float d0,d1,d2,d3;
	struct MAPPOINT *par;
	struct MAPPOINT *next;
	struct MAPPOINT *nearnode[4];
}*PMAPPOINT;

typedef struct MAPPOINTexe
{
	int ID111;
	int ID222;
	int ID333;
	double x;
	double y;
	struct MAPPOINTexe *par;
	struct MAPPOINTexe *next;
}*PMAPPOINTexe;

typedef struct 
{
	double x;
	double y;
	int inlane;
	int point_Id;
}CHARPOINT,*PCHARPOINT;

typedef struct 
{
	int exit_Id;
	int m;
	int n;
	int p;
}EXITT,*PEXITT;

typedef struct 
{
	int check_Id;
	int check_sn;
}CHECKPOINT,*PCHECKPOINT;

typedef struct  
{
	int lane_Id;
	int waypoints_num;
	int lane_width;
	int exit_num;
	//int exit_id;
	int stop_id;
	PEXITT pExit;
	PMAPPOINT pPoint;
	int charpoints_num;
	PCHARPOINT pCharpoint;
	int check_num;
	PCHECKPOINT pCheckpoint;
	string left_boundary;
}LANE,*PLANE;

typedef struct  
{
	int perimeter_Id;
	int perimeter_num;
	int exit_id;
	int exit_num;
	PEXITT pExit;
	PMAPPOINT pPoint;
}PERIMETER;

typedef struct  
{
	int spot_Id;
	int spot_width;
	MAPPOINT Point[2];
}SPOT,*PSPOT;

typedef struct  
{
	int segment_Id;
	int lane_num;
	int direction_num;
	string segment_name;
	PLANE pLane;
}SEGMENT,*PSEGMENT;

typedef struct  
{
	int zone_Id;
	int spots_num;
	int charpoints_num;	
	string zone_name;

	PERIMETER Perimeter;
	PSPOT pSpot;
	PCHARPOINT pCharpoint;
}ZONE,*PZONE;

typedef struct
{
	int segment_num;
	int zone_num;
	float format_version;
	string RNDF_name;
	string creation_date;
	PSEGMENT pSegment;
	PZONE pZone;
}MAPINFO;

template<class T>
void XnVarToString(string &result, const T &t)
{
	ostringstream oss;
	oss << t;
	result = oss.str();
}

template<class T>
string XnVarToString(const T &t)
{
	ostringstream oss;
	oss << t;  
	return oss.str();
}
 

class CLoadRNDF
{
public:
	CLoadRNDF(void);
public:
	~CLoadRNDF(void);
public:
	void Openfile(); 

private:
	string m_strMapData;

	void ReadData(string strMapData);
	void FindSegment(int iSeg,string segment);
	void FindLane(string lane,int iSeg,int iLan);
	void FindZone(int iZone,string zone);
	void FindSpot(string zone, int iZone,int iSpot);
	int FindEXITNUM(string exits);
	void FindEXIT(string exits,int iSeg,int iLan,int i);
	void FindZoneEXIT(string exits,int iZone,int i);
	void FindConnect();
	int FindCheckNUM(string check);
	void FindCheck(string check,int iSeg,int iLan,int i);
	double D(CvPoint2D64f p1, CvPoint2D64f p2);
	
	
public:
	MAPINFO m_mapInfo;
	CvPoint2D64f GetPoint(int Seg_id,int Lan_id, int P_id);
	CvPoint2D64f GetPerimeter(int Zone_id,int P_id);
	CvPoint2D64f GetSpot(int Zone_id,int Sp_id, int P_id);
	CHARPOINT GetCharPoint(int Seg_id,int Lan_id, int P_id);
	CHARPOINT GetCharPoint(int Zone_id, int P_id);
	CONNECT FindWay(CvPoint2D64f pos,double dir);
	double GetLaneDir(int seg_id,int lan_id);
	int GetCheckPoint(int check_num,CvPoint2D64f &point);
	int GetCheckSeg(int check_num,int &n);
	int GetLaneNum(int seg_id,int lan_id);
	int GetLaneShape(int seg_id,int lan_id);
	double distance(double wd1, double jd1, double wd2, double jd2);
	double GetAngle(CvPoint2D64f apoint, CvPoint2D64f bpoint);
	double rad( double lat1 );
	int GetCheckPointID(int check_num,int &id1,int &id2,int&id3);
	void Openfile1(CString FileName) ;
};
