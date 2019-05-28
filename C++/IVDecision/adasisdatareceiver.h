#pragma once
#include "canlib.h"
#include "IVDecision.h"
#include <vector>
#include "RNPointQueue.h"

using namespace std;

#define CAN_READ_WAIT 5000
#define CAN_DATA_SIZE 8

	//路网点
struct RNPoint
{
	double lng;		//经度
	double lat;		//纬度
	double height;	//高度
	int type;		//类型
};
struct Segment
{
	int id;
	int pathindex;
	int lanenum;
	int offset;
};

struct Position
{
	int pathindex;
	int offset;
};

struct Stub
{
	int id;
	int pathindex;
	int offset;
};

struct Longitute
{
	int pathindex;
	int offset;
	double val;
};

struct Latitude
{
	int pathindex;
	int offset;
	double val;
};


class CADASISDataReceiver
{
public:
	CADASISDataReceiver(void);
	~CADASISDataReceiver(void);

public:
	CanHandle m_hCan;
	HANDLE m_hThread;
	HANDLE m_hEvent;
	
	//vector<Segment> m_segments;
	//vector<Stub> m_stubs;
	//vector<Longitute> m_longitudes;
	//vector<Latitude> m_latitudes;

	Position m_curPosition;
	
	CRNPointQueue<Segment> m_segQueue;
	CRNPointQueue<Stub> m_stubQueue;
	CRNPointQueue<RNPoint> m_rnPts;

	RNPoint m_pt;

public:
	void OpenADASISCAN();
	CvMemStorage* storage;
	CvSeq *ADAS_points;

private:
	void InitCAN(int channel);
	static DWORD WINAPI OnReceive(LPVOID lparam);
};
