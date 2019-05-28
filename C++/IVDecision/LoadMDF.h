#include "IVDecision.h"
#include <string>
#include <sstream>
#include <math.h>

#pragma once

using namespace std;


typedef struct 
{
	int id;
	double min;
	double max;
	
}SpeedLimit;

typedef struct
{
	int num_checkpoints;
	int num_speed_limits;
	float format_version;
	string MDF_name;
	string RNDF;
	string creation_date;
}MDFINFO;

class CLoadMDF 
{
public:
	CLoadMDF(void);
public:
	~CLoadMDF(void);
public:
	void Openfile(); 

private:
	string m_strMapData;

	void ReadData(string strMapData);
	void FindSpeedLimit(string str);

	
public:
	MDFINFO m_mapInfo;
	
	queue<int>Check_Points;
	queue<SpeedLimit>Speed_Limits;


	
};
