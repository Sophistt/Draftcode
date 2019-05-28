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

struct state_struct
{
	int id;
	double lng;
	double lat;
	double height;
	int param1;
	int param2;
};

class CLoadLeadpt 
{
public:
	CLoadLeadpt(void);
public:
	~CLoadLeadpt(void);

public:
	void Openfile(); 

private:
	string m_strMapData;

	void ReadData(string strMapData);
public:
	MDFINFO m_mapInfo;
	
	queue<int>Lead_Points;
	queue<SpeedLimit>Speed_Limits;

	void  LoadLead();
	CListBox m_result;
	state_struct *LeadPt;
};
