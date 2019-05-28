#include "StdAfx.h"
#include "LoadRNDF.h"



CLoadRNDF::CLoadRNDF(void)
{
}

CLoadRNDF::~CLoadRNDF(void)
{
}

void CLoadRNDF::Openfile() 
{
	// TODO: Add your control notification handler code here
	CFileDialog dlg1(TRUE,"txt","*.txt"); 
	if(dlg1.DoModal() == IDOK)
	{
		CString path=dlg1.GetPathName();
		CFile mFile; 
		if(mFile.Open(path,CFile::modeRead)==0) 
			return; 
/*		TRACE("%d",mFile.GetLength());
		CArchive ar(&mFile,CArchive::load); 
		ar>>m_strMapData; 
		TRACE("%d",m_strMapData.GetLength());
		ar.Close(); */
		string str("");
		DWORD fileLen = mFile.GetLength();
		char *pBuf=new char[fileLen+1];
		pBuf[fileLen]=0;
		mFile.Read(pBuf,mFile.GetLength());
		m_strMapData = pBuf;
		delete []pBuf;
		mFile.Close();
	}
	else
		return;
	ReadData(m_strMapData);
//	MaxMAPPOINT();
//	TRACE("%f,%f",m_MaxPoint.x,m_MaxPoint.y);
//	DrawX();
}
void CLoadRNDF::Openfile1(CString FileName) 
{
	// TODO: Add your control notification handler code here
	//CFileDialog dlg1(TRUE,"txt","*.txt"); 
	//if(dlg1.DoModal() == IDOK)
	{
		CString path=FileName;
		CFile mFile; 
		if(mFile.Open(path,CFile::modeRead)==0) 
			return; 
/*		TRACE("%d",mFile.GetLength());
		CArchive ar(&mFile,CArchive::load); 
		ar>>m_strMapData; 
		TRACE("%d",m_strMapData.GetLength());
		ar.Close(); */
		string str("");
		DWORD fileLen = mFile.GetLength();
		char *pBuf=new char[fileLen+1];
		pBuf[fileLen]=0;
		mFile.Read(pBuf,mFile.GetLength());
		m_strMapData = pBuf;
		delete []pBuf;
		mFile.Close();
	}
	/*else
		return;*/
	ReadData(m_strMapData);
//	MaxMAPPOINT();
//	TRACE("%f,%f",m_MaxPoint.x,m_MaxPoint.y);
//	DrawX();
}
void CLoadRNDF::ReadData(string strMapData)
{
	int istartpos = strMapData.find("RNDF_name");
	int iendpos = strMapData.find("\r\n",istartpos);
	string rndfname = strMapData.substr(istartpos+9,iendpos-istartpos-9);
	m_mapInfo.RNDF_name = rndfname;
	//RNDF_name 

	istartpos = strMapData.find("format_version");
	iendpos = strMapData.find("\r\n",istartpos);
	string version = strMapData.substr(istartpos+15,iendpos-istartpos-15);
	m_mapInfo.format_version = atof(version.c_str());
	//format_version

	istartpos = strMapData.find("creation_date");
	iendpos = strMapData.find("\r\n",istartpos);
	string creationdate = strMapData.substr(istartpos+13,iendpos-istartpos-13);
	m_mapInfo.creation_date = creationdate;
	//creation_date

	istartpos = strMapData.find("num_segments");
	iendpos = strMapData.find("\r\n",istartpos);
	string segmentnum = strMapData.substr(istartpos+12,iendpos-istartpos-12);
	m_mapInfo.segment_num = atof(segmentnum.c_str());
	m_mapInfo.pSegment = new SEGMENT[m_mapInfo.segment_num];

	istartpos = strMapData.find("segment ");
	iendpos = strMapData.find("end_segment",istartpos);
	string segment = strMapData.substr(istartpos,iendpos+11-istartpos);	

	for(int i=1;i<=m_mapInfo.segment_num;i++)
	{
		FindSegment(i,segment);
		istartpos = iendpos + 11;
		iendpos = strMapData.find("end_segment",istartpos);
		segment = strMapData.substr(istartpos,iendpos+11-istartpos);	
	}
	//segment

	istartpos = strMapData.find("num_zones");
	iendpos = strMapData.find("format");
	if(istartpos < 0)
	{
		m_mapInfo.zone_num == 0;
		m_mapInfo.pZone = NULL;
	}
	else
	{
		string zonenum = strMapData.substr(istartpos+9,iendpos-istartpos-9);
		m_mapInfo.zone_num = atoi(zonenum.c_str());
		if(m_mapInfo.zone_num == 0)
			m_mapInfo.pZone = NULL;

		else
		{	
			m_mapInfo.pZone = new ZONE[m_mapInfo.zone_num];
			istartpos = strMapData.find("zone ");
			iendpos = strMapData.find("end_zone",istartpos);
			string zone = strMapData.substr(istartpos,iendpos+11-istartpos);	
			for(int i=1;i<=m_mapInfo.zone_num;i++)
			{
				FindZone(i,zone);
				istartpos = iendpos + 11;
				iendpos = strMapData.find("end_zone",istartpos);
				zone = strMapData.substr(istartpos,iendpos+8-istartpos);	
			}
		}
	}
	//zone


	//FindConnect();

}
void CLoadRNDF::FindSegment(int iSeg,string segment)
{

	//stringstream   sstream;   
	//sstream<<iSeg; 
	//string strfind = "segment " + sstream.str();
	//int istartpos = m_strMapData.find(strfind);
	//int iendpos = m_strMapData.find("end_segment",istartpos);
	//string segment = m_strMapData.substr(istartpos,iendpos-istartpos);

	m_mapInfo.pSegment[iSeg-1].direction_num = 2;//直接定义为双车道

	int	istartpos = segment.find("segment");
	int iendpos = segment.find("\r\n",istartpos);

	string segmentid = segment.substr(istartpos+7,iendpos-istartpos-7);
	m_mapInfo.pSegment[iSeg-1].segment_Id = atoi(segmentid.c_str());

	istartpos = segment.find("segment_name");
	iendpos = segment.find("\r\n",istartpos);
	string segmentname = segment.substr(istartpos+12,iendpos-istartpos-12);
	m_mapInfo.pSegment[iSeg-1].segment_name = segmentname;

	istartpos = segment.find("num_lanes");
	iendpos = segment.find("\r\n",istartpos);

	string lanenum = segment.substr(istartpos+9,iendpos-istartpos-9);
	m_mapInfo.pSegment[iSeg-1].lane_num = atoi(lanenum.c_str());
	m_mapInfo.pSegment[iSeg-1].pLane = new LANE[m_mapInfo.pSegment[iSeg-1].lane_num];
	
//	FindLane(segment,iSeg,1);
	//for(int i=1;i<=m_mapInfo.pSegment[iSeg-1].lane_num;i++)
	//	FindLane(segment,iSeg,i);

	istartpos = segment.find("lane ");
	iendpos = segment.find("end_lane",istartpos);
	string lane = segment.substr(istartpos,iendpos+8-istartpos);	
	int segid = m_mapInfo.pSegment[iSeg-1].segment_Id;
	for(int i=1;i<=m_mapInfo.pSegment[iSeg-1].lane_num;i++)
	{
		FindLane(lane,segid,i);
		istartpos = iendpos + 8;
		iendpos = segment.find("end_lane",istartpos);
		lane = segment.substr(istartpos,iendpos+8-istartpos);	
	}

}
void CLoadRNDF::FindLane(string lane, int iSeg,int iLan)
{
	//stringstream sstream;
	//sstream<<iSeg;
	//sstream<<'.';
	//sstream<<iLan; 
	//string strfind = "lane " + sstream.str();

	//int istartpos = m_strMapData.find(strfind);
	//int iendpos = m_strMapData.find("end_lane",istartpos);
	//string lane = m_strMapData.substr(istartpos,iendpos-istartpos);


	int istartpos = lane.find("lane " + XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+".");
	int iendpos = lane.find("\r\n",istartpos);
	string find = "lane " + XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+".";

	string laneid = lane.substr(istartpos+find.length(),iendpos-istartpos-find.length());
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id = atoi(laneid.c_str());

	istartpos = lane.find("num_waypoints");
	iendpos = lane.find("\r\n",istartpos);

	string waypoints = lane.substr(istartpos+13,iendpos-istartpos-13);
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].waypoints_num = atoi(waypoints.c_str()); 
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pPoint = new MAPPOINT[m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].waypoints_num];

	istartpos = lane.find("lane_width");
	iendpos = lane.find("\r\n",istartpos);
	string lane_width = lane.substr(istartpos+10,iendpos-istartpos-10);
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_width = atoi(lane_width.c_str());

	istartpos = lane.find("left_boundary");
	iendpos = lane.find("\r\n",istartpos);
	string left_boundary = lane.substr(istartpos+13,iendpos-istartpos-13);
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].left_boundary = left_boundary;

	istartpos = lane.find("stop");
	iendpos = lane.find("\r\n",istartpos);
	
	if(istartpos<0)
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].stop_id = 0;
	else
	{
		string lane_stop = lane.substr(istartpos,iendpos-istartpos);
		string findstop = XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id)+".";
		istartpos = lane.find(findstop);
		string stopid = lane.substr(istartpos+findstop.length(),2);
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].stop_id = atoi(stopid.c_str());
	}

	//istartpos = lane.find("lane");
	//iendpos = lane.find("num_waypoints");
	//float laneId = atof(lane.substr(istartpos+5,iendpos-istartpos-5).c_str());
	//m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id=int(laneId*10)%10;

	istartpos = lane.find("num_charpoints");
	iendpos = lane.find("\r\n",istartpos);

	if(istartpos<=0)
	{
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].charpoints_num = 0;
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCharpoint = NULL;
	}
	else
	{
		string charpoints = lane.substr(istartpos+14,iendpos-istartpos-14);
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].charpoints_num = atoi(charpoints.c_str()); 
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCharpoint = new CHARPOINT[m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].charpoints_num];
		istartpos = lane.find("\r\ncharpoint");
		iendpos = lane.find("end_charpoint");
		charpoints = lane.substr(istartpos+11,iendpos-istartpos-11);
		for(int i=1;i<=m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].charpoints_num;i++)
		{
			string strfind = XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id)+"."+XnVarToString(i)+" ";
			istartpos = charpoints.find(strfind.c_str())+strfind.length();
			iendpos = charpoints.find(" ",istartpos);
			m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCharpoint[i-1].x = atof(charpoints.substr(istartpos,iendpos-istartpos).c_str());
			istartpos=iendpos;
			iendpos=charpoints.find(" ",istartpos+1);
			m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCharpoint[i-1].y = atof(charpoints.substr(istartpos,iendpos).c_str());
			istartpos=iendpos;
			iendpos=charpoints.find("\r\n",istartpos+1);
			m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCharpoint[i-1].inlane = atof(charpoints.substr(istartpos,iendpos).c_str());

		}
	}	


	istartpos = lane.find("\r\nwaypoint");
	iendpos = lane.find("end_waypoint");
	waypoints = lane.substr(istartpos+10,iendpos-istartpos-10);
	for(int i=1;i<=m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].waypoints_num;i++)
	{
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pPoint[i-1].con_num = 0;
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pPoint[i-1].point_Id = i;
		string strfind = "\r\n"+XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id)+"."+XnVarToString(i)+" ";
		istartpos = waypoints.find(strfind.c_str());
		iendpos = waypoints.find(" ",istartpos+strfind.length());
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pPoint[i-1].x=atof(waypoints.substr(istartpos+strfind.length(),iendpos-istartpos-strfind.length()).c_str());
		istartpos=iendpos;
		iendpos=waypoints.find("\r\n",istartpos);
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pPoint[i-1].y = atof(waypoints.substr(istartpos,iendpos).c_str());
	}

	string exits = lane; 	
	int num = FindEXITNUM(exits);
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].exit_num = num;

	if(num == 0)
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pExit = NULL;
	else
	{
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pExit = new EXITT[m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].exit_num];
		istartpos = 0;
		for(int i = 0; i<m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].exit_num; i++)
		{
			istartpos = exits.find("exit",istartpos);
			iendpos = exits.find("\r\n",istartpos);
			string exitpoint = exits.substr(istartpos+4,iendpos-istartpos-4);
			FindEXIT(exitpoint,iSeg,iLan,i);
			istartpos = iendpos+2;
		}

	}

	/***************************
	解析所有checkpoint
	***************************/
	string check = lane; 	
	int checknum = FindCheckNUM(check);
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].check_num = checknum;

	if(checknum == 0)
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCheckpoint = NULL;
	else
	{
		m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCheckpoint = new CHECKPOINT[m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].check_num];
		istartpos = 0;
		for(int i = 0; i<m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].check_num; i++)
		{
			istartpos = check.find("checkpoint",istartpos);
			iendpos = check.find("\r\n",istartpos);
			string checkpoint = check.substr(istartpos+10,iendpos-istartpos-10);
			FindCheck(checkpoint,iSeg,iLan,i);
			istartpos = iendpos+2;
		}

	}

}
int CLoadRNDF::FindEXITNUM(string exits)
{
	int num=0;
	string temp = exits;
	while(1)
	{	
		int istartpos = temp.find("exit");
		if(istartpos == -1)break;

		temp = temp.substr(istartpos+4,temp.length()-istartpos-4);

		num++;

			
	}
	return num;
}
int CLoadRNDF::FindCheckNUM(string check)
{
	int num=0;
	string temp = check;
	while(1)
	{	
		int istartpos = temp.find("checkpoint");
		if(istartpos == -1)break;

		temp = temp.substr(istartpos+10,temp.length()-istartpos-10);

		num++;

			
	}
	return num;
}

void CLoadRNDF::FindEXIT(string exits,int iSeg,int iLan,int i)
{

	string findexit =  XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id)+".";
	int	istartpos = exits.find(findexit);
	string exitid = exits.substr(istartpos+findexit.length(),2);
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pExit[i].exit_Id = atoi(exitid.c_str());

	
	findexit =  XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pExit[i].exit_Id);
	istartpos = exits.find(findexit);
	string exitpoint = exits.substr(istartpos+findexit.length(),exitpoint.length()-istartpos-findexit.length());
	int m = atoi(exitpoint.c_str());
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pExit[i].m = m;
	
	findexit = XnVarToString(m)+".";
	istartpos = exitpoint.find(findexit);	
	exitpoint = exitpoint.substr(istartpos+findexit.length(),exitpoint.length()-istartpos-findexit.length());
	int n = atoi(exitpoint.c_str());
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pExit[i].n = n;

	findexit = XnVarToString(n)+".";
	istartpos = exitpoint.find(findexit);

	exitpoint = exitpoint.substr(istartpos+findexit.length(),exitpoint.length()-istartpos-findexit.length());
	int p = atoi(exitpoint.c_str());
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pExit[i].p = p;	
	
}

void CLoadRNDF::FindCheck(string check,int iSeg,int iLan,int i)
{

	string findcheck =  XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id)+".";
	int	istartpos = check.find(findcheck);
	string checkid = check.substr(istartpos+findcheck.length(),2);
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCheckpoint[i].check_Id = atoi(checkid.c_str());

	
	findcheck =  XnVarToString(m_mapInfo.pSegment[iSeg-1].segment_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].lane_Id)+"."+XnVarToString(m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCheckpoint[i].check_Id);
	istartpos = check.find(findcheck);
	string checkpoint = check.substr(istartpos+findcheck.length(),check.length()-istartpos-findcheck.length());
	int m = atoi(checkpoint.c_str());
	m_mapInfo.pSegment[iSeg-1].pLane[iLan-1].pCheckpoint[i].check_sn = m;
	
}
void CLoadRNDF::FindZoneEXIT(string exits,int iZone,int i)
{

	string findexit =  XnVarToString(m_mapInfo.pZone[iZone-1].zone_Id)+".0.";
	int	istartpos = exits.find(findexit);
	string exitid = exits.substr(istartpos+findexit.length(),2);
	m_mapInfo.pZone[iZone-1].Perimeter.exit_id = atoi(exitid.c_str());

	
	findexit =  XnVarToString(m_mapInfo.pZone[iZone-1].zone_Id)+".0."+XnVarToString(m_mapInfo.pZone[iZone-1].Perimeter.exit_id);
	istartpos = exits.find(findexit);
	string exitpoint = exits.substr(istartpos+findexit.length(),exitpoint.length()-istartpos-findexit.length());
	int m = atoi(exitpoint.c_str());
	m_mapInfo.pZone[iZone-1].Perimeter.pExit[i].m = m;
	
	findexit = XnVarToString(m)+".";
	istartpos = exitpoint.find(findexit);	
	exitpoint = exitpoint.substr(istartpos+findexit.length(),exitpoint.length()-istartpos-findexit.length());
	int n = atoi(exitpoint.c_str());
	m_mapInfo.pZone[iZone-1].Perimeter.pExit[i].n = n;

	findexit = XnVarToString(n)+".";
	istartpos = exitpoint.find(findexit);

	exitpoint = exitpoint.substr(istartpos+findexit.length(),exitpoint.length()-istartpos-findexit.length());
	int p = atoi(exitpoint.c_str());
	m_mapInfo.pZone[iZone-1].Perimeter.pExit[i].p = p;	
	
}
void CLoadRNDF::FindZone(int iZone,string zone)
{


	int	istartpos = zone.find("zone ");
	int iendpos = zone.find("\r\n",istartpos);

	string zoneid = zone.substr(istartpos+5,iendpos-istartpos-5);
	m_mapInfo.pZone[iZone-1].zone_Id = atoi(zoneid.c_str());

	istartpos = zone.find("num_spots");
	iendpos = zone.find("\r\n",istartpos);

	string spotnum = zone.substr(istartpos+9,iendpos-istartpos-9);
	m_mapInfo.pZone[iZone-1].spots_num = atoi(spotnum.c_str());
	m_mapInfo.pZone[iZone-1].pSpot = new SPOT[m_mapInfo.pZone[iZone-1].spots_num];


 	for(int i=1;i<=m_mapInfo.pZone[iZone-1].spots_num;i++)
		FindSpot(zone,iZone,i);
		
	istartpos = zone.find("perimeter");
	iendpos = zone.find("end_perimeter");
	
	string perimeter = zone.substr(istartpos+9,iendpos-istartpos-9);

	istartpos = perimeter.find("num_perimeterpoints");
	iendpos = perimeter.find("end_perimeter");
	string perimeternum = perimeter.substr(istartpos+19,iendpos-istartpos-19);
	m_mapInfo.pZone[iZone-1].Perimeter.perimeter_num = atoi(perimeternum.c_str());
	m_mapInfo.pZone[iZone-1].Perimeter.pPoint = new MAPPOINT[m_mapInfo.pZone[iZone-1].Perimeter.perimeter_num];
		
	string perpoints = perimeter;

	float perimeterId = atof(perimeter.substr(1,5).c_str());
	m_mapInfo.pZone[iZone-1].Perimeter.perimeter_Id = int(perimeterId*10)%10;
	for(int i=1;i<=m_mapInfo.pZone[iZone-1].Perimeter.perimeter_num;i++)
	{
		string strfind = XnVarToString(m_mapInfo.pZone[iZone-1].zone_Id)+"."+ XnVarToString(m_mapInfo.pZone[iZone-1].Perimeter.perimeter_Id) +"."+XnVarToString(i)+" ";
		istartpos = perpoints.find(strfind.c_str());
		iendpos = perpoints.find(" ",istartpos+strfind.length());
	
		m_mapInfo.pZone[iZone-1].Perimeter.pPoint[i-1].x=atof(perpoints.substr(istartpos+strfind.length(),iendpos-istartpos-strfind.length()).c_str());
		istartpos = iendpos;
		iendpos = perpoints.find("\r\n",istartpos);
		m_mapInfo.pZone[iZone-1].Perimeter.pPoint[i-1].y = atof(perpoints.substr(istartpos,iendpos).c_str());
		perpoints = perpoints.substr(iendpos,perpoints.length()-iendpos);//
	}

	istartpos = zone.find("num_charpoint");
	iendpos = zone.find("end_charpoint");

	if(istartpos<=0)
	{
		m_mapInfo.pZone[iZone-1].charpoints_num = 0;
		m_mapInfo.pZone[iZone-1].pCharpoint = NULL;
	}
	else
	{
		string charpoints = zone.substr(istartpos+14,iendpos-istartpos-14);
		m_mapInfo.pZone[iZone-1].charpoints_num = atoi(charpoints.c_str()); 
		m_mapInfo.pZone[iZone-1].pCharpoint = new CHARPOINT[m_mapInfo.pZone[iZone-1].charpoints_num];
		istartpos = charpoints.find("charpoint");
		iendpos = charpoints.find("end_charpoint");
		charpoints = charpoints.substr(istartpos+9,iendpos-istartpos-9);


		for(int i=1;i<=m_mapInfo.pZone[iZone-1].charpoints_num;i++)
		{
			string strfind = XnVarToString(m_mapInfo.pZone[iZone-1].zone_Id)+".0."+XnVarToString(i)+" ";
			istartpos = charpoints.find(strfind.c_str())+strfind.length();
			iendpos = charpoints.find(" ",istartpos);
			string charpoint = charpoints.substr(istartpos,iendpos-istartpos);
			m_mapInfo.pZone[iZone-1].pCharpoint[i-1].x = atof(charpoints.substr(istartpos,iendpos-istartpos).c_str());
			istartpos=iendpos;
			iendpos=charpoints.find(" ",istartpos+1);
			m_mapInfo.pZone[iZone-1].pCharpoint[i-1].y = atof(charpoints.substr(istartpos,iendpos).c_str());
			istartpos=iendpos;
			iendpos=charpoints.find("\r\n",istartpos+1);
			m_mapInfo.pZone[iZone-1].pCharpoint[i-1].inlane = atof(charpoints.substr(istartpos,iendpos).c_str());

		}
	}	
	
	string exits = zone; 	
	int num = FindEXITNUM(exits);
	m_mapInfo.pZone[iZone-1].Perimeter.exit_num = num;

	if(num == 0)
		m_mapInfo.pZone[iZone-1].Perimeter.pExit = NULL;
	else
	{
		m_mapInfo.pZone[iZone-1].Perimeter.pExit = new EXITT[m_mapInfo.pZone[iZone-1].Perimeter.exit_num];
		istartpos = 0;
		for(int i = 0; i<m_mapInfo.pZone[iZone-1].Perimeter.exit_num; i++)
		{
			istartpos = exits.find("exit",istartpos);
			iendpos = exits.find("\r\n",istartpos);
			string exitpoint = exits.substr(istartpos+4,iendpos-istartpos-4);
			FindZoneEXIT(exitpoint,iZone,i);
			istartpos = iendpos+2;
		}

	}




	for(int i=1;i<=m_mapInfo.pZone[iZone-1].spots_num;i++)
		FindSpot(zone,iZone,i);
	
}

void CLoadRNDF::FindSpot(string zone, int iZone,int iSpot)
{
	stringstream sstream;
	int iZ = m_mapInfo.pZone[iZone-1].zone_Id;
	sstream<<iZ;
	sstream<<'.';
	sstream<<iSpot; 
	string strfind = "spot " + sstream.str();

	int istartpos = m_strMapData.find(strfind);
	int iendpos = m_strMapData.find("end_spot",istartpos);
	string spot = m_strMapData.substr(istartpos,iendpos-istartpos);

	istartpos = spot.find("spot_width");
	iendpos = spot.find("end_spot");
	string spot_width = spot.substr(istartpos+10,iendpos-istartpos-10);
	m_mapInfo.pZone[iZone-1].pSpot[iSpot-1].spot_width = atoi(spot_width.c_str());

	istartpos = spot.find("spot");
	iendpos = spot.find("end_spot");
	float spotId = atof(spot.substr(istartpos+4,iendpos-istartpos-4).c_str());
	m_mapInfo.pZone[iZone-1].pSpot[iSpot-1].spot_Id = int(spotId*10)%10;


	for(int i=1;i<=2;i++)
	{
		strfind = XnVarToString(spotId)+"."+XnVarToString(i)+" ";
		istartpos = spot.find(strfind.c_str());
		iendpos = spot.find(" ",istartpos+strfind.length());
		m_mapInfo.pZone[iZone-1].pSpot[iSpot-1].Point[i].x = atof(spot.substr(istartpos+strfind.length(),iendpos-istartpos-strfind.length()).c_str());
		istartpos=iendpos;
		iendpos=spot.find("\r\n",istartpos);
		m_mapInfo.pZone[iZone-1].pSpot[iSpot-1].Point[i].y = atof(spot.substr(istartpos,iendpos).c_str());
	}
}

CvPoint2D64f CLoadRNDF::GetPoint(int Seg_id,int Lan_id, int P_id)
{
	CvPoint2D64f P = {0};
	for(int i = 0;i<m_mapInfo.segment_num;i++)
	{
		if(m_mapInfo.pSegment[i].segment_Id == Seg_id)
			for(int j = 0;j<m_mapInfo.pSegment[i].lane_num;j++)
			{
				if(m_mapInfo.pSegment[i].pLane[j].lane_Id == Lan_id)
				{
					P.x = m_mapInfo.pSegment[i].pLane[j].pPoint[P_id-1].x;
					P.y = m_mapInfo.pSegment[i].pLane[j].pPoint[P_id-1].y;
					return P;
				}
			}
	}
	return P;
}

CvPoint2D64f CLoadRNDF::GetPerimeter(int Zone_id,int P_id)
{
	CvPoint2D64f P = {0};
	for(int i = 0;i<m_mapInfo.zone_num;i++)
	{
		if(m_mapInfo.pZone[i].zone_Id == Zone_id)
			for(int j = 0;j<m_mapInfo.pZone[i].Perimeter.perimeter_num;j++)
			{

				P.x = m_mapInfo.pZone[i].Perimeter.pPoint[P_id-1].x;
				P.y = m_mapInfo.pZone[i].Perimeter.pPoint[P_id-1].y;
				return P;
				
			}
	}
	return P;
}
CvPoint2D64f CLoadRNDF::GetSpot(int Zone_id,int Sp_id, int P_id)
{
	CvPoint2D64f P = {0};
	for(int i = 0;i<m_mapInfo.zone_num;i++)
	{
		if(m_mapInfo.pZone[i].zone_Id == Zone_id)
			for(int j = 0;j<m_mapInfo.pZone[i].spots_num;j++)
			{
				if(m_mapInfo.pZone[i].pSpot[j].spot_Id == Sp_id)
				{
					P.x = m_mapInfo.pZone[i].pSpot[j].Point[P_id-1].x;
					P.y = m_mapInfo.pZone[i].pSpot[j].Point[P_id-1].y;
					return P;
				}
			}
	}
	return P;
}

CHARPOINT CLoadRNDF::GetCharPoint(int Seg_id,int Lan_id, int P_id)
{
	CHARPOINT P = {0};
	for(int i = 0;i<m_mapInfo.segment_num;i++)
	{
		if(m_mapInfo.pSegment[i].segment_Id == Seg_id)
			for(int j = 0;j<m_mapInfo.pSegment[i].lane_num;j++)
			{
				if(m_mapInfo.pSegment[i].pLane[j].lane_Id == Lan_id)
				{
					P.x = m_mapInfo.pSegment[i].pLane[j].pCharpoint[P_id-1].x;
					P.y = m_mapInfo.pSegment[i].pLane[j].pCharpoint[P_id-1].y;
					P.inlane = m_mapInfo.pSegment[i].pLane[j].pCharpoint[P_id-1].inlane;
					return P;
				}
			}
	}
	return P;
}

CHARPOINT CLoadRNDF::GetCharPoint(int Zone_id, int P_id)
{
	CHARPOINT P = {0};
	for(int i = 0;i<m_mapInfo.zone_num;i++)
	{
		if(m_mapInfo.pZone[i].zone_Id == Zone_id)
			for(int j = 0;j<m_mapInfo.pZone[i].Perimeter.perimeter_num;j++)
			{

				P.x = m_mapInfo.pZone[i].pCharpoint[P_id-1].x;
				P.y = m_mapInfo.pZone[i].pCharpoint[P_id-1].y;
				P.inlane = m_mapInfo.pZone[i].pCharpoint[P_id-1].inlane;
				return P;
			}
	}
	return P;
}
void CLoadRNDF::FindConnect( )
{
	int istartpos = m_strMapData.find("connection");
	int iendpos = m_strMapData.find("end_file",istartpos);
	string connection = m_strMapData.substr(istartpos,iendpos-istartpos);

	while(1)
	{
	
		istartpos = connection.find("connection");
		if(istartpos<0)
			break;
		iendpos = connection.find("\r\n",istartpos);
		string conpoint = connection.substr(istartpos+11,iendpos-istartpos-11);
		int m1 = atoi(conpoint.c_str());
		
		string findexit = XnVarToString(m1)+".";
		istartpos = conpoint.find(findexit);	
		conpoint = conpoint.substr(istartpos+findexit.length(),conpoint.length()-istartpos-findexit.length());
		int n1 = atoi(conpoint.c_str());

		findexit = XnVarToString(n1)+".";
		istartpos = conpoint.find(findexit);

		conpoint = conpoint.substr(istartpos+findexit.length(),conpoint.length()-istartpos-findexit.length());
		int p1 = atoi(conpoint.c_str());

		findexit = " ";
		istartpos = conpoint.find(findexit);

		conpoint = conpoint.substr(istartpos+findexit.length(),conpoint.length()-istartpos-findexit.length());		
		int m2 = atoi(conpoint.c_str());
		
		findexit = XnVarToString(m2)+".";
		istartpos = conpoint.find(findexit);	
		conpoint = conpoint.substr(istartpos+findexit.length(),conpoint.length()-istartpos-findexit.length());
		int n2 = atoi(conpoint.c_str());

		findexit = XnVarToString(n2)+".";
		istartpos = conpoint.find(findexit);

		conpoint = conpoint.substr(istartpos+findexit.length(),conpoint.length()-istartpos-findexit.length());
		int p2 = atoi(conpoint.c_str());

		
		m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1].connect[m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1].con_num].m = m2;
		m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1].connect[m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1].con_num].n = n2;
		m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1].connect[m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1].con_num].p = p2;
		m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1].con_num++;
		
		//m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1].connect[m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1].con_num].m = m1;
		//m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1].connect[m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1].con_num].n = n1;
		//m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1].connect[m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1].con_num].p = p1;
		//m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1].con_num++;
		
		istartpos = connection.find("\r\n");
		connection = connection.substr(istartpos+2,connection.length()-istartpos-2);
	}

}

double CLoadRNDF::D(CvPoint2D64f p1, CvPoint2D64f p2)
{
	double temp;
	temp = pow(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2),0.5);
	return temp;
}

CONNECT CLoadRNDF::FindWay(CvPoint2D64f pos,double dir)
{
	CONNECT temp;
	double dis = 1000;
	CvPoint2D64f wp;
	double ang;
	for(int i = 0;i<m_mapInfo.segment_num;i++)
	{
		
		for(int j = 0;j<m_mapInfo.pSegment[i].lane_num;j++)
		{
			for(int k = 0;k<m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				wp.x = m_mapInfo.pSegment[i].pLane[j].pPoint[k].x;
				wp.y = m_mapInfo.pSegment[i].pLane[j].pPoint[k].y;
				ang = atan((wp.y - pos.y)/(wp.x - pos.x));
				if((wp.x - pos.x)<0)
					ang = ang + 3.14;
				ang = ang * 180/3.14;
				if(abs(ang-dir)>60&&abs(ang-dir)<300)
					continue;
				if(dis > D(pos,wp))
				{
					dis = D(pos,wp);
					temp.m = i;
					temp.n = j;
					temp.p = k;
				}
				
			}
		}
	}

	return temp;
}

double CLoadRNDF::GetLaneDir(int seg_id,int lan_id)
{	
	int num = m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num;
	double lat1 = m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pPoint[num-1].x;
	double lng1 = m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pPoint[num-1].y;
	double lat2 = m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pPoint[0].x;
	double lng2 = m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pPoint[0].y;

    double a = lat1 - lat2;
    double b = lng1 - lng2;
	double t = atan(b/a);
	if (a<0)
		t = t + 3.1415926;

	t = t * 180 / 3.1415926;
	return t;
}
int CLoadRNDF::GetCheckPoint(int check_num,CvPoint2D64f &point)
{	
	for(int i = 0;i<m_mapInfo.segment_num;i++)
	{
		
		for(int j = 0;j<m_mapInfo.pSegment[i].lane_num;j++)
		{
			for(int k = 0;k<m_mapInfo.pSegment[i].pLane[j].check_num;k++)
			{
				if(m_mapInfo.pSegment[i].pLane[j].pCheckpoint[k].check_sn == check_num)
				{
					int check_id = m_mapInfo.pSegment[i].pLane[j].pCheckpoint[k].check_Id;
					point = GetPoint(i+1, j+1, check_id);
					return 1;
				}
				
			}
		}
	}
	return 0;
}

int CLoadRNDF::GetCheckPointID(int check_num,int &id1,int &id2,int&id3)
{	

	for(int i = 0;i<m_mapInfo.segment_num;i++)
	{
		
		for(int j = 0;j<m_mapInfo.pSegment[i].lane_num;j++)
		{
			for(int k = 0;k<m_mapInfo.pSegment[i].pLane[j].check_num;k++)
			{
				if(m_mapInfo.pSegment[i].pLane[j].pCheckpoint[k].check_sn == check_num)
				{
					id1 = i+1;
					id2 = j+1;
					id3 = m_mapInfo.pSegment[i].pLane[j].pCheckpoint[k].check_Id;
					
					return 1;
				}
				
			}
		}
	}
	return 0;
}

int CLoadRNDF::GetCheckSeg(int check_num,int &n)
{	
	for(int i = 0;i<m_mapInfo.segment_num;i++)
	{
		
		for(int j = 0;j<1/*m_mapInfo.pSegment[i].lane_num*/;j++)
		{
			for(int k = 0;k<m_mapInfo.pSegment[i].pLane[j].check_num;k++)
			{
				if(m_mapInfo.pSegment[i].pLane[j].pCheckpoint[k].check_sn == check_num)
				{
					n = m_mapInfo.pSegment[i].pLane[j].pCheckpoint[k].check_Id;
					return i;
				}
				
			}
		}
	}
	//return 0;
}
int CLoadRNDF::GetLaneNum(int seg_id,int lan_id)
{
	return m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num;
}

int CLoadRNDF::GetLaneShape(int seg_id,int lan_id)
{
	int num = m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num;
	double s1=0,s2;
	s2 = GetAngle(GetPoint(seg_id,lan_id,1),GetPoint(seg_id,lan_id,num));
	for(int i = 0;i<num-1;i++)
	{
		s1=GetAngle(GetPoint(seg_id,lan_id,i+1),GetPoint(seg_id,lan_id,i+2));
		if (abs(s1-s2)>3&&abs(360-abs(s1-s2)>3))
			return 0;
	}
	
	return 1;
}
double CLoadRNDF::distance(double wd1, double jd1, double wd2, double jd2) {// 根据经纬度坐标计算实际距离

	double PI = 3.1415926535898;
	double radLat1 = (wd1 * PI / 180.0);
	double radLat2 = (wd2 * PI / 180.0);
	double a = radLat1 - radLat2;
	double b = (jd1 * PI / 180.0) - (jd2 * PI / 180.0);
	double s = 2 * asin(sqrt(pow(sin(a/2),2) + 
		cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
	s = s * 6378.137*1000;
	//s = round(s * 10000) / 10000;
	return s;


}
double CLoadRNDF::GetAngle(CvPoint2D64f apoint, CvPoint2D64f bpoint)
{	
	double lat1 = apoint.x;
	double lng1 = apoint.y;
	double lat2 = bpoint.x;
	double lng2 = bpoint.y;
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double a = radLat1 - radLat2;
	double b = (rad(lng1) - rad(lng2))*cos(radLat1);
	double t=atan(b/a);
	if (a<0)
		t=t+ 3.1416;
	//t=PI/2-t;

	t = t * 180 / 3.1416;
	return t;
}

double CLoadRNDF::rad( double d )
{
	 return d * 3.1416 / 180.0;
}


