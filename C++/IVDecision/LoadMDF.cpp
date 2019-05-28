#include "StdAfx.h"
#include "LoadMDF.h"



CLoadMDF::CLoadMDF(void)
{
}

CLoadMDF::~CLoadMDF(void)
{
}

void CLoadMDF::Openfile() 
{
	// TODO: Add your control notification handler code here
	CFileDialog dlg(TRUE,"txt","*.txt"); 
	if(dlg.DoModal() == IDOK)
	{
		CString path1=dlg.GetPathName();
		CFile mFile1; 
		if(mFile1.Open(path1,CFile::modeRead)==0) 
			return; 
/*		TRACE("%d",mFile.GetLength());
		CArchive ar(&mFile,CArchive::load); 
		ar>>m_strMapData; 
		TRACE("%d",m_strMapData.GetLength());
		ar.Close(); */
		string str("");
		DWORD fileLen = mFile1.GetLength();
		char *pBuf=new char[fileLen+1];
		pBuf[fileLen]=0;
		mFile1.Read(pBuf,mFile1.GetLength());
		m_strMapData = pBuf;
		delete []pBuf;
		mFile1.Close();
	}
	else
		return;
	ReadData(m_strMapData);
//	MaxMAPPOINT();
//	TRACE("%f,%f",m_MaxPoint.x,m_MaxPoint.y);
//	DrawX();
}
void CLoadMDF::ReadData(string strMapData)
{
	int istartpos = strMapData.find("MDF_name");
	int iendpos = strMapData.find("\r\n",istartpos);
	string mdfname = strMapData.substr(istartpos+8,iendpos-istartpos-8);
	m_mapInfo.MDF_name = mdfname;
	//MDF_name 

	istartpos = strMapData.find("RNDF");
	iendpos = strMapData.find("\r\n",istartpos);
	string rndfname = strMapData.substr(istartpos+4,iendpos-istartpos-4);
	m_mapInfo.RNDF = rndfname;


	istartpos = strMapData.find("format_version");
	iendpos = strMapData.find("\r\n",istartpos);
	string fversion = strMapData.substr(istartpos+15,iendpos-istartpos-15);
	m_mapInfo.format_version = atof(fversion.c_str());
	//format_version

	istartpos = strMapData.find("creation_date");
	iendpos = strMapData.find("\r\n",istartpos);
	string creationdate = strMapData.substr(istartpos+13,iendpos-istartpos-13);
	m_mapInfo.creation_date = creationdate;
	//creation_date

	istartpos = strMapData.find("num_checkpoints");
	iendpos = strMapData.find("\r\n",istartpos);
	string num_checkpoints = strMapData.substr(istartpos+15,iendpos-istartpos-15);
	m_mapInfo.num_checkpoints = atof(num_checkpoints.c_str());
	
	istartpos = strMapData.find("num_speed_limits");
	iendpos = strMapData.find("\r\n",istartpos);
	string speednum = strMapData.substr(istartpos+16,iendpos-istartpos-16);
	m_mapInfo.num_speed_limits = atof(speednum.c_str());


	istartpos = strMapData.find("checkpoints");
	iendpos = strMapData.find("end_checkpoints",istartpos);
	string checkpoints = strMapData.substr(istartpos,iendpos+11-istartpos);	
	istartpos = checkpoints.find("\r\n");
	
	for(int i = 0; ; i++)
	{
		iendpos = checkpoints.find("\r\n",istartpos+2);	
		string checknum = checkpoints.substr(istartpos+2,iendpos);
		int check_id = atoi(checknum.c_str());
		if(check_id>0)
			Check_Points.push(check_id);
		if(Check_Points.size()==m_mapInfo.num_checkpoints)break;
		istartpos = iendpos;

	}

	istartpos = strMapData.find("speed_limits");
	iendpos = strMapData.find("end_speed_limits",istartpos);
	string speed_limits = strMapData.substr(istartpos,iendpos+11-istartpos);	

	istartpos = speed_limits.find("\r\n");
	for(int i = 0; ; i++)
	{
		
		iendpos = speed_limits.find("\r\n",istartpos+2);
		string speedlimit = speed_limits.substr(istartpos+2,iendpos);
		FindSpeedLimit(speedlimit);
		if(Speed_Limits.size()==m_mapInfo.num_speed_limits)
			break;
		istartpos = iendpos;
		

	}
}
void CLoadMDF::FindSpeedLimit(string str)
{
	int istartpos = 0;
	int iendpos = str.find(" ",istartpos);
	string strid = str.substr(istartpos,iendpos);
	int id = atoi(strid.c_str());

	istartpos = iendpos+1;
	iendpos = str.find(" ",istartpos);
	string strmin = str.substr(istartpos,iendpos);
	int min = atoi(strmin.c_str());

	istartpos = iendpos+1;
	iendpos = str.find(" ",istartpos);
	string strmax = str.substr(istartpos,iendpos);
	int max = atoi(strmax.c_str());
	
	if(id>0)
	{
		SpeedLimit temp;
		temp.id = id;
		temp.max = max;
		temp.min = min;
		Speed_Limits.push(temp);
	}
}