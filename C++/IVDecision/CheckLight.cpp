#include "StdAfx.h"
#include "CheckLight.h"
#include "IntersectionContext.h"
CCheckLight::CCheckLight(void)
{
}

CCheckLight::~CCheckLight(void)
{
}
void CCheckLight::CheckLightDrive()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->Get_Sign = true;
	CvPoint2D64f m_gps = m_GpsData.GetPosition();
//	this->Stop(m_gps);//待完善 注意
	Sleep(1000);
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



bool CCheckLight::GetSignal(int (&sign)[4])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int GREEN[4]={0};
	int RED[4]={0};
	
	//5/4/3/2/1->STOP/NOTLEFT/NOTRIGHT/LEFT/RIGHT
	int time = 1000;
	while(1)
	{
		if(app->Traffic_Sign.size()>=3)
			break;
		Sleep(5);
		time--;
		if(time == 0)
			return false;
	}
	
	for(int i = 0; i<3; i++)
	{
		int temp = app->Traffic_Sign.front();
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
	app->Get_Sign = false;
	
	int sign_size = app->Traffic_Sign.size();
	for(int i = 0; i < sign_size;i++)
	{
		app->Traffic_Sign.pop();
	}
	return true;

}