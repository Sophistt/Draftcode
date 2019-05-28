#include "stdafx.h"
#include "Control.h"
#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;
ofstream ly("ly.txt");
ofstream desirev("dv.txt");
ofstream carstates("carstates.txt");
ofstream czl("caizongliang.txt");
ofstream czl2("caizongliang2.txt");
ofstream dmbob("dmbob.txt");
ofstream xcjout("xcjtest.txt");
ofstream outstopline("stoplineMg.txt");

ofstream outofsdiffer("ofsdiffer.txt");

ofstream outqidongstatectrl("outqidongstatectrl.txt");

CControl::CControl(void)
	: m_lastM2(0)
	, m_lastBrake(0)
	, m_stopAtPoint(false)
	, m_stopAtLine(false)
	, m_stopLineSpeed0(0)
{
	m_roadPoint = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );
	//m_planRoad = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );

	memset(&udpdatarcv, 0, sizeof(Uudpdatarcv));

	m_controlParam.steerAngle = 0;
	m_controlParam.brake = 0;
	m_controlParam.driveTorque = 0;
	m_controlParam.hornOn = 0;
	m_controlParam.light = 0;
	m_uDesire = 0;

	m_pidLat.Kp = 35;//26
	m_pidLat.Ki = 0;
	m_pidLat.Kd = 0.1*5;//0.3;

	m_pidLat.Kc = 0;
	m_pidLat.calc = NULL;
	m_pidLat.Err = 0;
	m_pidLat.Fdb = 0;
	m_pidLat.Out = 0;
	m_pidLat.OutMax = 779.9;
	m_pidLat.OutMin = -780;
	m_pidLat.OutPreSat = 0;
	m_pidLat.Ref = 0;
	m_pidLat.SatErr = 0;
	m_pidLat.Ud = 0;
	m_pidLat.Ui = 0;
	m_pidLat.Up = 0;
	m_pidLat.Up1 = 0; 

	m_pidLatLdw=m_pidLat;
	m_pidLatLdw.OutMax = 90;
	m_pidLatLdw.OutMin = -90;
	m_pidLatLdw.Kp = 15;//30//26
	m_pidLatLdw.Kd = 0.3;//0.1//0.3
	
	m_pidDrive = m_pidLat;
	m_pidDrive.Kp = 150;
	m_pidDrive.Ki = 0.001/5;
	m_pidDrive.Kd = 1*5;
	m_pidDrive.Kc = 0;
	m_pidDrive.OutMax = 400;
	m_pidDrive.OutMin = -900;
	lasttorque=0;
	con=0;
	con1=0;
	con2=0;
	con3=0;
	con4=0;

	laststrldw=0;
	LKAlastAcc=0;
	LKAlastbrake=0;
	for(int i=0;i<5;i++)
		fdbhis[i]=-100;

	Lkahaikanglanecnt=0;

	halfwidth=1.725;//1.85

	//m_hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);//路径更新消息
	m_endControlThreadFlag = false;	//终止线程标志
	czl<< "车速" << " "<< "期望车速" << " " << "油门" << " "<< "制动" << " " << "加速度" <<" "<<"制动标志位"<<" "<<"终点停车标志"<< endl;

}


CControl::~CControl(void)
{
}


//计算控制量
//currentPosition：当前位置
//currentDirection：当前航向
//aimPosition：目标点位置
//aimDirection：目标航向
//speed：车速
void CControl::GetCtrlParameters(CvPoint2D64f currentPosition, double currentDirection, 
								 CvPoint2D64f *aimPosition, double aimDirection, double speed)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	float steerAngle = 0;
	UINT brake = 0;
	float torque = 0;
	steerAngle = lateralControl(currentPosition, currentDirection, aimPosition, aimDirection,speed);
    czl2 <<setprecision(11)<<aimPosition[4].x<<" "<<aimPosition[4].y<<" "<<currentPosition.x<<" "<<currentPosition.y<<endl;

	if (m_stopAtPoint)
	{
		//double s = m_GpsData.GetDistance(currentPosition.x, currentPosition.y, m_endPoint.x, m_endPoint.y);
		double s = VertDist(currentPosition,currentDirection,m_endPoint);
		//********1203dmb添加*******//
		if (app->stop)//优先判断前方障碍物
		{
			m_uDesire = 0;
			dmbob<<"与路口遇障碍停车1"<<endl;
		}
		//********1203dmb添加*******//
		else
		{
			if (s < 4)
			{
				m_uDesire = 0;
			}
			else
			{
				double a = speed * speed / s / 2.0;
				if (m_stopAtLine || s<15)
				{
					if (m_stopLineSpeed0 < 0.1)
					{
						m_stopLineSpeed0 = speed;
					}
					a = m_stopLineSpeed0 * m_stopLineSpeed0 / 7.0 / 2.0;
				}
				m_uDesire -= a * 0.1;
				if (m_uDesire < 1)
				{
					m_uDesire = 0;
				}
			}
		}
		double du1 = app->drive_state_update;
		double du2 = app->drive_curvature_update;
		double du3 = app->drive_obstacle_update;
		double du4 = app->drive_obstacle2_update;
		double du5 = app->limit_speed_update;

		m_uDesire = min(m_uDesire, du1);
		m_uDesire = min(m_uDesire, du2);
		m_uDesire = min(m_uDesire, du3);
		m_uDesire = min(m_uDesire, du4);
		m_uDesire = min(m_uDesire, du5);
	}
	else if (app->stop)
	{
		m_uDesire = 0;
		dmbob<<"与路口遇障碍停车2"<<endl;
	}
	else
	{
		double du1 = app->drive_state_update;
		double du2 = app->drive_curvature_update;
		double du3 = app->drive_obstacle_update;
		double du4 = app->drive_obstacle2_update;
		double du5 = app->limit_speed_update;

		m_uDesire = min(du1, du2);
		m_uDesire = min(m_uDesire, du3);
		m_uDesire = min(m_uDesire, du4);
		m_uDesire = min(m_uDesire, du5);
	}
	longitudinalControl(speed, torque, brake);//计算油门刹车
	m_controlParam.steerAngle = steerAngle;
	m_controlParam.driveTorque = torque;
	m_controlParam.brake = brake;
	lasttorque=torque;
	carstates << speed << '	' << m_uDesire << endl;	
	SYSTEMTIME t1;
	GetLocalTime(&t1);
	czl <<t1.wHour<<" "<<t1.wMinute<<" "<<t1.wSecond<<" "<<t1.wMilliseconds<<" "<< speed<< " "<< m_uDesire<<" "<<torque<<" "<<brake<<" "<<app->GPS_LonAcc<<" "<<app->brake_flag<<" "<<app->goal_flag<< endl;
}

//计算侧向误差
//currentPosition：当前位置
//currentDirection：当前航向
//aimPosition：目标点位置
//返回值：侧向误差
double CControl::GetLatError(CvPoint2D64f currentPosition, double currentDirection, CvPoint2D64f aimPosition)
{
	double aimDirection = m_GpsData.GetAngle(aimPosition.x, aimPosition.y, currentPosition.x, currentPosition.y);
	double angleError = (aimDirection - currentDirection) * PI / 180;
	double dist = m_GpsData.GetDistance(currentPosition.x, currentPosition.y, aimPosition.x, aimPosition.y);
	double angleerrtmp=aimDirection - currentDirection;
	double laterrnow=dist * sin(angleError);
	if(angleerrtmp>360)
		angleerrtmp=angleerrtmp-360;
	else if(angleerrtmp<-360)
		angleerrtmp=angleerrtmp+360;
	if(angleerrtmp>180)
		angleerrtmp=angleerrtmp-360;
	else if(angleerrtmp<-180)
		angleerrtmp=angleerrtmp+360;
	if(angleerrtmp>90||angleerrtmp<-90)
		laterrnow=dist * angleerrtmp/90;
	return laterrnow;
}

//纵向控制
//speed：车速
//torque：输出对地扭矩
//brake：输出制动量
void CControl::longitudinalControl(double speed, float &torque, UINT &brake)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	double desireSpeed = 0;
	if (m_uDesire < 2/3.6 && speed < 1)
	{
		torque = 0;
		m_pidDrive.Ui = 0;
		if (speed < 0.3)
		{
			brake = 12;
		} 
		else
		{
			/*brake = m_lastBrake - 1;
			if (brake < 0)
			{
				brake = 0;
			}*/
			brake = m_lastBrake;
		}
		if(brake<12)
			brake=12;
	} 
	else
	{
		//车速PID控制
		if (m_uDesire < 12/3.6)
		{
			m_pidDrive.Kp = 300;
		}
		else if(m_uDesire < 7)
		{
			m_pidDrive.Kp = 200;//200
		}
		else
		{
			m_pidDrive.Kp = 200;
		}
		if (abs(m_uDesire - speed) > 2 || speed < 0.1)
		{
			m_pidDrive.Ki = 0;
		}
		else
		{
			m_pidDrive.Ki = 0.03;
		}
		if (m_uDesire > 32/3.6)
		{
			m_pidDrive.OutMax = 700;
		}
		else
		{
			m_pidDrive.OutMax = 500;
		}
		if (m_uDesire - speed > 4)
		{	//加速阶跃不超过3m/s
			desireSpeed = speed + 4;
		}
		else
		{
			desireSpeed = m_uDesire;
		}
		if (((desireSpeed - speed) > -4/3.6) && (m_uDesire > 0.5) && (!((app->continue_brake)&&(m_lastBrake>0))))	//控制油门
		{
			app->brake_flag==false;
			app->goal_flag==false;
			con=0;
			con1=0;
		    con2=0;
			con3=0;
			con4=0;
			m_pidDrive.Ref = desireSpeed;
			m_pidDrive.Fdb = speed;
			pid_reg3_calc(&m_pidDrive);
			torque = m_pidDrive.Out;

			if(app->continue_spdred)
			{
				float torqred = (app->spdredacc)*1635*0.307 + ((0.015*9.8*1635+2.28*0.295*1.204*speed*speed/2)*0.307);
				if(torqred>0)
					torqred=0;
				if(torqred<torque)
				{
					torque = torqred;
					//m_pidDrive.Ui = 0;
				}
			}
			torque = torque*0.8 + lasttorque*0.2;

			//torque = slidingModeControlForSpeed(speed, m_lastM2);	//车速滑模控制
			/*if(lasttorque*torque<0)
			{
              torque=lasttorque;
			}*/


			if (torque < m_canMsgRecv.TqGndNegMax)
			{
				torque = m_canMsgRecv.TqGndNegMax;
			}
			brake = 0;
			m_brakeSpeed0 = speed;


			//if(torque<0)
			//{
			//	if(con4<3)
			//	{
			//		torque=lasttorque;
			//	}
			//	con4++;
			//}
			//else
			//{
			//	con4=0;
			//}
		} 
		else	//控制制动
		{
		    torque = 0;
		    int maxBrake = 10;
			m_pidDrive.Ui = 0;
			//if(app->goal_flag==true)
			if(app->goal_flag==true && m_uDesire>=4/3.6)
		   {
		        if(m_brakeSpeed0>20/3.6)
		        {
			        maxBrake = 2;
		        }
		        else if(m_brakeSpeed0<=20/3.6)
		        {
			        maxBrake = 1;
		        }
		   }

		   else 
		   {
			   if((m_brakeSpeed0>25/3.6)&&(app->brake_flag==true&& m_uDesire>=4/3.6))//舒适制动
			   {
				   //maxBrake = 8;
				   con2++;
				   if (con2>4)
				   {
					   maxBrake = 12;
					   torque = lasttorque*0.8;
					   if(abs(torque)<1)
						   torque=0;
				   }
				   else
				   {
					   maxBrake = 0;
					   torque=lasttorque;

				   }
			   }
			   else if((m_brakeSpeed0>25/3.6)&&(app->brake_flag==false||m_uDesire<4/3.6))//舒适制动
			   {
				   //maxBrake = 8;
				   con2++;
				   if (con2>4)
				   {
					   maxBrake = 20;
					   torque = lasttorque*0.8;
					   if(abs(torque)<1)
						   torque=0;
				   }
				   else
				   {
					   maxBrake = 0;
					   torque=lasttorque;

				   }
			   }
			   else if((m_brakeSpeed0>21/3.6)&&(app->brake_flag==true&& m_uDesire>=4/3.6))//舒适制动
			   {
				   //maxBrake = 8;
				   con2++;
				   if (con2>4)
				   {
					   maxBrake = 9.5;
					   torque = lasttorque*0.8;
					   if(abs(torque)<1)
						   torque=0;
				   }
				   else
				   {
					   maxBrake = 0;
					   torque=lasttorque;

				   }
			   }
			   else if((m_brakeSpeed0>21/3.6)&&(app->brake_flag==false||m_uDesire<4/3.6))//紧急制动
			   {
				   con++;
				   if (con>4)
				   {
                       maxBrake = 16;
					   torque = lasttorque*0.8;
					   if(abs(torque)<1)
						   torque=0;
				   }
				   else
				   {
				      maxBrake = 0;
					  torque=lasttorque;

				   }
				  
			   }
			   else if((m_brakeSpeed0<=21/3.6)&&(app->brake_flag==true&& m_uDesire>=4/3.6))
			   {
				   //maxBrake =6;
				   con3++;
				   if (con3>4)
				   {
					   maxBrake = 9.5;
					   torque = lasttorque*0.8;
					   if(abs(torque)<1)
						   torque=0;
				   }
				   else
				   {
					   maxBrake = 0;
					   torque=lasttorque;

				   }
			   }
			   else if((m_brakeSpeed0<=21/3.6)&&(app->brake_flag==false||m_uDesire<4/3.6))
			   {
				   con1++;
				   if (con1>4)
				   {
					   maxBrake = 12;
					   torque = lasttorque*0.8;
					   if(abs(torque)<1)
						   torque=0;
					   
				   }
				   else
				   {
					   maxBrake = 0;
					   torque=lasttorque;
				   }
			   }

		   }

		   if(app->stop)
		   {
			   if(app->spdredacc>-((app->GPS_Speed)*(app->GPS_Speed))/4)
				   app->spdredacc=-((app->GPS_Speed)*(app->GPS_Speed))/4;
		   }
		   
		   if(app->continue_spdred || app->stop)
		   {
			   double a = -app->spdredacc;
			   double p=0;
			   double braketmp=0;

/*
			   if(app->bisaiwaiburoad)
			   {
				   if(a>=3)
					   a=4;
			   }
			   */

			   if(app->left_right==0)
			   {
				   if(app->road_speed2ctrl<42 || app->m_task2ctrl==5002)
				   {
					   if(a>4)
						   a=4;
				   }
				   else if(app->road_speed2ctrl<45)
				   {
					   if(a>4)
						   a=4;
				   }
			   }
			   else
			   {
				   if(a>5)
					   a=5;
			   }
			   if(a<0.615)
				   p=0.5;
			   else if(a>9)
				   p=8;
			   else
				   p=0.8886*a-0.0465;
			   if(p<0.4)
				   braketmp=2;
			   else if(p<3)
				   braketmp=18.4615*p-5.3846;
			   else if(p<8)
				   braketmp=10*p+20;
			   else
				   braketmp=100;
			   if(braketmp>maxBrake)
				   maxBrake=braketmp;
		   }

		   if((app->continue_brake)&&(m_lastBrake>0)&&(m_lastBrake<40))
		   {
			   if(maxBrake<m_lastBrake)
				   maxBrake=m_lastBrake;
		   }
		   

		   /*if((m_brakeSpeed0>20/3.6)&&(app->brake_flag==true))
		   {
               maxBrake = 14;
		   }
		   else if((m_brakeSpeed0>20/3.6)&&(app->brake_flag==false))
		   {
                maxBrake = 26;
		   }
		   else if((m_brakeSpeed0<=20/3.6)&&(app->brake_flag==true))
		   {
			   maxBrake = 10;
		   }
		   else if((m_brakeSpeed0<=20/3.6)&&(app->brake_flag==false))
		   {
			   maxBrake = 14;
		   }
		   else if((m_brakeSpeed0>20/3.6)&&(app->goal_flag==true))
		   {
			   maxBrake = 12;
		   }
		   else if((m_brakeSpeed0<=20/3.6)&&(app->goal_flag==true))
		   {
			   maxBrake = 10;
		   }*/


           /*if (((desireSpeed - m_brakeSpeed0) <= -3/3.6) && ((desireSpeed - m_brakeSpeed0) >= -10/3.6)&&(m_brakeSpeed0>=20/3.6))
		   {
                  maxBrake = 14;
		   }
		   else if(((desireSpeed - m_brakeSpeed0) <= -3/3.6) && ((desireSpeed - m_brakeSpeed0) >= -10/3.6)&&(m_brakeSpeed0<20/3.6))
		   {
			   maxBrake = 8;
		   }

		   else if(((desireSpeed - m_brakeSpeed0) <-10/3.6)&&(m_brakeSpeed0>=20/3.6))
		   {
                  maxBrake = 26;
		   } 

		   else if(((desireSpeed - m_brakeSpeed0) <-10/3.6)&&(m_brakeSpeed0<20/3.6))
		   {
			   maxBrake = 10;
		   } */


			/*if (m_brakeSpeed0 > 30/3.6)
			{
				maxBrake = 26;
			}
			else if (m_brakeSpeed0 > 18/3.6)
			{
				maxBrake = 14;
			}
			else
			{
				maxBrake = 8;
			}*/
		   brake = maxBrake;
			//torque = 0;
			m_pidDrive.Ui = 0;
			m_pidDrive.Up1 = 0;
		}
	}
	desirev<<brake<<endl;
}

void CControl::longitudinalControlLKA(double disirespd, double spdLKAacc, double speed, double &torque, double &brake)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	if (disirespd < 1 && speed < 1)
	{
		torque = 0;
		m_pidDrive.Ui = 0;
		if (speed < 0.3)
		{
			brake = 12;
		} 
		else
		{
			brake = LKAlastbrake;
		}
		if(brake<12)
			brake=12;
	} 
	else
	{
		if (disirespd < 12/3.6)
		{
			m_pidDrive.Kp = 300;
		}
		else if(disirespd < 7)
		{
			m_pidDrive.Kp = 200;
		}
		else
		{
			m_pidDrive.Kp = 200;
		}
		if (abs(disirespd - speed) > 2 || speed < 0.1)
		{
			m_pidDrive.Ki = 0;
		}
		else
		{
			m_pidDrive.Ki = 0.03;
		}
		if (disirespd > 32/3.6)
		{
			m_pidDrive.OutMax = 700;
		}
		else
		{
			m_pidDrive.OutMax = 500;
		}
		if (disirespd - speed > 3)
		{
			disirespd = speed + 3;
		}
		if (((disirespd - speed) > -3/3.6) && (disirespd > 0.5) && (spdLKAacc==0))	//控制油门
		{
			m_pidDrive.Ref = disirespd;
			m_pidDrive.Fdb = speed;
			pid_reg3_calc(&m_pidDrive);
			torque = m_pidDrive.Out;
			brake = 0;
		} 
		else
		{
		    torque = 0;
		    double maxBrake = 10;
		   
		   if(spdLKAacc<0)
		   {
			   double a = -spdLKAacc;
			   double p=0;
			   double braketmp=0;

			   if(a<0.615)
				   p=0.5;
			   else if(a>9)
				   p=8;
			   else
				   p=0.8886*a-0.0465;
			   if(p<0.4)
				   braketmp=2;
			   else if(p<3)
				   braketmp=18.4615*p-5.3846;
			   else if(p<8)
				   braketmp=10*p+20;
			   else
				   braketmp=100;
			   if(braketmp>maxBrake)
				   maxBrake=braketmp;
		   }
		   
		   if(maxBrake<LKAlastbrake)
			   maxBrake=LKAlastbrake;

		   brake = maxBrake;
		   m_pidDrive.Ui = 0;
		   m_pidDrive.Up1 = 0;
		}
	}
	LKAlastbrake=brake;

	desirev<<brake<<endl;
}

//侧向控制
//currentPosition：当前位置
//currentDirection：当前航向
//aimPosition：目标点位置
//aimDirection：目标航向
//speed：车速
//返回值：方向盘转角
float CControl::lateralControl(CvPoint2D64f currentPosition, double currentDirection,
							   CvPoint2D64f *aimPosition, double aimDirection, double speed)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	float steerAngle = 0;
	double pidOut = 0;
	double lateralError[5];
	double meanLatErr = 0;


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
	
	if (m_uDesire < 1 && speed < 1 && app->stop)
	{
		if(app->fangxiangpanbuhuizheng)
			steerAngle=m_controlParam.steerAngle;
		else
			steerAngle = 0;
	} 
	else
	{
		/*double north_dir = m_GpsData.GetAngle(aimPosition.x, aimPosition.y, currentPosition.x, currentPosition.y);
		double head_path_angle = (north_dir - currentDirection)*PI/180;
		double dist = m_GpsData.GetDistance( currentPosition.x, currentPosition.y, aimPosition.x, aimPosition.y );*/
		for (int i = 0; i < 5; i++)
		{
			lateralError[i] = GetLatError(currentPosition, currentDirection, aimPosition[i]);
			meanLatErr += lateralError[i];
		}
		meanLatErr /= 5;
		m_pidLat.Ref = 0; 
		m_pidLat.Fdb = meanLatErr;
		pid_reg3_calc(&m_pidLat);
		pidOut = m_pidLat.Out;
		/*if (speed < 5)
		{
			speed = 5;
		}*/
		if (speed > 17/3.6)/*13*///17
		{
			if (pidOut > 0)
			{
				steerAngle = pidOut * 2.7 * (1 + 0.0043 * 1.7 * speed * speed) / speed + 5.5;	//矫正零位偏差
			} 
			else
			{
				steerAngle = pidOut * 2.7 * (1 + 0.0043 * 1.7 * speed * speed) / speed + 5.5;	//矫正零位偏差 0.003
			}
		}
		else
		{
			steerAngle = pidOut * 2.7 + 5.5;
		}
	}
	ly << meanLatErr << '	' << lateralError[0] << endl;
    xcjout<<steerAngle<<endl;
	return steerAngle;
}

//计算预瞄距离
//speed：车速
//返回值：预瞄距离
float CControl::GetAimLen(double speed)
{
	float aimLength = 10;
	if (speed < 6/3.6)
	{
		aimLength = 6;
	}
	else if (speed < 16/3.6)
	{
		aimLength = 8;
	}
	else if (speed < 20/3.6)
	{
		aimLength = 10;
	}
	else if (speed < 40/3.6)
	{
		aimLength = 15;
	}
	else if (speed < 60/3.6)
	{
		aimLength = 22;
	}
	else
	{
		aimLength = 25;
	}
	return aimLength;
}

//计算预瞄点
//currentPosition：当前位置
//currentDirection：当前航向
//aimLength：预瞄距离
//aimPosition：输出预瞄点位置
//aimDirection：输出目标航向
//返回值：找到预瞄点返回true，路点到头返回false
bool CControl::GetAimPoint(CvPoint2D64f currentPosition, double currentDirection, float aimLength,
						   double speed, CvPoint2D64f &aimPosition, double &aimDirection)
{
	//默认路点存储顺序与车辆前进方向一致
	double distance;
	int pointNumber = m_roadPoint->total;//总路点数，路点存储按行走顺序
	CvPoint3D64f aimPoint_temp = {0};	//候选预瞄点
	CvPoint3D64f aimPoint_1 = {0};
	CvPoint3D64f aimDirectionPoint_temp = {0};	//预瞄点前方某点，用来计算目标航向
	CvPoint2D64f pointtemp = {0};
	CvPoint2D64f pointtempgps = {0};
	int count = -1;

	for (int j = 0; j < m_roadPoint->total; j++)
	{
		aimPoint_temp = *(CvPoint3D64f*)cvGetSeqElem( m_roadPoint, j );
		pointtempgps.x=aimPoint_temp.x;
		pointtempgps.y=aimPoint_temp.y;
		pointtemp=m_GpsData.APiontConverD(currentPosition,pointtempgps,currentDirection);
		if(pointtemp.y>412)
			count = j;
		else
			break;
	}
	if(m_roadPoint->total-count-1<6)
	{
		count = -1;
		double distmin = 9999999;
		for (int j = 0; j < m_roadPoint->total; j++)
		{
			aimPoint_temp = *(CvPoint3D64f*)cvGetSeqElem( m_roadPoint, j );
			distance = m_GpsData.GetDistance( currentPosition.x, currentPosition.y, aimPoint_temp.x, aimPoint_temp.y );
			if(distance<distmin)
			{
				count = j;
				distmin = distance;
			}
			if (distance >100)
			{
				break;
			}
		}
	}
	for (int k = 0; k<=count; k++)
	{
		cvSeqPopFront( m_roadPoint,NULL );
	}

	pointNumber = m_roadPoint->total;

	aimDirection = currentDirection;

	if (pointNumber <= 1)
	{
		aimPosition.x = currentPosition.x;
		aimPosition.y = currentPosition.y;
		return false;
	}
	if (pointNumber < 6)
	{
		aimPoint_temp = *(CvPoint3D64f*)cvGetSeqElem(m_roadPoint, 1);
		aimPosition.x = aimPoint_temp.x;
		aimPosition.y = aimPoint_temp.y;
		return false;
	}
	aimPoint_temp = *(CvPoint3D64f*)cvGetSeqElem(m_roadPoint, pointNumber-1);
	aimDirection = currentDirection;
	distance = m_GpsData.GetDistance(currentPosition.x, currentPosition.y, aimPoint_temp.x, aimPoint_temp.y);
	if (distance < 3)
	{
		aimPosition.x = currentPosition.x;
		aimPosition.y = currentPosition.y;
		return false;
	}
	aimPoint_1 = *(CvPoint3D64f*)cvGetSeqElem(m_roadPoint, 0);
	distance = m_GpsData.GetDistance(aimPoint_1.x, aimPoint_1.y, aimPoint_temp.x, aimPoint_temp.y);
	if (distance < 1)
	{
		aimPosition.x = currentPosition.x;
		aimPosition.y = currentPosition.y;
		return false;
	}
	int i = 0;	
	for (i = 0; i < m_roadPoint->total; i++)
	{
		aimPoint_temp = *(CvPoint3D64f*)cvGetSeqElem( m_roadPoint, i );
		distance = m_GpsData.GetDistance( currentPosition.x, currentPosition.y, aimPoint_temp.x, aimPoint_temp.y );
		if (distance >= aimLength)
		{
			i++;
			break;
		}
	}
	if(i+4<m_roadPoint->total)
	{
		aimDirectionPoint_temp = *(CvPoint3D64f*)cvGetSeqElem(m_roadPoint, i + 4);
		aimDirection = m_GpsData.GetAngle(aimDirectionPoint_temp.x, aimDirectionPoint_temp.y, aimPoint_temp.x, aimPoint_temp.y);
	}
	else if(i==m_roadPoint->total)
	{
		aimDirectionPoint_temp = *(CvPoint3D64f*)cvGetSeqElem(m_roadPoint, i - 2);
		aimDirection = m_GpsData.GetAngle(aimPoint_temp.x, aimPoint_temp.y, aimDirectionPoint_temp.x, aimDirectionPoint_temp.y);
	}
	else
	{
		aimDirectionPoint_temp = *(CvPoint3D64f*)cvGetSeqElem(m_roadPoint, m_roadPoint->total-1);
		aimDirection = m_GpsData.GetAngle(aimDirectionPoint_temp.x, aimDirectionPoint_temp.y, aimPoint_temp.x, aimPoint_temp.y);
	}
	aimPosition.x = aimPoint_temp.x;
	aimPosition.y = aimPoint_temp.y;
	return true;	
}

UINT CControl::sendCtrlParamToCanBus(void)
{
	return 1;
}

// 符号函数
int CControl::sign(double data)
{
	if (data > 0)
	{
		return 1;
	}
	else if (data < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

// 停车,pointEndFlag：路点结束标志
void CControl::ctrlStop(bool pointEndFlag)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	m_uDesire = 0;
	if (pointEndFlag)
	{
		double speed = app->GPS_Speed;
		//m_endControlThreadFlag = true;
		
		
			CvPoint2D64f currentPosition = cvPoint2D64f(0, 0);
			double currentDirection = 0;
			CvPoint2D64f aimPointBeforFilter={0};
			double aimDirection = 0;
			app->critical_section.Lock();
			currentPosition = app->GPS_Point;
			currentDirection = app->GPS_Direction /*- 2*/;
			speed = app->GPS_Speed;
			
			app->critical_section.Unlock();
			

			//longitudinalControl(speed, m_controlParam.driveTorque, m_controlParam.brake);
			m_controlParam.driveTorque = 0;
			/*m_controlParam.brake = 8;
			m_lastBrake = m_controlParam.brake;*/
			if(speed>25/3.6)
			{
		       m_controlParam.brake = 20;
			}
			else if(speed>20/3.6)
			{
				m_controlParam.brake = 16;
			}
			else if(speed> 10/3.6)
			{
               m_controlParam.brake = 10;
			}
			else if(speed> 5/3.6)
			{
              m_controlParam.brake = 6;
			}
			else
			{
              m_controlParam.brake = 4;
			}
			m_pidDrive.Ui = 0;
                 
			/*else if(speed>5/3.6)
                 m_controlParam.brake = 16;
			else
				m_controlParam.brake = 20;*/
			m_lastBrake = m_controlParam.brake;



			if (speed < 1 || abs(m_controlParam.steerAngle) < 10)
			{
				m_controlParam.steerAngle = 0;
			} 
			else
			{
				m_controlParam.steerAngle *= 0.8;
			}
			
		
	} 
}

// 饱和函数，输入乘以factor以后计算饱和，饱和输出-1或1，不饱和输出in*factor
int CControl::sat(double in, double factor)
{
	if (in * factor > 1)
	{
		return 1;
	} 
	else if (in * factor < -1)
	{
		return -1;
	}
	else
	{
		return in * factor;
	}
}


//speed：车速
//lastM2：上一步的M2值
//返回值：对地转矩
float CControl::slidingModeControlForSpeed(double speed, double &lastM2)
{
	const double kesi = 2;
	const double beta = 30;
	const double satFactor = 2;
	const double filterCoef = 0.5;	//滤波系数，越接近1，转矩上升越平缓
	double speedError = m_uDesire - speed;
	if (speedError > 3)
	{
		speedError = 3;
	}
	double M1 = beta * speedError * speedError;
	double M2 = (1 - filterCoef) * kesi * m_uDesire * m_uDesire + filterCoef * lastM2;	//滤波器，防止对地扭矩跳变
	double torque = (M1 + M2) * sat(speedError, satFactor);
	lastM2 = M2;
	return torque;
}


// 预瞄点滤波
//pt：待滤波的GPS点序列
//aimPoint：输出滤波后的预瞄点
void CControl::filterByTime(CvPoint2D64f* pt, CvPoint2D64f &aimPoint)
{
	double k1[] = {-0.02205882352941176, -0.01911764705882353, -0.01617647058823529, -0.01323529411764706, 
				   -0.01029411764705882, -0.007352941176470588, -0.004411764705882353, -0.001470588235294118, 
				   0.001470588235294118, 0.004411764705882353,  0.007352941176470588, 0.01029411764705882, 
				   0.01323529411764706, 0.01617647058823529, 0.01911764705882353, 0.02205882352941176};
	double k2[] = {0.25, 0.225, 0.2, 0.175, 0.15, 0.125, 0.1, 0.075, 0.05, 0.025, 0, -0.025, -0.05, -0.075,
					-0.1, -0.125};
	double kx = 0;
	double bx = 0;
	double ky = 0;
	double by = 0;

	for (int i = 0; i < 16; i++)
	{
		kx += k1[i] * pt[i].x;
		bx += k2[i] * pt[i].x;
		ky += k1[i] * pt[i].y;
		by += k2[i] * pt[i].y;
	}

	aimPoint.x = kx * 16.0 + bx;
	aimPoint.y = ky * 16.0 + by;

	pt[15] = aimPoint;
	for (int i = 0; i < 15; i++)
	{
		pt[i] = pt[i + 1];
	}
}

void CControl::ClearzStop(bool bclearall)
{
	m_stopAtPoint = false;
	m_stopAtLine = false;
	if(bclearall)
		m_stopLineSpeed0 = 0;
}


int CControl::zStop(CvPoint2D64f end )	//控制改
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	m_endPoint = end;
	m_stopAtPoint = true;
	//while(1)
	//{
	//	if(app->light_res!=2)
	//	{
	//		m_stopAtPoint = false;
	//		m_stopAtLine = false;
	//		m_stopLineSpeed0 = 0;
	//		return 8;
	//	}
	//	
	//	if(app->stop_map)
	//	{
	//		m_stopAtLine = true;
	//		outstopline<<"收到stopline"<<endl;
	//	}

	//}
	//m_stopAtPoint = false;
	//m_stopAtLine = false;
	//return 0;
	//********1205*********//
	if (app->light_res == 2)
	{
		m_stopAtPoint = true;
		app->secondDecison = "停止点停车";
		if(app->stop_map)
		{
			m_stopAtLine = true;
			app->secondDecison = "停止线停车";
			outstopline<<"收到stopline"<<endl;
		}
		//return 8;
	}
	else
	{
		m_stopAtPoint = false;
		m_stopAtLine = false;
		//m_stopLineSpeed0 = 0;
		//return 8;	
	}
	return 8;
	//********1205*********//
}


double CControl::SearchFrontOb(I_Map *map,double range)//正前方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=-1;m>-range*5;m--)
		for(int n=-4/*6*/;n<=4/*6*/;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);

			}
		}

		return 0;

}
double CControl::SearchRearOb(I_Map *map,double range)//正hou方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=1;m<range*2;m++)
		for(int n=-4/*6*/;n<=4/*6*/;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/2.0);

			}
		}

		return 0;

}
double CControl::SearchRearObForApa(I_Map *map,double range)//正hou方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=1;m<range*10;m++)
		for(int n=-3/*6*/;n<=3/*6*/;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/10.0);

			}
		}

		return 0;

}
double CControl::SearchrightFrontOb(I_Map *map,double range)//you前方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=-1;m>-4*5;m--)
		for(int n=-2;n<=2;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18 ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);
			}
		}

	x = 256;
	y = 396;
	int path_width = 4;
	double radius=5.3256;

	for(int m=-1;m>-(range-3)*5;m--)
	{
		for(int n=-path_width;n<=path_width;n++)
		{	
			double radiustmp=radius;
			if(0)
				radiustmp=radiustmp+n*0.2;
			else
				radiustmp=radiustmp-n*0.2;
			double angle=abs(m*0.2)/radiustmp;
			int xtmp=x+n+radiustmp*(1-cos(angle))*5;
			if(0)
				xtmp=x+n-radiustmp*(1-cos(angle))*5;
			int ytmp=y-radiustmp*sin(angle)*5;

			if(ytmp>511||ytmp<0||xtmp>511||xtmp<0)
				continue;
			if(map->MapPoint[ytmp][xtmp] == 8 || map->MapPoint[ytmp][xtmp] == 18 || map->MapPoint[ytmp][xtmp] == 28)
			{
				return abs((m-15)/5.0);
			}
		}
	}
	
	return 0;

}
double CControl::SearchleftFrontOb(I_Map *map,double range)//you前方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=-1;m>-4*5;m--)
		for(int n=-2;n<=2;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18 ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);
			}
		}

	x = 256;
	y = 396;
	int path_width = 4;
	double radius=5.3256;

	for(int m=-1;m>-(range-3)*5;m--)
	{
		for(int n=-path_width;n<=path_width;n++)
		{	
			double radiustmp=radius;
			if(1)
				radiustmp=radiustmp+n*0.2;
			else
				radiustmp=radiustmp-n*0.2;
			double angle=abs(m*0.2)/radiustmp;
			int xtmp=x+n+radiustmp*(1-cos(angle))*5;
			if(1)
				xtmp=x+n-radiustmp*(1-cos(angle))*5;
			int ytmp=y-radiustmp*sin(angle)*5;

			if(ytmp>511||ytmp<0||xtmp>511||xtmp<0)
				continue;
			if(map->MapPoint[ytmp][xtmp] == 8 || map->MapPoint[ytmp][xtmp] == 18 || map->MapPoint[ytmp][xtmp] == 28)
			{
				return abs((m-15)/5.0);
			}
		}
	}
	
	return 0;

}

