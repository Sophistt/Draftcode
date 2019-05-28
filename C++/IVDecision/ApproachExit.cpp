#include "StdAfx.h"
#include "ApproachExit.h"
#include "LaneDriver.h"
#include "UrbanRoadContext.h"
CApproachExit::CApproachExit(void)
{
	temp_map = new I_Map;
}

CApproachExit::~CApproachExit(void)
{
	delete temp_map;
}


void CApproachExit::Approach_intersection()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "�ӽ�·��";
	CvPoint2D64f P;//��ʼ����ӳ���
	double r;//��ʼ���ڵ�ͼ����ϵ����x��������ļн�
	double d = 0;//������ʼ����ǰ�ӳ����룬ÿ�ӳ�һ������-5�����ӳ�10�ף�������dΪ-50.
	double e = 300;//���ù켣����ǰ�ӳ����룬ÿ�ӳ�һ������5�����ӳ�10�ף�������eΪ50.
	double k4;

	if( LeadPoint[seq_num-1].param2 ==3)
		intersection_direction = 3/*fx*/;//1 ֱ�� 2 ��ת 3 ��ת
	else if( LeadPoint[seq_num-1].param2 ==2)
		intersection_direction = 2;
	else
		intersection_direction = 1;
	//startpoint = cvPoint2D64f(31.847466884,117.120801086 );//31.847466884,117.120901086
	//as = 179.206547962;

	lane_appro.GetLaneMap(*CVehicle::vel_Map,*temp_map);
	lane_appro.ProcCenterLane(temp_map,CVehicle::lane_mid_map);
	lane_appro.KeepLane(realtime_Gps,realtime_Dir,MidGpsPoint);
	double dir = m_GpsData.GetAngle(MidGpsPoint[199].x,MidGpsPoint[199].y,MidGpsPoint[180].x,MidGpsPoint[180].y);


	Search_intersection_startpoint(vel_Map,intersection_direction,startpoint,as);//�����as����ʼ���GPS�����
	as = dir;

	app->critical_section.Lock();//��ס
	currentpoint = app->GPS_Point;
	ac = app->GPS_Direction;//��ȡ��ǰλ��

	app->critical_section.Unlock();

	
	
	WayPoint1[0].x = 256;
	WayPoint1[0].y = 412;//��ǰλ���ڵ�ͼ����ϵ�е����꣬�������270��

	WayPoint1[4] = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//����ʼ��GPS����ת��Ϊ��ͼ����
	//r = ((int)(270 + as - ac))%360;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;
	r = 270 + as - ac;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;


	if(r != 270)
	{
		r = rad(r);
	    k4 = tan(r);//����ʼ��ķ����ת��Ϊ��ͼ����ϵ�е�б��

		P.x = WayPoint1[4].x - d*cos(r);
		P.y = WayPoint1[4].y - d*sin(r);//�����ʼ����ӳ���ĵ�ͼ����


		WayPoint1[4] = P;

		solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);//�ڽӽ�·�ڵĹ����У���ǰ�㡢��ʼ����ӳ��㡢����������߶��ϵĲ����ĵ�ͼ����

		if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))//��������£�����ֱ������������ɱ������켣
		{
			WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);
		
			Bezier(WayPoint1,4,MidGpsPoint);//�ڽӽ�·�ڵĹ����У��õ�ǰ�㡢��ʼ����ӳ��㡢����������߶��ϵĲ�������ɵı���������
		}
		else
		{
			solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint);
		}

	}

	else if(r == 270)
	{
		r = rad(r);

		P.x = WayPoint1[4].x - d*cos(r);
		P.y = WayPoint1[4].y - d*sin(r);//�����ʼ����ӳ���ĵ�ͼ����


		WayPoint1[4] = P;

		solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
		WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
		WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
		WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
		WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
		WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

		Bezier(WayPoint1,4,MidGpsPoint);
	}

	bool edge;
	int edgepoint_x,edgepoint_y;
	edge = Search_edge(MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);//�ж����ɵĹ켣�Ƿ����·��
	while(edge == 1)
	{
		//avoid_edge(startpoint, as, MidGpsPoint);
		startpoint = correct_point(startpoint, as);//���ɵĹ켣������·�أ�������·�ڽ�����GPS���꣨����ƽ��1�ף����������ɹ켣

		app->critical_section.Lock();//��ס
		currentpoint = app->GPS_Point;
		ac = app->GPS_Direction;//��ȡ��ǰλ��

		app->critical_section.Unlock();



		WayPoint1[0].x = 256;
		WayPoint1[0].y = 412;//��ǰλ���ڵ�ͼ����ϵ�е����꣬�������270��

		WayPoint1[4] = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//����ʼ��GPS����ת��Ϊ��ͼ����
		//r = ((int)(270 + as - ac))%360;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;
		r = 270 + as - ac;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;


		if(r != 270)
		{
			r = rad(r);
			k4 = tan(r);//����ʼ��ķ����ת��Ϊ��ͼ����ϵ�е�б��

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//�����ʼ����ӳ���ĵ�ͼ����		


			WayPoint1[4] = P;

			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);//�ڽӽ�·�ڵĹ����У���ǰ�㡢��ʼ����ӳ��㡢����������߶��ϵĲ����ĵ�ͼ����

			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))//��������£�����ֱ������������ɱ������켣
			{
				WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);//�ڽӽ�·�ڵĹ����У��õ�ǰ�㡢��ʼ����ӳ��㡢����������߶��ϵĲ�������ɵı���������
			}
			else
			{
				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
				WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);
			}

		}

		else if(r == 270)
		{
			r = rad(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//�����ʼ����ӳ���ĵ�ͼ����


			WayPoint1[4] = P;

			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint);
		}

		edge = Search_edge(MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);
	}


	app->critical_section.Lock();//��ס
	currentpoint = app->GPS_Point;
	ac = app->GPS_Direction;//��ȡ��ǰλ��

	app->critical_section.Unlock();

	CvPoint2D64f m_startpoint = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//����ʼ��GPS����ת��Ϊ��ͼ����
	CvPoint2D64f y_startpoint;//�ӳ��Ĺ켣��
	r = 270 + as - ac;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;
	if(r!=270)
	{
		r = r - 180;
		r = rad(r);
		y_startpoint.x = m_startpoint.x - e*cos(r);
		y_startpoint.y = m_startpoint.y - e*sin(r);
	}
	else if (r==270)
	{
		y_startpoint.x = m_startpoint.x;
		y_startpoint.y = m_startpoint.y - e;
	}
	y_startpoint = m_GpsData.MaptoGPS(currentpoint,ac,y_startpoint);
	double dx = (y_startpoint.x - startpoint.x)/200;
	double dy = (y_startpoint.y - startpoint.y)/200;
	CvPoint2D64f y_MidGpsPoint[400];
	for(int i =0;i<200;i++)
	{
		y_MidGpsPoint[i] = MidGpsPoint[i];
	}
	for (int j=200;j<400;j++)
	{
		y_MidGpsPoint[j].x = y_MidGpsPoint[j-1].x + dx;
		y_MidGpsPoint[j].y = y_MidGpsPoint[j-1].y + dy;
	}
	edge = Search_edge(y_MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);//�ж��ӳ��Ĺ켣�Ƿ����·��

	while (edge == 1)
	{
		as = as - 1;//������ȡ����ʼ���GPS����ǣ��������ɹ켣

		app->critical_section.Lock();//��ס
		currentpoint = app->GPS_Point;
		ac = app->GPS_Direction;//��ȡ��ǰλ��

		app->critical_section.Unlock();



		WayPoint1[0].x = 256;
		WayPoint1[0].y = 412;//��ǰλ���ڵ�ͼ����ϵ�е����꣬�������270��

		WayPoint1[4] = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//����ʼ��GPS����ת��Ϊ��ͼ����
		//r = ((int)(270 + as - ac))%360;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;
		r = 270 + as - ac;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;


		if(r != 270)
		{
			r = rad(r);
			k4 = tan(r);//����ʼ��ķ����ת��Ϊ��ͼ����ϵ�е�б��

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//�����ʼ����ӳ���ĵ�ͼ����


			WayPoint1[4] = P;

			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);//�ڽӽ�·�ڵĹ����У���ǰ�㡢��ʼ����ӳ��㡢����������߶��ϵĲ����ĵ�ͼ����

			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))//��������£�����ֱ������������ɱ������켣
			{
				WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);//�ڽӽ�·�ڵĹ����У��õ�ǰ�㡢��ʼ����ӳ��㡢����������߶��ϵĲ�������ɵı���������
			}
			else
			{
				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
				WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);
			}

		}

		else if(r == 270)
		{
			r = rad(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//�����ʼ����ӳ���ĵ�ͼ����


			WayPoint1[4] = P;

			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = currentpoint;//�ֽ��������ĵ�ͼ����ת��ΪGPS����
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint);
		}

		m_startpoint = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//����ʼ��GPS����ת��Ϊ��ͼ����
		r = 270 + as - ac;//����ʼ��ķ����ת��Ϊ��ͼ����ϵ����x��������н�;
		if(r!=270)
		{
			r = r - 180;
			r = rad(r);
			y_startpoint.x = m_startpoint.x - e*cos(r);
			y_startpoint.y = m_startpoint.y - e*sin(r);
		}
		else if (r==270)
		{
			y_startpoint.x = m_startpoint.x;
			y_startpoint.y = m_startpoint.y - e;
		}
		y_startpoint = m_GpsData.MaptoGPS(currentpoint,ac,y_startpoint);
		dx = (y_startpoint.x - startpoint.x)/200;
		dy = (y_startpoint.y - startpoint.y)/200;
		y_MidGpsPoint[400];
		for(int i =0;i<200;i++)
		{
			y_MidGpsPoint[i] = MidGpsPoint[i];
		}
		for (int j=200;j<400;j++)
		{
			y_MidGpsPoint[j].x = y_MidGpsPoint[j-1].x + dx;
			y_MidGpsPoint[j].y = y_MidGpsPoint[j-1].y + dy;
		}
		edge = Search_edge(y_MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);//�ж��ӳ��Ĺ켣�Ƿ����·��
	}
	MidGpsPoint[199] = y_startpoint;
	Bezier(MidGpsPoint,199,MidGpsPoint);
	

	/*��ʱ�����Ǻ��̵����*/

	//�����̵�;
	//if(����P && ���)
	//{
	//	ͣ����ʼ��;
	//	�����̵�;
	//	if(���)
	//	{
	//		wait();
	//		�����̵�;
	//	}
	//}
	//if(����P && �̵�)
	//{
	//	Intersection_Driver();
	//}

	//if(P.y >= 412)//������ʼ��ǰ3�ף��л�����·��״̬
	//{
	//    Intersection_Driver();
	//}

}



void CApproachExit::solvepoints(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//��ʼλ�˺�Ŀ��λ�������һ������£��ɵ�ǰλ�˺���ֹλ������������ɱ��������ߵ�����������
{
	P3.x = 256;
	P3.y = a2*256 - a2*P2.x + P2.y;
	P4.x = (P3.x + P1.x)/2;
	P4.y = (P3.y + P1.y)/2;
	P5.x = (P3.x + P2.x)/2;
	P5.y = (P3.y + P2.y)/2;
}

void CApproachExit::solvepoints1(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//��ʼλ�˺�Ŀ��λ������ڶ�������£��ɵ�ǰλ�˺���ֹλ������������ɱ��������ߵ�����������
{
	P3.x = (256 + P2.x)/2;
	P3.y = (412 + P2.y)/2;
	P4.x = 256;
	P4.y = P3.y;
	P5.x = (a2*P2.x - P2.y + P3.x/a2 + P3.y)/(a2 + 1/a2);
	P5.y = a2*(P5.x - P2.x) + P2.y;
}

void CApproachExit::solvepoints2(CvPoint2D64f P1,CvPoint2D64f P2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//��ʼλ�˺�Ŀ��λ���������������£��ɵ�ǰλ�˺���ֹλ������������ɱ��������ߵ�����������
{
	P3.x = (256 + P2.x)/2;
	P3.y = (412 + P2.y)/2;
	P4.x = 256;
	P4.y = P3.y;
	P5.x = P2.x;
	P5.y = P3.y;
}

void CApproachExit::Search_intersection_startpoint(I_Map *map, int t,CvPoint2D64f &qqq,double &kkk)//t:ת��0��ת1ֱ��2��ת��qqq����ȡ�ĵ㣻kkk����ȡ���GPS�����,S:��ʾ������Ƿ���90����270�ȣ�0���ǣ�1��
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();//��ס
	/*currentpoint = app->GPS_Point;
	ac = app->GPS_Direction;*/

	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();

	CvPoint2D64f Q1,Q2;
	double U;//��ȡ��б��
	double V;//��ȡ�ĵ��ڵ�ͼ�����еķ����
	int S;//S=0�����ȡ�ĵ��ڵ�ͼ�����еķ���ǲ���270�Ȼ���90�ȣ�1�������270�Ȼ���90��

	if(t==3)//��ת
	{
		for (int j=0;j<412;j++)
		{
			for(int i=512;i>0;i--)
			{
				if(map->MapPoint[j][i] == 1 || map->MapPoint[j][i]==11)
				{
					Q1.x = i;
					Q1.y = j;
					goto search11_over;
				}
			}
		}
search11_over:
		for(int j=0;j<412;j++)
		{
			for(int i=0;i<512;i++)
			{
				if(map->MapPoint[j][i] == 2 || map->MapPoint[j][i]==12)
				{
					Q2.x = i;
					Q2.y = j;
					goto search12_over;
				}
			}
		}
	}
search12_over:
	if(t==1)//ֱ��
	{
		for(int j=0;j<412;j++)
		{
			for(int i=512;i>0;i--)
			{
				if(map->MapPoint[j][i] == 2 || map->MapPoint[j][i]==12)
				{
					Q1.x = i;
					Q1.y = j;
					goto search21_over;
				}
			}
		}
search21_over:
		for(int j=0;j<412;j++)
		{
			for(int i=0;i<512;i++)
			{
				if(map->MapPoint[j][i] == 3 || map->MapPoint[j][i]==13)
				{
					Q2.x = i;
					Q2.y = j;
					goto search22_over;
				}
			}
		}
	}
search22_over:
	if(t==2)//��ת
	{
		for(int j=0;j<412;j++)
		{
			for(int i=512;i>0;i--)
			{
				if(map->MapPoint[j][i] == 2 || map->MapPoint[j][i]==12)
				{
					Q1.x = i;
					Q1.y = j;
					goto search31_over;
				}
			}
		}
search31_over:
		for(int j=0;j<412;j++)
		{
			for(int i=0;i<512;i++)
			{
				if(map->MapPoint[j][i] == 3 || map->MapPoint[j][i]==13)
				{
					Q2.x = i;
					Q2.y = j;
					goto search32_over;
				}
			}
		}
	}
search32_over:
	qqq.x = (Q1.x +Q2.x)/2;
	qqq.y = (Q1.y +Q2.y)/2;

	qqq = m_GpsData.MaptoGPS(m_gps,m_gpsdir,qqq);//ת��ΪGPS����


	//����С���˷���ϵ�һ�������ߵ�б��
	int X[20]={0};
	int Y[20]={0};
	int n=0;
	double x_sum_average=0;//����X20��Ԫ�ص�ƽ��ֵ
	double y_sum_average=0;//����Y20��Ԫ�ص�ƽ��ֵ
	double x_square_sum=0;//����X20��Ԫ�ص�ƽ����
	double x_multiply_y=0;//����X��Y��ӦԪ�س˻�֮��

	for (int j=1;j<512;j++)
	{
		for(int i=1;i<512;i++)
		{
			if(map->MapPoint[j][i] == 1 || map->MapPoint[j][i] == 11)
			{
				X[n]=i;
				Y[n]=j;
				n++;	
				if(n>19)
				{
					goto search_roadline_over;
				}
				break;
			}
		}
	}
search_roadline_over:
	for(int i=0;i<n;i++)
	{
		x_sum_average = x_sum_average + X[i];
		y_sum_average = y_sum_average + Y[i];
		x_square_sum = x_square_sum + X[i]*X[i];
		x_multiply_y = x_multiply_y + X[i]*Y[i];
	}
	x_sum_average = x_sum_average/n;
	y_sum_average = y_sum_average/n;

	if((x_square_sum - n * x_sum_average * x_sum_average)!=0)
	{
		U = (x_multiply_y - n * x_sum_average * y_sum_average)/(x_square_sum - n * x_sum_average * x_sum_average);
		S=0;
	}
	else if((x_square_sum - n * x_sum_average * x_sum_average)==0)
	{
		S=1;
	}


	if(S == 0)
	{
		V = atan(U)*180/3.1415926;
		if(V>=0)
		{
			V = V+180;
		}
		else if(V<0)
		{
			V = V+360;
		}
	}
	else if(S == 1)
	{
		V = 270;
	}

	kkk = V + m_gpsdir - 270;//ת��ΪGPS�����
}

bool CApproachExit::Search_edge(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &edge_x,int &edge_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	app->critical_section.Lock();//��ס
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

	int m_up = 412-up*10;
	int m_down = 411 -down*10;

	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<6;m++)
			for(int n=-10;n<10;n++)
			{
				if(y+m>360&&y+m<430)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 18)
				{
					edge_x = x+n;
					edge_y = y+m;
					return true;
			
				}
			}
	}
	return false;
}

//int CApproachExit::avoid_edge(CvPoint2D64f s_gps, double s_gpsdir,CvPoint2D64f WayPoint[])//�˴��Ĳ�����moveway�������岻ͬ��s_gps��s_gpsdir��·�����������ͷ��򣬱������ڹ켣֮�ϣ����켣����ƽ��1��(�ڵ�ͼ������Ϊ5��)
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	
//	app->critical_section.Lock();//��ס
//	CvPoint2D64f m_gps = app->GPS_Point;
//	double m_gpsdir = app->GPS_Direction;
//	
//	app->critical_section.Unlock();
//
//	s_gps = m_GpsData.APiontConverD(m_gps,s_gps,m_gpsdir);
//	s_gpsdir = s_gpsdir + 270 - m_gpsdir - 180;
//	s_gpsdir = rad(s_gpsdir);
//
//	CvPoint2D64f s1_gps;
//	s1_gps.x = (int)( s_gps.x - 5*sin(s_gpsdir) );
//	s1_gps.y = (int)( s_gps.y + 5*cos(s_gpsdir) );//ƽ��1��
//
//	s1_gps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,s1_gps);
//	s_gps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,s_gps);
//	
//	double dx = s_gps.x - s1_gps.x;
//	double dy = s_gps.y - s1_gps.y;
//	
//	//outn1<<"ƫ��"<<dx*100000<<", "<<dy*100000<<endl;
//	for(int i=0;i<200;i++)
//	{
//		WayPoint[i].x -= dx;
//		WayPoint[i].y -= dy;
//
//	}
//	return 1;
//}


CvPoint2D64f CApproachExit::correct_point(CvPoint2D64f s_gps, double s_gpsdir)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		
	app->critical_section.Lock();//��ס
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
		
	app->critical_section.Unlock();
	
	s_gps = m_GpsData.APiontConverD(m_gps,s_gps,m_gpsdir);
	s_gpsdir = s_gpsdir + 270 - m_gpsdir - 180;
	s_gpsdir = rad(s_gpsdir);
	
	CvPoint2D64f s1_gps;
	s1_gps.x = (int)( s_gps.x - 5*sin(s_gpsdir) );
	s1_gps.y = (int)( s_gps.y + 5*cos(s_gpsdir) );//ƽ��1��

	s1_gps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,s1_gps);

	return s1_gps;
}

