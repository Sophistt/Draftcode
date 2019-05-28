#include "StdAfx.h"
#include "Vlist.h"

#include "My_Timer.h"

#include <fstream>
#include <time.h>
#define  DIFFX 100                   //种类最大值
#define  DIFFY 100                   //列表中最大障碍物数
#define  DOWNLIMIT 10
#define  UPLIMIT   2000
#define  DISTANCE 3
#define  HDL_IBEO_DISTANCE 13
#define  DELTA 3
//ofstream outlist("outlist.txt");
//ofstream listtime("listtime.txt");
//ofstream outdir("dir.txt");
//My_Timer up_timer;
//ofstream out_error("error_obstacle.txt");//存物数据不正常的储障碍数据
//ofstream out_dataerror("data_error.txt");//存储中心点位置错误的障碍物
ofstream ibeo_data("ibeodata.txt");  //记录所有的ibeo障碍物信息
ofstream problem("problem.txt");

Vlist::Vlist(void)
{
	 l_variancex=10;
	 l_variancey=10;
	 r_variancex=10;
	 r_variancey=10;
	 m_lIsBelievable=1;
	 m_rIsBelievable=1;
	 m_imap=new I_Map;
	 for(int i = 0; i< 512; i++)
	{
		r_roadedge[i].x = 0;
		r_roadedge[i].y = 0;
		l_roadedge[i].x = 0;
		l_roadedge[i].y = 0;
	}


}

Vlist::~Vlist(void)
{
	delete m_imap;
}

double Vlist::ComparaeOb(DyObstacle listob,DyObstacle newob)
{	
//int k=0;
//    while(k<10)
//	{
//		if (listob.GetArea(k)!=0)
//		{
//			break;
//		}
//		k++;
//	}
// 
//	ASSERT(k>=0&&k<10);
//	double distance=m_GpsData.GetDistance(listob.GetCenter(k).x,listob.GetCenter(k).y,newob.GetCenter().x,newob.GetCenter().y);
//	ASSERT(distance)
//	
//	if (listob.GetObCount()<10)
//	{
//		int time=listob.GetObCount()+k-9;
//		ASSERT(time>0&&time<=10);
//        double  dif=distance/time;
//		ASSERT(distance<80);
//		return dif;  //返回中心距离	
//	}
//	if (listob.GetObCount()>9)
//	{
//		int time=k+1;
//		ASSERT(time>0&&time<=10);
//		double dif=distance/time;
//		return dif;
//	}
		DyObstacle ob1=listob;
		DyObstacle  ob2=newob;

		CvPoint2D64f center1;
		CvPoint2D64f center2;
		center1.x=0;
		center1.y=0;
		center2.x=0;
		center2.y=0;
		double dx1=0;
		double dy1=0;
		double dx2=0;
		double dy2=0;
		double distance;

		int k=0;
		int area1=0;
		while(k<10)
		{
			area1=ob1.GetArea(k);
			if (area1!=0)
			{
				break;
			}
			k++;
		}
		CvPoint2D64f p[SIZE]={0};
		ob1.GetPos(k,p);

		for (int i=0;i<area1;i++)
		{	
			center1.x=center1.x+p[i].x;
			center1.y=center1.y+p[i].y;	
		}
		dx1=center1.x/area1;
		dy1=center1.y/area1;

		int area2=ob2.GetArea();

		/*if (area2-area1>30)
		{
		return 100;
		}*/
		CvPoint2D64f m[SIZE]={0};
		ob2.GetPos(m);
		for (int j=0; j<area2;j++)
		{
			center2.x=center2.x+m[j].x;
			center2.y=center2.y+m[j].y;
		}
		dx2=center2.x/area2;                        //第一个障碍物的中心位置
		dy2=center2.y/area2;                        //第二个障碍物的中心位置
		/*DISTANCE=fabs(dx1-dx2)*10+fabs(dy1-dy2);
		areadiff=fabs(ob1.GetArea()-ob2.GetArea());

		diffenrence=DISTANCE+areadiff;
		return DISTANCE;*/
		int s=0;
		for (int i=0; i<SIZE; i++)
		{
			if (p[i].x!=m[i].x||p[i].y!=m[i].y)
			{
				s++;
			}
		}
		distance=m_GpsData.GetDistance(dx1,dy1,dx2,dy2);
		ASSERT(distance<80);
		/*double ob_dir=CountDir(ob2,ob1);*/
		/*if (ob1.GetMovingDir()-ob_dir>25)
		{
		return 100;
		}*/
		if (ob1.GetObCount()<10)
		{
			double  dif=sqrt(distance/(ob1.GetObCount()+k-9));
			return dif;  //返回中心距离
		}
		if (ob1.GetObCount()>9)
		{
			double dif=distance/(k+1);
			return dif;
		}
}


void Vlist::Update(I_Map *map)
{

}

void Vlist::SearchRLine(CvPoint2D64f (&line)[512],int &l_num)
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();

	for (int i=450; i>0; i--)
	{
		for (int j=400; j>256; j--)
		{
			if (m_imap->MapPoint[i][j]==18)
			{
				line[l_num].x=j;
				line[l_num].y=i;
				l_num++;
				break;
			}
		}
	}

}

void Vlist::SearchLLine(CvPoint2D64f (&line)[512],int &l_num)
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();


	for (int i=450; i>0; i--)
	{
		for (int j=256; j>100; j--)
		{
			if (m_imap->MapPoint[i][j]==18)
			{
				line[l_num].x=j;
				line[l_num].y=i;
				l_num++;
				break;
			}
		}
	}
}

void Vlist::CountLine(CvPoint2D64f *line,int l_num,double &k,double &b)
{
	double sumx=0;
	double sumy=0;
	double xmean=0;
	double ymean=0;
	double sumxx=0;
	double sumxy=0;

	for (int i=0; i<l_num; i++)
	{
		sumx+=line[i].x;
		sumy+=line[i].y;
	}
	xmean=sumx/l_num;
	ymean=sumy/l_num;

	for (int i=0; i<l_num; i++)
	{
		sumxx+=(line[i].y-xmean)*(line[i].y-xmean);
		sumxy+=(line[i].y-xmean)*(line[i].x-ymean);
	}

	k=sumxx/sumxy;
	b=ymean-k*xmean;

}

void Vlist::ObInRoad()
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock(); 



	bool r_IsVertical=0;
	bool l_IsVertical=0;

	/************************************************************************/
	/* 右侧路沿拟合                                                                     */
	/************************************************************************/
	CvPoint2D64f r_line[512]={0};
	int r_num=0;

	
	
	SearchRLine(r_line,r_num);
	ASSERT(r_num>=0&&r_num<=512);
	if (r_num>30)
	{
		r_variancex=0;
		r_variancey=0;

		CountLine(r_line,r_num,m_rk,m_rb);
		for (int i=0; i<r_num; i++)
		{
			r_meanx+=r_line[i].x;
			r_meany+=r_line[i].y;
			/*outrline<<r_line[i].x<<"\t"<<r_line[i].y<<endl;*/
		}
		r_meanx=r_meanx/r_num;
		r_meany=r_meany/r_num;

		for (int i=0; i<r_num; i++)
		{
			r_variancex=(r_line[i].x-r_meanx)*(r_line[i].x-r_meanx);
			r_variancey=(r_line[i].y-r_meany)*(r_line[i].y-r_meany);
		}
		r_variancex=r_variancex/r_num;
		r_variancey=r_variancey/r_num;
	}

	/*outvariance<<r_variancex<<"\t"<<r_variancey<<endl;*/
	double r_angle=atan(m_rk)*180/PI;

	if (r_angle<0)
	{
		r_angle=r_angle+180;
	}
	ASSERT(r_angle<180&&r_angle>-180);
	/*outRway<<r_num<<"\t"<<r_angle<<"\t"<<r_variancex<<endl;*/


	/************************************************************************/
	/* 左侧路沿拟合                                                               */
	/************************************************************************/
	CvPoint2D64f l_line[512]={0};
	int l_num=0;
	

	SearchLLine(l_line,l_num);
	ASSERT(l_num>=0&&l_num<=512);
	if (l_num>30)
	{
		l_variancex=0;
		l_variancey=0;
		CountLine(l_line,l_num,m_lk,m_lb);
		for (int i=0; i<l_num; i++)
		{
			l_meanx+=l_line[i].x;
			l_meany+=l_line[i].y;
			/*outrline<<r_line[i].x<<"\t"<<r_line[i].y<<endl;*/
		}
		l_meanx=l_meanx/l_num;
		l_meany=l_meany/l_num;

		for (int i=0; i<l_num; i++)
		{
			l_variancex=(l_line[i].x-l_meanx)*(l_line[i].x-l_meanx);
			l_variancey=(l_line[i].y-l_meany)*(l_line[i].y-l_meany);
		}
		l_variancex=l_variancex/l_num;
		l_variancey=l_variancey/l_num;
	}

	double l_angle=atan(m_lk)*180/PI;

	if (l_angle<0)
	{
		l_angle=l_angle+180;
	}
	/* if (m_lk>5000)
	{
	l_angle=90;
	} */
	ASSERT(l_angle<180&&l_angle>-180);
	/*outLway<<l_num<<"\t"<<l_angle<<"\t"<<l_variancex<<endl;*/


	//if (r_angle==0&&l_angle!=0)
	//{
	//	m_rk=m_lk;
	//}
	//if (l_angle==0&&r_angle!=0)
	//{
	//	m_lk=m_rk;
	//}


    CList<DyObstacle,DyObstacle> m_dylist;
	POSITION pos=fusionlist.GetHeadPosition();
	int i=fusionlist.GetCount();
	if (i>0)
	{
		int ssssss=0;
	}
	while (pos!=NULL)
	{
		
		DyObstacle obstacle=fusionlist.GetNext(pos);
		CvPoint2D64f center=obstacle.GetIbeoCenter();
		center=m_GpsData.APiontConverD(m_gps,center,dir);
		int center_x=(center.x*10+5)/10;
		int center_y=(center.y*10+5)/10;

		if (r_variancex<3/*&&m_rk!=0*//*&&r_num>10*/)
		{
			r_IsVertical=1;
		}
		if (l_variancex<3/*&&m_lk!=0*//*&&l_num>10*/)
		{
			l_IsVertical=1;
		}



		if (r_IsVertical==1&&l_IsVertical==1)                //都垂直
		{
			if (center_x-l_meanx>10&&r_meanx-center_x>10)
			{
				m_dylist.AddTail(obstacle);
			}
			/*if (obstacle.GetObCount()>5&&l_meanx-center_x>0)
			{
				m_lIsBelievable=0;
			}*/
		}
		if (r_IsVertical==1&&l_IsVertical==0&&m_lk!=0)     // 右边垂直，左边不垂直，斜率不为零
		{
			if (r_meanx-center_x>10&&center_x-(center_y-m_lb)/m_lk>10)
			{
				m_dylist.AddTail(obstacle);
			}
		}

		if (r_IsVertical==1&&l_IsVertical==0&&m_lk==0)    //右侧垂直，左边不垂直，斜率为零
		{
			if (r_meanx-center_x>10)
			{
				m_dylist.AddTail(obstacle);
			}
		}


		if (r_IsVertical==0&&l_IsVertical==1&&m_rk!=0) //右边不垂直，斜率不为零，左边垂直
		{
			if ((center_y-m_rb)/m_rk-center_x>10&&center_x-l_meanx>10)
			{
				m_dylist.AddTail(obstacle);
			}
		}

		if (r_IsVertical==0&&l_IsVertical==0&&m_rk!=0&&m_lk!=0)   //左右都不垂直，斜率都不为零
		{
			if ((center_y-m_rb)/m_rk-center_x>10&&center_x-(center_y-m_lb)/m_lk>10)
			{
				m_dylist.AddTail(obstacle);
			}
		}
		if (r_IsVertical==0&&l_IsVertical==0&&m_rk==0&&m_lk!=0)   //左右都不垂直，右侧斜率为零，左侧不为零
		{
			if (center_x-(center_y-m_lb)/m_lk>10)
			{
				m_dylist.AddTail(obstacle);
			}
		}

		if (r_IsVertical==0&&l_IsVertical==0&&m_rk!=0&&m_lk==0)  //左右都不垂直，右侧斜率不为零，左侧为零
		{
			if ((center_y-m_rb)/m_rk-center_x>10)
			{
				m_dylist.AddTail(obstacle);
			}
		}

		if (r_IsVertical==0&&m_rk==0&&l_IsVertical==1)          //左边垂直，右边没有道路边沿
		{
			if (center_x-l_meanx>10)
			{
				m_dylist.AddTail(obstacle);
			}
		}
		//if (r_IsVertical==0&&m_rk==0&&l_IsVertical==0&&m_rk!=0) //左边不垂直，右边没有道路边沿
		//{
		//	if (center_x-(center_y-m_lb)/m_lk>5)
		//	{
		//		m_dylist.AddTail(obstacle);
		//	}
		//}
		//if (r_IsVertical==1&&l_IsVertical==0&&m_lk==0)          //右侧垂直，左侧无道路边沿
		//{
		//	if (r_meanx-center_x>5)
		//	{
		//		m_dylist.AddTail(obstacle);
		//	}
		//}
		/*if (r_IsVertical==0&&m_rk!=0&&l_IsVertical==0&&m_lk==0)
		{
			if ()
			{
			}
		}*/
		if (r_IsVertical==0&&l_IsVertical==0&&m_rk==0&&m_lk==0)   //左右都不垂直，左右斜率都为零
		{
			m_dylist.AddTail(obstacle);
		}
	}
	

	fusionlist.RemoveAll();
	POSITION newpos=m_dylist.GetHeadPosition();
	while (newpos!=NULL)
	{
		DyObstacle newobstacle=m_dylist.GetNext(newpos);
		fusionlist.AddTail(newobstacle);
	}

}

void Vlist::ObInPath()
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	CList<DyObstacle,DyObstacle> temp_list;
	CvPoint2D64f path[200]={0};
	int length=0;
	app->critical_planningroad.Lock();
	length=planning_road->total;
    /*pt = *(CvPoint2D64f*)cvGetSeqElem( m_roadseq, n-1 );
	planning_road->delta_elems*/
	for (int i=0; i<200; i++)
	{
		path[i]=*(CvPoint2D64f*)cvGetSeqElem( planning_road, i);
	}
	app->critical_planningroad.Unlock();    

	POSITION pos=newlist.GetHeadPosition();
	while (pos!=NULL)
	{
		DyObstacle obstacle=newlist.GetNext(pos);
		CvPoint2D64f ob_point=obstacle.GetCenter();
		int size=obstacle.GetArea();
		double nearest=200;
		for (int index=0; index!=length; index++)
		{
			double distance=m_GpsData.GetDistance(path[index].x,path[index].y,ob_point.x,ob_point.y);
			if (nearest>distance)
			{
				nearest=distance;
			}
		}

		if (nearest<5)
		{
			temp_list.AddTail(obstacle);
		}
	}
    newlist.RemoveAll();
	POSITION temp_pos=temp_list.GetHeadPosition();
	while (temp_pos!=NULL)
	{
		DyObstacle obstacle=temp_list.GetNext(temp_pos);
		newlist.AddTail(obstacle);
	}
	
}

//DyObstacle Vlist::FindDyobstacle(CvPoint2D64f point)
//{
//	POSITION pos=newlist.GetHeadPosition();
//	while(pos!=NULL)
//	{
//		CvPoint2D64f ob_point;
//		CvPoint2D64f ob_points[2000];
//		DyObstacle obstacle=newlist.GetNext(pos);
//		obstacle.GetPos(ob_points);
//		for (int i=0; i<obstacle.GetArea(); i++)
//		{
//			if (ob_points[i].x==point.x&&ob_points[i].y==point.y)
//			{
//				return obstacle;
//			}
//		}
//
//	}
//}

void Vlist::Update_IbeoList(ObjectWT *ObjectWT1 ,int num)     
{
	/*CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	double current_dir=app->GPS_Direction;
	CvPoint2D64f current_pos=app->GPS_Point;
	double current_v=m_GpsData.GetSpeed();
	app->critical_section.Unlock();*/
    //problem<<1<<endl;
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();   //读取汽车方向，坐标，速度
	app->critical_section.Lock();
	xinhuang_dir=app->GPS_Direction;
	xinhuang_pos=app->GPS_Point;
	double current_dir = xinhuang_dir;
	CvPoint2D64f current_pos = xinhuang_pos;
	double current_v=m_GpsData.GetSpeed();
	app->critical_section.Unlock();
	

//for (int i=0; i<num; i++)//存储ibeo数据
//	{
//		int ob_ID=ObjectWT1[i].Object_Id; //编号
//
//		int ob_length_x=ObjectWT1[i].ObjectBox_SizeX;//框大小
//		int ob_length_y=ObjectWT1[i].ObjectBox_SizeY;
//
//		int ob_center_x=ObjectWT1[i].ObjectBox_CenterX;//中心坐标
//		int ob_center_y=ObjectWT1[i].ObjectBox_CenterY;
//
//		double ob_vx=ObjectWT1[i].Relative_VelocityX;//速度分量
//		double ob_vy=ObjectWT1[i].Relative_VelocityY;
//
//		int ob_age=ObjectWT1[i].Object_Age; //存在时间
//		int ob_orientation=ObjectWT1[i].ObjectBox_orientation;//方向
//
//		ob_vx=ob_vx/3.6;
//		ob_vy=ob_vy/3.6;
//
///************************************************************************/
///*         将ibeo坐标转化为hdl坐标方便观察                                                                     */
///************************************************************************/
//		ob_center_x=256-ob_center_x/20;
//		ob_center_y=412-HDL_IBEO_DISTANCE-ob_center_y/20;
//		//ibeo_data<<current_v<<"\t"<<current_dir<<"\t"<<ob_ID<<"\t"<<ob_length_x<<"\t"<<ob_length_y<<"\t"<<ob_center_x<<"\t"<<ob_center_y<<"\t"<<ob_vx<<"\t"<<ob_vy<<"\t"<<ob_age<<"\t"<<ob_orientation<<endl;
//	}

/****************************************
    将上一帧的ibeolist存入temp_list
*****************************************/
	temp_list.RemoveAll();
	POSITION temp_pos=ibeolist.GetHeadPosition();
	while(temp_pos!=NULL)
	{
		DyObstacle obstacle = ibeolist.GetNext(temp_pos);
		temp_list.AddTail(obstacle);
	}

	ibeolist.RemoveAll();       //移除上一帧的数据

	
	for (int i=0; i!=num; i++)
	{
		ObjectWT myobject=ObjectWT1[i];  //获取第i个障碍物
		DyObstacle obstacle;             //定义存储障碍物的存储单元
		int kind = ObjectWT1[i].Classification;
		//obstacle.SetObstacleKind(kind);
		//ibeo_data<<ObjectWT1[i].Object_Id<<"    "<<ObjectWT1[i].Classification<<endl;
		//if (kind == 3)
		//{
		//	obstacle.IsMan = 1;
		//	//obstacle.SetObstacleKind(3);
		//}

		CvPoint2D64f center={0};//中心点
		CvPoint2D64f ob_center={0};  //存储障碍物中心点地图坐标
		int area=0;
		
		center.x=ObjectWT1[i].ObjectBox_CenterX/20;
		center.y=ObjectWT1[i].ObjectBox_CenterY/20;
		
		int danger = ObjectWT1[i].danger;

		CvPoint2D64f jilucenter = center; //障碍物中心在ibeo坐标系中的坐标
		
		
		Transfer(center);
        /************************************************************************/
        /* 通过实验添加矫正参数                                                                     */
        /************************************************************************/
		center.x = center.x+3;
		center.y = center.y+3;

		CvPoint2D64f obstacle_map_center=center;
		

		double length_x=ObjectWT1[i].ObjectBox_SizeX/20;//框大小
		double length_y=ObjectWT1[i].ObjectBox_SizeY/20;

		obstacle.SetOriX(length_x);
		obstacle.SetOriY(length_y);
		if (ObjectWT1[i].ObjectBox_SizeX<100)//ibeo的精度比较高，框的大小可能小于20cm
		{
			length_x=5;
		}
		if (ObjectWT1[i].ObjectBox_SizeY<100)
		{
			length_y=5;
		}

		if(kind != 3)
		{
		length_x=length_x*8;
		length_y=length_y*6;
		}
		
		if(length_x>50)       //一般的障碍物大小不会长不会超过10m，宽不会超过5m
		{
			length_x=75;
		}
		if(length_y>25)
		{
			length_y=38;
		}


		double test_x=ObjectWT1[i].ObjectBox_SizeX; //为了过滤大小很小的框而定义的框大小
		double test_y=ObjectWT1[i].ObjectBox_SizeY;
		/*if(test_x==0&&test_y==0)
		{
			int ssssss=0;
		}*/
		/*if(length_x==0)
		{
			double tttt;
		}*/

		int id=ObjectWT1[i].Object_Id;//编号
		ASSERT(id>=0);
		obstacle.SetObNum(id);

		int age=ObjectWT1[i].Object_Age;//存在时间，即存在置信度
		//ASSERT(age!=0);
		obstacle.SetMovingConfidence(age);
		/*if (age>2)
		{
			int aaage=0;
		}*/
		/*if(age>4)
		{
			obstacle.SetExistConfidence(10);
		}*/

		double dir=ObjectWT1[i].ObjectBox_orientation;//相对于本车的航向,还需要转换
		double  compare_v;
		double  juedui_v;
		double mydir;
	
		mydir=dir;
		//将ibeo数据中的航向转换为gps航向
		dir = -dir;
		if (dir<0)
		{
			dir=dir+360;
		}
		dir=dir-90;
		if (dir<0)
		{
			dir=dir+360;
		}
		double my_dir=dir;
		obstacle.SetRelativaDir(my_dir); //地图中的角度  
		
		/*if (my_dir111<30||my_dir222<30||my_dir333<30)
		{
			
		}*/
		dir=dir+current_dir-270;
		/*dir=dir+180-270;*/
		if (dir<0)
		{
			dir=dir+360;
		}
		if (dir>=360)
		{
			dir=dir-360;
		}
	/*	if((70<dir)&&(dir<110))
		{
			int aaaaaa=1;
		}
		if((250<dir)&&(dir<290))
		{
			int bbbbbbb=1;
		}*/
		

		double velocity_x=ObjectWT1[i].Relative_VelocityX/3.6;//相对速度，需要转换
		double velocity_y=ObjectWT1[i].Relative_VelocityY/3.6;
		/*if (velocity_x!=0)
		{
			int aaaaa=0;
		}*/
		/*if (velocity_y!=0)
		{
			int sssss=0;
		}*/
		/*if ((velocity_x!=0)&&(velocity_y!=0))
		{
			int ttttttttttttt=0;
		}*/

	//	double v=sqrt(velocity_x*velocity_x+velocity_y*velocity_y);
		//double  v=sqrt(velocity_x*velocity_x+(velocity_y+current_v)*(velocity_y+current_v));
		//current_v = 2;
		double  v=sqrt(velocity_y*velocity_y+(velocity_x+current_v)*(velocity_x+current_v));
		compare_v=sqrt(velocity_x*velocity_x+velocity_y*velocity_y);
		obstacle.SetRelativeSpeed(compare_v);
		obstacle.SetDanger(compare_v);
		juedui_v = v;



		double vx_bei=velocity_x*cos(rad(xinhuang_dir));
		double vx_dong=velocity_x*sin(rad(xinhuang_dir));
		double vy_bei=velocity_y*cos(rad(xinhuang_dir-90));
		double vy_dong=velocity_y*sin(rad(xinhuang_dir-90));

		double vv_dong=app->GPS_EastSpeed;
		double vv_bei=app->GPS_NorthSpeed;
		double vv_hecheng = app->GPS_Speed;
		if(vv_hecheng<1)
		{
			vv_dong = 0;
			vv_bei = 0;
		}

		double juedui_sudu_dong = vx_dong+vy_dong+vv_dong;
		double juedui_sudu_bei = vx_bei+vy_bei+vv_bei;
		if (juedui_sudu_bei==0)
		{
			juedui_sudu_bei = 0.01;
		}
		double juedui_sudu = sqrt(juedui_sudu_dong*juedui_sudu_dong+juedui_sudu_bei*juedui_sudu_bei);
		double juedui_fangxiang = (atan(juedui_sudu_dong/juedui_sudu_bei))*180/PI;
		if ((juedui_sudu_dong>0)&&(juedui_sudu_bei<0))
		{
			juedui_fangxiang = juedui_fangxiang+180;
		}
		else if((juedui_sudu_dong<0)&&(juedui_sudu_bei<0))
		{
			juedui_fangxiang = juedui_fangxiang+180;
		}
		else if ((juedui_sudu_dong<0)&&(juedui_sudu_bei>0))
		{
			juedui_fangxiang = juedui_fangxiang+360;
		}
		/*if (!(velocity_x==0&&velocity_y==0)&&(juedui_fangxiang == 0))
		{
			int aaaaaaaaaa=1;
		}*/
		
		double juedui_dir;
		if(velocity_y!=0)
		{
			juedui_dir=atan((velocity_x+current_v)/velocity_y);
			if(juedui_dir<0)
			{
				juedui_dir = juedui_dir+PI;
			}
			juedui_dir = juedui_dir+PI;
		}
		else if((velocity_x+current_v)>0)
		{
			juedui_dir=1.5*PI;
		}
		else if((velocity_x+current_v)<0)
		{
			juedui_dir=0.5*PI;
		}
		juedui_dir = juedui_dir*180/PI;
		
		dir=juedui_dir+current_dir-270;
		/*dir=dir+180-270;*/
		if (dir<0)
		{
			dir=dir+360;
		}
		if (dir>=360)
		{
			dir=dir-360;
		}
		//obstacle.SetMovingDir(dir);//加入gps方向
		obstacle.SetMovingDir(juedui_fangxiang);

		double my_dir111=abs(juedui_fangxiang-xinhuang_dir);
		double my_dir222=abs(360-my_dir111);
		//double my_dir333=abs(mydir-360);


		/*double  v=sqrt(velocity_x*velocity_x+velocity_y*velocity_y);*/
		/*double  v=velocity_y;*/
		//obstacle.SetV(v);
		obstacle.SetV(juedui_sudu);

		area=ObjectWT1[i].iContour_Points;//障碍物点个数
		obstacle.SetArea(area);

		double xishu;//回溯系数
		double yanshenjuli;//径向延伸距离，单位：格
		yanshenjuli = 5;//延伸1米试试看
		if(my_dir<110&&my_dir>70)//障碍物向我们开过来，回溯系数设置大一些
		{
			xishu = 1.3;
		}
		else if (my_dir<20||my_dir>340)//障碍物驶离我们，回溯系数设置小一点
		{
			xishu = 0.4;
		}
		else
		{
			xishu = 0.8;
		}

	/*	if(compare_v>12)
		{
			xishu = 0.8;
		}
		else if(compare_v>8)
		{
			xishu = 0.6;
		}
		else
		{
			xishu = 0.3;
		}*/


		//CvPoint2D64f my_pos[2000]={0};                 //轮廓位置
		//for (int j=0; j<area; j++)
		//{
		//	my_pos[j].x=(ObjectWT1[i].iContour_X[j])/20;
		//	my_pos[j].y=(ObjectWT1[i].iContour_Y[j])/20;
		//	/*Transfer(pos[j]);*/
		//	my_pos[j].x=256-my_pos[j].x-compare_v*xishu*cos(my_dir*PI/180);
		//	my_pos[j].y=412-HDL_IBEO_DISTANCE-my_pos[j].y-compare_v*xishu*sin(my_dir*PI/180);
		//	my_pos[j]=m_GpsData.MaptoGPS(current_pos,current_dir,my_pos[j]);
		//}
		//obstacle.SetPos(my_pos);

  /************************************************************************/
  /* 根据障碍物速度方向                                                                     */
  /************************************************************************/
		center.x=center.x-compare_v*xishu*cos(my_dir*PI/180);
		center.y=center.y-compare_v*xishu*sin(my_dir*PI/180);
		double dis_ob=sqrt((center.x-256)*(center.x-256)+(center.y-412)*(center.y-412));
		double bili_xishu=(dis_ob+yanshenjuli)/dis_ob;
		center.x = (center.x-256)*bili_xishu+256;
		center.y = (center.y-412)*bili_xishu+412;
		ob_center=center;


		center=m_GpsData.MaptoGPS(current_pos,current_dir,center);
		obstacle.SetIbeoCenter(center);

		int fangxiang_cha_biaoji = 0;
        double v_change = 0;
		 int box_v_change = 0;

		/************************************************************************/
			/* 记录障碍物框的大小，将框大小设定为历史上最大的尺寸       */
       /************************************************************************/
		POSITION  box_size_pos=temp_list.GetHeadPosition();
		DyObstacle box_obstacle;
		while(box_size_pos!=0)
		{
			box_obstacle=temp_list.GetNext(box_size_pos);
			int box_id=box_obstacle.GetObNum();
			if(box_id==id)
			{
				double temple_fangxiang = box_obstacle.GetMovingDir();
				double fangxiang_cha = abs(juedui_fangxiang-temple_fangxiang);
				if(fangxiang_cha>45&&fangxiang_cha<315)
				{
					fangxiang_cha_biaoji = 1;//此种障碍物不要
				}
/************************************************************************/
/*根据速度大小变化进行障碍物过滤                                                                     */
/************************************************************************/
				double temp_v = box_obstacle.GetV();
				double v_change = temp_v - juedui_sudu;
			    box_v_change = box_obstacle.GetVchange();
				if (v_change > 3)
				{
					box_v_change = box_v_change + 1;
					obstacle.SetVchange(box_v_change);
				}


				
				/*if (box_obstacle.IsMan == 1)
				{
					obstacle.IsMan = 1;
					obstacle.SetObstacleKind(3);
				}*/
ibeo_data<<"#"<<box_obstacle.GetObNum()<<"    "<<box_obstacle.IsMan<<endl;




				double box_length_x=box_obstacle.GetLength_X();
				double box_length_y=box_obstacle.GetLength_Y();
				if(box_length_x>length_x)
				{
					length_x=box_length_x;
				}
				if(box_length_y>length_y)
				{
					length_y=box_length_y;
				}

			  //CvPoint2D64f box_center=box_obstacle.GetIbeoCenter();
			  //int box_age=box_obstacle.GetMovingConfidence();
			  //ASSERT(box_age!=0);
     //         double center_distace=m_GpsData.GetDistance(box_center.x,box_center.y,center.x,center.y);//获取前后两帧中中心点的距离
			  //double center_distace1=m_GpsData.GetDistance(center.x,center.y,box_center.x,box_center.y);
			  //if (center_distace>15||center_distace1>15)
			  //{
				 // out_dataerror<<id<<"\t"<<box_age<<"\t"<<age<<"\t"<<box_center.x<<"\t"<<box_center.y<<"\t"<<center_distace<<"\t"<<center.x<<"\t"<<center.y<<"\t"<<obstacle_map_center.x<<"\t"<<obstacle_map_center.y<<"\t"<<area<<"\t"<<ObjectWT1[i].ObjectBox_SizeX<<"\t"<<ObjectWT1[i].ObjectBox_SizeY<<endl;//输出中心距离、中心gps坐标、障碍物中心地图坐标、障碍物点个数、框大小
			  //}
			}
		}

		/*if(length_x<5)
		{
			length_x=5;
		}
		if(length_y<5)
		{
			length_y=5;
		}*/
		obstacle.SetLength_X(length_x);
		obstacle.SetLength_Y(length_y);

		/************************************************************************/
		/* 这里出现了错误                                                                     */
		/************************************************************************/
		double angle=juedui_fangxiang+270-current_dir;//在地图中的方向

		double b=length_x;
		double a=length_y;
		double r1=atan(a/b)*180/PI;
		double r2=atan(b/a)*180/PI;
		CvPoint2D64f point[4]={0};

		/*point[0].x=ob_center.x-sin((90-angle-r1)*PI/180)*sqrt((a*a+b*b)/4);
		point[0].y=ob_center.y-cos((90-angle-r1)*PI/180)*sqrt((a*a+b*b)/4);*/

		point[0].x=ob_center.x+sin((90-angle-r1)*PI/180)*sqrt((a*a+b*b)/4);
		point[0].y=ob_center.y+cos((90-angle-r1)*PI/180)*sqrt((a*a+b*b)/4);


		point[1].x=ob_center.x+cos((r2-90+angle)*PI/180)*sqrt((a*a+b*b)/4);
		point[1].y=ob_center.y+sin((r2-90+angle)*PI/180)*sqrt((a*a+b*b)/4);

		point[2].x=ob_center.x-sin((90-angle-r1)*PI/180)*sqrt((a*a+b*b)/4);
		point[2].y=ob_center.y-cos((90-angle-r1)*PI/180)*sqrt((a*a+b*b)/4);

		point[3].x=ob_center.x-cos((r2-90+angle)*PI/180)*sqrt((a*a+b*b)/4);
		point[3].y=ob_center.y-sin((r2-90+angle)*PI/180)*sqrt((a*a+b*b)/4);
		for(int i=0;i<4;i++)
		{
			point[i]=m_GpsData.MaptoGPS(current_pos,current_dir,point[i]);
		}



		obstacle.SetBoxPoints(point);
		//obstacle.SetMovingConfidence(10);
	    //obstacle.SetDanger(danger);

		obstacle.SetObCount(1);
		

		//if ((jilucenter.x==0&&jilucenter.y==0)||(test_x==0&&test_y==0)||area==0||(jilucenter.x>1000||jilucenter.x<-1000||jilucenter.y>1000||jilucenter.y<-1000))  //记录原始数据垃圾造成的错误
		//{
		//	out_error<<id<<"\t"<<jilucenter.x<<"\t"<<jilucenter.y<<"\t"<<area<<"\t"<<ObjectWT1[i].ObjectBox_SizeX<<"\t"<<ObjectWT1[i].ObjectBox_SizeY<<endl;
		//}

		int temp_age = 10;
		/*if (app->on_road == 0)
		{
			temp_age = 6;
		}*/

		double mandirflag = 0;

		if ((my_dir111>50&&my_dir111<130)||(my_dir222>50&&my_dir222<130)/*||my_dir333<40*/)
		{
			mandirflag = 1;
		}


		int ibeoare = ObjectWT1[i].iContour_Points;
		int man111 = 0;

		if (mandirflag ==1 && ibeoare >2 &&ibeoare <50 &&obstacle.GetV()>0.5&& obstacle.GetV()<5)
		{
			obstacle.SetObstacleKind(3);
			man111 =1 ;
		}

		/*if (obstacle.IsMan ==1&&obstacle.GetV()>0.5&&obstacle.GetV()<3)
		{
			obstacle.notman_count = 0;
			double man_age = obstacle.man_count;
			man_age++;
			obstacle.man_count = man_age;
		}
		else
		{
			obstacle.man_count = 0;
			double notman_age = obstacle.notman_count;
			notman_age++;
			obstacle.notman_count = notman_age;
		}*/

	/*	if(obstacle.man_count>5&&obstacle.notman_count<20)
		{
			obstacle.SetObstacleKind(3);
		}*/

		if(/*(obstacle.man_count>5&&obstacle.notman_count<20)*/man111 == 1||(man111!=1&&(fangxiang_cha_biaoji==0&&(!(jilucenter.x==0&&jilucenter.y==0))&&(ob_center.x<512&&ob_center.x>=0&&ob_center.y<512&&ob_center.y>=0)&&age>temp_age&&juedui_sudu>1/*&&(age*juedui_sudu)>20*/&&box_v_change < 3)))//对加入链表中的Ibeo障碍物按速度进行初步过滤
		{
			ibeolist.AddTail(obstacle);//将动态障碍物添加入列表
		}	
	}
	/*if(ibeolist.GetCount()==0)
	{
		int aaaaaaa=1;
		num;
		int bbbbbbbbbbb=1;
	}*/


	//bool match=0;
	//POSITION temp_repair_pos=temp_list.GetHeadPosition();
	//while(temp_repair_pos!=NULL)
	//{
	//	match=0;
	//	DyObstacle temp_obstacle=temp_list.GetNext(temp_repair_pos); //从上一帧的ibeolist中取一个障碍物
	//	int temp_index=temp_obstacle.GetObNum();


	//	POSITION ibeo_repair_pos=ibeolist.GetHeadPosition();
	//	while(ibeo_repair_pos!=NULL)
	//	{
	//		DyObstacle ibeo_obstacle=ibeolist.GetNext(ibeo_repair_pos);//从现在的ibeolist中取一个障碍物
	//		int ibeo_index=ibeo_obstacle.GetObNum();

	//		CvPoint2D64f repair_center,new_center; 
	//		repair_center=temp_obstacle.GetIbeoCenter();
	//		new_center = ibeo_obstacle.GetIbeoCenter();
	//		double xinshu1=1.2;
	//		double temp_dir=temp_obstacle.GetRLDir();
	//		double temp_v=temp_obstacle.GetRLSpeed();
	//		repair_center=m_GpsData.APiontConverD(current_pos,repair_center,current_dir);
	//		repair_center.x=(int)(repair_center.x+0.5+cos((temp_dir/*-current_dir+270*/)*PI/180)*xinshu1*temp_v);
	//		repair_center.y=(int)(repair_center.y+0.5+sin((temp_dir/*-current_dir+270*/)*PI/180)*xinshu1*temp_v);
	//		repair_center=m_GpsData.MaptoGPS(current_pos,current_dir,repair_center);
	//		double distance_cha = m_GpsData.GetDistance(repair_center.x,repair_center.y,new_center.x,new_center.y);

	//		if((temp_index==ibeo_index)&&(distance_cha<15))
	//		{
	//			/*CvPoint2D64f a = ibeo_obstacle.GetCenter();
	//			CvPoint2D64f b = temp_obstacle.GetCenter();				
	//			double c = m_GpsData.GetDistance(a.x,a.y,b.x,b.y);*/
	//		
	//			/*if(c<20)
	//			{
	//				int aaaaaaaaaaaaaaaaa=1;*/
	//				match=1;	
	//			    break;
	//			//}
	//			
	//			
	//		}
	//	}
	//	/************************************************************************/
	//	/* 如果上一帧中的temp_obstacle没有在这一帧中找到编号一样的障碍物，将上一帧的temp_obstacle进行预测处理后填入ibeolist                                                                     */
	//	/************************************************************************/
	//	if(match==0)
	//	{   
	//		/*CvPoint2D64f repair_point[2000]={0};*/
	//		CvPoint2D64f repair_center;            //中心点
	//		CvPoint2D64f repair_box_points[4]={0};  //框定点
	//		DyObstacle repair_obstacle;             //预测得到的障碍物

	//		repair_obstacle.SetObNum(temp_index);   //设定编号
	//		/*temp_obstacle.GetPos(repair_point);*/
	//		

	//		int temp_size=temp_obstacle.GetArea();
	//		repair_obstacle.SetArea(temp_size);      //设定点的数量


	//		double temp_dir=temp_obstacle.GetMovingDir();
	//		if (temp_dir==0)
	//		{
	//			int bbbbbbbbbb=1; 
	//		}
	//		repair_obstacle.SetMovingDir(temp_dir);//设定方向

	//		double temp_v=temp_obstacle.GetV();        
	//		repair_obstacle.SetV(temp_v); //设定速度

	//		double l_x=temp_obstacle.GetLength_X(); //设定x长度
	//		repair_obstacle.SetLength_X(l_x);

	//		double l_y=temp_obstacle.GetLength_Y();//设定y长度
	//		repair_obstacle.SetLength_Y(l_y);

	//		int temp_age=temp_obstacle.GetMovingConfidence();
	//		repair_obstacle.SetMovingConfidence(temp_age);


	//		

	//		temp_dir=temp_obstacle.GetRLDir();
	//		repair_obstacle.SetRelativaDir(temp_dir);//设定相对方向
	//		temp_v=temp_obstacle.GetRLSpeed();
	//		repair_obstacle.SetRelativeSpeed(temp_v);//设定相对速度
	//		

	//		
	//		
	//		/*for(int temp_i=0; temp_i<temp_size; temp_i++)
	//		{
	//			repair_point[temp_i]=m_GpsData.APiontConverD(current_pos,repair_point[temp_i],current_dir);
	//			repair_point[temp_i].x=(int)(repair_point[temp_i].x+0.5+cos((temp_dir-current_dir+270)*PI/180)*0.85*temp_v);
	//			repair_point[temp_i].y=(int)(repair_point[temp_i].y+0.5+sin((temp_dir-current_dir+270)*PI/180)*0.85*temp_v);
	//			repair_point[temp_i]=m_GpsData.MaptoGPS(current_pos,current_dir,repair_point[temp_i]);


	//		}
	//		repair_obstacle.SetPos(repair_point);*/

	//	/************************************************************************/
	//	/* 设定四个顶点                                                                     */
	//	/************************************************************************/
	//		double xinshu1=1.2;
	//		temp_obstacle.GetBoxPoints(repair_box_points);
	//		for(int box_i=0; box_i<4; box_i++)
	//		{
	//			repair_box_points[box_i]=m_GpsData.APiontConverD(current_pos,repair_box_points[box_i],current_dir);
	//			repair_box_points[box_i].x=(int)(repair_box_points[box_i].x+0.5+cos((temp_dir/*-current_dir+270*/)*PI/180)*xinshu1*temp_v);
	//			repair_box_points[box_i].y=(int)(repair_box_points[box_i].y+0.5+sin((temp_dir/*-current_dir+270*/)*PI/180)*xinshu1*temp_v);
	//			repair_box_points[box_i]=m_GpsData.MaptoGPS(current_pos,current_dir,repair_box_points[box_i]);
	//		}
	//		repair_obstacle.SetBoxPoints(repair_box_points);
	//		
	//	/************************************************************************/
	//	/* 设定中心点                                                                     */
	//	/************************************************************************/	
	//		repair_center=temp_obstacle.GetIbeoCenter();
	//		repair_center=m_GpsData.APiontConverD(current_pos,repair_center,current_dir);
	//		repair_center.x=(int)(repair_center.x+0.5+cos((temp_dir/*-current_dir+270*/)*PI/180)*xinshu1*temp_v);
	//		repair_center.y=(int)(repair_center.y+0.5+sin((temp_dir/*-current_dir+270*/)*PI/180)*xinshu1*temp_v);
	//		repair_center=m_GpsData.MaptoGPS(current_pos,current_dir,repair_center);
	//		repair_obstacle.SetIbeoCenter(repair_center);

	//	/************************************************************************/
	//	/*  设定存在置信度                                                                     */
	//	/************************************************************************/
	//		int temp_existence=temp_obstacle.GetExistConfidence();
	//		temp_existence=temp_existence-1;
	//		repair_obstacle.SetExistConfidence(temp_existence);
	//		repair_obstacle.SetObCount(1);
 //           
	//		if(temp_existence>4)
	//		{
	//			//ibeolist.AddTail(repair_obstacle);
	//		}	
	//	}

	//}


	
//problem<<2<<endl;

}
void Vlist::Transfer(CvPoint2D64f &input_point)
{
	/*CvPoint2D64f out_point;*/
	input_point.x=256-input_point.x;
	input_point.y=412-HDL_IBEO_DISTANCE-input_point.y;//需要重新标定，此值ibeo坐标系圆心到hdl坐标系圆点的距离；
	//ASSERT(out_point.x>0&&out_point.y>0);7
}

void Vlist::Fuse(I_Map *map)
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	//app->critical_section.Lock();
	/*xinhuang_dir=app->GPS_Direction;
	xinhuang_pos=app->GPS_Point;*/
	double current_dir = xinhuang_dir;
	CvPoint2D64f current_pos = xinhuang_pos;
	double current_v=m_GpsData.GetSpeed();
	//app->critical_section.Unlock();

    fusionlist.RemoveAll();//清空上一帧的融合列表

	cluster.Setdetect(80,0,40,40);
	cluster.Init(map);


	int index=0;
    int ob_size=0;
	POSITION ibeo_pos=ibeolist.GetHeadPosition();
	
	vector <ObPoint> ob_points;   //存储所有在方框中的障碍物块的障碍物点信息,将其定义从下面的循环中提出来 by h,2013 6.19 22:06
	while(ibeo_pos!=NULL)
	{
		bool find_ob=0;  //是否在框中的标志

		DyObstacle ibeo_obstacle=ibeolist.GetNext(ibeo_pos);//从ibeolist中取一个障碍物

		
		double length=ibeo_obstacle.GetLength_X();
		double width=ibeo_obstacle.GetLength_Y();


		CvPoint2D64f box_points[4];
		ibeo_obstacle.GetBoxPoints(box_points);
		for(int box_points_index=0; box_points_index<4; box_points_index++)
		{
			box_points[box_points_index]=m_GpsData.APiontConverD(current_pos,box_points[box_points_index],current_dir);
		}


		CvPoint2D64f ibeo_center=ibeo_obstacle.GetIbeoCenter();
		ibeo_center=m_GpsData.APiontConverD(current_pos,ibeo_center,current_dir);
		
       
		ob_points.clear();  //ob_points对不同的ibeolist中的obstacle都清空一次

		for(int index=0;index<cluster.cluster_num;index++)
		{
			find_ob=0;
			vector <ObPoint> inner_points;
			inner_points.clear();

			for (int j=0; j<cluster.ob_count; j++)
			{
				if (cluster.ObPoints[j].GetClusterId()==index)
				{
					inner_points.push_back(cluster.ObPoints[j]);//将一个类的所有点加入到一个向量中
					CvPoint2D64f cluster_obpoint;
					cluster_obpoint.x=cluster.ObPoints[j].Getx();
					cluster_obpoint.y=cluster.ObPoints[j].Gety();

					if(find_ob==0)
					{
						if(IbeoExpansion2(box_points,cluster_obpoint,length,width))
						{
							find_ob=1;
						}
						/*if(IbeoExpansion(box_points,cluster_obpoint))
						{
							find_ob=1;
						}*/
					}
					
				}
			}

			if (find_ob==1)   //如果这个类中的点至少有一个在框中，将这个类的所有点都加入到ob_points中         
			{
				int  inner_size=inner_points.size();
				for (int i=0; i<inner_size; i++)
				{
					ob_points.push_back(inner_points[i]);
				}

			}
        }


			int vector_size=ob_points.size();
			if(vector_size<=2000&&vector_size>0)
			{
				DyObstacle fusion_obstacle;
				CvPoint2D64f point_pos[2000]={0};
				//CvPoint2D64f point_pos_map[2000];

				for(int cluster_point_index=0; cluster_point_index<vector_size; cluster_point_index++)
				{
					point_pos[cluster_point_index].x=ob_points[cluster_point_index].Getx();
					point_pos[cluster_point_index].y=ob_points[cluster_point_index].Gety();
					//point_pos_map[cluster_point_index].x=ob_points[cluster_point_index].Getx();
					//point_pos_map[cluster_point_index].y=ob_points[cluster_point_index].Gety();
					point_pos[cluster_point_index]=m_GpsData.MaptoGPS(current_pos,current_dir,point_pos[cluster_point_index]);
				}
				ob_points.clear();


				for(int box_points_index=0; box_points_index<4; box_points_index++)
				{
					box_points[box_points_index]=m_GpsData.MaptoGPS(current_pos,current_dir,box_points[box_points_index]);
				}

				ibeo_center=m_GpsData.MaptoGPS(current_pos,current_dir,ibeo_center);
				fusion_obstacle.SetPos(point_pos);
				fusion_obstacle.SetV(ibeo_obstacle.GetV());
				fusion_obstacle.SetMovingDir(ibeo_obstacle.GetMovingDir());
				fusion_obstacle.SetMovingConfidence(ibeo_obstacle.GetMovingConfidence());
				fusion_obstacle.SetArea(vector_size);
				fusion_obstacle.SetObNum(ibeo_obstacle.GetObNum());
				fusion_obstacle.SetBoxPoints(box_points);
				fusion_obstacle.SetIbeoCenter(ibeo_center);
				fusion_obstacle.SetLength_X(length);
				fusion_obstacle.SetLength_Y(width);
				fusion_obstacle.SetObCount(1);
				fusionlist.AddTail(fusion_obstacle);		
		}

	}
	
	cluster.Clear();
	
}

void Vlist::Fuse2(I_Map *map)
{
	//problem<<3<<endl;
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	double current_dir = xinhuang_dir;
	CvPoint2D64f current_pos = xinhuang_pos;
	double current_v=m_GpsData.GetSpeed();
	m_imap=map;
	CountRoadEdge();

	fusionlist.RemoveAll();//清空上一帧的融合列表
	//cluster.Setdetect(80,0,40,40);
	if(app->on_road == 0)
	{
		cluster.Setgrow(-5,6);
	}
	else
	{
		cluster.Setgrow(-1,2);
	}
	cluster.Init(map);                       //聚类

	POSITION ibeo_pos=ibeolist.GetHeadPosition();
	while(ibeo_pos!=NULL)
	{
		DyObstacle ibeo_obstacle=ibeolist.GetNext(ibeo_pos);
		DyObstacle fuse_obstacle;
		CvPoint2D64f ibeo_center=ibeo_obstacle.GetIbeoCenter();
		ibeo_center=m_GpsData.APiontConverD(current_pos,ibeo_center,current_dir);
		int center_x=ibeo_center.x+0.5;
		int center_y=ibeo_center.y+0.5;
		CvPoint2D64f box_points[4]={0};
		ibeo_obstacle.GetBoxPoints(box_points);
		for(int box_points_index=0; box_points_index<4; box_points_index++)
		{
			box_points[box_points_index]=m_GpsData.APiontConverD(current_pos,box_points[box_points_index],current_dir);
		}
		int length_x=ibeo_obstacle.GetLength_X();
		int length_y=ibeo_obstacle.GetLength_Y();
		int radius=(length_x+length_y)/2;
		vector<int> cluster_index;
		cluster_index.clear();
		for (int i=-radius; i<radius; i++)
		{
			for (int j=-radius; j<radius; j++)
			{				
				if ((center_x+i)<512&&(center_x+i>=0)&&(center_y+j<512)&&(center_y+j>=0)&&cluster.obmap->obstacle[center_y+j][center_x+i].isob==8&&cluster.obmap->obstacle[center_y+j][center_x+i].clusterId>=0)
				{
					CvPoint2D64f point={0};
					point.x=ibeo_center.x+i;
					point.y=ibeo_center.y+j;

					if (IbeoExpansion2(box_points, point, length_x, length_y))
					{
						int Id=cluster.obmap->obstacle[center_y+j][center_x+i].clusterId;
						int inner_flag=0;
						
						for (int inner_i=0; inner_i<cluster_index.size(); inner_i++)//查找和向量中类别数不同的类
						{
							if (cluster_index[inner_i]==Id)
							{
								inner_flag=1;
								break;
							}
						}
							  //遇到不同的就将inner_i设为1，跳出循环
							
						if (inner_flag==0)//如果在向量中没有找到这个类别，就将这个类别加入cluster_index
						{
							cluster_index.push_back(Id);
							inner_flag=0;
						}

					}
				}
			}
		}//for循环结束，取到了框内点的类别号

		vector<CvPoint2D64f> ob_points;
		CvPoint2D64f temp_points[2000] = {0};
		for (int find_index = 0; find_index < cluster_index.size(); find_index++)//将所有框内的小于2000的类加入向量中
		{
			if (cluster.obclusters[cluster_index[find_index]].GetSize() < 600/*cluster.obclusters[cluster_index[find_index]].GetFlag() == 0*//*&&cluster.obclusters[cluster_index[find_index]].GetSize() <= 600*/)
			{
				cluster.obclusters[cluster_index[find_index]].GetPoint(temp_points);
				for (int i = 0; i < cluster.obclusters[cluster_index[find_index]].GetSize(); i++)
				{
					ob_points.push_back(temp_points[i]);
				}
			}
		}


		if (ob_points.size() > 600/*2000*/)
		{
			ob_points.clear();
			for (int i=-radius; i<radius; i++)
			{
				for (int j=-radius; j<radius; j++)
				{				
					if ((center_x+i)<512&&(center_x+i>=0)&&(center_y+j<512)&&(center_y+j>=0)&&cluster.obmap->obstacle[center_y+j][center_x+i].isob==8&&cluster.obmap->obstacle[center_y+j][center_x+i].clusterId>=0)
					{
						CvPoint2D64f point={0};
						point.x=ibeo_center.x+i;
						point.y=ibeo_center.y+j;
						if (IbeoExpansion2(box_points, point, length_x, length_y))
						{
							ob_points.push_back(point);
						}
					}
				}
			}
		}



		/*for (int i=0; i<cluster_index.size(); i++)
		{
			for (int j=0; j<cluster.ob_count; j++)
			{
				if (cluster.ObPoints[j].GetClusterId()==cluster_index[i])
				{
					CvPoint2D64f ob_point;
					ob_point.x=cluster.ObPoints[j].Getx();
					ob_point.y=cluster.ObPoints[j].Gety();
					ob_points.push_back(ob_point);
				}
			}
		}*/

		/*int mmmmm=ob_points.size();
		if (ob_points.size() > 2000)
		{
			ob_points.clear();
			for (int i=-radius; i<radius; i++)
			{
				for (int j=-radius; j<radius; j++)
				{
					if ((center_x+i)<512&&(center_x+i>=0)&&(center_y+j<512)&&(center_y+j>=0)&&cluster.obmap->obstacle[center_y+j][center_x+i].isob==8&&cluster.obmap->obstacle[center_y+j][center_x+i].clusterId>=0)
					{
						CvPoint2D64f point={0};
						point.x=ibeo_center.x+i;
						point.y=ibeo_center.y+j;

						if (IbeoExpansion2(box_points, point, length_x, length_y))
						{
							ob_points.push_back(point);
						}

					}
				}
			}
		}*/

		/*if (ob_points.size()>2000||ob_points.size() == 0)
		{
			continue;
		}*/


		/*ASSERT(ob_points.size()<=2000);*/

		CvPoint2D64f find_points[2000];

		for (int ob_index=0; ob_index<ob_points.size(); ob_index++)
		{
			int aaa = ob_points.size();

			find_points[ob_index]=ob_points[ob_index];
			find_points[ob_index]=m_GpsData.MaptoGPS(current_pos,current_dir,find_points[ob_index]);

		}

		fuse_obstacle.SetPos(find_points); //障碍物点
		//fuse_obstacle.SetObCount(ibeo_obstacle.GetObCount());//编号
		fuse_obstacle.SetObNum(ibeo_obstacle.GetObNum());
		fuse_obstacle.SetIbeoCenter(ibeo_obstacle.GetIbeoCenter());//中心点
		CvPoint2D64f fuse_box[4];
		ibeo_obstacle.GetBoxPoints(fuse_box);
		fuse_obstacle.SetBoxPoints(fuse_box);//框顶点
		fuse_obstacle.SetMovingDir(ibeo_obstacle.GetMovingDir());//方向
		fuse_obstacle.SetV(ibeo_obstacle.GetV());//速度
		fuse_obstacle.SetLength_X(ibeo_obstacle.GetLength_X());//长度
		fuse_obstacle.SetLength_Y(ibeo_obstacle.GetLength_Y());//宽度
		fuse_obstacle.SetExistConfidence(ibeo_obstacle.GetExistConfidence());//存在置信度
		fuse_obstacle.SetMovingConfidence(ibeo_obstacle.GetMovingConfidence());
		fuse_obstacle.SetArea(ob_points.size());//障碍物点数量
		double c = ibeo_obstacle.GetDanger();
		fuse_obstacle.SetDanger(c);
		fuse_obstacle.SetOriX(ibeo_obstacle.GetOriX());
		fuse_obstacle.SetOriY(ibeo_obstacle.GetOriY());
		fuse_obstacle.SetObstacleKind(ibeo_obstacle.GetObstacleKind());
		fuse_obstacle.SetObCount(1);
		fusionlist.AddTail(fuse_obstacle);
	}
	cluster.Clear();
	//problem<<4<<endl;
	if (app->on_road != 0)
	{
		ObInRoad();
		//ObInRoad2();
	}
	

}


bool Vlist::IbeoExpansion(CvPoint2D64f box_points[4],CvPoint2D64f point)
{
   int flag1=0;
   flag1=box_points[0].y+(box_points[1].y-box_points[0].y)*(point.x-box_points[0].x)/(box_points[1].x-box_points[0].x)-point.y;
   int flag2=0;
   flag2=box_points[2].y+(box_points[3].y-box_points[2].y)*(point.x-box_points[2].x)/(box_points[3].x-box_points[1].x)-point.y;
   int flag3=0;
   flag3=box_points[1].y+(box_points[2].y-box_points[1].y)*(point.x-box_points[1].x)/(box_points[2].x-box_points[1].x)-point.y;
    int flag4=0; 
   flag4=box_points[0].y+(box_points[3].y-box_points[0].y)*(point.x-box_points[0].x)/(box_points[3].x-box_points[0].x)-point.y;
   if (flag1*flag2<0&&flag3*flag4<0)
   {
	   return 1;
   }
   else
	   return 0;

}
bool Vlist::IbeoExpansion2(CvPoint2D64f box_points[4],CvPoint2D64f point,int length,int width)
{
	double dis1,dis2,dis3,dis4,discha;
	double a,b,c;
	/*for(int box_points_index=0; box_points_index; box_points_index++)
		{
			box_points[box_points_index]=m_GpsData.APiontConverD(xinhuang_pos,box_points[box_points_index],xinhuang_dir);
		}*/
	a=(box_points[0].y-box_points[1].y)/(box_points[0].x-box_points[1].x);
	b=-1;
	c=box_points[0].y-(box_points[0].x*(box_points[0].y-box_points[1].y)/(box_points[0].x-box_points[1].x));
	dis1 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	a=(box_points[1].y-box_points[2].y)/(box_points[1].x-box_points[2].x);
	b=-1;
	c=box_points[1].y-(box_points[1].x*(box_points[1].y-box_points[2].y)/(box_points[1].x-box_points[2].x));
	dis2 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	a=(box_points[2].y-box_points[3].y)/(box_points[2].x-box_points[3].x);
	b=-1;
	c=box_points[2].y-(box_points[2].x*(box_points[2].y-box_points[3].y)/(box_points[2].x-box_points[3].x));
	dis3 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	a=(box_points[3].y-box_points[0].y)/(box_points[3].x-box_points[0].x);
	b=-1;
	c=box_points[3].y-(box_points[3].x*(box_points[3].y-box_points[0].y)/(box_points[3].x-box_points[0].x));
	dis4 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	discha = abs(dis1+dis2+dis3+dis4-length-width);

	if (discha<=DELTA)
	{
		return 1;
	}
	else
	{
		return 0;
	}
	

}


void Vlist::Intersection()
{
	CList<DyObstacle, DyObstacle> t_list;
	vector<int> around_num;
	POSITION pos = ibeolist.GetHeadPosition();
	while (pos!=NULL)
	{
		t_list.RemoveAll();
		//int around_count = 0;
		DyObstacle obstacle = ibeolist.GetNext(pos);
		obstacle.visted = obstacle.GetObNum();
		t_list.AddHead(obstacle);
		POSITION pos1 = ibeolist.GetHeadPosition();
		while (pos1 != NULL)
		{
			DyObstacle obstacle1 = ibeolist.GetNext(pos1);
			double ob_distance =GetObDistace(obstacle, obstacle1);
			if (ob_distance < 2)
			{
				obstacle1.visted = obstacle.visted;
				t_list.AddHead(obstacle1);
				//around_count++;
			}
		}

		if (t_list.GetCount() > 4)
		{
			around_num.push_back(obstacle.visted);
		}

	}

	vector<DyObstacle> ob_tlist;
	POSITION pos4 = ibeolist.GetHeadPosition();
	while (pos4 != NULL)
	{
		int flag = 0;
		DyObstacle obstacle = ibeolist.GetNext(pos4);
		for (int i= 0; i<around_num.size(); i++)
		{
			if (obstacle.visted == around_num[i])
			{
				flag = 1;
				break;
			}
		}
		if (flag == 0)
		{
			ob_tlist.push_back(obstacle);
		}
	}

	ibeolist.RemoveAll();

	for (int j= 0; j<ob_tlist.size(); j++)
	{
		ibeolist.AddTail(ob_tlist[j]);
	}

}

void Vlist::SetAAA(double a[6])
{
	if(a[5] >= 0.002||a[2] >= 0.002)
		{
			return;
		}
	for(int i = 0; i<6; i++)
	{
		aaa[i] = a[i];
	}
}

void Vlist::CountRoadEdge()
{
	
	for(int i = 412; i > 0; i--)
	{
		for(int j = 256; j>0; j--)
		{
			if(m_imap->MapPoint[i][j] == 18)
			{
				l_roadedge[i].x = j;
				l_roadedge[i].y = i;
				break;
			}
		}
	}

	for(int i = 412; i>0; i--)
	{
		for(int j = 256; j<512; j++)
		{
			if(m_imap->MapPoint[i][j] == 18)
			{
				r_roadedge[i].x = j;
				r_roadedge[i].y = i;
				break;
			}
		}
	}
}

double Vlist::GetObDistace(DyObstacle ob1, DyObstacle ob2)
{
	CvPoint2D64f center1 = ob1.GetIbeoCenter();
	CvPoint2D64f center2 = ob2.GetIbeoCenter();
	double ob_distance = m_GpsData.GetDistance(center1.x,center1.y,center2.x,center2.y);
	return ob_distance;
}

double Vlist::ClearOb()
{
	ibeolist.RemoveAll();
	fusionlist.RemoveAll();
	return 0;
}

void  Vlist::ObInRoad2()
{
	//POSITION pos = ibeolist.GetHeadPosition();
	//vector<DyObstacle> vector_list;
	//int a1, b1, c1, a2, b2, c2;
	//	a1 = aaa[2];
	//	b1 = aaa[1];
	//	c1 = aaa[0];

	//	a2 = aaa[5];
	//	b2 = aaa[4];
	//	c2 = aaa[3];
	//while (pos != NULL)
	//{
	//	DyObstacle obstacle = ibeolist.GetNext(pos);
	//	CvPoint2D64f ibeo_center = obstacle.GetIbeoCenter();
	//	Map_Point Map_center = m_GpsData.APiontConver(xinhuang_pos, ibeo_center, xinhuang_dir);		
	//	//double result = (a1*Map_center.x*Map_center.x + b1*Map_center.x + c1 - Map_center.y)*(a2*Map_center.x*Map_center.x + b2*Map_center.x + c2 - Map_center.y);
	//	
	//	double result = (a1*Map_center.y*Map_center.y + b1*Map_center.y + c1 - Map_center.x)*(a2*Map_center.y*Map_center.y + b2*Map_center.y + c2 - Map_center.x);
	//	if (result < 0)
	//	{
	//		vector_list.push_back(obstacle);
	//	}
	//}

	//ibeolist.RemoveAll();
	//for (int i = 0; i < vector_list.size(); i++)
	//{
	//	ibeolist.AddTail(vector_list[i]);
	//}
	CList<DyObstacle,DyObstacle> temp_fusionlist;

	POSITION pos = fusionlist.GetHeadPosition();
	if(fusionlist.GetCount() != 0)
	{int s=0;}
	while(pos != NULL)
	{
		DyObstacle obstacle = fusionlist.GetNext(pos);
		CvPoint2D64f center = obstacle.GetIbeoCenter();
		center = m_GpsData.APiontConverD(xinhuang_pos, center, xinhuang_dir);
		int center_x = (int)(center.x+0.5);
		int center_y = (int)(center.y+0.5);

		int r_x = r_roadedge[center_y].x;
		int l_x = l_roadedge[center_y].x;
		if((center_x-l_x)*(center_x-r_x)<0)
		{
			temp_fusionlist.AddHead(obstacle);
		}
	}
	fusionlist.RemoveAll();

	POSITION pos2 = temp_fusionlist.GetHeadPosition();
	while(pos2!=NULL)
	{
		DyObstacle temp_obstacle = temp_fusionlist.GetNext(pos2);
		fusionlist.AddHead(temp_obstacle);
	}
}