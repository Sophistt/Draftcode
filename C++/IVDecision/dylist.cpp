#include "StdAfx.h"
#include "DyList.h"
#include <fstream>
#define  DIFFX 10                   //�������ֵ
#define  DIFFY 20                   //�б�������ϰ�����
#define  DOWNLIMIT 30
#define  UPLIMIT   400
#define  DISTANCE 3

DyList::DyList(void):head(NULL),tail(NULL),num(0)
{
	dy_obstacle_num=0;
	mapcount=0;
}

DyList::~DyList(void)
{
}

bool DyList::isEmpty()
{
	if((head==NULL)&&(tail==NULL))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

ObstacleNode *DyList::GetHead()
{
	return head;
}

ObstacleNode *DyList::GetTail()
{
	return tail;
}

int DyList::GetNum()
{
	return this->num;
}
ObstacleNode *DyList::ItemAt(int position)
{
	if ((position<0)||(position>num))
	{
		AfxMessageBox(_T("�������"));
	}

	else
	{
		int i=0;
		ObstacleNode *p=head;
		while(i<position)                        //���position=0����õ���ͷ�����position-1���򷵻ر�β
		{
			p=p->GetNext();
			i++;
		}
		return p;
	}
}

ObstacleNode *DyList::Search(DyObstacle dyobstaclenode)
{
	if(isEmpty())
	{
		AfxMessageBox(_T("����Ϊ��"));
	}
	else
	{
		ObstacleNode *p=head;
		while((p!=NULL)&&!(p->GetData()==dyobstaclenode))
		{
			p=p->GetNext();
		}
		if (p!=NULL)
		{
			return p;
		}
		else
		{
			return NULL;
		}
	}
}

void DyList::RemoveHead()
{
	if (this->isEmpty())
	{
		AfxMessageBox(_T("�ϰ����б�Ϊ��"));
	}
	else
	{
		if (head==tail)
		{
			head=NULL;
			tail=NULL;
			num--;
		}
		else
		{
			ObstacleNode *tem=head;
			head=head->GetNext();
			num--;
			delete tem;
		}
	}
}

void DyList::RemoveTail()
{
	if (isEmpty())
	{
		AfxMessageBox(_T("����Ϊ��"));
	}
	else
	{
		if (head==tail)
		{
			head=NULL;
			tail=NULL;
			num--;
		}
		else
		{
			ObstacleNode *p=ItemAt(num-2);              //�õ����±�βǰһ���ڵ�ĵ�ַ,��Ҫע��˴�������num-2����num-1
			ObstacleNode *tem=tail;
			tail=p;
			tail->SetNext(NULL);
			num--;
			delete tem;
		}
	}
}
void DyList::Remove(int x)                              //x��0��num-1,��ItemAt(x)�Ƴ�
{
	if (isEmpty())
	{
		AfxMessageBox(_T("����Ϊ��"));
	}
	else
	{
		if (x==0)
		{
		   RemoveHead();
		}
		else
		{
			ObstacleNode *p=ItemAt(x-1);
			ObstacleNode *p1=ItemAt(x);
			p->SetNext(p1->GetNext());
			num--;
			delete p1;
		}
	}
}

void DyList::InsertHead(ObstacleNode obstaclenode)
{
	if (isEmpty())
	{
		ObstacleNode *p=new ObstacleNode(obstaclenode);
		head=tail=p;
		num++;
	}
	else
	{
		ObstacleNode *p=new ObstacleNode(obstaclenode);
		p->SetNext(head);
		head=p;
		num++;
	}
}
void DyList::InsertTail(ObstacleNode obstaclenode)
{
	if (isEmpty())
	{
		ObstacleNode *p=new ObstacleNode(obstaclenode);
		/*p->SetData(obstaclenode);*/
		tail=head=p;
		num++;
	}
	else
	{
		ObstacleNode *p=new ObstacleNode(obstaclenode);
		tail->SetNext(p);
		p->SetNext(NULL);
		tail=p;
		num++;
	}
}
void DyList::Insert(ObstacleNode *p, int x)         //x��0��num-1
{
	ObstacleNode *pt=ItemAt(x);
	ObstacleNode *q=pt->GetNext();
	p->SetNext(q);
	pt->SetNext(p);
	num++;
}

double DyList::ComPare(DyObstacle ob1,DyObstacle ob2)
{
	double diffenrence=0;
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
	dx2=center2.x/area2;                        //��һ���ϰ��������λ��
	dy2=center2.y/area2;                        //�ڶ����ϰ��������λ��
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
	/*double ob_dir=CountDir(ob2,ob1);*/
	/*if (ob1.GetMovingDir()-ob_dir>25)
	{
		return 100;
	}*/
	if (ob1.GetObCount()<10)
	{
        double  dif=sqrt(double((area1-area2)*(area1-area2)/2500))+distance/(ob1.GetObCount()+k-9);
		return dif;  //�������ľ���
	}
	if (ob1.GetObCount()>9)
	{
		double dif=sqrt(double((area1-area2)*(area1-area2)/2500))+distance/(k+1);
		return dif;
	}
   	
}

double DyList::V_Dir(DyObstacle ob1,DyObstacle ob2)
{
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

	int area1=ob1.GetArea();
	CvPoint2D64f p[SIZE]={0};
	ob1.GetPos(p);
	for (int i=0;i<area1;i++)
	{	
		center1.x=center1.x+p[i].x;
		center1.y=center1.y+p[i].y;

	}
	dx1=center1.x/area1;
	dy1=center1.y/area1;

	int area2=ob2.GetArea();
	CvPoint2D64f m[SIZE]={0};
	ob2.GetPos(m);
	for (int j=0; j<area2;j++)
	{
		center2.x=center2.x+m[j].x;
		center2.y=center2.y+m[j].y;
	}
	dx2=center2.x/area2;                        //��һ���ϰ��������λ��
	dy2=center2.y/area2;                        //�ڶ����ϰ��������λ��

	double dir=m_GpsData.GetAngle(center1,center2);
	return dir;
}

double DyList::CountV(DyObstacle ob1, DyObstacle ob2)
{

	double diffenrence=0;
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
	dx2=center2.x/area2;                        //��һ���ϰ��������λ��
	dy2=center2.y/area2;                        //�ڶ����ϰ��������λ��
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
	double ob_dir=CountDir(ob2,ob1);
	/*if (ob1.GetMovingDir()-ob_dir>25)
	{
		return 100;
	}*/
	if (ob1.GetObCount()<10)
	{
        double  dif=distance/(ob1.GetObCount()+k-9)*10;
		return dif;  //�������ľ���
	}
	if (ob1.GetObCount()>9)
	{
		double dif=distance/(k+1)*10;
		return dif;
	}
}

double DyList::CountDir(DyObstacle ob1,DyObstacle ob2)
{

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

	int area1=ob1.GetArea();
	CvPoint2D64f p[SIZE]={0};
	ob1.GetPos(p);
	for (int i=0;i<area1;i++)
	{	
		center1.x=center1.x+p[i].x;
		center1.y=center1.y+p[i].y;

	}
	dx1=center1.x/area1;
	dy1=center1.y/area1;

	int area2=ob2.GetArea();
	CvPoint2D64f m[SIZE]={0};
	ob2.GetPos(m);
	for (int j=0; j<area2;j++)
	{
		center2.x=center2.x+m[j].x;
		center2.y=center2.y+m[j].y;
	}
	dx2=center2.x/area2;                        //��һ���ϰ��������λ��
	dy2=center2.y/area2;                        //�ڶ����ϰ��������λ��
	double dir=m_GpsData.GetAngle(dx1,dy1,dx2,dy2);
	return dir;
}

void DyList::UpDate(I_Map *map)
{
	//dy_obstacle_num=0;                           //��ʼ���ϰ�����Ϊ0
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();             //��ȡ��ͼ����������Ϣ

	GrowCluster cluster;
	cluster.Init(map);
	
	if (this->isEmpty())
	{	                                       
		for (int i=1; i<=cluster.cluster_num; i++)
		{	
			CvPoint2D64f p[SIZE]={0};                    //�洢������м����
			ObstacleNode node;                          //����һ���м�ڵ�
			DyObstacle dy_ob;                           //�洢�ϰ�����Ϣ���м����
			int k=0;                                    //������ 
			for (int j=0; j<cluster.ob_count; j++)
			{
				if (cluster.ObPoints[j].GetClusterId()==i)
				{
					p[k].x=cluster.ObPoints[j].Getx();
					p[k].y=cluster.ObPoints[j].Gety();
					p[k]=m_GpsData.MaptoGPS(m_gps,dir,p[k]);
					k++;
				}
			}
			if (k<20)                               //����ϰ���С��20���������ϰ���
			{
				continue;
			}
			 dy_ob.SetPos(p);                       //�����ϰ���λ��
			 dy_ob.SetObNum(dy_obstacle_num);       //�ϰ����ŵ�������
             dy_ob.SetArea(k);                      //�ϰ���ռդ����
             dy_ob.SetObCount(1);                   //�����Ѿ��洢��֡��
			 node.SetData(dy_ob);
			 this->InsertTail(node);
			 dy_obstacle_num++;
		}	                      
	}

              /*����ǿ�*/

	else
	{ 		 
			  for (int i=1; i<=cluster.cluster_num; i++)
			  {
				  DyObstacle newob;
				  CvPoint2D64f q[SIZE]={0};
				  int newk=0;
				  for (int j=0; j<cluster.ob_count; j++)
				  {
					  if (cluster.ObPoints[j].GetClusterId()==i)
					  {
						  q[newk].x=cluster.ObPoints[j].Getx();
						  q[newk].y=cluster.ObPoints[j].Gety();
						  q[newk]=m_GpsData.MaptoGPS(m_gps,dir,q[newk]);
						  newk++;
					  }
				  } 
				  if (newk<20)                           //����ϰ���С��20,�������ϰ���
				  {
					  continue;
				  }
				  newob.SetPos(q);
				  newob.SetObNum(dy_obstacle_num);
				  newob.SetArea(newk);
				  newob.SetObCount(1);

				  int pos=0;
				  double dif[100];
				  for (int i=0; i<100; i++)
				  {
					  dif[i]=100;
				  }
				  int dif_num=0;
				  while(pos<this->GetNum())                             //���ϰ����б��е�ÿһ���ϰ���Աȣ��洢����ֵ
				  {
					  if (this->ItemAt(pos)->GetUpdateFlag()==0)        //����ڵ�û�и���
					  {
					 ObstacleNode  *listnode=this->ItemAt(pos);         //��ȡ��pos���ϰ���
					 DyObstacle listob=listnode->GetData();
					 dif[pos]=ComPare(listob,newob);                    //���ص��������������ĵ�ľ���
					  }
					  else 
						  dif[pos]=100;                                 //����Ѿ�ƥ������򽫾�������Ϊ100m 
					 pos++;		
				  }
					 double smallest=dif[0];                            //������Сֵ
				
				  int small_num=0;                                      //�洢��Сֵ������
				  for (int i=0; i<pos; i++)                             //ͨ�������������������Сֵ
				  {
					  if (smallest>dif[i])
					  {
						  smallest=dif[i];
						  small_num=i;
					  }
				  }

				  
				  if (smallest<=DISTANCE)                                            //�����С����С��DISTANCEm����ô��Ϊ����ϰ������small_num���ϰ�����ͬһ���ϰ����������ϰ������Ϣ
				  {
					  DyObstacle update_ob=this->ItemAt(small_num)->GetData();  //ȡ����small_num���ϰ����к��е��ϰ�����Ϣ
					  CvPoint2D64f pos_stack[SIZE];
					  CvPoint2D64f p1[SIZE]={0};
						  newob.GetPos(p1);
					  for (int i=0; i<SIZE; i++)
					  {						
						  pos_stack[i].x=p1[i].x;
						  pos_stack[i].y=p1[i].y;
					  }
					  double v_dir=this->V_Dir(update_ob,newob);
					  update_ob.SetPos(q);                                   //�����һ֡�ϰ���λ��
					  update_ob.SetArea(newk);                               //���դ������
					  update_ob.SetV(smallest*10);                              //�����ٶ�
					  if (smallest>0.01)                                        //����ٶȴ���1m/s�����趨Ϊ�˶�
					  {
						  update_ob.SetMovingFlag(true);
					  }
					  
					  update_ob.SetMovingDir(v_dir);                                //���ٶȷ����趨Ϊ0
					  update_ob.SetAccel(0);                                    //�����ٶ��趨Ϊ0
					  update_ob.SetExistConfidence(1);                          //�������Ŷȼ�1
					  if (smallest>0.01)
					  {
						 update_ob.SetMovingConfidence(1);                      //���λ�ñ仯����0.1m�˶����Ŷȼ�1
					  }
					
					  update_ob.SetObCount(1);                          //�洢��֡����1
					  this->ItemAt(small_num)->SetData(update_ob);              //�����ϰ�����Ϣ
					  this->ItemAt(small_num)->SetUpdateFlag(1); 
				  }
				  if (smallest>DISTANCE&&newob.GetArea()>20)                                                            //��С���벻����Ҫ�󣬽����ϰ��ﵱ���µ��ϰ���ڵ���붯̬�ϰ����б�
				  {
					  ObstacleNode newnode;	  
					  newob.SetObNum(this->dy_obstacle_num);
					  newnode.SetData(newob);
					  this->InsertTail(newnode);                                //�����µĽڵ�
					  dy_obstacle_num++;	
				  }
			  }		
	}

	for (int i=0; i<this->GetNum(); i++)                                         //������û�еõ����µĽڵ㣬������һ֡����Ϣ
	{		
		if (this->ItemAt(i)->GetUpdateFlag()==0)
		{
			DyObstacle obstacle=this->ItemAt(i)->GetData();
			CvPoint2D64f newpos[SIZE]={0};
			obstacle.SetPos(newpos);                                              //����һ֡��λ����Ϊ0
			obstacle.SetArea(0);                                                  //����һ֡�����Ϊ0��
			obstacle.SetAccel(0);                                                 //����һ֡�ļ��ٶ���Ϊ0
			obstacle.SetExistConfidence(-1);
			obstacle.SetV(0);                                                     //�ٶ���Ϊ0
			obstacle.SetAccel(0);                                                 //���ٶ���Ϊ0
			obstacle.SetMovingDir(0);                                             //���ٶ���Ϊ0
			int obcount=obstacle.GetObCount();                                    //��ȡ�Ѿ���¼��֡��
			obstacle.SetMovingFlag(false);                                        //�˶�״̬���
			obstacle.SetObCount(obcount+1);                                       //�Ѿ��۲��֡��
			this->ItemAt(i)->SetData(obstacle);	
		}

	}
	for (int i=0; i<this->GetNum(); i++)                                         //���������Ŷ�С��3���ϰ���ڵ��Ƴ�
	{
		if (this->ItemAt(i)->GetData().GetExistConfidence()<3)
		{
			this->Remove(i);                     
		}
	}

	/*�����������нڵ�ĸ��±�־����Ϊ0 */
	for (int i=0; i<this->GetNum(); i++)
	{
		this->ItemAt(i)->SetUpdateFlag(0);
	}

	mapcount++;                                      
	
}

void DyList::UpDate2(I_Map *map)
{
	/*double time1=clock();
	double time2=0;*/
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();             //��ȡ��ͼ����������Ϣ

	GrowCluster cluster;
	cluster.Init(map);
	cluster.Clear();

	/*if (cluster.cluster_num>1)
	{
		AfxMessageBox(_T("������1"));
	}*/
	
	   //     if (mapcount>10)
	   //     {
				//AfxMessageBox(_T("H"));
	   //     }

	if (this->isEmpty())
	{	 int i=0;                                     
		while (i<cluster.cluster_num)
		{	
			CvPoint2D64f p[2000]={0};                    //�洢������м����
			
			ObstacleNode node;                          //����һ���м�ڵ�
			DyObstacle dy_ob;                           //�洢�ϰ�����Ϣ���м����
			CvPoint2D64f center;
			center.x=0;
			center.y=0;
			int k=0;                                    //������ 
			for (int j=0; j<cluster.ob_count; j++)
			{
				if (cluster.ObPoints[j].GetClusterId()==i)
				{
					p[k].x=cluster.ObPoints[j].Getx();
					p[k].y=cluster.ObPoints[j].Gety();
					p[k]=m_GpsData.MaptoGPS(m_gps,dir,p[k]);
					center.x+=p[k].x;
					center.y+=p[k].y;
					k++;
				}
			}
			i++;
			if (k<DOWNLIMIT||k>UPLIMIT)                               //����ϰ���С��30���������ϰ���
				continue;
			center.x=center.x/k;
			center.y=center.y/k;
			CvPoint2D64f m[SIZE]={0};
			for (int w=0; w<SIZE; w++)
			{
				m[w].x=p[w].x;
				m[w].y=p[w].y;
			}
			dy_ob.SetPos(m);                       //�����ϰ���λ��
			dy_ob.SetObNum(dy_obstacle_num);       //�ϰ����ŵ�������
			dy_ob.SetArea(k);                      //�ϰ���ռդ����
			dy_ob.SetCenter(center);
			dy_ob.SetObCount(1);                   //�����Ѿ��洢��֡��
			node.SetData(dy_ob);
			this->InsertTail(node);
			dy_obstacle_num++;
		}	                      
	}
	else
	{ 
		Diff diff[DIFFX][DIFFY];
		for (int i=0; i<DIFFX; i++)
		{
			for (int j=0; j<DIFFY; j++)
			{
				diff[i][j].d=100;
				diff[i][j].visited=0;
			}
		}
		for (int i=0; i<cluster.cluster_num; i++)                  //����ÿһ���ϰ�����б����ϰ���Ĳ�𣬴����һ����ά������
		{
			DyObstacle newob;
			CvPoint2D64f q[2000]={0};
			CvPoint2D64f q2[SIZE]={0};
			int newk=0;
			
			for (int j=0; j<cluster.ob_count; j++)
			{
				if (cluster.ObPoints[j].GetClusterId()==i)
				{
					q[newk].x=cluster.ObPoints[j].Getx();
					q[newk].y=cluster.ObPoints[j].Gety();
					q[newk]=m_GpsData.MaptoGPS(m_gps,dir,q[newk]);
					
					newk++;
				}
			}
			if (newk<UPLIMIT&&newk>DOWNLIMIT)                           //����ϰ������Ŀ����30��С��SIZE���洢������ʹ����һ���ϰ���ڵ�
			{
				for (int f=0; f<SIZE; f++)
				{
					q2[f].x=q[f].x;
					q2[f].y=q[f].y;
				}
			}
			else
			{
				continue;
			}
			
			newob.SetPos(q2);
			newob.SetArea(newk);
			newob.SetObCount(1);
			int pos=0;
			while(pos<this->GetNum())                             //���ϰ����б��е�ÿһ���ϰ���Աȣ��洢����ֵ
			{	
					ObstacleNode  *listnode=this->ItemAt(pos);         //��ȡ��pos���ϰ���
					DyObstacle listob=listnode->GetData();
					diff[i][pos].d=ComPare(listob,newob);                    //���ص��������������ĵ�ľ���
				    pos++;		
			}
       }
        double smallest;
		int small_cluster=0;
		int small_list=0;
		int list_match[50];                                        //�洢�������Ѿ�ƥ��������
		for (int l=0; l<50; l++)
		{
			list_match[l]=-1;
		}
		int ob_match[50];                                          //�洢�Ѿ�ƥ���ϵ��ϰ������
		for (int m=0; m<50; m++)
		{
			ob_match[m]=-1;
		}
		int lmatch_num=0;                                          //�洢�Ѿ�ƥ���ϵ��б��ϰ�����Ŀ
		int omatch_num=0;                                          //�洢�Ѿ�ƥ���ϵ��ϰ�����Ŀ
		while(1)                                                    //����С���С��DISTANCE������ѭ��
		{
			smallest=100;                                                      //����Сֵ����ֵ
			for (int i=0; i<cluster.cluster_num; i++)
			{
				for (int j=0; j<this->num; j++)
				{
					if (diff[i][j].visited==0&&diff[i][j].d<smallest)
					{
						smallest=diff[i][j].d;
						small_cluster=i;
						small_list=j;
					}
				}
			}
			if (smallest>DISTANCE)                                      //�����С�������DISTANCE������ѭ��
			{
				break;
			}
			;
			for (int m=0; m<cluster.cluster_num; m++)
			{
				diff[m][small_list].visited=1;
			}
			for (int n=0; n<this->num; n++)
			{
				diff[small_cluster][n].visited=1;
			}
            
			CvPoint2D64f r[SIZE];
			int r_count=0;
			ObstacleNode *update_node=ItemAt(small_list);
			DyObstacle update_ob=update_node->GetData();  //ȡ����small_list���ϰ����к��е��ϰ�����Ϣ
			DyObstacle newobstacle;
			CvPoint2D64f center;
			center.x=0; 
			center.y=0;
            for (int i=0; i<cluster.ob_count; i++)
            {
				if (cluster.ObPoints[i].GetClusterId()==small_cluster)
				{
					r[r_count].x=cluster.ObPoints[i].Getx();
					r[r_count].y=cluster.ObPoints[i].Gety();
					r[r_count]=m_GpsData.MaptoGPS(m_gps,dir,r[r_count]);
					center.x+=r[r_count].x;
					center.y+=r[r_count].y;
					r_count++;
				}
            }
			center.x=center.x/r_count;
			center.y=center.y/r_count;
				newobstacle.SetPos(r);
				newobstacle.SetArea(r_count);
				/*double ob_v=CountV(update_ob,newobstacle);*/
			   /* double ob_dir=CountDir(newobstacle,update_ob);*/
				/*update_ob.SetMovingDir(ob_dir);*/
				update_ob.SetPos(r);
				update_ob.SetArea(r_count);
				update_ob.SetCenter(center);
				update_ob.SetV(smallest*10);
				if (update_ob.GetExistConfidence()<15)
				{
					update_ob.SetExistConfidence(1);
				}
				if (update_ob.GetMovingConfidence()<15)
				{update_ob.SetMovingConfidence(1);
				}
				
				update_ob.SetObCount(1);
				this->ItemAt(small_list)->SetData(update_ob);
				list_match[lmatch_num]=small_list;
				lmatch_num++;
				ob_match[omatch_num]=small_cluster;
				omatch_num++;
			}

		/************************************************************************/
		/* ������ϰ���                                                                     */
		/************************************************************************/

		for (int i=0; i<cluster.cluster_num; i++)
		{
			bool pipei=0;
			for (int j=0; j<omatch_num; j++)
			{
				if (i==ob_match[j])
				{
					pipei=1;
					break;
				}
			}

			if (pipei==0)
			{
				ObstacleNode newnode;
				CvPoint2D64f temp[2000]={0};
				CvPoint2D64f newpos[SIZE]={0};
				int newk=0;
				DyObstacle newobstacle;
				CvPoint2D64f center;
				center.x=0; 
				center.y=0;
				for (int j=0; j<cluster.ob_count; j++)
				{						
					if (cluster.ObPoints[j].GetClusterId()==i)
					{
						temp[newk].x=cluster.ObPoints[j].Getx();
						temp[newk].y=cluster.ObPoints[j].Gety();
						temp[newk]=m_GpsData.MaptoGPS(m_gps,dir,temp[newk]);
						center.x+=temp[newk].x;
						center.y+=temp[newk].y;
						newk++;
					}
				}

				if (newk<DOWNLIMIT||newk>UPLIMIT)                                //�������С��30�����������400��������ǰѭ��
				{
					continue;                                  
				}
				center.x=center.x/newk;
				center.y=center.y/newk;
				for (int m=0; m<UPLIMIT; m++)
				{
					newpos[m].x=temp[m].x;
					newpos[m].y=temp[m].y;
				}
				/************************************************************************/
				/*                    �����ϰ�������б�                                                  */
				/************************************************************************/
				newobstacle.SetPos(newpos);
				newobstacle.SetArea(newk);
				newobstacle.SetCenter(center);
				newobstacle.SetObCount(1);
				newobstacle.SetObNum(dy_obstacle_num);
				dy_obstacle_num++;
				newnode.SetData(newobstacle);
				this->InsertTail(newnode);	

			}
		}

		/************************************************************************/
		/* �µ�ƥ�䷽��                                                                     */
		/************************************************************************/
//            
//			for (int i=0; i<small_cluster; i++)           //������ж�û��ƥ���ϣ�����ϰ���ʱ���ϰ�������б�
//			{
//				int cluster_count=0;
//				for (int j=0; j<small_list; j++)
//				{
//					if (diff[i][j].visited==0)
//					{
//						cluster_count++;
//					}
//				}
//				if (cluster_count=this->num)                
//				{
//					ObstacleNode newnode;
//					CvPoint2D64f temp[1000]={0};
//					CvPoint2D64f newpos[SIZE]={0};
//					int newk=0;
//					DyObstacle newobstacle;
//					for (int j=0; j<this->GetNum(); j++)
//					{						
//						if (cluster.ObPoints[j].GetClusterId()==i)
//						{
//							temp[newk].x=cluster.ObPoints[j].Getx();
//							temp[newk].y=cluster.ObPoints[j].Gety();
//							temp[newk]=m_GpsData.MaptoGPS(m_gps,dir,temp[newk]);
//							newk++;
//						}
//					}
//				
//					if (newk<DOWNLIMIT||newk>UPLIMIT)                                //�������С��30�����������400��������ǰѭ��
//					{
//						continue;                                  
//					}
//					for (int m=0; m<UPLIMIT; m++)
//					{
//						newpos[m].x=temp[m].x;
//						newpos[m].y=temp[m].y;
//					}
///************************************************************************/
///*                    �����ϰ�������б�                                                  */
///************************************************************************/
//					newobstacle.SetPos(newpos);
//					newobstacle.SetArea(newk);
//					newobstacle.SetObCount(1);
//					newobstacle.SetObNum(dy_obstacle_num);
//					dy_obstacle_num++;
//					newnode.SetData(newobstacle);
//					this->InsertTail(newnode);	
//			    }		    
//			}

		/************************************************************************/
		/* Ϊ������û��ƥ���ϵĵ�����µ���Ϣ                                                                    */
		/************************************************************************/

		for (int i=0; i<this->num; i++)
		{
			int pipei=0; 
			for (int j=0; j<lmatch_num; j++)
			{
				if (list_match[j]==i)
				{
					pipei=1;
					break;
				}
			}
			if (pipei==0)
			{
				CvPoint2D64f zero[SIZE]={0};
				CvPoint2D64f center;
				center.x=0;
				center.y=0;
				ObstacleNode *listnode=ItemAt(i);
				DyObstacle listob=listnode->GetData();
				listob.SetPos(zero);
				listob.SetArea(0);
				listob.SetCenter(center);
				listob.SetExistConfidence(-2);
				listob.SetMovingDir(0);
				listob.SetMovingFlag(0);
				listob.SetV(0);
				listob.SetAccel(0);
				listob.SetMovingConfidence(-1);
				listob.SetObCount(1);
				this->ItemAt(i)->SetData(listob);   
			}
		}


			   //for (int i=0; i<this->num; i++)                       //������ж�û��ƥ���ϣ����б��еĴ���������һ֡��û����֮��Ӧ�����壬Ϊ���������һ֡����Ϣ
			   //{
				  // int list_count=0;
				  // for (int j=0; j<cluster.cluster_num; j++)
				  // {
					 //  if (diff[j][i].visited==0)
					 //  {
						//   list_count++;
					 //  }
				  // }
				  // if (list_count==cluster.cluster_num)
				  // {
					 //  /************************************************************************/
					 //  /*                    Ϊ��һ֡�����µ���Ϣ                                                  */
					 //  /************************************************************************/
					 //  CvPoint2D64f zero[SIZE]={0};
					 //  DyObstacle listob=this->ItemAt(i)->GetData();
					 //  listob.SetPos(zero);
					 //  listob.SetArea(0);
					 //  listob.SetExistConfidence(-2);
					 //  listob.SetMovingDir(0);
					 //  listob.SetMovingFlag(0);
					 //  listob.SetV(0);
					 //  listob.SetAccel(0);
					 //  listob.SetMovingConfidence(-1);
					 //  listob.SetObCount(1);
					 //  this->ItemAt(i)->SetData(listob);   
				  // }
				  // 
			   //}         
			
	
}
	for (int i=0; i<this->GetNum(); i++)                                         //���������Ŷ�С��3���ϰ���ڵ��Ƴ�
	{
		if (this->ItemAt(i)->GetData().GetExistConfidence()<8)
		{
			/*this->Remove(i);   */                  
		}
	}	

   /* time2=clock();
	double time=(time2-time1)/CLOCKS_PER_SEC;*/
	mapcount++;    
	ofstream timec("D\:zhen.txt");
	timec<<mapcount;
	if (mapcount==154)
	{
		AfxMessageBox("  ");
	}
	
}

