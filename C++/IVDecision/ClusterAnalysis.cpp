#include "StdAfx.h"
#include "ClusterAnalysis.h"
#include <fstream>
#include <math.h>
#include "ComuLcm.h"
#include "PointShowDlg.h"
ofstream outcluster("cluster.txt");
ClusterAnalysis::ClusterAnalysis(void)
{
}

ClusterAnalysis::~ClusterAnalysis(void)
{
}


//��������
void ClusterAnalysis::Init(I_Map *map, double radius, int minPTs)
{
	this->radius=radius;                                   //���ð뾶
	this->minPTs=minPTs;                                   //����������С���ݸ���
	this->dimNum=DIME_NUM;                                 //��������ά��
	MPoint mpoint[10000]={0};
	int k=0;
    for (int i=100; i<452; i++)
    {
		for (int j=100; j<300; j++)
		{
			if (map->MapPoint[i][j]==18)
			{
				mpoint[k].x=j;
				mpoint[k].y=i;
				k++;
			}
		}
    }
    unsigned long m=0;            //���ݸ���ͳ��


	while (m<=k)
	{
	   DataPoint tempDP;                                   //��ʱ���ݶ���
	   double tempDimData[DIME_NUM];                       //��ʱ���ݵ�ά����Ϣ
	
		tempDimData[0]=mpoint[m].x;
		tempDimData[1]=mpoint[m].y;
	    tempDP.SetDimension(tempDimData);                        //��ά����Ϣ�������ݵ����


	    tempDP.SetDpId(m);                                       //�����ݵ����ID����Ϊi
		tempDP.SetVisited(false);                                //���ݵ����isVisited����Ϊfalse
		tempDP.SetClusterId(-1);                                 //����Ĭ�ϴ�IDΪ-1
		dadaSets.push_back(tempDP);                              //������ѹ�����ݼ�������
		m++;                                                     //����+1
	}
	dataNum=m-1;                                                   //�������ݶ��󼯺ϴ�СΪi
	
	
	
	//�������ݵ������ڶ���
	for (unsigned long s=0; s<dataNum; s++)
	{
		//SetArrivalPoints(dadaSets[s]);                          //�������ݵ������ڶ���
		for (unsigned long i=0; i<dataNum; i++)
		{
			double distance=GetDistance(dadaSets[i],dadaSets[s]);            //��ȡ���ض���֮��ľ���
			if (distance<=radius&&i!=dadaSets[s].GetDpId())                  //������С�ڰ뾶�������ض����id��dp��id��ͬ��ִ������Ĵ���
			{
				dadaSets[s].GetArrivalPoints().push_back(i);                 //���ض���idѹ��dp�������б���
			}
		}

		dadaSets[s].SetKey(false);                                       //���Ǻ��Ķ�����dp���Ķ����־λ��Ϊfalse

		if (dadaSets[s].GetArrivalPoints().size()>=minPTs)               //��dp���������ݵ���������miniPTsִ���������
		{
			dadaSets[s].SetKey(true);                                    //��dp���Ķ����־λ��Ϊtrue
		}
	}
	AfxMessageBox(_T("WENTI"));
}



//�������
double ClusterAnalysis::GetDistance(DataPoint &dp1, DataPoint dp2)
{
	double distance=0;               //��ʼ������Ϊ0
	for (int i=0; i<DIME_NUM; i++)
	{
		distance+=pow(dp1.GetDimension()[i]-dp2.GetDimension()[i],2);
	}
	return pow(distance,0.5);       //�������ؾ���
}



//�������ݵ��������б�
//void ClusterAnalysis::SetArrivalPoints(DataPoint &dp)
//{
//	for (unsigned long i=0; i<dataNum; i++)
//	{
//		double distance=GetDistance(dadaSets[i],dp);            //��ȡ���ض���֮��ľ���
//		if (distance<=radius&&i!=dp.GetDpId())                  //������С�ڰ뾶�������ض����id��dp��id��ͬ��ִ������Ĵ���
//		{
//			dp.GetArrivalPoints().push_back(i);                 //���ض���idѹ��dp�������б���
//		}
//	}
//
//    dp.SetKey(false);                                       //���Ǻ��Ķ�����dp���Ķ����־λ��Ϊfalse
//
//	if (dp.GetArrivalPoints().size()>=minPTs)               //��dp���������ݵ���������miniPTsִ���������
//	{
//		dp.SetKey(true);                                    //��dp���Ķ����־λ��Ϊtrue
//	}
//}


//�����ݵ������ڵĵ�ִ�о�����������õݹ鷽�����������
void ClusterAnalysis::KeyPointCluster(unsigned long dpID, unsigned long clusterId)
{
	//DataPoint& srcDp=dadaSets[dpID];                           //��ȡ���ݶ���
	if (!dadaSets[dpID].IsKey())     
	{
		return;
	}
	vector<unsigned long>& arrvalPoints=dadaSets[dpID].GetArrivalPoints();       //��ȡ���������ڵ�ID�б�
	for (unsigned long i=0; i<arrvalPoints.size(); i++)
	{
		//DataPoint& desDp=dadaSets[arrvalPoints[i]];           //��ȡ�����ڵ����ݵ�
		if (!dadaSets[arrvalPoints[i]].isVisited())
		{
			dadaSets[arrvalPoints[i]].SetClusterId(clusterId);                   //���øö��������ص�IDΪclusterId,�����ö����������
			dadaSets[arrvalPoints[i]].SetVisited(true);                          //���ö����ѱ�����
			if (dadaSets[arrvalPoints[i]].IsKey())
			{
				KeyPointCluster(dadaSets[arrvalPoints[i]].GetDpId(),clusterId);   //�ݹ�ضԸ���������ݵ������ڵĵ�ִ�о������������������ȷ���
			}
		}
	}
}

//ִ�о������
bool ClusterAnalysis::DoDBSCANRecursive()
{
	unsigned long clusterId=0;                       //����id��������ʼ��Ϊ0
	for (unsigned long i=0; i<dataNum; i++)          //��ÿһ�����ݵ�ִ��
	{
		DataPoint& dp=dadaSets[i];                   //ȡ����i�����ݵ����
		if (!dp.isVisited()&&dp.IsKey())             //������û�����ʹ��������Ǻ��Ķ���ִ�����´���
		{
			dp.SetClusterId(clusterId);              //���øö���������IDΪclusterId
			dp.SetVisited(true);                     //���øö����ѱ����ʹ�
			KeyPointCluster(i,clusterId);            //�Ըö��������ڵ���о���
			clusterId++;                             //clusterId����1
		}
	}
	clusterNum=clusterId;
	return true;                                    //����
}

//���Ѿ��������㷨��������ݼ���д���ļ�

void ClusterAnalysis::WriteToFile()
{
	for(int j=0; j<clusterNum; j++)
	{
	for(unsigned long i=0; i<dataNum;i++)                 //�Դ������ÿ�����ݵ�д���ļ�
		{
			if(dadaSets[i].GetClusterId()==j)
			{
				for(int d=0; d<DIME_NUM ; d++)                    //��ά����Ϣд���ļ�
				outcluster<<dadaSets[i].GetDimension()[d]<<'\t';
				outcluster<< dadaSets[i].GetClusterId() <<endl;       //��������IDд���ļ�
			} 
		}
	}
	PointShowDlg pointShowDlg;

	for (int i=0; i<clusterNum; i++)
	{
		pointShowDlg.mpoint[i].x=dadaSets[i].GetDimension()[0];
		pointShowDlg.mpoint[i].y=dadaSets[i].GetDimension()[1];
		pointShowDlg.mpoint[i].clusterID=dadaSets[i].GetClusterId();
	}
	pointShowDlg.Show(clusterNum,dataNum);
	pointShowDlg.DoModal();
	}
	

