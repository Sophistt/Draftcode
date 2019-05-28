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


//读入数据
void ClusterAnalysis::Init(I_Map *map, double radius, int minPTs)
{
	this->radius=radius;                                   //设置半径
	this->minPTs=minPTs;                                   //设置领域最小数据个数
	this->dimNum=DIME_NUM;                                 //设置数据维度
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
    unsigned long m=0;            //数据个数统计


	while (m<=k)
	{
	   DataPoint tempDP;                                   //临时数据对象
	   double tempDimData[DIME_NUM];                       //临时数据点维度信息
	
		tempDimData[0]=mpoint[m].x;
		tempDimData[1]=mpoint[m].y;
	    tempDP.SetDimension(tempDimData);                        //将维度信息存入数据点对象


	    tempDP.SetDpId(m);                                       //将数据点对象ID设置为i
		tempDP.SetVisited(false);                                //数据点对象isVisited设置为false
		tempDP.SetClusterId(-1);                                 //设置默认簇ID为-1
		dadaSets.push_back(tempDP);                              //将对象压入数据集合容器
		m++;                                                     //计数+1
	}
	dataNum=m-1;                                                   //设置数据对象集合大小为i
	
	
	
	//计算数据点领域内对象
	for (unsigned long s=0; s<dataNum; s++)
	{
		//SetArrivalPoints(dadaSets[s]);                          //计算数据点领域内对象
		for (unsigned long i=0; i<dataNum; i++)
		{
			double distance=GetDistance(dadaSets[i],dadaSets[s]);            //获取与特定点之间的距离
			if (distance<=radius&&i!=dadaSets[s].GetDpId())                  //若距离小于半径，并且特定点的id与dp的id不同，执行下面的代码
			{
				dadaSets[s].GetArrivalPoints().push_back(i);                 //将特定点id压入dp的领域列表中
			}
		}

		dadaSets[s].SetKey(false);                                       //若非核心对象，则将dp核心对象标志位设为false

		if (dadaSets[s].GetArrivalPoints().size()>=minPTs)               //若dp领域内数据点数量大于miniPTs执行下面代码
		{
			dadaSets[s].SetKey(true);                                    //将dp核心对象标志位设为true
		}
	}
	AfxMessageBox(_T("WENTI"));
}



//计算距离
double ClusterAnalysis::GetDistance(DataPoint &dp1, DataPoint dp2)
{
	double distance=0;               //初始化距离为0
	for (int i=0; i<DIME_NUM; i++)
	{
		distance+=pow(dp1.GetDimension()[i]-dp2.GetDimension()[i],2);
	}
	return pow(distance,0.5);       //开方返回距离
}



//设置数据点的领域点列表
//void ClusterAnalysis::SetArrivalPoints(DataPoint &dp)
//{
//	for (unsigned long i=0; i<dataNum; i++)
//	{
//		double distance=GetDistance(dadaSets[i],dp);            //获取与特定点之间的距离
//		if (distance<=radius&&i!=dp.GetDpId())                  //若距离小于半径，并且特定点的id与dp的id不同，执行下面的代码
//		{
//			dp.GetArrivalPoints().push_back(i);                 //将特定点id压入dp的领域列表中
//		}
//	}
//
//    dp.SetKey(false);                                       //若非核心对象，则将dp核心对象标志位设为false
//
//	if (dp.GetArrivalPoints().size()>=minPTs)               //若dp领域内数据点数量大于miniPTs执行下面代码
//	{
//		dp.SetKey(true);                                    //将dp核心对象标志位设为true
//	}
//}


//对数据点领域内的点执行聚类操作，采用递归方法，深度优先
void ClusterAnalysis::KeyPointCluster(unsigned long dpID, unsigned long clusterId)
{
	//DataPoint& srcDp=dadaSets[dpID];                           //获取数据对象
	if (!dadaSets[dpID].IsKey())     
	{
		return;
	}
	vector<unsigned long>& arrvalPoints=dadaSets[dpID].GetArrivalPoints();       //获取对象领域内点ID列表
	for (unsigned long i=0; i<arrvalPoints.size(); i++)
	{
		//DataPoint& desDp=dadaSets[arrvalPoints[i]];           //获取领域内点数据点
		if (!dadaSets[arrvalPoints[i]].isVisited())
		{
			dadaSets[arrvalPoints[i]].SetClusterId(clusterId);                   //设置该对象所属簇的ID为clusterId,即将该对象吸入簇中
			dadaSets[arrvalPoints[i]].SetVisited(true);                          //设置对象已被访问
			if (dadaSets[arrvalPoints[i]].IsKey())
			{
				KeyPointCluster(dadaSets[arrvalPoints[i]].GetDpId(),clusterId);   //递归地对该领域点数据的领域内的点执行聚类操作，采用深度优先方法
			}
		}
	}
}

//执行聚类操作
bool ClusterAnalysis::DoDBSCANRecursive()
{
	unsigned long clusterId=0;                       //聚类id计数，初始化为0
	for (unsigned long i=0; i<dataNum; i++)          //对每一个数据点执行
	{
		DataPoint& dp=dadaSets[i];                   //取到第i个数据点对象
		if (!dp.isVisited()&&dp.IsKey())             //若对象没被访问过，并且是核心对象执行以下代码
		{
			dp.SetClusterId(clusterId);              //设置该对象所属簇ID为clusterId
			dp.SetVisited(true);                     //设置该对象已被访问过
			KeyPointCluster(i,clusterId);            //对该对象领域内点进行聚类
			clusterId++;                             //clusterId自增1
		}
	}
	clusterNum=clusterId;
	return true;                                    //返回
}

//将已经过聚类算法处理的数据集合写回文件

void ClusterAnalysis::WriteToFile()
{
	for(int j=0; j<clusterNum; j++)
	{
	for(unsigned long i=0; i<dataNum;i++)                 //对处理过的每个数据点写入文件
		{
			if(dadaSets[i].GetClusterId()==j)
			{
				for(int d=0; d<DIME_NUM ; d++)                    //将维度信息写入文件
				outcluster<<dadaSets[i].GetDimension()[d]<<'\t';
				outcluster<< dadaSets[i].GetClusterId() <<endl;       //将所属簇ID写入文件
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
	

