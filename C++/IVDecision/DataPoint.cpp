#include "StdAfx.h"
#include "DataPoint.h"

DataPoint::DataPoint()
{
}

DataPoint::~DataPoint(void)
{
}

DataPoint::DataPoint(unsigned long dpID,double* dimension ,bool isKey):isKey(isKey),dpID(dpID)  //构造函数
{
	//传递维度数据
	for (int i=0; i<DIME_NUM; i++)
	{
		this->dimension[i]=dimension[i];
	}
}

//设置维度数据
void DataPoint::SetDimension(double *dimension)
{
	for (int i=0; i<DIME_NUM; i++)
	{
		this->dimension[i]=dimension[i];
	}
}

//获取维度数据
double* DataPoint::GetDimension()
{
	return this->dimension;
}

//获取是否为核心对象
bool DataPoint::IsKey()
{
	return this->isKey;
}

//设置核心对象标志
void DataPoint::SetKey(bool isKey)
{
	this->isKey=isKey;
}

//获取DpId
unsigned long DataPoint::GetDpId()
{
	return this->dpID;
}

//设置DpId
void DataPoint::SetDpId(unsigned long dpID)
{
	this->dpID=dpID;
}

//判断访问
bool DataPoint::isVisited()
{
	return this->visited;
}

//设置访问标志
void DataPoint::SetVisited(bool visited)
{
	this->visited=visited;
}

//获取数据集合序号
long DataPoint::GetClusterId()
{
	return this->clusterId;
}

//设定数据集合序号
void DataPoint::SetClusterId(long classId)
{
	this->clusterId=classId;
}

//获取领域内点序号列表
vector<unsigned long>& DataPoint::GetArrivalPoints()
{
	return arrivalPoints;
}