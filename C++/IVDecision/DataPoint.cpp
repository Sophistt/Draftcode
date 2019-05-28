#include "StdAfx.h"
#include "DataPoint.h"

DataPoint::DataPoint()
{
}

DataPoint::~DataPoint(void)
{
}

DataPoint::DataPoint(unsigned long dpID,double* dimension ,bool isKey):isKey(isKey),dpID(dpID)  //���캯��
{
	//����ά������
	for (int i=0; i<DIME_NUM; i++)
	{
		this->dimension[i]=dimension[i];
	}
}

//����ά������
void DataPoint::SetDimension(double *dimension)
{
	for (int i=0; i<DIME_NUM; i++)
	{
		this->dimension[i]=dimension[i];
	}
}

//��ȡά������
double* DataPoint::GetDimension()
{
	return this->dimension;
}

//��ȡ�Ƿ�Ϊ���Ķ���
bool DataPoint::IsKey()
{
	return this->isKey;
}

//���ú��Ķ����־
void DataPoint::SetKey(bool isKey)
{
	this->isKey=isKey;
}

//��ȡDpId
unsigned long DataPoint::GetDpId()
{
	return this->dpID;
}

//����DpId
void DataPoint::SetDpId(unsigned long dpID)
{
	this->dpID=dpID;
}

//�жϷ���
bool DataPoint::isVisited()
{
	return this->visited;
}

//���÷��ʱ�־
void DataPoint::SetVisited(bool visited)
{
	this->visited=visited;
}

//��ȡ���ݼ������
long DataPoint::GetClusterId()
{
	return this->clusterId;
}

//�趨���ݼ������
void DataPoint::SetClusterId(long classId)
{
	this->clusterId=classId;
}

//��ȡ�����ڵ�����б�
vector<unsigned long>& DataPoint::GetArrivalPoints()
{
	return arrivalPoints;
}