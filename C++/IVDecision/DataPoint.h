#pragma once
#include <vector>
using namespace std;
const int DIME_NUM=2;       //����ά�ȣ������������������࣬���Զ���Ϊ��ά
class DataPoint
{
private:
	unsigned long dpID;    //���ݵ�ı��
	double dimension[DIME_NUM];  //ά������
	long clusterId;              //��������ı��
	bool isKey;                  //�Ƿ���Ķ���
	bool visited;               //�Ƿ��ѷ���
	vector <unsigned long> arrivalPoints; //�������ݵ��id�б�

public:
	DataPoint();                 //Ĭ�Ϲ��캯��
	DataPoint(unsigned long dpID,double* dimension ,bool isKey);//���캯��
	unsigned long GetDpId();       //��ȡ���ݵ���
	void SetDpId(unsigned long dpID); //�������ݵ���
	double *GetDimension();          //��ȡά������
	void SetDimension(double * dimension);   //����ά������
	bool IsKey();                    //�ж��Ƿ��Ǻ��Ķ���
	void SetKey(bool isKey);          //���ú��Ķ���
	bool isVisited();                 //�ж��Ƿ��Ѿ����ʹ�
	void SetVisited(bool visited);     //�����Ѿ����ʹ��ı�־
	long GetClusterId();                //��ȡ���ݼ��ı��
	void SetClusterId(long classId);      //�������ݼ��ϱ��
	vector<unsigned long>& GetArrivalPoints();  //��¼��Χ�ڵĵ�id�б�
public:
	~DataPoint(void);
};
