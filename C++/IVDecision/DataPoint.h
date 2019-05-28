#pragma once
#include <vector>
using namespace std;
const int DIME_NUM=2;       //数据维度，本程序用于坐标点聚类，所以定义为二维
class DataPoint
{
private:
	unsigned long dpID;    //数据点的编号
	double dimension[DIME_NUM];  //维度数据
	long clusterId;              //所属聚类的编号
	bool isKey;                  //是否核心对象
	bool visited;               //是否已访问
	vector <unsigned long> arrivalPoints; //领域数据点的id列表

public:
	DataPoint();                 //默认构造函数
	DataPoint(unsigned long dpID,double* dimension ,bool isKey);//构造函数
	unsigned long GetDpId();       //获取数据点编号
	void SetDpId(unsigned long dpID); //设置数据点编号
	double *GetDimension();          //获取维度数据
	void SetDimension(double * dimension);   //设置维度数据
	bool IsKey();                    //判断是否是核心对象
	void SetKey(bool isKey);          //设置核心对象
	bool isVisited();                 //判定是否已经访问过
	void SetVisited(bool visited);     //设置已经访问过的标志
	long GetClusterId();                //获取数据集的标号
	void SetClusterId(long classId);      //设置数据集合标号
	vector<unsigned long>& GetArrivalPoints();  //记录范围内的点id列表
public:
	~DataPoint(void);
};
