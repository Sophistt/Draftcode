#pragma once
#include "structhead.h"
#include <vector>
#include "ObPoint.h"
#include "ObCluster.h"
typedef struct ObstaclePoint
{
	int isob;                             //�Ƿ����ϰ���
	int isreached;                        //�Ƿ����
	int clusterId;                        //�����ϰ������
};
typedef struct Pos
{
	int x;
	int y;
};
typedef struct Relate
{
	int x;
	int y;
};

typedef struct ObMap
{
	ObstaclePoint obstacle[512][512];     //�Զ����ϰ����ͼ�����Է���Ĳ���
};
class GrowCluster
{
private:
	int m_bf;
	int m_bh;
	int m_lt;
	int m_rt;
	int m_width,m_lengh;     
public:
	GrowCluster(void);
	void Init(I_Map *map);
	void Init(I_Map *map,ObjectWT *ibcpWT, int ib_num);
	void Init_history(I_Map *map,ObjectWT *ibcpWT, int ib_num);
    int ob_count;                        //�ϰ�������
	int cluster_num;
	ObMap *obmap;
	int xmin;
	int xmax;
	int ymin;
	int ymax;

	float v_x_clusterId[5000];
	float v_y_clusterId[5000];

	/*vector <ObPoint> ObPoints1;  */
    vector <ObPoint> ObPoints; 
	vector <ObCluster> obclusters;
	
	void Setdetect(double before,double behind, double left, double right);
	void Setgrow(double width, double length);
	void Clear();
public:
	~GrowCluster(void);
};
