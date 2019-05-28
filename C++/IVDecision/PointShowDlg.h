#pragma once
#include "DataPoint.h"
#include "ClusterAnalysis.h"
#include "new_uturn.h"
// PointShowDlg 对话框
struct ClusterPoint
{
	int x;
	int y;
	int clusterID;
};
class PointShowDlg : public CDialog
{
	DECLARE_DYNAMIC(PointShowDlg)

public:
	PointShowDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~PointShowDlg();
    //vector <DataPoint> PointSets;       //数据集合
    //void DataPlot(vector <DataPoint> PointSets);      //数据集合
	ClusterPoint mpoint[10000];
	void Show(int i, int j);
// 对话框数据
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
};
