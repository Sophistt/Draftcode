#pragma once
#include "DataPoint.h"
#include "ClusterAnalysis.h"
#include "new_uturn.h"
// PointShowDlg �Ի���
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
	PointShowDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~PointShowDlg();
    //vector <DataPoint> PointSets;       //���ݼ���
    //void DataPlot(vector <DataPoint> PointSets);      //���ݼ���
	ClusterPoint mpoint[10000];
	void Show(int i, int j);
// �Ի�������
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
};
