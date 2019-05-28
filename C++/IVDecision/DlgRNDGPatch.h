#pragma once
#include "afxwin.h"

#include "ZPictureCtrl.h"
#include "GetGPSData.h"
#include "VisionCK.h"
//#include "IVExecution.h"

// CDlgRNDGPatch 对话框

class CDlgRNDGPatch : public CDialog
{
	DECLARE_DYNAMIC(CDlgRNDGPatch)

public:
	CDlgRNDGPatch(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CDlgRNDGPatch();

// 对话框数据
	enum { IDD = IDD_DLG_RNDF };

public:
//	CIVExecution execution;
//	void GetExeParam( CIVExecution &ex );
	CLoadRNDF RNDF;//GPS坐标
	CLoadRNDF RNDFXY;//XY坐标
	struct MAPPOINT *GPath;

	void GetRNDFParam( CLoadRNDF &rndf, struct MAPPOINT *path);

	//RNDF地图
	IplImage *rndfImage;
	double rndf_centerx, rndf_centery;
	//RNDF地图
	IplImage *rndfxyImage;
	double rndfxy_centerx, rndfxy_centery;

	int DIM_M;

	int b_load_rndf;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnRndfqjgps();
public:
	afx_msg void OnBnClickedBtnRndfgbxy();
public:
	CZPictureCtrl m_static_rndf_gpsmap;
public:
	CZPictureCtrl m_static_rndf_xymap;
public:
	afx_msg void OnBnClickedBtnRndfexit();
public:
	virtual BOOL OnInitDialog();
public:
	afx_msg void OnDestroy();
public:
	afx_msg void OnBnClickedBtnRndfmatch();
public:
	double m_edit_rot;
public:
	afx_msg void OnBnClickedBtnRndfrot();
public:
	afx_msg void OnBnClickedBtnFliph();
public:
	afx_msg void OnBnClickedBtnFlipv();
public:
	afx_msg void OnBnClickedGlobalPlan();
};
