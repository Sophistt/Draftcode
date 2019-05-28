// IVDecisionDlg.h : ͷ�ļ�
//

#pragma once
#include "SocketTransfer.h"
#include "GetGPSData.h"
#include "DlgRNDGPatch.h"
#include "VisionCK.h"
#include "ZPictureCtrl.h"
#include "afxwin.h"
//#include"adasisdatareceiver.h"
//#include "card.h"
//#include "IVExeNew.h"
#include "ComuLcm.h"
#include "VehicleExe.h"
#include "vehicle.h"
#include <vector>
// CIVDecisionDlg �Ի���
class CIVDecisionDlg : public CDialog
{
// ����
public:
	CIVDecisionDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_IVDECISION_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��
//2011 new
	void zDrawLanes_dst( I_Map *map, I_Map *map2, I_Map *map3, CvSeq *points, CvPoint2D64f gps, double m_dir, IplImage *dst, int b_draw_center );
	
////
//��Ϣ��ʾ,��ͼ��ʾ
public:
	CZPictureCtrl m_static_localmap;
	IplImage *LaneDisp;
	CvMemStorage *LaneDispStorage;
	CvSeq *LaneDispSeq;//�洢�㡣

	//RNDF��ͼ
	CZPictureCtrl m_static_rndfmap;
	IplImage *rndfImage;
	IplImage *traceImage;
	double rndf_centerx, rndf_centery;
	int disp_centerx, disp_centery;
	int b_load_rndf;
	double vspeed;
	double aimspeed;
	CvPoint2D64f showp;
	int miss_result;
	void DisplayRNDF( CLoadRNDF RNDF, IplImage *dst, double &centerx, double &centery );//�����ͼ����
	void AddTraceInRndfDisp( IplImage *src, double px, double py, double centerx, double centery );//��ǰ�����rndfͼ��
	void DispTraceInRndf(double px, double py);//���뾭γ�ȣ��켣��rndf��ͼ����ʾ
	/*****************************************
	��ʾͼ���
	*****************************************/
	void resetLaneDisp();
	/*******************************************
	��һ������ʾͼ�ϣ�����ʾ��ͼ���С512*512��ͼ�����  LaneDisp ,  �洢����  LaneDispSeq
	��ǰ�����GPS�㱣���ڶ����У����Ƶ�ͼʱ���»�һ�顣�Ե�ǰ��Ϊ����(256,256)
	����
	px,py ����GPS�������,����
	py -- GPS�����꣬γ��
	rDirection -- INS ����
	RGB -- ��ɫ���ã�0-��ɫ��1-��ɫ��2-��ɫ
	*******************************************/
	void AddPointInLaneDisp(  double px, double py, double rDirection, int RGB );
	void AddPointInLaneDisp2( double px, double py, int id, int RGB );
	void SendMess(CString str);
	//RNDF��ͼ����Ի���
	CDlgRNDGPatch m_rndfpatch;
	void InitCvtRNDF2Img();
	IplImage *rndfmap;
	CvPoint2D64f intergps[200];
	CvPoint2D64f rndfgps[200];
	void Readrndf_FilePath(CString FileName);
	void Readgps_FilePath(CString FileName);

	BOOL blanekeephspd;
	int lukoushuxingnum;
// ʵ��
public:
	CSocketTransfer sockettransfer;
	//CARD card;
	//ComuLcm gps;
//�˶�����ʵ��
public:
	//CIVExeNew m_exetask;
	BOOL b_initgps;
	CvMemStorage* storagemap;

	CvMemStorage* storageApaPath1;
	CvMemStorage* storageApaPath2;
	CvMemStorage* storageApaPath3;

protected:
	HICON m_hIcon;
private:
	///2012.6
	CVehicleExe m_VehicleExe;
	CLaneDriver LaneDriver;
	CVehicle m_Vehicle;
	I_Map *view_vel_Map;
	I_Map *view_ob_Map;
	I_Map *view_danger_Map;
	I_Map *view_v_Map;
	CvSeq* view_planning_road;
	I_Map *view_lane_mid_map;
	CvMemStorage* view_storage_road;
	bool AimFlag;
	int view_seq_num;
	int view_id1;
	int view_id2;
	ComuLcm gps;
	//���沿��
	//XTPPaintTheme m_eTheme;
	void SetTheme();
	void RepositionControls();
	CRect m_rcBorders;
	BOOL m_bInRepositionControls, m_bInitDone;
	IplImage *m_disp;
	//���沿�֡���
	CvMemStorage *storagem;
	CvSeq *seq_road;
	CvSeq *plan_road;
	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnDestroy();

public:
	afx_msg void OnBnClickedDatasendtest();
public:
	afx_msg void OnBnClickedViewonoff();
public:
	afx_msg void OnTimer(UINT_PTR nIDEvent);
public:
	double m_Lat;
	double m_Lon;
	double m_Azimuth;
public:
	afx_msg void OnBnClickedViewoff();
public:
	afx_msg void OnBnClickedReadgps();
public:
	afx_msg void OnBnClickedExecutionstart();
public:
	afx_msg void OnBnClickedExecutionstop();
public:
	afx_msg void OnBnClickedStarttest();
public:
	afx_msg void OnBnClickedConnectGps();
public:
	afx_msg void OnBnClickedPathFollow();

public:
	double m_ccc;
	long cPulse1,cPulse2;
	double card1,card2,c_speed;
	bool sended;
public:
	afx_msg void OnEnChangeEdit1();
public:
	int state11;


	void zDrawLanes(CvSeq *points, CvPoint2D64f gps, double m_dir );
public:
	afx_msg void OnBnClickedButtonzxzb();
public:
	afx_msg void OnBnClickedButtonzdzb();
public:
	afx_msg void OnBnClickedButtonjt();
public:
	afx_msg void OnBnClickedButtonfw();
public:
	afx_msg void OnBnClickedButtonzl();

public:
	CString m_TopDecision;
	CString m_leftfrontdist;
	CString m_leftfrontspd;
	CString m_leftreardist;
	CString m_leftrearspd;
	CString m_rightfrontdist;
	CString m_rightfrontspd;
	CString m_rightreardist;
	CString m_rightrearspd;
	CString m_middlefrontdist;
	CString m_middlefrontspd;
	
	CString m_bobleftlchgnotalooweddisp;
	CString m_bobrightlchgnotalooweddisp;
	CString m_bobfrontlchgnotalooweddisp;
	CString m_bobfrontlchgreqdisp;
public:
	CString m_SecondDecision;
public:
	afx_msg void OnEnChangeEdit6();

public:
	BOOL m_b_see_greenlight;
public:
	afx_msg void OnEnChangeEdit15();
public:
	afx_msg void OnEnChangeEditCtr();
	//CADASISDataReceiver ADASISReceiver;
public:
	afx_msg void OnBnClickedSetSpeed();
public:
	afx_msg void OnEnChangeSetSpeed1();
public:
	afx_msg void OnEnChangeSecondDecision();
public:
	afx_msg void OnBnClickedRadio4();
public:
	afx_msg void OnBnClickedRadio5();
public:
	afx_msg void OnBnClickedRadio6();
public:
	afx_msg void OnBnClickedCheck1();
public:
	afx_msg void OnBnClickedlukouzhi();
public:
	afx_msg void OnBnClickedlukouzuo();
public:
	afx_msg void OnBnClickedlukouyou();
public:
	afx_msg void OnBnClickedlukoudiaotou();
public:
	afx_msg void OnBnClickedlukouchu();
public:
	afx_msg void OnBnClickedSwan();
public:
	afx_msg void OnBnClickedperson();
public:
	afx_msg void OnBnClickedsetlane();
public:
	afx_msg void OnBnClickedlanechgqian();
public:
	afx_msg void OnBnClickedlanechghou();
public:
	afx_msg void OnBnClickedshigong();
public:
	afx_msg void OnBnClickedCheck2();
public:
	afx_msg void OnBnClickedCheck3();
public:
	afx_msg void OnBnClickedapapath1();
public:
	afx_msg void OnBnClickedapapath2();
public:
	afx_msg void OnBnClickedapapath3();
};
