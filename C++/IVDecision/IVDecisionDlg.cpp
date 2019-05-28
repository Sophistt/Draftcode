// IVDecisionDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "IVDecision.h"
#include "IVDecisionDlg.h"
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

#include <vector>

#include "atlbase.h"
#include "atlstr.h"
#include "comutil.h"
#include <time.h>

CStdioFile file;
using namespace std;
ofstream out123("ceshi3.txt");
ofstream out780("huatu.txt");
ofstream outdmb("dmb.txt");
ofstream outpp("outp.txt");
ofstream outpass(L"C:\\Documents and Settings\\Administrator.LENOVO-02B9C1DF\\桌面\\经过的任务点.txt",ios::app);
ofstream outRealSpeed("RealSpeedData.txt");
ofstream outAimSpeed("AimSpeedData.txt");
ofstream outRealGps("RealGpsData.txt");

ofstream outRongheData("outRongheData.txt");

ofstream outbiandaoconditiondata("outbiandaoconditiondata.txt");

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// CIVDecisionDlg 对话框




CIVDecisionDlg::CIVDecisionDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CIVDecisionDlg::IDD, pParent)
	
	, m_ccc(0)
	, state11(0)
	, m_TopDecision(_T(""))
	, m_SecondDecision(_T(""))
	, m_b_see_greenlight(FALSE)

	, m_leftfrontdist(_T(""))
	, m_leftfrontspd(_T(""))
	, m_leftreardist(_T(""))
	, m_leftrearspd(_T(""))
	, m_rightfrontdist(_T(""))
	, m_rightfrontspd(_T(""))
	, m_rightreardist(_T(""))
	, m_rightrearspd(_T(""))
	, m_middlefrontdist(_T(""))
	, m_middlefrontspd(_T(""))

	, m_bobleftlchgnotalooweddisp(_T(""))
	, m_bobrightlchgnotalooweddisp(_T(""))
	, m_bobfrontlchgnotalooweddisp(_T(""))
	, m_bobfrontlchgreqdisp(_T(""))
{
	AimFlag=false;

	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	//界面部分
	m_rcBorders.SetRectEmpty();
	m_bInRepositionControls = FALSE;
	m_bInitDone = FALSE;
	m_Lat=0;
	m_Lon=0;
	m_Azimuth=0;
	card1 = 0;
	card2 = 0;
	c_speed = 0;
	sended = false;
  CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->zuo_huandao = false;
	app->you_hundao = false;
	app->topDecison = "车辆启动中";
	app->secondDecison = "车辆启动中";
	view_storage_road = cvCreateMemStorage(0);
	view_planning_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), view_storage_road );
	view_vel_Map = new I_Map;
	view_lane_mid_map = new I_Map;
	view_seq_num = 0;
	m_disp = cvCreateImage( cvSize(512, 512), 8, 3);
	storagem = cvCreateMemStorage(0);
	seq_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storagem );
	plan_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storagem );
	storagemap = cvCreateMemStorage(0);
	app->gpsmap_road = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint5D64f_type), storagemap);

	storageApaPath1 = cvCreateMemStorage(0);
	storageApaPath2 = cvCreateMemStorage(0);
	storageApaPath3 = cvCreateMemStorage(0);

	app->gpsmap_road_ApaPath1 = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint2D64f), storageApaPath1);
	app->gpsmap_road_ApaPath2 = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint2D64f), storageApaPath2);
	app->gpsmap_road_ApaPath3 = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint2D64f), storageApaPath3);

	blanekeephspd=false;
	lukoushuxingnum=0;
}

void CIVDecisionDlg::DoDataExchange(CDataExchange* pDX)
{
	DDX_Text(pDX, IDC_LAT, m_Lat);
	DDX_Text(pDX, IDC_LON, m_Lon);
	DDX_Text(pDX, IDC_AZIMUTH, m_Azimuth);
	DDX_Control(pDX, IDC_STATIC_MAP, m_static_localmap);
	CDialog::DoDataExchange(pDX);
	//DDX_Text(pDX, IDC_EDIT6, m_exetask.seq_num);
	DDX_Text(pDX, IDC_EDIT6, /*m_VehicleExe.seq_numm_Vehicle.getSeqNum()*/view_seq_num);
	DDX_Text(pDX, IDC_EDIT2, vspeed);
	DDX_Text(pDX, IDC_EDIT15, aimspeed);
	DDX_Text(pDX, IDC_CARD1, card1);
	DDX_Text(pDX, IDC_CARD2, card2);
	DDX_Text(pDX, IDC_C_SPEED, c_speed);
	DDX_Text(pDX, IDC_TOP_DECISION, m_TopDecision);
	DDX_Text(pDX, IDC_SECOND_DECISION, m_SecondDecision);
	
	DDX_Text(pDX, IDC_leftfrontdist, m_leftfrontdist);
	DDX_Text(pDX, IDC_leftfrontspd, m_leftfrontspd);
	DDX_Text(pDX, IDC_leftreardist, m_leftreardist);
	DDX_Text(pDX, IDC_leftrearspd, m_leftrearspd);
	DDX_Text(pDX, IDC_rightfrontdist, m_rightfrontdist);
	DDX_Text(pDX, IDC_rightfrontspd, m_rightfrontspd);
	DDX_Text(pDX, IDC_rightreardist, m_rightreardist);
	DDX_Text(pDX, IDC_rightrearspd, m_rightrearspd);
	DDX_Text(pDX, IDC_middlefrontdist, m_middlefrontdist);
	DDX_Text(pDX, IDC_middlefrontspd, m_middlefrontspd);
	
	DDX_Text(pDX, IDC_bobleftlchgnotalooweddisp, m_bobleftlchgnotalooweddisp);
	DDX_Text(pDX, IDC_bobrightlchgnotalooweddisp, m_bobrightlchgnotalooweddisp);
	DDX_Text(pDX, IDC_bobfrontlchgnotalooweddisp, m_bobfrontlchgnotalooweddisp);
	DDX_Text(pDX, IDC_bobfrontlchgreqdisp, m_bobfrontlchgreqdisp);
}

BEGIN_MESSAGE_MAP(CIVDecisionDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_DATASENDTEST, &CIVDecisionDlg::OnBnClickedDatasendtest)
	ON_BN_CLICKED(IDC_VIEWONOFF, &CIVDecisionDlg::OnBnClickedViewonoff)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_EXECUTIONSTART, &CIVDecisionDlg::OnBnClickedExecutionstart)
	ON_BN_CLICKED(IDC_EXECUTIONSTOP, &CIVDecisionDlg::OnBnClickedExecutionstop)
	ON_BN_CLICKED(IDC_STARTTEST, &CIVDecisionDlg::OnBnClickedStarttest)
	ON_BN_CLICKED(IDC_BUTTONZXZB, &CIVDecisionDlg::OnBnClickedButtonzxzb)
	ON_BN_CLICKED(IDC_BUTTONZDZB, &CIVDecisionDlg::OnBnClickedButtonzdzb)
	ON_BN_CLICKED(IDC_BUTTONJT, &CIVDecisionDlg::OnBnClickedButtonjt)
	ON_BN_CLICKED(IDC_BUTTONFW, &CIVDecisionDlg::OnBnClickedButtonfw)
	ON_BN_CLICKED(IDC_BUTTONZL, &CIVDecisionDlg::OnBnClickedButtonzl)
	ON_EN_CHANGE(IDC_EDIT6, &CIVDecisionDlg::OnEnChangeEdit6)
	ON_EN_CHANGE(IDC_EDIT15, &CIVDecisionDlg::OnEnChangeEdit15)
	ON_BN_CLICKED(IDC_SET_SPEED, &CIVDecisionDlg::OnBnClickedSetSpeed)
	ON_EN_CHANGE(IDC_SET_SPEED1, &CIVDecisionDlg::OnEnChangeSetSpeed1)
	ON_EN_CHANGE(IDC_SECOND_DECISION, &CIVDecisionDlg::OnEnChangeSecondDecision)
	ON_BN_CLICKED(IDC_CONNECT_GPS, &CIVDecisionDlg::OnBnClickedConnectGps)
	ON_BN_CLICKED(IDC_PATH_FOLLOW, &CIVDecisionDlg::OnBnClickedPathFollow)
	ON_BN_CLICKED(IDC_RADIO4, &CIVDecisionDlg::OnBnClickedRadio4)
	ON_BN_CLICKED(IDC_RADIO5, &CIVDecisionDlg::OnBnClickedRadio5)
	ON_BN_CLICKED(IDC_RADIO6, &CIVDecisionDlg::OnBnClickedRadio6)
	ON_BN_CLICKED(IDC_CHECK1, &CIVDecisionDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_lukou_zhi, &CIVDecisionDlg::OnBnClickedlukouzhi)
	ON_BN_CLICKED(IDC_lukou_zuo, &CIVDecisionDlg::OnBnClickedlukouzuo)
	ON_BN_CLICKED(IDC_lukou_you, &CIVDecisionDlg::OnBnClickedlukouyou)
	ON_BN_CLICKED(IDC_lukou_diaotou, &CIVDecisionDlg::OnBnClickedlukoudiaotou)
	ON_BN_CLICKED(IDC_lukouchu, &CIVDecisionDlg::OnBnClickedlukouchu)
	ON_BN_CLICKED(IDC_Swan, &CIVDecisionDlg::OnBnClickedSwan)
	ON_BN_CLICKED(IDC_person, &CIVDecisionDlg::OnBnClickedperson)
	ON_BN_CLICKED(IDC_setlane, &CIVDecisionDlg::OnBnClickedsetlane)
	ON_BN_CLICKED(IDC_lanechgqian, &CIVDecisionDlg::OnBnClickedlanechgqian)
	ON_BN_CLICKED(IDC_lanechghou, &CIVDecisionDlg::OnBnClickedlanechghou)
	ON_BN_CLICKED(IDC_shigong, &CIVDecisionDlg::OnBnClickedshigong)
	ON_BN_CLICKED(IDC_CHECK2, &CIVDecisionDlg::OnBnClickedCheck2)
	ON_BN_CLICKED(IDC_CHECK3, &CIVDecisionDlg::OnBnClickedCheck3)
	ON_BN_CLICKED(IDC_apapath1, &CIVDecisionDlg::OnBnClickedapapath1)
	ON_BN_CLICKED(IDC_apapath2, &CIVDecisionDlg::OnBnClickedapapath2)
	ON_BN_CLICKED(IDC_apapath3, &CIVDecisionDlg::OnBnClickedapapath3)
END_MESSAGE_MAP()


// CIVDecisionDlg 消息处理程序

BOOL CIVDecisionDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	CFont *font;
	//font.CreatePointFont(120,"隶书");
	font = new CFont;
	font->CreateFont(30,        //设置静态对话框的字体大小
		10,0,0,600, //设置字体颜色
		FALSE,FALSE,   
		0,   
		ANSI_CHARSET,              // nCharSet
		OUT_DEFAULT_PRECIS,        // nOutPrecision
		CLIP_DEFAULT_PRECIS,       // nClipPrecision
		DEFAULT_QUALITY,           // nQuality
		FF_SWISS, _T("Arial")); //设置字体
	GetDlgItem(IDC_Title)->SetFont(font);

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	//this-> ShowWindow(SW_SHOWMAXIMIZED);

	//初始化
	LaneDisp = cvCreateImage(cvSize(512,512), 8, 3);
	cvSet(LaneDisp, CV_RGB(255,255,255));
	m_static_localmap.zShowImage(LaneDisp);

	LaneDispStorage = cvCreateMemStorage(0); 
	LaneDispSeq = cvCreateSeq(CV_32FC3,sizeof(CvSeq),sizeof(CvPoint3D32f),LaneDispStorage); //保存经过的路点

	rndfImage = cvCreateImage(cvSize(10000,10000), 8, 3);
	traceImage = cvCreateImage(cvSize(10000,10000), 8, 3);
	cvZero(traceImage);
	rndf_centerx = 0;
	rndf_centery = 0;
	disp_centerx = 2000;
	disp_centery = 2000;

	b_load_rndf = 0;

	//GPS建立及获得数据 6.1日改动  combobox被我误删了
	//((CComboBox*)GetDlgItem(IDC_COMBO_COM))->ResetContent();
	//for(int i=1;i<10;i++)
	//{
	//	CString strTemp;
	//	strTemp.Format("COM%d",i);
	//	((CComboBox*)GetDlgItem(IDC_COMBO_COM))->AddString(strTemp);
	//}
	//((CComboBox*)GetDlgItem(IDC_COMBO_COM))->SetCurSel(2);//设置第n行内容为显示的内容。

	//b_initgps = FALSE;//////6.1日改动

	//gps.Set_SendAddress("udpm://239.255.76.69:7669?ttl=1");
	//gps.Set_SendChannel("GPS");
	//CString filename1 = "D:\\file2\\lead.txt";
	//Readgps_FilePath(filename1);
	//CString filename2 = "D:\\file2\\rndf.txt";
	//Readrndf_FilePath(filename2);
	CEdit* m_Edit1, *m_Edit2, *m_Edit3, *m_Edit4, *m_Edit5;  
	m_Edit1 = (CEdit*) GetDlgItem(IDC_SET_SPEED1);
	m_Edit2 = (CEdit*) GetDlgItem(IDC_SET_SPEED2);
	m_Edit3 = (CEdit*) GetDlgItem(IDC_SET_SPEED3);
	m_Edit4 = (CEdit*) GetDlgItem(IDC_SET_SPEED4);
	m_Edit5 = (CEdit*) GetDlgItem(IDC_SET_SPEED5);
	m_Edit1->SetWindowText(_T("75"));
	m_Edit2->SetWindowText(_T("20"));
	m_Edit3->SetWindowText(_T("10"));
	m_Edit4->SetWindowText(_T("10"));
	m_Edit5->SetWindowText(_T("10"));

	CEdit* m_Edit_laneseq, *m_Edit_lanenum, *m_Edit_spdiniset, *m_Edit_laneexist;  
	m_Edit_laneseq = (CEdit*) GetDlgItem(IDC_laneseq);
	m_Edit_lanenum = (CEdit*) GetDlgItem(IDC_lanenum);
	m_Edit_spdiniset = (CEdit*) GetDlgItem(IDC_speediniset);
	m_Edit_laneexist = (CEdit*) GetDlgItem(IDC_laneexist);

	m_Edit_laneseq->SetWindowText(_T("1"));
	m_Edit_lanenum->SetWindowText(_T("1"));
	m_Edit_spdiniset->SetWindowText(_T("10"));
	m_Edit_laneexist->SetWindowText(_T("1"));

	CEdit *m_Edit_ApaStrtLat, *m_Edit_ApaStrtLng, *m_Edit_ApaEndLat, *m_Edit_ApaEndLng;  
	m_Edit_ApaStrtLat = (CEdit*) GetDlgItem(IDC_ApaStrtLat);
	m_Edit_ApaStrtLng = (CEdit*) GetDlgItem(IDC_ApaStrtLng);
	m_Edit_ApaEndLat = (CEdit*) GetDlgItem(IDC_ApaEndLat);
	m_Edit_ApaEndLng = (CEdit*) GetDlgItem(IDC_ApaEndLng);

	m_Edit_ApaStrtLat->SetWindowText(_T("0"));
	m_Edit_ApaStrtLng->SetWindowText(_T("0"));
	m_Edit_ApaEndLat->SetWindowText(_T("0"));
	m_Edit_ApaEndLng->SetWindowText(_T("0"));

	((CButton *)GetDlgItem(IDC_RADIO6))->SetCheck(true);

	lukoushuxingnum=0;

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

//界面部分

void CIVDecisionDlg::RepositionControls()
{
	if (m_bInRepositionControls || !m_bInitDone)
		return;
	
}

void CIVDecisionDlg::SetTheme()
{
	//XTPPaintTheme m_eTheme = (XTPPaintTheme)(xtpThemeOffice2003);
	//CXTPPaintManager::SetTheme(m_eTheme);
	//RedrawWindow(0, 0, RDW_ALLCHILDREN|RDW_INVALIDATE);
}
//界面部分——

void CIVDecisionDlg::OnDestroy()
{

	CDialog::OnDestroy();

	// TODO: 在此处添加消息处理程序代码
	if(LaneDisp)  cvReleaseImage(&LaneDisp);
	if(rndfImage)  cvReleaseImage(&rndfImage);
	if(traceImage)  cvReleaseImage(&traceImage);
	cvClearSeq(LaneDispSeq);
	cvReleaseMemStorage(&LaneDispStorage);
	//2012.7.24
	delete view_vel_Map ;
	delete view_lane_mid_map ;
	if(m_disp) cvReleaseImage(&m_disp);
	cvReleaseMemStorage(&view_storage_road);
	cvReleaseMemStorage(&storagem);
	cvReleaseMemStorage(&storagemap);

	cvReleaseMemStorage(&storageApaPath1);
	cvReleaseMemStorage(&storageApaPath2);
	cvReleaseMemStorage(&storageApaPath3);
}

void CIVDecisionDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CIVDecisionDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标显示。
//
HCURSOR CIVDecisionDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CIVDecisionDlg::OnBnClickedConnectGps()
{
	bool bGpsConnect = m_VehicleExe.m_GpsData.SetCom( "COM3", "115200" );
	SetTimer(4, 100, NULL);
}

void CIVDecisionDlg::OnBnClickedPathFollow()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint5D64f_type aim_point_his;
	aim_point_his.x=0;
	aim_point_his.y=0;
	using namespace std;
	bool bwuxiaodian=0;
	if(AimFlag)
	{
		AfxMessageBox("已载入！");
	}
	else
	{
		CString FileExtension;
		CString DefExtension;
		FileExtension="TXT File(*.txt)|*.txt|";
		FileExtension+="All Files(*.*)|*.*|";
		CFileDialog openFileDialog
			(true,
			DefExtension,
			NULL,
			OFN_PATHMUSTEXIST   |   OFN_HIDEREADONLY   |   OFN_ALLOWMULTISELECT,
			FileExtension,this);
		if( openFileDialog.DoModal ()==IDOK )
		{
			CString FileName,FileExt;
			FileName=openFileDialog.GetPathName();
			ifstream f1(FileName); 

			if (!f1) {
				AfxMessageBox("map.txt file not open!"); 
				exit(1); 
			}

			string temp = "";
			int j;

			while(!f1.eof())
			{
				CString strLatitude = ""; 
				CString strLongitude = "";
				CString strlaneseq = "";
				CString strlanenum = "";

				CString strLatitude_left = ""; 
				CString strLongitude_left = "";
				CString strLatitude_right = ""; 
				CString strLongitude_right = "";
				CString bflag_left = "";
				CString bflag_right = "";

				CString strspdiniset = "";
				CString strlaneexist = "";

				int SectionID = 0;
				f1>>temp;

				int ct = temp.size();

				for(j = 0;j<ct;j++)
				{
					if(',' == temp[j])
					{
						SectionID++;
						continue;
					}
					if(temp[j]==NULL)break;
					else
					{
						switch(SectionID)
						{ 
						case 0: 
							strLatitude += temp[j];
							break;	

						case 1: 
							strLongitude += temp[j];
							break;

						case 2: 
							strLatitude_left += temp[j];
							break;

						case 3: 
							strLongitude_left += temp[j];
							break;

						case 4: 
							strLatitude_right += temp[j];
							break;

						case 5: 
							strLongitude_right += temp[j];
							break;

						case 6: 
							bflag_left += temp[j];
							break;

						case 7: 
							bflag_right += temp[j];
							break;

						case 8: 
							strspdiniset += temp[j];
							break;

						case 9: 
							strlaneexist += temp[j];
							break;

						default:
							break;
						}
					}
				}
				CvPoint5D64f_type aim_point;
				aim_point.x = atof(strLatitude);
				aim_point.y = atof(strLongitude);
				aim_point.z = 0;
				aim_point.laneseq = 0;
				aim_point.lanenum_ = 0;
				aim_point.x_left = atof(strLatitude_left);
				aim_point.y_left = atof(strLongitude_left);
				aim_point.x_right = atof(strLatitude_right);
				aim_point.y_right = atof(strLongitude_right);
				aim_point.b_left = atoi(bflag_left);
				aim_point.b_right = atoi(bflag_right);
				/*
				aim_point.spdiniset = atoi(strspdiniset);
				aim_point.laneexist = atoi(strlaneexist);
				*/
				aim_point.spdiniset = 10/3.6;
				aim_point.laneexist = 1;

				if(aim_point.x==11111&&aim_point.y==11111)
					aim_point_his.z=1;
				else if(aim_point.x==22222&&aim_point.y==22222)
					aim_point_his.z=2;
				else if(aim_point.x==33333&&aim_point.y==33333)
					aim_point_his.z=3;
				else if(aim_point.x==44444&&aim_point.y==44444)
					aim_point_his.z=4;
				else if(aim_point.x==66666&&aim_point.y==66666)
					aim_point_his.z=6;
				else if(aim_point.x==77777&&aim_point.y==77777)
					aim_point_his.z=7;
				else if(aim_point.x==88888&&aim_point.y==88888)
					aim_point_his.z=8;
				else if(aim_point.x==99999&&aim_point.y==99999)
					aim_point_his.z=9;
				else if(aim_point.x==292929&&aim_point.y==292929)
					aim_point_his.z=29;
				else if(aim_point.x==303030&&aim_point.y==303030)
				{
					aim_point_his.z=30;
					bwuxiaodian=0;
				}
				else
				{
					if(bwuxiaodian==0 && (aim_point_his.x!=0) && (aim_point_his.y!=0))
						cvSeqPush(app->gpsmap_road, &aim_point_his);
					if(aim_point_his.z==29)
						bwuxiaodian=1;
					aim_point_his=aim_point;
				}
			}
			f1.close();
			AimFlag = true;
			app->bGpsMapLoad = true;
			AfxMessageBox("目标点已载入");
		}
	}
}

void CIVDecisionDlg::OnBnClickedDatasendtest()
{
	// TODO: 在此添加控件通知处理程序代码
	
	
	//app->lanekeeptime = 10000;
	SetTimer(2,100,NULL);
	
}

void CIVDecisionDlg::OnBnClickedViewonoff()
{
	//// TODO: 在此添加控件通知处理程序代码
	//LRESULT errcode=SUCCESS;
	//errcode = card.CARD_ON();
	//if(errcode != SUCCESS)
	//{
	//	//AfxMessageBox("板卡打开失败");
	//}
	
	//app->lanekeeptime = 10000;
	SetTimer(1,100,NULL);

}

void CIVDecisionDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	if(nIDEvent==5)
	{
		app->systemtimer10ms++;
	}
	if(nIDEvent==2)
	{
		if(app->systemtimer10ms<app->systemtimer10ms_his+10)
			app->systemtimer10ms=app->systemtimer10ms_his+10;
		app->systemtimer10ms_his=app->systemtimer10ms;

		if(1)
			app->realsteerangle=-m_VehicleExe.control.udpdatarcv.realSteerAngle;
	}
	if(nIDEvent==4)
	{
		app->critical_section.Lock();
		double lat = app->GPS_Point.x;
		double lon = app->GPS_Point.y;
		double azim = app->GPS_Direction;
		double speed = app->GPS_Speed;
		app->critical_section.Unlock();
		AddPointInLaneDisp(lat, lon, azim, 2);
	}
	if(nIDEvent==1)
	{
		if(app->leftturnreq==false)
			app->leftturnreq_proc=false;
		if(app->rightturnreq==false)
			app->rightturnreq_proc=false;
		if((app->leftturnreq==false)&&(app->rightturnreq==false))
			app->lanekeeptime=app->lanekeeptime+0.1;
		if(app->leftturnreq||app->rightturnreq)
			app->lanekeeptime=0;
		if((app->leftturnreq_proc==false)&&(app->rightturnreq_proc==false)&&(app->leftturnreq||app->rightturnreq||app->lanekeeptime<10))
			app->lanekeepreq=true;
		else
			app->lanekeepreq=false;

		app->lanekeepreqhspdapp=blanekeephspd;

		CString str;
		
		if (m_VehicleExe.able_ChangeLane == 0 )
		{
			str = "饶章过程中";
		}
		else
		{
			 str= "不饶张";
		}
		
		CString str111;
		if(app->light_res==1)
			str111 = "绿灯";
		else if(app->light_res==2)
			str111 = "红灯";
		else
			str111 = "no";
		GetDlgItem(IDC_EDIT13)->SetWindowTextA(str111);

		CString strfx;
		if(m_VehicleExe.m_taskpublic == IVTASK_LANE)
			strfx = "路上";
		else if(m_VehicleExe.lukou_fx == 1)
			strfx = "直行";
		else if(m_VehicleExe.lukou_fx == 3)
			strfx = "左转";
		else if(m_VehicleExe.lukou_fx == 2)
			strfx = "右转";
		else if(m_VehicleExe.lukou_fx == 4)
			strfx = "掉头";
		else if(m_VehicleExe.lukou_fx == 6)
			strfx = "施工路段";
		else if(m_VehicleExe.lukou_fx == 7)
			strfx = "行人";
		else if(m_VehicleExe.lukou_fx == 8)
			strfx = "S弯";
		GetDlgItem(IDC_EDIT14)->SetWindowTextA(strfx);

		CTime t = CTime::GetCurrentTime();
		CString strTime = t.Format("%Y-%m-%d %H:%M:%S");
		m_Lat=app->GPS_Point.x;
		m_Lon=app->GPS_Point.y;
		m_Azimuth=app->GPS_Direction;
		outRealGps<<setprecision(11)<<m_Lat<<"	"<<m_Lon<<"	"<<m_Azimuth<<"	"<<strTime<<endl;
		vspeed = app->GPS_Speed*3.6;
		outRealSpeed<<vspeed<<"	"<<strTime<<endl;
		aimspeed = m_VehicleExe.control.m_uDesire*3.6;
		outAimSpeed<<aimspeed<<"	"<<strTime<<endl;
		CString gpstodecision;
		
		gpstodecision.Format("%f,%f,%f;" , app->GPS_Direction,app->GPS_Point.x ,app->GPS_Point.y );
		gps.send_messagegps(gpstodecision);

		app->critical_adasisupdate.Lock();
		m_VehicleExe.SendtoPerc();
		miss_result = m_VehicleExe.PassMissionPoint(app->GPS_Point,app->GPS_Direction);
		app->critical_adasisupdate.Unlock();
		{
			SetEvent(m_VehicleExe.m_MissionEvent);
		}

		
		cvSet(m_disp, cvScalarAll(255));
		//view_vel_Map = m_Vehicle.getVelMap();
		//ASSERT(view_vel_Map);
		view_planning_road = m_Vehicle.getPlanRoad();
		ASSERT(view_planning_road);
		//view_lane_mid_map = m_Vehicle.getLaneMidMap();
		app->critical_xinmap.Lock();
		
		/*view_lane_mid_map = m_VehicleExe.dy_map;
		view_ob_Map = m_VehicleExe.dy_map;*/
		//view_lane_mid_map = m_VehicleExe.getDyMap();
		view_ob_Map = m_VehicleExe.getDyMap();
		view_danger_Map = m_Vehicle.getDyMap();
	
		app->critical_xinmap.Unlock();
		ASSERT(view_lane_mid_map);
		//view_ob_Map = m_Vehicle.getXinMap();
		
		
		//view_v_Map = m_Vehicle.getVMap();
		
		////////////////2011 new
		//CString gpstodecision;
		
		//gpstodecision.Format(/*%sGGPS*/"%f,%f,%f;" ,/*"%" ,*/ app->GPS_Direction,app->GPS_Point.x ,app->GPS_Point.y );
		//gps.send_messagegps(gpstodecision);

		CString str1;
		str1.Format("%f   %d   %f    %0.2f",m_VehicleExe.control.m_controlParam.steerAngle, m_VehicleExe.control.m_controlParam.brake,
					m_VehicleExe.control.m_controlParam.driveTorque, m_VehicleExe.control.m_uDesire);
		GetDlgItem(IDC_EDIT_CTR)->SetWindowTextA(str1);
		
/////////////////2012.6
		//card.read_1784_CNT(1);
		//cPulse1=card.cnt_in[1];
		//app->cardl=double(cPulse1)*2.08/4000;

		//card.read_1784_CNT(3);
		//cPulse2=-card.cnt_in[3];
		//app->cardr=double(cPulse2)*2.08/4000;


		////app->cspeed = -(5*(card1 - app->cardl) + 5*(card2 - app->cardr));
		//app->cspeed = -(10*(card1 - app->cardl) );
		//c_speed = app->cspeed*3.6;
		//card1 = app->cardl;
		//card2 = app->cardr;
		//	

		m_leftfrontdist.Format("%f",m_VehicleExe.lanedriverstate.y_left_front[0]);
		m_leftfrontspd.Format("%f",m_VehicleExe.lanedriverstate.spdrelleftfront*3.6);
		m_leftreardist.Format("%f",m_VehicleExe.lanedriverstate.y_left_rear[0]);
		m_leftrearspd.Format("%f",m_VehicleExe.lanedriverstate.spdrelleftrear*3.6);
		m_rightfrontdist.Format("%f",m_VehicleExe.lanedriverstate.y_right_front[0]);
		m_rightfrontspd.Format("%f",m_VehicleExe.lanedriverstate.spdrelrightfront*3.6);
		m_rightreardist.Format("%f",m_VehicleExe.lanedriverstate.y_right_rear[0]);
		m_rightrearspd.Format("%f",m_VehicleExe.lanedriverstate.spdrelrightrear*3.6);
		m_middlefrontdist.Format("%f",m_VehicleExe.lanedriverstate.y_middle_front[0]);
		m_middlefrontspd.Format("%f",m_VehicleExe.lanedriverstate.spdrelmidfront*3.6);
		
		m_bobleftlchgnotalooweddisp.Format("%d",m_VehicleExe.lanedriverstate.bobleftlchgnotalooweddisp);
		m_bobrightlchgnotalooweddisp.Format("%d",m_VehicleExe.lanedriverstate.bobrightlchgnotalooweddisp);
		m_bobfrontlchgnotalooweddisp.Format("%d",m_VehicleExe.lanedriverstate.bobfrontlchgnotalooweddisp);
		m_bobfrontlchgreqdisp.Format("%d",m_VehicleExe.lanedriverstate.bobfrontlchgreqdisp);

		outbiandaoconditiondata<<m_VehicleExe.m_taskpublic<<","<<m_VehicleExe.able_ChangeLane<<","<<app->GPS_Speed<<","
			<<app->spdredacc<<","<<app->continue_brake<<","<<app->continue_spdred<<","
			<<app->drive_state<<","<<app->drive_obstacle_update<<","<<app->drive_obstacle2_update<<","
			<<m_VehicleExe.lanedriverstate.y_left_front[0]<<","<<m_VehicleExe.lanedriverstate.spdrelleftfront*3.6<<","
			<<m_VehicleExe.lanedriverstate.y_left_rear[0]<<","<<m_VehicleExe.lanedriverstate.spdrelleftrear*3.6<<","
			<<m_VehicleExe.lanedriverstate.y_right_front[0]<<","<<m_VehicleExe.lanedriverstate.spdrelrightfront*3.6<<","
			<<m_VehicleExe.lanedriverstate.y_right_rear[0]<<","<<m_VehicleExe.lanedriverstate.spdrelrightrear*3.6<<","
			<<m_VehicleExe.lanedriverstate.y_middle_front[0]<<","<<m_VehicleExe.lanedriverstate.spdrelmidfront*3.6<<","
			<<m_VehicleExe.lanedriverstate.bobleftlchgnotalooweddisp<<","
			<<m_VehicleExe.lanedriverstate.bobrightlchgnotalooweddisp<<","
			<<m_VehicleExe.lanedriverstate.bobfrontlchgnotalooweddisp<<","
			<<m_VehicleExe.lanedriverstate.bobfrontlchgreqdisp<<","
			<<m_VehicleExe.control.m_uDesire<<endl;

		//m_ccc= m_exetask.m_task;
		state11 = app->state1;
		m_TopDecision = app->topDecison;
		m_SecondDecision = app->secondDecison;
		CString m_SecondDecisiontmp;
		m_SecondDecisiontmp.Format("%s:%d,%d,%d,%d,%d,%d",m_SecondDecision,(int)(app->leftturnreq), (int)(app->leftturnreq_proc),(int)(app->rightturnreq), (int)(app->rightturnreq_proc), (int)(app->lanekeepreq), (int)(app->lanekeepreqhspdapp));
		m_SecondDecision.Format("%s",m_SecondDecisiontmp);
			app->critical_planningroad.Lock();
		//if(view_planning_road->total > 1)
		{	
			//outpp<<"1"<<endl;
				
			zDrawLanes_dst( view_ob_Map,view_danger_Map, view_v_Map,view_planning_road, app->GPS_Point, app->GPS_Direction, m_disp, 0);
					
			//zDrawLanes_dst( view_lane_mid_map,view_planning_road, app->GPS_Point, app->GPS_Direction, m_disp, 1);
			//outpp<<"2"<<endl;		
			app->critical_planningroad.Unlock();

			app->critical_planningroad.Lock();
			m_static_localmap.zShowImage(m_disp);
			
		
		}
		app->critical_planningroad.Unlock();

	}

	
    UpdateData(false);
	CDialog::OnTimer(nIDEvent);
}

void CIVDecisionDlg::OnBnClickedViewoff()
{
	// TODO: 在此添加控件通知处理程序代码
	//card.CARD_OFF();
	KillTimer(1);
}

void CIVDecisionDlg::OnBnClickedReadgps()
{
	// TODO: 在此添加控件通知处理程序代码
	SendMess("%G100T0V0B-25500S0K0A37H0L0END;");
	

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	if(AimFlag||!app->Lead_pt.empty())
	{
		AfxMessageBox("已载入！");
	}

	
	else
	{
		// TODO: 在此添加命令处理程序代码
		// TODO: 在此添加命令处理程序代码、
		CString FileExtension;
		CString DefExtension;
		//DefExtension="accdb";
		FileExtension="TXT File(*.txt)|*.txt|";
		FileExtension+="All Files(*.*)|*.*|";
		CFileDialog openFileDialog
		(true,//
		DefExtension,//
		NULL,//
		OFN_PATHMUSTEXIST   |   OFN_HIDEREADONLY   |   OFN_ALLOWMULTISELECT,//
		FileExtension,this);//
		//openFileDialog.SetDefExt();
		if( openFileDialog.DoModal ()==IDOK )
		{
			CString FileName,FileExt;
			FileName=openFileDialog.GetFileName();
			//FileExt=openFileDialog.GetFileExt();
			
			ifstream f1(FileName); 
			
			if (!f1) { //当f1打开失败时进行错误处理 
				cerr<<"map.txt file not open!"<<endl; 
				exit(1); 
			}
			char temp[50];
			
			//string temp = "";
			int i,j;
			while(!f1.eof())
			{
				CString point_id = "";
				CString strLatitude = ""; 
				CString strLongitude = "";
				CString strheight = "";
				CString strparam1 = "";
				CString strparam2 = "";
				int SectionID = 0;
				f1.getline(temp,50);
				//f1>>temp;
				if(temp[0]==NULL)
						break;
				for(j = 0;j<50;j++)
				{
					if('	' == temp[j])
					{
						SectionID++;
						continue;
					}
					if('/n' == temp[j])
					{
						SectionID++;
						break;
					}
					if(temp[j]==NULL)
						break;
					else // 数据
					{
						switch(SectionID)
						{ 

						case 0: 
							point_id += temp[j];
							break;	

						case 1: 
							strLongitude += temp[j];
							break;

						case 2: 
							strLatitude += temp[j];
							break;
						case 3: 
							strheight += temp[j];
							break;
						case 4: 
							strparam1 += temp[j];
							break;
						case 5: 
							strparam2 += temp[j];
							break;
						default:
							break;
						}
					}
				}
				LEAD lead_point;
				lead_point.id = atof(point_id);
				lead_point.lat = atof(strLatitude);
				lead_point.lng = atof(strLongitude);
				lead_point.height = atof(strheight);
				lead_point.param1 = atof(strparam1);
				lead_point.param2 = atof(strparam2);
				//aim_point.z = atof(strAzimuth);
				//double Dir = atof(strAzimuth);
				AddPointInLaneDisp2( lead_point.lat, lead_point.lng, lead_point.id, 1 );

				app->Lead_pt.push(lead_point);
			}
			f1.close();
			AimFlag = TRUE;
			AfxMessageBox("目标点已载入");
		}

	}

}


void CIVDecisionDlg::OnBnClickedExecutionstart()
{
	// TODO: 在此添加控件通知处理程序代码
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	//app->lanekeeptime = 10000;

	app->systemtimer10ms=0;
	app->systemtimer10ms_his=0;

	// 启动GPS
	b_initgps = m_VehicleExe.m_GpsData.SetCom( "COM3", "115200" );
	Sleep(1000);
	while(1)
	{
		if(app->GPS_Point.x!=0&&app->GPS_Point.y!=0)
			break;
		Sleep(100);
	}
	
	if (m_VehicleExe.control.startCAN() != 0)	//控制改
	{//启动CAN
		exit(0);
	}
	AfxMessageBox("CAN启动成功");


	m_VehicleExe.StartUdpSent();
	//sockettransfer.OpenSignListen();
	Sleep(200);
	//***********//
	m_VehicleExe.StartProc();
	Sleep(50);
	m_VehicleExe.StartCtrl();
	Sleep(50);
	m_VehicleExe.StartPath();
	Sleep(50);
	m_VehicleExe.StartLight();
	Sleep(50);
	m_VehicleExe.StartIbeoMap();
	Sleep(50);
	m_VehicleExe.StartUdpRcv();

	Sleep(100);
	
	m_VehicleExe.StartIbeo2();//前方ibeo

	SetTimer(5,10,NULL);
	
	
	//m_exetask.StartSendGPS();
//	sockettransfer.OpenSignListen();
	//Sleep(50);
	
}

void CIVDecisionDlg::OnBnClickedExecutionstop()
{
	// TODO: 在此添加控件通知处理程序代码
	//ADASISReceiver.OpenADASISCAN();
	KillTimer(2);
}

/*****************************************
显示图清除
*****************************************/
void CIVDecisionDlg::resetLaneDisp()
{
	cvSet(LaneDisp, CV_RGB(255,255,255));
	m_static_localmap.zShowImage(LaneDisp);
	cvClearSeq(LaneDispSeq);
}

/*******************************************
画一点在显示图上，并显示。图像大小512*512。图像变量  LaneDisp ,  存储变量  LaneDispSeq
当前输入的GPS点保存在队列中，绘制地图时重新画一遍。以当前点为中心(256,256)
输入
    px,py －－GPS点的坐标,经度
	py -- GPS点坐标，纬度
	rDirection -- INS 方向
	RGB -- 颜色设置，0-红色，1-绿色，2-蓝色
*******************************************/
void CIVDecisionDlg::AddPointInLaneDisp( double px, double py, double rDirection, int RGB )
{		
	CvPoint3D32f pt = cvPoint3D32f(px, py, double(RGB));
	cvSeqPush(LaneDispSeq, &pt);
	cvSet(LaneDisp, CV_RGB(255,255,255));
	CvPoint2D64f rPosition,a_point;
	a_point.x = pt.x;
	a_point.y = pt.y;
	if(LaneDispSeq->total > 0)
	{
		for(int i=0; i<LaneDispSeq->total; i++)
		{
			CvPoint3D32f point = * (CvPoint3D32f*)cvGetSeqElem( LaneDispSeq, i ); 
			rPosition.x = point.x;
			rPosition.y = point.y;
			Map_Point mp = m_Vehicle.m_GpsData.APiontConver( a_point, rPosition, rDirection );

			if( (int)(point.z+0.5) == 1 )
				cvCircle( LaneDisp, cvPoint(mp.x,mp.y), 1, CV_RGB(0,255,0), 1);
			else if((int)(point.z+0.5) == 2)
				cvCircle( LaneDisp, cvPoint(mp.x,mp.y), 1, CV_RGB(0,0,255), 1);
			else
				cvCircle( LaneDisp, cvPoint(mp.x,mp.y), 1, CV_RGB(255,0,0), 1);
		}
		
	}
	m_static_localmap.zShowImage(LaneDisp);

	if(LaneDispSeq->total >= 100000)
	{
		cvSeqPop(LaneDispSeq);
	}
	
}
void CIVDecisionDlg::AddPointInLaneDisp2( double px, double py, int id, int RGB )
{		
	CvPoint3D32f pt = cvPoint3D32f(px, py, id);
	cvSeqPush(LaneDispSeq, &pt);
	cvSet(LaneDisp, CV_RGB(255,255,255));
	CvPoint2D64f rPosition,a_point;
	a_point.x = pt.x;
	a_point.y = pt.y;
	if(LaneDispSeq->total>0)
	{
		for(int i=0; i<LaneDispSeq->total; i++)
		{
			CvPoint3D32f point = * (CvPoint3D32f*)cvGetSeqElem( LaneDispSeq, i ); 
			rPosition.x = point.x;
			rPosition.y = point.y;
			Map_Point mp = m_Vehicle.m_GpsData.APiontConver( rPosition, a_point, 0 );

			
			cvCircle( LaneDisp, cvPoint(mp.x/3,mp.y/3), 3, CV_RGB(0,255,0), 2);

			CString str;
			str.Format("%d",(int)point.z);
			CvFont font;

			double hscale = 0.8;
			double vscale = 0.8;
			int linewidth = 2;
			cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);
			cvPutText(LaneDisp, str, cvPoint(mp.x/3,mp.y/3), &font,CV_RGB(200, 200, 100));
			
			m_static_rndfmap.zShowImage(LaneDisp);
		}
		
	}
	m_static_rndfmap.zShowImage(LaneDisp);

	if(LaneDispSeq->total >= 100000)
	{
		cvSeqPop(LaneDispSeq);
	}


	
}
void CIVDecisionDlg::OnBnClickedStarttest()
{
	// TODO: 在此添加控件通知处理程序代码
	//DataTest a;
	//a.DoModal();

}


void CIVDecisionDlg::DisplayRNDF( CLoadRNDF RNDF, IplImage *dst, double &centerx, double &centery )
{
	//5000米×5000米地图，每个像素代表0.5米，粗略为经纬度的小数点后5位
	cvSet(dst, CV_RGB(255,255,255));
	int nsegment = RNDF.m_mapInfo.segment_num;

	//第一段的第一点做为起始点，是图像中心，(5000,5000)
	centerx = 0;
	centery = 0;
	for(int i=0; i<nsegment; i++)
	{
		int nlane = RNDF.m_mapInfo.pSegment[i].lane_num;
		for(int j=0; j<nlane; j++)//每条路
		{
			int nwaypoint = RNDF.m_mapInfo.pSegment[i].pLane[j].waypoints_num;//路点
			for (int k1=0; k1<nwaypoint; k1++)
			{
				double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].x;
				double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].y;
				if(i==0 && j==0 && k1==0)
				{
					centerx = wx;
					centery = wy;

					int px = cvRound((wx-centerx)*200000)+5000;
					int py = cvRound((wy-centery)*200000)+5000;

					cvCircle(dst, cvPoint(px,py), 5, CV_RGB(0,255,100), CV_FILLED);
				}
				int px = cvRound((wx-centerx)*200000/10*2)+5000;
				int py = cvRound((wy-centery)*200000/10*2)+5000;
				cvCircle(dst, cvPoint(px,py), 3, CV_RGB(255,0,0));
				if(k1 > 0)
				{
					double wx0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1-1].x;
					double wy0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1-1].y;

					int px0 = cvRound((wx0-centerx)*200000/10*2)+5000;
					int py0 = cvRound((wy0-centery)*200000/10*2)+5000;
					cvLine(dst, cvPoint(px,py), cvPoint(px0,py0), CV_RGB(0,0,255), 2);
				}

			}
		}
	}


	//for(int i=0; i<nsegment; i++)//每段
	//{
	//	int nlane = RNDF.m_mapInfo.pSegment[i].lane_num;
	//	for(int j=0; j<nlane; j++)//每条路
	//	{
	//		int nwaypoint = RNDF.m_mapInfo.pSegment[i].pLane[j].waypoints_num;//路点
	//		int ncharpoint = RNDF.m_mapInfo.pSegment[i].pLane[j].charpoints_num;//特征点
	//		int nexit = RNDF.m_mapInfo.pSegment[i].pLane[j].exit_num;//出口点
	//		int exitid = 0;
	//		//exitid = RNDF.m_mapInfo.pSegment[i].pLane[j].exit_id;

	//		for (int k1=0; k1<nwaypoint; k1++)
	//		{
	//			double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].x;
	//			double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].y;

	//			if(i==0 && j==0 && k1==0)
	//			{
	//				centerx = wx;
	//				centery = wy;

	//				int px = cvRound((wx-centerx)*200000)+5000;
	//				int py = cvRound((wy-centery)*200000)+5000;

	//				cvCircle(dst, cvPoint(px,py), 5, CV_RGB(0,255,100), CV_FILLED);
	//			}
	//			int px = cvRound((wx-centerx)*200000)+5000;
	//			int py = cvRound((wy-centery)*200000)+5000;
	//			cvCircle(dst, cvPoint(px,py), 3, CV_RGB(0,255,0));

	//			//connect连线
	//			int n_con = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].con_num;
	//			for(int k2=0; k2<n_con; k2++)
	//			{
	//				int m = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].connect[k2].m-1;
	//				int n = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].connect[k2].n-1;
	//				int p = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].connect[k2].p-1;

	//				double wx0 = RNDF.m_mapInfo.pSegment[m].pLane[n].pPoint[p].x;
	//				double wy0 = RNDF.m_mapInfo.pSegment[m].pLane[n].pPoint[p].y;

	//				int px0 = cvRound((wx0-centerx)*200000)+5000;
	//				int py0 = cvRound((wy0-centery)*200000)+5000;
	//				cvLine(dst, cvPoint(px,py), cvPoint(px0,py0), CV_RGB(0,0,255), 2);

	//			}
	//			//同一路段上连线
	//			if(k1 > 0)
	//			{
	//				double wx0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1-1].x;
	//				double wy0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1-1].y;

	//				int px0 = cvRound((wx0-centerx)*200000)+5000;
	//				int py0 = cvRound((wy0-centery)*200000)+5000;
	//				cvLine(dst, cvPoint(px,py), cvPoint(px0,py0), CV_RGB(0,0,255), 2);
	//			}

	//			//文字
	//		CString str;
	//			str.Format("%d.%d.%d", i+1, j+1, k1+1);

	//			CvFont font;

	//			double hscale = 0.5;
	//			double vscale = 0.5;
	//			int linewidth = 2;
	//			cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);
	//			cvPutText(dst, str, cvPoint(px,py), &font,CV_RGB(30, 200, 100));
	//		}
	//		for (int k2=0; k2<ncharpoint; k2++)
	//		{
	//			double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pCharpoint[k2].x;
	//			double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pCharpoint[k2].y;

	//			int px = cvRound((wx-centerx)*200000)+5000;
	//			int py = cvRound((wy-centery)*200000)+5000;
	//			cvCircle(dst, cvPoint(px,py), 5, CV_RGB(255,0,0), CV_FILLED);
	//		}

	//			//连线
	//		for (int k3=0; k3<nexit; k3++)
	//			{
	//				int m = RNDF.m_mapInfo.pSegment[i].pLane[j].pExit[k3].m;
	//				int n = RNDF.m_mapInfo.pSegment[i].pLane[j].pExit[k3].n;
	//				int p = RNDF.m_mapInfo.pSegment[i].pLane[j].pExit[k3].p;

	//				if(m<1 || n<1 || p<1) continue;
	//				double wx0 = RNDF.m_mapInfo.pSegment[m-1].pLane[n-1].pPoint[p-1].x;
	//				double wy0 = RNDF.m_mapInfo.pSegment[m-1].pLane[n-1].pPoint[p-1].y;

	//				int px0 = cvRound((wx0-centerx)*200000)+5000;
	//				int py0 = cvRound((wy0-centery)*200000)+5000;


	//				double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[exitid-1].x;
	//				double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[exitid-1].y;

	//				int px = cvRound((wx-centerx)*200000)+5000;
	//				int py = cvRound((wy-centery)*200000)+5000;

	//				cvLine(dst, cvPoint(px,py), cvPoint(px0,py0), CV_RGB(0,0,255), 2);
	//			}
	//	}
	//}

}

//在全局地图中加入车辆行驶轨迹
void CIVDecisionDlg::AddTraceInRndfDisp( IplImage *src, double px, double py, double centerx, double centery )
{		
	int DIM_M = 4000;
	double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
	double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
	double angle;
	double dist;

	GetXYFromGPS( centerx, centery, px, py, pxx, pyy, angle, dist);

	int pnx = int(pxx) + DIM_M/2;
	int pny = int(pyy) + DIM_M/2;

	cvCircle(src, cvPoint(pnx, pny), 3, CV_RGB(0,255,255), CV_FILLED);
}

//输入经纬度，轨迹在rndf地图中显示
void CIVDecisionDlg::DispTraceInRndf(double px, double py)
{
	int DIM_M = 4000;
	AddTraceInRndfDisp(traceImage, px, py, rndf_centerx, rndf_centery);

	double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
	double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
	double angle;
	double dist;

	GetXYFromGPS( rndf_centerx, rndf_centery, px, py, pxx, pyy, angle, dist);
	//int pxx = (int)distance(rndf_centerx, rndf_centery, rndf_centerx, py)+DIM_M/2;
	//int pyy = (int)distance(rndf_centerx, rndf_centery, px, rndf_centery)+DIM_M/2;

	if(abs(pxx - disp_centerx) > 200)
	{
		if(pxx - disp_centerx > 0)
		{
			if(256+pxx >= DIM_M)
			{
				disp_centerx = DIM_M-256-1;
			}
			else
			{
				disp_centerx = pxx;
			}
		}
		else
		{
			if(pxx - 256 < 0)
			{
				disp_centerx = 256;
			}
			else disp_centerx = pxx;

		}
	}
	if(abs(pyy - disp_centery) > 200)
	{
		if(pyy - disp_centery > 0)
		{
			if(256+pyy >= DIM_M)
			{
				disp_centery = DIM_M-256-1;
			}
			else
			{
				disp_centery = pyy;
			}

		}
		else
		{
			if(pyy - 256 < 0)
			{
				disp_centery = 256;
			}
			else disp_centery = pyy;

		}

	}

	//显示
	IplImage *disp = cvCreateImage(cvSize(512,512), 8, 3);
	CvRect rect = cvRect(disp_centerx - 256, disp_centery - 256, 512,512);
	cvSetImageROI(rndfImage, rect);
	cvSetImageROI(traceImage, rect);
	cvSub(rndfImage, traceImage, disp);
	cvResetImageROI(traceImage);
	cvResetImageROI(rndfImage);
	m_static_rndfmap.zShowImage(disp);
	cvReleaseImage(&disp);
}


void CIVDecisionDlg::OnEnChangeEdit1()
{
	// TODO:  如果该控件是 RICHEDIT 控件，则它将不会
	// 发送该通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CIVDecisionDlg::zDrawLanes(CvSeq *points, CvPoint2D64f gps, double m_dir )
{
	IplImage *disp = cvCreateImage( cvSize(512, 512), 8, 3);
	cvSet(disp, cvScalarAll(255));
	//for( int i=0; i<512; i++)
	//{
	//	for( int j=0; j<512; j++)
	//	{
	//		int a = map->MapPoint[i][j];
	//		if( a == 1)
	//		{
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3] = 255;
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3+1] = 0;
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3+2] = 0;
	//		}
	//		else if( a == 9)
	//		{
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3] = 0;
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3+1] = 255;
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3+2] = 0;
	//		}
	//		else
	//		{
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3] = 255;
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3+1] = 255;
	//			((uchar*)(disp->imageData + disp->widthStep*i))[j*3+2] = 255;
	//		}
	//	}
	//}
	//
	//CvMemStorage *storagem = cvCreateMemStorage(0);
	//CvSeq *seq_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storagem );

	if( points->total > 0 ) seq_road = cvCloneSeq(points, storagem);

	//CString str;
	//str.Format("%d",seq_road->total);
	//GetDlgItem(IDC_EDIT_RUNSTAT)->SetWindowTextA(str);
	//gps = cvPoint2D64f(31.849874,117.121857);
	//m_dir = 90;
	if(seq_road->total > 0)
	{
		for ( int i=0; i<seq_road->total; i++)
		{
			CvPoint2D64f dstgps;
			dstgps = *(CvPoint2D64f*)cvGetSeqElem( seq_road, i );
			Map_Point pt = m_Vehicle.m_GpsData.APiontConver(gps, dstgps, m_dir);
			cvCircle(disp, cvPoint(pt.x,pt.y), 2, CV_RGB(0,0,255), CV_FILLED);
		}

		m_static_localmap.zShowImage(disp);

		
		cvReleaseImage(&disp);
	}
}
void CIVDecisionDlg::SendMess(CString str)
{
	HWND hWnd = ::FindWindow(NULL,"无人车控制平台");
	//HWND hWnd = ::FindWindow(NULL,"调试窗口 2012.5.31");
	if(hWnd==NULL)
	{
		//AfxMessageBox("没有找到接受窗体");
		 return;
	}

	//CString str="%G200ZXZB";
	const size_t newsize = 200;

	str += "\0";
	//
    size_t strsize = strlen(str) + 1;
    size_t convertedChars = 0;
    wchar_t wcstring[newsize];
    mbstowcs_s(&convertedChars, wcstring, strsize, str, _TRUNCATE);

	LPWSTR strSendMsg=wcstring;

	char cSendMsg[200];

	DWORD dwNum = WideCharToMultiByte(CP_OEMCP,NULL,strSendMsg,-1,NULL,0,NULL,FALSE);

	WideCharToMultiByte(CP_OEMCP,NULL,strSendMsg,-1,cSendMsg,dwNum,NULL,FALSE);

	if(hWnd!=NULL) {
		COPYDATASTRUCT cpd; /*给COPYDATASTRUCT结构赋值*/
		cpd.dwData = 0;
		cpd.cbData = strlen(cSendMsg);
		cpd.lpData = (void*)cSendMsg;
		::SendMessageTimeout(hWnd,WM_COPYDATA,NULL,(LPARAM)&cpd,SMTO_NORMAL,30,NULL);//发送！
		
		
	}
}

/////////////////////////////
//决策按钮控制指令：
//   %G200ZXZB 转向准备
//   %G200ZDZB 制动准备
//   %G200JT    急停
//   %G200FW   复位
//   %G200ZL    找零位
////////////////////////////////

void CIVDecisionDlg::OnBnClickedButtonzxzb()//转向准备
{	
	//SendMess("%G100T0V0B-46000S0K0A37H0L0END;");
	//SendMess("%G200ZXZB");
	
}

void CIVDecisionDlg::OnBnClickedButtonzdzb()//制动准备
{
	
}

void CIVDecisionDlg::OnBnClickedButtonjt()
{
	//SendMess("%G200JT");	//控制改
}

void CIVDecisionDlg::OnBnClickedButtonfw()
{
	//SendMess("%G200FW");	//控制改
}

void CIVDecisionDlg::OnBnClickedButtonzl()
{
	//SendMess("%G100T0V0B-25500S0K0A37H0L0END;");
	
}


void CIVDecisionDlg::zDrawLanes_dst( I_Map *map,  I_Map *map2,I_Map *map3,CvSeq *points, CvPoint2D64f gps, double m_dir, IplImage *dst, int b_draw_center )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f dstgps;
	app->critical_section.Lock();
	double current_dir=app->GPS_Direction;
	CvPoint2D64f current_pos=app->GPS_Point;
	app->critical_section.Unlock();

	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX,0.8,0.8,0,2,2); 

	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			int a = map->MapPoint[i][j];
			
			if( a==8 )
			{
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3] = 255;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+1] = 120;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+2] = 0;
			}
			else if( a == 18)
			{
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3] = 255;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+1] = 0;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+2] = 180;
			}
			else if(a == 28)
			{
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3] = 0;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+1] = 0;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+2] = 255;
			}
			else if( a == 46)
			{
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3] = 210;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+1] = 180;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+2] = 0;
			}
			else if( a == 45)
			{
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3] = 150;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+1] = 180;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+2] = 0;
			}
			else if(a == 44)
			{
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3] = 255;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+1] = 255;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+2] = 0;
			}
			else
			{
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3] = 255;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+1] = 255;
				((uchar*)(dst->imageData + dst->widthStep*i))[j*3+2] = 255;
			}
		}
	}
	if (map->MapPoint[254][0]==1)
	{
		cvCircle(dst,cvPoint(450,50),25,cvScalarAll(200),CV_FILLED);
	}
	if (map->MapPoint[254][0]==10)
	{
		cvCircle(dst,cvPoint(450,50),25,cvScalarAll(100),CV_FILLED);
	}
	
	if (map->MapPoint[411][0]==255)
	{
		cvCircle(dst,cvPoint(50,411),25,cvScalarAll(0),CV_FILLED);
	}
	if (map->MapPoint[411][511]==255)
	{
		cvCircle(dst,cvPoint(50,411),25,cvScalarAll(0),CV_FILLED);
	}
	
	if (map->MapPoint[412][0]==255)
	{
		cvCircle(dst,cvPoint(460,411),25,cvScalarAll(100),CV_FILLED);
	}
	if (map->MapPoint[412][511]==255)
	{
		cvCircle(dst,cvPoint(460,411),25,cvScalarAll(100),CV_FILLED);
	}

	cvCircle(dst, cvPoint(255,411),3, CV_RGB(0,0,255), CV_FILLED);//标出无人车位置

	

	int ob_num=m_VehicleExe.iObject1test1 + m_VehicleExe.iObject2test1;
	CString ob_numstr; //障碍物数量
	ob_numstr.Format("%d",ob_num);
	cvPutText(dst, ob_numstr, cvPoint(20,20), &font, CV_RGB(255,0,0));//显示障碍物数量

	CvPoint2D64f ibpoint;

	int yibeomin=0;
	double spdyibeomin;

	app->critical_winsockread2.Lock();
	
	for(int ibi=0; ibi<m_VehicleExe.iObject2test1; ibi++)
	{
		//for(int ibj=0; ibj<300; ibj++)
		//{
		//	ibpoint.x=256+m_VehicleExe.ObjectWT1[ibi].iContour_X[ibj]/20;
		//	ibpoint.y=412+10+m_VehicleExe.ObjectWT1[ibi].iContour_Y[ibj]/20;
		//	cvCircle(dst, cvPoint(ibpoint.x,ibpoint.y), 1, CV_RGB(0,0,0), CV_FILLED);//标出无人车位置
		//}
		CvPoint2D64f ibcenter;
		ibcenter.x=m_VehicleExe.ObjectWT2test1[ibi].ObjectBox_CenterX;
		ibcenter.y=m_VehicleExe.ObjectWT2test1[ibi].ObjectBox_CenterY;
		if((ibcenter.x>=228 && ibcenter.x<284)&&((ibcenter.y>=(421+(abs(ibcenter.x-256)/tan(50*PI/180))))||(ibcenter.y<=(397-(abs(ibcenter.x-256)/tan(50*PI/180))))))
		{
			cvCircle(dst, cvPoint(ibcenter.x,ibcenter.y), 1, CV_RGB(0,255,0), CV_FILLED);//标出无人车位置

			//double uv=sqrt(pow(m_VehicleExe.ObjectWT2test1[ibi].Relative_VelocityX + app->GPS_Speed,2)+pow(m_VehicleExe.ObjectWT2test1[ibi].Relative_VelocityY,2))*3.6;
			double uv=m_VehicleExe.ObjectWT2test1[ibi].Relative_VelocityX*3.6;
			//if(abs(uv)>3)
			{
				CString uv_str;
				uv_str.Format("%.1f",uv);
				cvPutText(dst, uv_str, cvPoint(ibcenter.x,ibcenter.y), &font, CV_RGB(55,55,55));
			}

			if(ibcenter.x>=253 && ibcenter.x<=259 && ibcenter.y>=312 && ibcenter.y<412)
			{
				if(yibeomin<ibcenter.y)
				{
					yibeomin=ibcenter.y;
					spdyibeomin=m_VehicleExe.ObjectWT2test1[ibi].Relative_VelocityX*3.6;
				}
			}

			for(int j=0;j<m_VehicleExe.ObjectWT2test1[ibi].iContour_Points;j++)
			{
				ibcenter.x=m_VehicleExe.ObjectWT2test1[ibi].iContour_X[j];
				ibcenter.y=m_VehicleExe.ObjectWT2test1[ibi].iContour_Y[j];
				cvCircle(dst, cvPoint(ibcenter.x,ibcenter.y), 1, CV_RGB(0,255,0), CV_FILLED);//标出无人车位置

				if(ibcenter.x>=253 && ibcenter.x<=259 && ibcenter.y>=312 && ibcenter.y<412)
				{
					if(yibeomin<ibcenter.y)
					{
						yibeomin=ibcenter.y;
						spdyibeomin=m_VehicleExe.ObjectWT2test1[ibi].Relative_VelocityX*3.6;
					}
				}

			}
		}

	}

	app->critical_winsockread2.Unlock();

	app->critical_winsockread.Lock();
	
	for(int ibi=0; ibi<m_VehicleExe.iObject1test1; ibi++)
	{
		//for(int ibj=0; ibj<300; ibj++)
		//{
		//	ibpoint[ibj].x=256+m_VehicleExe.ObjectWT1[ibi].iContour_X[ibj]/20;
		//	ibpoint[ibj].y=412+10+m_VehicleExe.ObjectWT1[ibi].iContour_Y[ibj]/20;
		//	cvCircle(dst, cvPoint(ibpoint[ibj].x,ibpoint[ibj].y), 1, CV_RGB(0,0,0), CV_FILLED);//标出无人车位置
		//}
		CvPoint2D64f ibcenter;
		ibcenter.x=m_VehicleExe.ObjectWT1test1[ibi].ObjectBox_CenterX;
		ibcenter.y=m_VehicleExe.ObjectWT1test1[ibi].ObjectBox_CenterY;
		if((ibcenter.x>=228 && ibcenter.x<284)&&((ibcenter.y>=(421+(abs(ibcenter.x-256)/tan(50*PI/180))))||(ibcenter.y<=(397-(abs(ibcenter.x-256)/tan(50*PI/180))))))
		{
			cvCircle(dst, cvPoint(ibcenter.x,ibcenter.y), 1, CV_RGB(0,255,0), CV_FILLED);//标出无人车位置

			//double uv=sqrt(pow(m_VehicleExe.ObjectWT1test1[ibi].Relative_VelocityX + app->GPS_Speed,2)+pow(m_VehicleExe.ObjectWT1test1[ibi].Relative_VelocityY,2))*3.6;
			double uv=m_VehicleExe.ObjectWT1test1[ibi].Relative_VelocityX*3.6;
			//if(abs(uv)>3)
			{
				CString uv_str;
				uv_str.Format("%.1f",uv);
				cvPutText(dst, uv_str, cvPoint(ibcenter.x,ibcenter.y), &font, CV_RGB(55,55,55));
			}

			if(ibcenter.x>=253 && ibcenter.x<=259 && ibcenter.y>=312 && ibcenter.y<412)
			{
				if(yibeomin<ibcenter.y)
				{
					yibeomin=ibcenter.y;
					spdyibeomin=m_VehicleExe.ObjectWT1test1[ibi].Relative_VelocityX*3.6;
				}
			}

			for(int j=0;j<m_VehicleExe.ObjectWT1test1[ibi].iContour_Points;j++)
			{
				ibcenter.x=m_VehicleExe.ObjectWT1test1[ibi].iContour_X[j];
				ibcenter.y=m_VehicleExe.ObjectWT1test1[ibi].iContour_Y[j];
				cvCircle(dst, cvPoint(ibcenter.x,ibcenter.y), 1, CV_RGB(0,255,0), CV_FILLED);//标出无人车位置

				if(ibcenter.x>=253 && ibcenter.x<=259 && ibcenter.y>=312 && ibcenter.y<412)
				{
					if(yibeomin<ibcenter.y)
					{
						yibeomin=ibcenter.y;
						spdyibeomin=m_VehicleExe.ObjectWT2test1[ibi].Relative_VelocityX*3.6;
					}
				}

			}
		}
	}
	app->critical_winsockread.Unlock();

	outRongheData<<spdyibeomin<<",  "<<yibeomin<<",";

	bool flagtes=0;

	for(int y=412;y>=312;y--)
	{
		for(int x=253;x<=259;x++)
		{
			if(map->MapPoint[y][x]==8)
			{
				outRongheData<<y<<",  ";
				flagtes=true;
				break;
			}
		}
		if(flagtes)
			break;
	}

	if( points->total > 0 ) 
	{
		cvClearSeq( seq_road );
		seq_road = cvCloneSeq(points, storagem);
	}
	
	if(seq_road->total > 0)
	{
		for ( int i=0; i<seq_road->total; i++)
		{
			
			dstgps = *(CvPoint2D64f*)cvGetSeqElem( seq_road, i );
			Map_Point pt = m_Vehicle.m_GpsData.APiontConver(gps, dstgps, m_dir);
			cvCircle(dst, cvPoint(pt.x,pt.y), 1, CV_RGB(255,0,0), CV_FILLED);
		}
	}
	for( int i=0; i<200; i++)
	{
		intergps[i] = m_Vehicle.m_GpsData.APiontConverD(gps,m_VehicleExe.CrossPath[i],m_dir);
		cvCircle(dst, cvPoint(intergps[i].x,intergps[i].y), 1, CV_RGB(255,120,132), CV_FILLED);
	}
	for( int i=0; i<200; i++)
	{
		rndfgps[i] = m_Vehicle.m_GpsData.APiontConverD(gps,m_VehicleExe.rndfbezier[i],m_dir);
		cvCircle(dst, cvPoint(rndfgps[i].x,rndfgps[i].y), 1, CV_RGB(123,0,132), CV_FILLED);
	}
	if((m_VehicleExe.able_ChangeLane)&&((m_VehicleExe.m_taskpublic==5001)||(m_VehicleExe.m_taskpublic==5005)))
	{
		for( int i=0; i<200; i++)
		{
			rndfgps[i] = m_Vehicle.m_GpsData.APiontConverD(gps,m_VehicleExe.rndfbezierlanechangeinitialdisp[i],m_dir);
			cvCircle(dst, cvPoint(rndfgps[i].x,rndfgps[i].y), 1, CV_RGB(0,255,0), CV_FILLED);
		}
	}


	showp = m_Vehicle.m_GpsData.APiontConverD(gps,m_VehicleExe.gps_aimpoint,m_dir);
	cvCircle(dst, cvPoint(m_VehicleExe.aim_point.x,m_VehicleExe.aim_point.y), 7, CV_RGB(0,255,255), CV_FILLED);
	//cvCircle(dst, cvPoint(showp.x,showp.y), 7, CV_RGB(155,0,188), CV_FILLED);
	double dir = app->GPS_Direction - m_VehicleExe.gps_aimdir+90;
	CvPoint2D64f aaa;
	aaa.x = m_VehicleExe.aim_point.x + 50*cos(dir*3.14/180);
	aaa.y = m_VehicleExe.aim_point.y - 50*sin(dir*3.14/180);
	cvLine(dst,cvPoint(m_VehicleExe.aim_point.x,m_VehicleExe.aim_point.y),cvPoint(aaa.x,aaa.y),CV_RGB(50,180,255),3,8,0);
}


void CIVDecisionDlg::OnEnChangeEdit6()
{
	// TODO:  如果该控件是 RICHEDIT 控件，则它将不会
	// 发送该通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}
void CIVDecisionDlg::InitCvtRNDF2Img()
{
	CvtRNDF2ImgMap( m_VehicleExe.RNDFGPS, rndfImage, rndf_centerx, rndf_centery, 1, 0 );
}

void CIVDecisionDlg::Readgps_FilePath(CString FileName)
{
	// TODO: 在此添加控件通知处理程序代码
	//SendMess("%G100T0V0B-25500S0K0A37H0L0END;");
	

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	if(!app->Lead_pt.empty())
	{
		AfxMessageBox("已载入！");
	}

	
	else
	{
		// TODO: 在此添加命令处理程序代码
		// TODO: 在此添加命令处理程序代码、
		CString FileExtension;
		CString DefExtension;
		//DefExtension="accdb";
		//FileExtension="TXT File(*.txt)|*.txt|";
		//FileExtension+="All Files(*.*)|*.*|";
		//CFileDialog openFileDialog
		//(true,//
		//DefExtension,//
		//NULL,//
		//OFN_PATHMUSTEXIST   |   OFN_HIDEREADONLY   |   OFN_ALLOWMULTISELECT,//
		//FileExtension,this);//
		//openFileDialog.SetDefExt();
	//	if( openFileDialog.DoModal ()==IDOK )
		{
	//		CString FileName,FileExt;
	//		FileName=openFileDialog.GetFileName();
			//FileExt=openFileDialog.GetFileExt();
			
			ifstream f1(FileName); 
			
			if (!f1) { //当f1打开失败时进行错误处理 
				cerr<<"map.txt file not open!"<<endl; 
				exit(1); 
			}
			char temp[50];
			
			//string temp = "";
			int i,j;
			while(!f1.eof())
			{
				CString point_id = "";
				CString strLatitude = ""; 
				CString strLongitude = "";
				CString strheight = "";
				CString strparam1 = "";
				CString strparam2 = "";
				int SectionID = 0;
				f1.getline(temp,50);
				//f1>>temp;
				if(temp[0]==NULL)
						break;
				for(j = 0;j<50;j++)
				{
					if('	' == temp[j])
					{
						SectionID++;
						continue;
					}
					if('/n' == temp[j])
					{
						SectionID++;
						break;
					}
					if(temp[j]==NULL)
						break;
					else // 数据
					{
						switch(SectionID)
						{ 

						case 0: 
							point_id += temp[j];
							break;	

						case 1: 
							strLongitude += temp[j];
							break;

						case 2: 
							strLatitude += temp[j];
							break;
						case 3: 
							strheight += temp[j];
							break;
						case 4: 
							strparam1 += temp[j];
							break;
						case 5: 
							strparam2 += temp[j];
							break;
						default:
							break;
						}
					}
				}
				LEAD lead_point;
				lead_point.id = atof(point_id);
				lead_point.lat = atof(strLatitude);
				lead_point.lng = atof(strLongitude);
				lead_point.height = atof(strheight);
				lead_point.param1 = atof(strparam1);
				lead_point.param2 = atof(strparam2);
				//aim_point.z = atof(strAzimuth);
				//double Dir = atof(strAzimuth);
				AddPointInLaneDisp2( lead_point.lat, lead_point.lng, lead_point.id, 1 );
		/*		if(lead_point.param1 == 3)
				{	
					lead_point.lat = app->End_Pt.x;
					lead_point.lng = app->End_Pt.y;
				}*/
				if (lead_point.param1 == 4)
				{
					app->mission_pt.push(lead_point);
				}
				else
					app->Lead_pt.push(lead_point);
			}
			f1.close();
			AimFlag = TRUE;
			AfxMessageBox("模式设置完成");
		}

	}

}
void CIVDecisionDlg::Readrndf_FilePath(CString FileName)
{
	m_VehicleExe.RNDFGPS.Openfile1(FileName);

	AfxMessageBox("RNDF加载完成");
}


void CIVDecisionDlg::OnEnChangeEdit15()
{
	// TODO:  如果该控件是 RICHEDIT 控件，则它将不会
	// 发送该通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}

void CIVDecisionDlg::OnBnClickedSetSpeed()
{
	CString str1,str2,str3,str4,str5;
	GetDlgItemText(IDC_SET_SPEED1,str1);
	GetDlgItemText(IDC_SET_SPEED2,str2);
	GetDlgItemText(IDC_SET_SPEED3,str3);
	GetDlgItemText(IDC_SET_SPEED4,str4);
	GetDlgItemText(IDC_SET_SPEED5,str5);
	m_VehicleExe.max_speed = atoi(str1)/3.6;
	m_VehicleExe.road_speed = atoi(str2)/3.6;
	m_VehicleExe.cross_speed1 = atoi(str3)/3.6;
	m_VehicleExe.cross_speed2 = atoi(str4)/3.6;
	m_VehicleExe.obstacle_speed = atoi(str5)/3.6;

	CString strStrtLat,strStrtLng,strEndLat,strEndLng;
	GetDlgItemText(IDC_ApaStrtLat,strStrtLat);
	GetDlgItemText(IDC_ApaStrtLng,strStrtLng);
	GetDlgItemText(IDC_ApaEndLat,strEndLat);
	GetDlgItemText(IDC_ApaEndLng,strEndLng);
	m_VehicleExe.ApaStrt.x = atof(strStrtLat);
	m_VehicleExe.ApaStrt.y = atof(strStrtLng);
	m_VehicleExe.ApaEnd.x = atof(strEndLat);
	m_VehicleExe.ApaEnd.y = atof(strEndLng);

	//*********813********//
	m_VehicleExe.temp_max = atoi(str1)/3.6;
	m_VehicleExe.temp_road = atoi(str2)/3.6;
	m_VehicleExe.temp_cross1 = atoi(str3)/3.6;
	m_VehicleExe.temp_cross2 = atoi(str4)/3.6;
	m_VehicleExe.temp_obstacle = atoi(str5)/3.6;
	//*********813********//
	// TODO: 在此添加控件通知处理程序代码
}

void CIVDecisionDlg::OnEnChangeSetSpeed1()
{
	// TODO:  如果该控件是 RICHEDIT 控件，则它将不会
	// 发送该通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}

void CIVDecisionDlg::OnEnChangeSecondDecision()
{
	// TODO:  如果该控件是 RICHEDIT 控件，则它将不会
	// 发送该通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}

void CIVDecisionDlg::OnBnClickedRadio4()
{
	// TODO: 在此添加控件通知处理程序代码
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	bool leftturnreq_his=app->leftturnreq;
	app->leftturnreq = ((CButton *)GetDlgItem(IDC_RADIO4))->GetCheck();
	if(app->leftturnreq)
	{
		((CButton *)GetDlgItem(IDC_RADIO5))->SetCheck(false);
		((CButton *)GetDlgItem(IDC_RADIO6))->SetCheck(false);
		app->rightturnreq=false;
		app->rightturnreq_proc=false;
		if(!leftturnreq_his)
		{
			app->leftturnreq_proc=true;
			app->lanekeepreq=false;
		}
	}
}

void CIVDecisionDlg::OnBnClickedRadio5()
{
	// TODO: 在此添加控件通知处理程序代码
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	bool rightturnreq_his=app->rightturnreq;
	app->rightturnreq = ((CButton *)GetDlgItem(IDC_RADIO5))->GetCheck();
	if(app->rightturnreq)
	{
		((CButton *)GetDlgItem(IDC_RADIO4))->SetCheck(false);
		((CButton *)GetDlgItem(IDC_RADIO6))->SetCheck(false);
		app->leftturnreq=false;
		app->leftturnreq_proc=false;
		if(!rightturnreq_his)
		{
			app->rightturnreq_proc=true;
			app->lanekeepreq=false;
		}
	}
}

void CIVDecisionDlg::OnBnClickedRadio6()
{
	// TODO: 在此添加控件通知处理程序代码
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	if(((CButton *)GetDlgItem(IDC_RADIO6))->GetCheck())
	{
		app->leftturnreq=false;
		app->rightturnreq=false;
		((CButton *)GetDlgItem(IDC_RADIO4))->SetCheck(false);
		((CButton *)GetDlgItem(IDC_RADIO5))->SetCheck(false);
	}
}

void CIVDecisionDlg::OnBnClickedCheck1()
{
	blanekeephspd=((CButton *)GetDlgItem(IDC_CHECK1))->GetCheck();
	// TODO: 在此添加控件通知处理程序代码
}

void CIVDecisionDlg::OnBnClickedlukouzhi()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=1;
}

void CIVDecisionDlg::OnBnClickedlukouzuo()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=3;
}

void CIVDecisionDlg::OnBnClickedlukouyou()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=2;
}

void CIVDecisionDlg::OnBnClickedlukoudiaotou()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=4;
}

void CIVDecisionDlg::OnBnClickedSwan()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=8;
}

void CIVDecisionDlg::OnBnClickedlukouchu()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=9;
}

void CIVDecisionDlg::OnBnClickedperson()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=7;
}

void CIVDecisionDlg::OnBnClickedsetlane()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CString str_laneseq,str_lanenum,str_spdiniset,str_laneexist;
	GetDlgItemText(IDC_laneseq,str_laneseq);
	GetDlgItemText(IDC_lanenum,str_lanenum);
	GetDlgItemText(IDC_speediniset,str_spdiniset);
	GetDlgItemText(IDC_laneexist,str_laneexist);

	app->laneseq_set = atoi(str_laneseq);
	app->lanenum_set = atoi(str_lanenum);
	app->speediniset = atoi(str_spdiniset);
	app->laneexist = atoi(str_laneexist);
}

void CIVDecisionDlg::OnBnClickedlanechgqian()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=29;
}

void CIVDecisionDlg::OnBnClickedlanechghou()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=30;
}

void CIVDecisionDlg::OnBnClickedshigong()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lukoudirnum=6;
}

void CIVDecisionDlg::OnBnClickedCheck2()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->bGpsGanrao=((CButton *)GetDlgItem(IDC_CHECK2))->GetCheck();
}

void CIVDecisionDlg::OnBnClickedCheck3()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->bLKAsuidao=((CButton *)GetDlgItem(IDC_CHECK3))->GetCheck();
}

void CIVDecisionDlg::OnBnClickedapapath1()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	using namespace std;

	if(app->AimFlag_ApaPath1)
	{
		AfxMessageBox("已载入！");
	}
	else
	{
		CString FileExtension;
		CString DefExtension;
		FileExtension="TXT File(*.txt)|*.txt|";
		FileExtension+="All Files(*.*)|*.*|";
		CFileDialog openFileDialog
			(true,
			DefExtension,
			NULL,
			OFN_PATHMUSTEXIST   |   OFN_HIDEREADONLY   |   OFN_ALLOWMULTISELECT,
			FileExtension,this);
		if( openFileDialog.DoModal ()==IDOK )
		{
			CString FileName,FileExt;
			FileName=openFileDialog.GetPathName();
			ifstream f1(FileName); 

			if (!f1) {
				AfxMessageBox("map.txt file not open!"); 
				exit(1); 
			}

			string temp = "";
			int j;

			while(!f1.eof())
			{
				CString strLatitude = ""; 
				CString strLongitude = "";

				int SectionID = 0;
				f1>>temp;

				int ct = temp.size();

				for(j = 0;j<ct;j++)
				{
					if(',' == temp[j])
					{
						SectionID++;
						continue;
					}
					if(temp[j]==NULL)break;
					else
					{
						switch(SectionID)
						{ 
						case 0: 
							strLatitude += temp[j];
							break;	

						case 1: 
							strLongitude += temp[j];
							break;

						default:
							break;
						}
					}
				}
				CvPoint2D64f aim_point;
				aim_point.x = atof(strLatitude);
				aim_point.y = atof(strLongitude);

				cvSeqPush(app->gpsmap_road_ApaPath1, &aim_point);
			}
			f1.close();
			app->AimFlag_ApaPath1 = true;
			AfxMessageBox("泊车路径1已载入");
		}
	}
}


void CIVDecisionDlg::OnBnClickedapapath2()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	using namespace std;

	if(app->AimFlag_ApaPath2)
	{
		AfxMessageBox("已载入！");
	}
	else
	{
		CString FileExtension;
		CString DefExtension;
		FileExtension="TXT File(*.txt)|*.txt|";
		FileExtension+="All Files(*.*)|*.*|";
		CFileDialog openFileDialog
			(true,
			DefExtension,
			NULL,
			OFN_PATHMUSTEXIST   |   OFN_HIDEREADONLY   |   OFN_ALLOWMULTISELECT,
			FileExtension,this);
		if( openFileDialog.DoModal ()==IDOK )
		{
			CString FileName,FileExt;
			FileName=openFileDialog.GetPathName();
			ifstream f1(FileName); 

			if (!f1) {
				AfxMessageBox("map.txt file not open!"); 
				exit(1); 
			}

			string temp = "";
			int j;

			while(!f1.eof())
			{
				CString strLatitude = ""; 
				CString strLongitude = "";

				int SectionID = 0;
				f1>>temp;

				int ct = temp.size();

				for(j = 0;j<ct;j++)
				{
					if(',' == temp[j])
					{
						SectionID++;
						continue;
					}
					if(temp[j]==NULL)break;
					else
					{
						switch(SectionID)
						{ 
						case 0: 
							strLatitude += temp[j];
							break;	

						case 1: 
							strLongitude += temp[j];
							break;

						default:
							break;
						}
					}
				}
				CvPoint2D64f aim_point;
				aim_point.x = atof(strLatitude);
				aim_point.y = atof(strLongitude);

				cvSeqPush(app->gpsmap_road_ApaPath2, &aim_point);
			}
			f1.close();
			app->AimFlag_ApaPath2 = true;
			AfxMessageBox("泊车路径2已载入");
		}
	}
}

void CIVDecisionDlg::OnBnClickedapapath3()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	using namespace std;

	if(app->AimFlag_ApaPath3)
	{
		AfxMessageBox("已载入！");
	}
	else
	{
		CString FileExtension;
		CString DefExtension;
		FileExtension="TXT File(*.txt)|*.txt|";
		FileExtension+="All Files(*.*)|*.*|";
		CFileDialog openFileDialog
			(true,
			DefExtension,
			NULL,
			OFN_PATHMUSTEXIST   |   OFN_HIDEREADONLY   |   OFN_ALLOWMULTISELECT,
			FileExtension,this);
		if( openFileDialog.DoModal ()==IDOK )
		{
			CString FileName,FileExt;
			FileName=openFileDialog.GetPathName();
			ifstream f1(FileName); 

			if (!f1) {
				AfxMessageBox("map.txt file not open!"); 
				exit(1); 
			}

			string temp = "";
			int j;

			while(!f1.eof())
			{
				CString strLatitude = ""; 
				CString strLongitude = "";

				int SectionID = 0;
				f1>>temp;

				int ct = temp.size();

				for(j = 0;j<ct;j++)
				{
					if(',' == temp[j])
					{
						SectionID++;
						continue;
					}
					if(temp[j]==NULL)break;
					else
					{
						switch(SectionID)
						{ 
						case 0: 
							strLatitude += temp[j];
							break;	

						case 1: 
							strLongitude += temp[j];
							break;

						default:
							break;
						}
					}
				}
				CvPoint2D64f aim_point;
				aim_point.x = atof(strLatitude);
				aim_point.y = atof(strLongitude);

				cvSeqPush(app->gpsmap_road_ApaPath3, &aim_point);
			}
			f1.close();
			app->AimFlag_ApaPath3 = true;
			AfxMessageBox("泊车路径3已载入");
		}
	}
}
