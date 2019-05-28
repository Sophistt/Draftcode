// IVDecision.cpp : 定义应用程序的类行为。
//

#include "stdafx.h"
#include "IVDecision.h"
#include "IVDecisionDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CIVDecisionApp

BEGIN_MESSAGE_MAP(CIVDecisionApp, CWinApp)
	ON_COMMAND(ID_HELP, &CWinApp::OnHelp)
END_MESSAGE_MAP()


// CIVDecisionApp 构造

CIVDecisionApp::CIVDecisionApp()
{
		limit_speed = 75/3.6;
		limitflag = false;
		lanekeeptime = 10000;
		lanekeepreqhspdapp=false;
		bdirectrunreq=false;
		blanechanging=false;
		lukoudirnum=0;
		lukoulanechangereq=0;
		ob_width_111=0;
		braohui=0;
		inter_UTURN=0;
		bchulukoufound=1;
		bbanmaxiannoraozhang=0;
		bjianruiuturn=0;

		bthirduturn=0;

		fangxiangpanbuhuizheng=0;

		bqishiluduan=1;

		bspecialuturnluduan=0;

		bspecialnoleftturnluduan=0;

		bleftturnban=0;
		brightturnban=0;

		bnobiandaoraozhang=0;

		bleftsidelane=0;
		brightsidelane=0;

		bGpsGanrao=0;
		bLKAsuidao=0;

		bApaActive=0;
		PXApaActive=0;
		bSuidao1Active=0;
		bSuidao2Active=0;

		uturndaoche=0;

		bspecialraozhangcanshu=0;

		kuaisulufulu=0;

		rouchestatus=0;

		realsteerangle=0;

		systemtimer10ms=0;
		systemtimer10ms_his=0;

	AimFlag_ApaPath1=false;
	AimFlag_ApaPath2=false;
	AimFlag_ApaPath3=false;

	bjiedaochaoche=0;

	bcheweistrtsearch=false;
	bcheweistrtsearchPX=false;

	// TODO: 在此处添加构造代码，
	// 将所有重要的初始化放置在 InitInstance 中
}


// 唯一的一个 CIVDecisionApp 对象

CIVDecisionApp theApp;


// CIVDecisionApp 初始化

BOOL CIVDecisionApp::InitInstance()
{
	// 如果一个运行在 Windows XP 上的应用程序清单指定要
	// 使用 ComCtl32.dll 版本 6 或更高版本来启用可视化方式，
	//则需要 InitCommonControlsEx()。否则，将无法创建窗口。
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// 将它设置为包括所有要在应用程序中使用的
	// 公共控件类。
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);
	VERIFY( 1 == InitSkinMagicLib( AfxGetInstanceHandle(), /*NULL*/NULL ,NULL,NULL ) );
	LoadSkinFile("skins\\corona.smf");
	CWinApp::InitInstance();
	//VERIFY( 1 == LoadSkinFile(_T("Skins\\AlphaOS.smf"))); 


	//SetDialogSkin( "Dialog" );

//	VERIFY( 1 == SetWindowSkin( m_pMainWnd->m_hWnd , "MainFrame" ));
	VERIFY( 1 == SetDialogSkin( "Dialog" ) );


	//VERIFY( 1 == SetDialogSkin( _T("Dialog") ) );
	//EnableWindowScrollbarSkin( m_hWnd , SB_BOTH );
	AfxEnableControlContainer();

	// 标准初始化
	// 如果未使用这些功能并希望减小
	// 最终可执行文件的大小，则应移除下列
	// 不需要的特定初始化例程
	// 更改用于存储设置的注册表项
	// TODO: 应适当修改该字符串，
	// 例如修改为公司或组织名
	SetRegistryKey(_T("应用程序向导生成的本地应用程序"));

	CIVDecisionDlg dlg;
	m_pMainWnd = &dlg;
	INT_PTR nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: 在此处放置处理何时用“确定”来关闭
		//  对话框的代码
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: 在此放置处理何时用“取消”来关闭
		//  对话框的代码
	}

	lane_Map = new I_Map;

	//lane_Map = vel_Map;
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			lane_Map->MapPoint[i][j] = 0;

		}
	}

	left_right = 0;
	sendflag1 = false;
	vehicle_speed = 0;
	ob_hxerror = 1;
	drive_curvature = 50;
	stop_map = false;
	Get_Sign = true;
	light_res = 0;
	perceptimelast = 0;
	percepIbeoMaptimelast = 0;
	IbeoMaptimeStamp = 0;
	proctimelast = 0;
	proctimeStamp = 0;
	inter_small = false;
	continue_brake = false;
	continue_braketmp = false;

	obb_right = false;
	obb_left = false;
	bzheng = false;

	lanechangenum = 0;
	
	continue_spdred = false;
	continue_spdredtmp = false;
	spdredacc = 0;
	spdredacctmp = 0;

	bGpsMapLoad = 0;

	drive_state_update = 0;
	drive_curvature_update = 0;
	drive_obstacle_update = 0;
	drive_obstacle2_update = 0;
	limit_speed_update = 0;

	leftturnreq = 0;
	rightturnreq = 0;

	leftturnreq_proc = 0;
	rightturnreq_proc = 0;

	lanekeepreq = 0;
	lanekeeptime = 10000;
	lanekeepreqhspdapp=false;

	bdirectrunreq=false;
	blanechanging=false;
	lukoudirnum=0;
	lukoulanechangereq=0;
	ob_width_111=0;
	braohui=0;

	inter_UTURN=0;

	bchulukoufound=1;
	bbanmaxiannoraozhang=0;
	laneturndir = 0;

	bjianruiuturn=0;

	bthirduturn=0;

	fangxiangpanbuhuizheng=0;

	bqishiluduan=1;

	bspecialuturnluduan=0;

	bspecialnoleftturnluduan=0;

	bleftturnban=0;
	brightturnban=0;

	bnobiandaoraozhang=0;

	bleftsidelane=0;
	brightsidelane=0;

	bGpsGanrao=0;
	bLKAsuidao=0;

	bApaActive=0;
	PXApaActive=0;
	bSuidao1Active=0;
	bSuidao2Active=0;

	uturndaoche=0;

	bspecialraozhangcanshu=0;

	kuaisulufulu=0;

	rouchestatus=0;

	realsteerangle=0;

	systemtimer10ms=0;
	systemtimer10ms_his=0;

	AimFlag_ApaPath1=false;
	AimFlag_ApaPath2=false;
	AimFlag_ApaPath3=false;

	bjiedaochaoche=0;

	bcheweistrtsearch=false;
	bcheweistrtsearchPX=false;

	// 由于对话框已关闭，所以将返回 FALSE 以便退出应用程序，
	//  而不是启动应用程序的消息泵。
	return FALSE;
}
