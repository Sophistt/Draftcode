// IVDecision.cpp : ����Ӧ�ó��������Ϊ��
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


// CIVDecisionApp ����

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

	// TODO: �ڴ˴���ӹ�����룬
	// ��������Ҫ�ĳ�ʼ�������� InitInstance ��
}


// Ψһ��һ�� CIVDecisionApp ����

CIVDecisionApp theApp;


// CIVDecisionApp ��ʼ��

BOOL CIVDecisionApp::InitInstance()
{
	// ���һ�������� Windows XP �ϵ�Ӧ�ó����嵥ָ��Ҫ
	// ʹ�� ComCtl32.dll �汾 6 ����߰汾�����ÿ��ӻ���ʽ��
	//����Ҫ InitCommonControlsEx()�����򣬽��޷��������ڡ�
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// ��������Ϊ��������Ҫ��Ӧ�ó�����ʹ�õ�
	// �����ؼ��ࡣ
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

	// ��׼��ʼ��
	// ���δʹ����Щ���ܲ�ϣ����С
	// ���տ�ִ���ļ��Ĵ�С����Ӧ�Ƴ�����
	// ����Ҫ���ض���ʼ������
	// �������ڴ洢���õ�ע�����
	// TODO: Ӧ�ʵ��޸ĸ��ַ�����
	// �����޸�Ϊ��˾����֯��
	SetRegistryKey(_T("Ӧ�ó��������ɵı���Ӧ�ó���"));

	CIVDecisionDlg dlg;
	m_pMainWnd = &dlg;
	INT_PTR nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: �ڴ˴����ô����ʱ�á�ȷ�������ر�
		//  �Ի���Ĵ���
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: �ڴ˷��ô����ʱ�á�ȡ�������ر�
		//  �Ի���Ĵ���
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

	// ���ڶԻ����ѹرգ����Խ����� FALSE �Ա��˳�Ӧ�ó���
	//  ����������Ӧ�ó������Ϣ�á�
	return FALSE;
}
