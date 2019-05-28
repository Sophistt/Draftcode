// DlgRNDGPatch.cpp : 实现文件
//

#include "stdafx.h"
#include "IVDecision.h"
#include "DlgRNDGPatch.h"


// CDlgRNDGPatch 对话框

IMPLEMENT_DYNAMIC(CDlgRNDGPatch, CDialog)

CDlgRNDGPatch::CDlgRNDGPatch(CWnd* pParent /*=NULL*/)
	: CDialog(CDlgRNDGPatch::IDD, pParent)
	, m_edit_rot(0)
{

}

CDlgRNDGPatch::~CDlgRNDGPatch()
{
}

void CDlgRNDGPatch::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_STATIC_RNDF_GPSMAP, m_static_rndf_gpsmap);
	DDX_Control(pDX, IDC_STATIC_RNDF_XYMAP, m_static_rndf_xymap);
	DDX_Text(pDX, IDC_EDIT_RNDF_ROT, m_edit_rot);
}


BEGIN_MESSAGE_MAP(CDlgRNDGPatch, CDialog)
	ON_BN_CLICKED(IDC_BTN_RNDFQJGPS, &CDlgRNDGPatch::OnBnClickedBtnRndfqjgps)
	ON_BN_CLICKED(IDC_BTN_RNDFGBXY, &CDlgRNDGPatch::OnBnClickedBtnRndfgbxy)
	ON_BN_CLICKED(IDC_BTN_RNDFEXIT, &CDlgRNDGPatch::OnBnClickedBtnRndfexit)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BTN_RNDFMATCH, &CDlgRNDGPatch::OnBnClickedBtnRndfmatch)
	ON_BN_CLICKED(IDC_BTN_RNDFROT, &CDlgRNDGPatch::OnBnClickedBtnRndfrot)
	ON_BN_CLICKED(IDC_BTN_FLIPH, &CDlgRNDGPatch::OnBnClickedBtnFliph)
	ON_BN_CLICKED(IDC_BTN_FLIPV, &CDlgRNDGPatch::OnBnClickedBtnFlipv)
	ON_BN_CLICKED(IDC_GLOBAL_PLAN, &CDlgRNDGPatch::OnBnClickedGlobalPlan)
END_MESSAGE_MAP()


// CDlgRNDGPatch 消息处理程序

void CDlgRNDGPatch::OnBnClickedBtnRndfqjgps()
{
	// TODO: 在此添加控件通知处理程序代码
	/*RNDF.Openfile();*/

	CvtRNDF2ImgMap( RNDF, rndfImage, rndf_centerx, rndf_centery, 1, 0 );
	CvtGPATH2ImgMap( RNDF, rndfImage, GPath, rndf_centerx, rndf_centery, 1, 0 );

	//cvFlip( rndfImage, NULL, 1 );
	m_static_rndf_gpsmap.zShowImage(rndfImage);

	//CString path = ReturnPath();
	//cvSaveImage(path+"\\"+"RNDFGPS.jpg", rndfImage);
}

void CDlgRNDGPatch::OnBnClickedBtnRndfgbxy()
{
	// TODO: 在此添加控件通知处理程序代码
	RNDFXY.Openfile();

	IplImage * tmpImage = cvCreateImage(cvSize(DIM_M,DIM_M), 8, 1);
	CvtRNDF2ImgMap( RNDFXY, tmpImage, rndfxy_centerx, rndfxy_centery, 0, 0 );

	CvRect rect = cvBoundingRect( tmpImage );
	rect.x -= 32; if(rect.x < 0) rect.x = 0;
	rect.y -= 32; if(rect.y < 0) rect.y = 0;
	rect.width += 64; if( rect.x+rect.width > tmpImage->width ) rect.width = tmpImage->width-rect.x;
	rect.height += 64; if( rect.y+rect.height > tmpImage->height ) rect.height = tmpImage->height-rect.y;
	if(rndfxyImage)  cvReleaseImage(&rndfxyImage);

	rndfxyImage = cvCreateImage(cvSize(rect.width,rect.height), 8, 1);
	cvSetImageROI(tmpImage, rect);
	cvCopy(tmpImage, rndfxyImage);
	cvResetImageROI(tmpImage);

	m_static_rndf_xymap.zShowImage(rndfxyImage);

	CString path = ReturnPath();
	//cvSaveImage(path+"\\"+"RNDFXY.jpg", rndfxyImage);
}

void CDlgRNDGPatch::OnBnClickedBtnRndfrot()
{
	// TODO: 在此添加控件通知处理程序代码
	if(!rndfxyImage)  return ;

	UpdateData(TRUE);
	if( !m_edit_rot ) return;
	zRotateImage( rndfxyImage, rndfxyImage, m_edit_rot );

	m_static_rndf_xymap.zShowImage(rndfxyImage);

	CString path = ReturnPath();
	//cvSaveImage(path+"\\"+"RNDFXY.jpg", rndfxyImage);
}

void CDlgRNDGPatch::OnBnClickedBtnRndfmatch()
{
	// TODO: 在此添加控件通知处理程序代码
	//缩小匹配
	IplImage *roitmp = cvCreateImage(cvSize(rndfxyImage->width/2, rndfxyImage->height/2), 8, 1);
	cvPyrDown( rndfxyImage, roitmp );
	IplImage *srctmp = cvCreateImage(cvSize(rndfImage->width/2, rndfImage->height/2), 8, 1);
	cvPyrDown( rndfImage, srctmp );
	//////
	CvMemStorage* storage;
	CvSeq *pTemplate;

	storage = cvCreateMemStorage(0);
	pTemplate = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );

	IplImage *roitmpdisp = cvCloneImage(roitmp);
	zCreatePyrRotTemplate( roitmp, roitmpdisp, 30, 100, 3, pTemplate, -5, 5, 0.2, 3);

	double fsimilar;
	CvPoint2D32f pos;
	double fangle;

	zCalImagePrySimilar( srctmp, pTemplate, 0.7, fsimilar, pos, fangle );

	cvCircle(rndfImage, cvPoint(int(pos.x*2),int(pos.y*2)), 9, cvScalar(100), CV_AA);

	m_static_rndf_gpsmap.zShowImage(rndfImage);

	cvReleaseMemStorage(&storage);
	cvReleaseImage(&roitmp);
	cvReleaseImage(&srctmp);
	cvReleaseImage(&roitmpdisp);

	CString str;
	str.Format("similar = %.3f, angle = %.3f", fsimilar, fangle);
	AfxMessageBox(str);
}


void CDlgRNDGPatch::GetRNDFParam( CLoadRNDF &rndf,struct MAPPOINT *path )
{
	RNDF = rndf;
	GPath = path;
	DIM_M = GetMapSize(RNDF)+200;
	//DIM_M = 4000;

	rndfImage = cvCreateImage(cvSize(DIM_M,DIM_M), 8, 3);
	rndf_centerx = 0;
	rndf_centery = 0;

	//rndfxyImage = cvCreateImage(cvSize(DIM_M,DIM_M), 8, 3);
}
void CDlgRNDGPatch::OnBnClickedBtnRndfexit()
{
	// TODO: 在此添加控件通知处理程序代码
	CDlgRNDGPatch::OnOK();
}

BOOL CDlgRNDGPatch::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  在此添加额外的初始化

	rndfxy_centerx = 0;
	rndfxy_centery = 0;

	b_load_rndf = 0;


	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}

void CDlgRNDGPatch::OnDestroy()
{
	CDialog::OnDestroy();

	// TODO: 在此处添加消息处理程序代码
	cvReleaseImage(&rndfImage);
	cvReleaseImage(&rndfxyImage);
}

void CDlgRNDGPatch::OnBnClickedBtnFliph()
{
	// TODO: 在此添加控件通知处理程序代码
	cvFlip( rndfxyImage, NULL, 1 );

	m_static_rndf_xymap.zShowImage(rndfxyImage);

	CString path = ReturnPath();
	//cvSaveImage(path+"\\"+"RNDFXY.jpg", rndfxyImage);
}

void CDlgRNDGPatch::OnBnClickedBtnFlipv()
{
	// TODO: 在此添加控件通知处理程序代码
	cvFlip( rndfxyImage );

	m_static_rndf_xymap.zShowImage(rndfxyImage);

	CString path = ReturnPath();
	//cvSaveImage(path+"\\"+"RNDFXY.jpg", rndfxyImage);
}

void CDlgRNDGPatch::OnBnClickedGlobalPlan()
{
	// TODO: 在此添加控件通知处理程序代码

}
