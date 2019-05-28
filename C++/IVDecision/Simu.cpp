// Simu.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "IVDecision.h"
#include "Simu.h"
#include "UTurn.h"
int i=0;
CRect *re;
// CSimu �Ի���

IMPLEMENT_DYNAMIC(CSimu, CDialog)

CSimu::CSimu(CWnd* pParent /*=NULL*/)
	: CDialog(CSimu::IDD, pParent)
	, slidernum(0)
{
i=50;
}

CSimu::~CSimu()
{
}

void CSimu::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_SLIDER1, slider);
}


BEGIN_MESSAGE_MAP(CSimu, CDialog)
	ON_BN_CLICKED(IDOK, &CSimu::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON1, &CSimu::OnBnClickedButton1)
	ON_WM_MOUSEWHEEL()
//	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER1, &CSimu::OnNMCustomdrawSlider1)
//	ON_NOTIFY(NM_THEMECHANGED, IDC_SLIDER1, &CSimu::OnNMThemeChangedSlider1)
//ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER1, &CSimu::OnNMCustomdrawSlider1)
END_MESSAGE_MAP()


// CSimu ��Ϣ�������

void CSimu::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
CUTurn uturn;
	
	CFile file;
	char szFileFilter[]=
		"ȫ������|*.txt;*.jpg|"
	
		"txt File(*.txt)|*.txt|"
		"jpg File(*.JPEG)|*.jpg|"

		"All File|(*.*)|";

	CFileDialog dlg(TRUE, NULL, NULL,OFN_HIDEREADONLY, szFileFilter);

	if (dlg.DoModal()==IDOK)
	{
		CString PathName=dlg.GetPathName();
		PathName.MakeUpper();
		file.Open(PathName,CFile::modeReadWrite);
		//CArchive archive(&file,CArchive::load);
		//archive.
		char A[100];
		file.Read(A,file.GetLength());
		file.Close();
		CString str;
		str.Format("%s",A);
		MessageBox(str);


		
	}


}

void CSimu::OnBnClickedButton1()
{
	
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	//CFile imgfile;
	//char szFileFilter[]=
	////"ȫ������|*.bmp|"
	//"bmp File(*.bmp)|*.bmp|"
	//"All File|(*.*)|";

	//CFileDialog imgdlg(TRUE, NULL, NULL, OFN_HIDEREADONLY, szFileFilter);
	//if (imgdlg.DoModal()==IDOK)
	//{
		//CString PathName=imgdlg.GetPathName();
		CString PathName("D:\\uturn1.jpg");
		//imgfile.Open(PathName, CFile::modeReadWrite);

       /* CBitmap bitmap;
		HBITMAP hBitmap=(HBITMAP)LoadImage(NULL, PathName, IMAGE_BITMAP,0,0,LR_LOADFROMFILE);
		bitmap.Attach(hBitmap);
		CClientDC *pDC=new CClientDC(this);
		CDC memDC;
		memDC.CreateCompatibleDC(pDC);
		memDC.SelectObject(&bitmap);
		
		BITMAP bitinfo;
		bitmap.GetObject(sizeof(bitinfo),&bitinfo);
		pDC->BitBlt(0,0,bitinfo.bmWidth,bitinfo.bmHeight,&memDC,0,0,SRCCOPY);*/

		IPicture *m_picture;
		OLE_XSIZE_HIMETRIC m_width;
		OLE_YSIZE_HIMETRIC m_height;
		CFile m_file(PathName,CFile::modeRead);

		DWORD m_filelen=m_file.GetLength();

		HGLOBAL m_hglobal=GlobalAlloc(GMEM_MOVEABLE,m_filelen);

		LPVOID pvdata=NULL;
		pvdata=GlobalLock(m_hglobal);
		m_file.Read(pvdata,m_filelen);
        IStream* m_stream;
		GlobalUnlock(m_hglobal);
		CreateStreamOnHGlobal(m_hglobal,TRUE,&m_stream);
		OleLoadPicture(m_stream,m_filelen,TRUE,IID_IPicture,(LPVOID*)&m_picture);
		m_picture->get_Width(&m_width);
		m_picture->get_Height(&m_height);

		CDC *dc=GetDC();

		CRect rect;
		GetClientRect(rect);
		SetScrollRange(SB_VERT,0,(int)(m_height/26.45)-rect.Height());
		SetScrollRange(SB_HORZ,0,(int)(m_width/26.45)-rect.Width());

		m_picture->Render(*dc,10,80,(int)(m_width/i),(int)(m_height/i),0,m_height,m_width,-m_height,NULL);

      

		

	//}




}

BOOL CSimu::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	//InvalidateRect(re,TRUE);
	
		//i=50+0.01*zDelta;
	
	

	return CDialog::OnMouseWheel(nFlags, zDelta, pt);
}

//void CSimu::OnNMCustomdrawSlider1(NMHDR *pNMHDR, LRESULT *pResult)
//{
//	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
//	// TODO: �ڴ���ӿؼ�֪ͨ����������
//	slider.SetPos(0);
//	slider.SetRange(-10,10);
//	UpdateData(TRUE);
//	
//	i=slidernum+50;
//	
//	CString str;
//	str.Format("%d",slidernum);
//	//MessageBox(str);
//	*pResult = 0;
//}

//void CSimu::OnNMThemeChangedSlider1(NMHDR *pNMHDR, LRESULT *pResult)
//{
//	// �ù���Ҫ��ʹ�� Windows XP ����߰汾��
//	// ���� _WIN32_WINNT ���� >= 0x0501��
//	// TODO: �ڴ���ӿؼ�֪ͨ����������
//
//	
//	//CSimu::OnBnClickedButton1();
//	CString str;
//	str.Format("%d",slidernum);
//	MessageBox(str);
//	*pResult = 0;
//}

