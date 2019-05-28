// ZPictureCtrl.cpp : implementation file
//

#include "stdafx.h"

#include "ZPictureCtrl.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CZPictureCtrl

CZPictureCtrl::CZPictureCtrl()
{
	
	///////////////////////////cv/////////////////////////////
	srcImage = NULL;
	displayImage = NULL;
	displayImage = cvCreateImage(cvSize(1024,768), 8, 3);

	displayRect = CRect( 0, 0, 0, 0 );

	m_RectTracker.m_nStyle = CRectTracker::resizeOutside|CRectTracker::solidLine;
	m_RectTracker.m_nHandleSize = 6;
	m_RectTracker.m_rect.SetRect(0,0,0,0);

	b_right_down = FALSE;
}

CZPictureCtrl::~CZPictureCtrl()
{
	////////////////////////cv//////////////////////////////
	if (displayImage)
		cvReleaseImage(&displayImage);

	if (srcImage)
		cvReleaseImage(&srcImage);
}

BOOL CZPictureCtrl::Create(LPCTSTR lpszText, DWORD dwStyle, const RECT& rect,
							 CWnd* pParentWnd, UINT nID /* = 0xffff */)
{
	BOOL bCreate = CStatic::Create(lpszText, dwStyle, rect, pParentWnd, nID);
	if (!bCreate)
	{
		return FALSE;
	}
	
	return TRUE;
}


BEGIN_MESSAGE_MAP(CZPictureCtrl, CStatic)
	//{{AFX_MSG_MAP(CZPictureCtrl)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_PAINT()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_SETCURSOR()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CZPictureCtrl message handlers

void CZPictureCtrl::OnLButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
//	AfxMessageBox("Lbutton down");

	int nIn; //定义一个鼠标的点击值；
	nIn=m_RectTracker.HitTest(point); //看看点到了哪了
	if(nIn<0)  //不在四边形区域内；
	{
		m_RectTracker.TrackRubberBand(this,point,TRUE);
		m_RectTracker.m_rect.NormalizeRect();

		Invalidate(); //引起OnDraw函数的发生；
	}
	else 
		//在四边形区域内：
	{
		m_RectTracker.Track(this,point,TRUE);
		Invalidate();
	}
	
	CStatic::OnLButtonDown(nFlags, point);
}

void CZPictureCtrl::OnLButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	
	CStatic::OnLButtonUp(nFlags, point);
}

void CZPictureCtrl::OnMouseMove(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	if (b_right_down)
	{
		draw_point(point.x, point.y);
	}
	
	CStatic::OnMouseMove(nFlags, point);
}

void CZPictureCtrl::OnPaint() 
{
	CPaintDC dc(this); // device context for painting
	
	// TODO: Add your message handler code here
	if (displayRect.Width() < 1 || displayRect.Height() < 1 )
		GetClientRect( &displayRect );

	if (displayImage){
		zDisplay(displayImage, &dc, displayRect);
	}

	//dc.Draw3dRect(m_RectTracker.m_rect,RGB(255,0,0),RGB(255,0,0));//0x000000,0x000000);  //画矩形边界

	// Do not call CStatic::OnPaint() for painting messages
}

void CZPictureCtrl::OnRButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	b_right_down = TRUE;
	
	CStatic::OnRButtonDown(nFlags, point);
}

void CZPictureCtrl::OnRButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	b_right_down = FALSE;
	
	CStatic::OnRButtonUp(nFlags, point);
}

BOOL CZPictureCtrl::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message) 
{
	// TODO: Add your message handler code here and/or call default

	if (pWnd == this && m_RectTracker.SetCursor(this, nHitTest))
	{
		return TRUE;
	}

	
	return CStatic::OnSetCursor(pWnd, nHitTest, message);
}

//////////////////////////////////////////cv/////////////////////////////////////////

/////////////////////////////////////////////////////////////
//***********           显示           *********************
/////////////////////////////////////////////////////////////


// CImageView 消息处理程序
/*填充DIB图像信息*/
void CZPictureCtrl::FillBitmapInfo(BITMAPINFO *bmi, int width, int height, int bpp, int origin)
{
	VERIFY( bmi && width >= 0 && height >= 0 && (bpp == 8 || bpp == 24 || bpp == 32));

	BITMAPINFOHEADER* bmih = &(bmi->bmiHeader);

	memset( bmih, 0, sizeof(*bmih));
	bmih->biSize = sizeof(BITMAPINFOHEADER); 
	bmih->biWidth = width;
	bmih->biHeight = origin ? abs(height) : -abs(height);
	bmih->biPlanes = 1; 
	bmih->biBitCount = (unsigned short)bpp;
	bmih->biCompression = BI_RGB;

	if( bpp == 8 )
	{
		RGBQUAD* palette = bmi->bmiColors;
		int i;
		for( i = 0; i < 256; i++ )
		{
			palette[i].rgbBlue = palette[i].rgbGreen = palette[i].rgbRed = (BYTE)i;
			palette[i].rgbReserved = 0;
		}
	}
}
/*显示图像
参数Src为输入图像
参数pDC为设备指针
参数rect为图像大小*/
void CZPictureCtrl::zDisplay(IplImage *src, CDC *pDC, CRect rect)
{
	if( src && src->depth == IPL_DEPTH_8U )
	{
		uchar buffer[sizeof(BITMAPINFOHEADER) + 1024];
		BITMAPINFO* bmi = (BITMAPINFO*)buffer;
		int bmp_w = src->width, bmp_h = src->height;

		int bpp = (src->depth & 255)*src->nChannels;
		FillBitmapInfo( bmi, bmp_w, bmp_h, bpp, src->origin );

		if (src->width == rect.right-rect.left && src->height == rect.bottom -rect.top){
			SetDIBitsToDevice(
				pDC->m_hDC, rect.left, rect.top, bmp_w, bmp_h, 0, 0, 0, bmp_h,
				src->imageData,
				bmi, DIB_RGB_COLORS );
		}
		else{
			pDC->SetStretchBltMode(COLORONCOLOR);
			StretchDIBits(pDC->GetSafeHdc(),
				rect.left,
				rect.top,
				rect.right-rect.left,				//显示窗口宽度
				rect.bottom-rect.top,				//显示窗口高度
				0,
				0,
				src->width,				//图像宽度
				src->height,				//图像高度
				src->imageData,		//图像数据
				bmi,			//BMP图像描述信息
				DIB_RGB_COLORS,
				SRCCOPY
				);
		}
	}
}

void CZPictureCtrl::zShowImage(IplImage *src, CRect rect)
{
	if ( displayImage )    cvReleaseImage( &displayImage );
	if (src->nChannels == 3)
		displayImage = cvCloneImage(src);
	else
	{
		displayImage = cvCreateImage(cvGetSize(src), 8, 3);
		cvCvtColor(src, displayImage, CV_GRAY2BGR);
	}

	displayRect = rect;
	Invalidate(false);
}

void CZPictureCtrl::zShowImage(IplImage *src)
{
	if ( displayImage )    cvReleaseImage( &displayImage );
	if (src->nChannels== 3)
	{
		//displayImage = cvCreateImage(cvGetSize(src), 8, 3);
		
		displayImage = cvCloneImage(src);
	}
	else
	{
		displayImage = cvCreateImage(cvGetSize(src), 8, 3);
		cvCvtColor(src, displayImage, CV_GRAY2BGR);
	}

	GetClientRect( &displayRect );

	Invalidate(false);
}
///////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////

void CZPictureCtrl::zGetSelectRect( CRect &rect )
{
	if (displayImage)
	{
		float dw = (float)displayImage->width/((float)displayRect.Width());
		float dh = (float)displayImage->height/((float)displayRect.Height());

		rect.left = int(m_RectTracker.m_rect.left*dw);
		rect.top = int(m_RectTracker.m_rect.top*dh);
		rect.right = int(m_RectTracker.m_rect.right*dw);
		rect.bottom = int(m_RectTracker.m_rect.bottom*dh);
	}
	else{
		rect = m_RectTracker.m_rect;
	}
}

////////////////////////////////////////////////////////////
//  画点
////////////////////////////////////////////////////////////
void CZPictureCtrl::draw_point(int x,int y){
	CRect rect;
	GetClientRect(&rect);

	int imx = int(displayImage->width * (float(x) / rect.Width()));
	int imy = int(displayImage->height * (float(y) / rect.Height()));
	//Draw a circle where is the mouse
	cvCircle(displayImage, cvPoint(imx,imy), 1, CV_RGB(0,0,0), -1, 4, 0);

	Invalidate(FALSE);
}

void CZPictureCtrl::get_draw_image(IplImage *dst)
{
	CvSize size1 = cvGetSize(displayImage);
	CvSize size2 = cvGetSize(dst);
	if ( (size1.width != size2.width) || (size1.height != size2.height))  return;

	cvCvtColor(displayImage, dst, CV_BGR2GRAY);
}