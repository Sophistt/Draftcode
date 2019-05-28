#if !defined(AFX_ZPICTURECTRL_H__6C8128EE_71BC_4AF9_81D4_15FCCAE28A5B__INCLUDED_)
#define AFX_ZPICTURECTRL_H__6C8128EE_71BC_4AF9_81D4_15FCCAE28A5B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// ZPictureCtrl.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CZPictureCtrl window

class CZPictureCtrl : public CStatic
{
// Construction
public:
	CZPictureCtrl();
	BOOL Create(LPCTSTR lpszText, DWORD dwStyle, const RECT& rect,
		CWnd* pParentWnd, UINT nID  = 0xffff );

// Attributes
	////////////////////////////////cv//////////////////////////////

	//显示
public:
	IplImage *srcImage;//原始图像

	void zShowImage(IplImage *src, CRect rect);
	void zShowImage(IplImage *src);

	CRectTracker m_RectTracker;

	//获得选择区域
	void zGetSelectRect( CRect &rect );
	
protected:
	CRect displayRect;
	IplImage *displayImage;
	void FillBitmapInfo(BITMAPINFO *bmi, int width, int height, int bpp, int origin);
	void zDisplay(IplImage *src, CDC *pDC, CRect rect);

//OCR
public:
	void draw_point(int x,int y);
	BOOL b_right_down;
	void get_draw_image(IplImage *dst);
// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CZPictureCtrl)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CZPictureCtrl();

	// Generated message map functions
protected:
	//{{AFX_MSG(CZPictureCtrl)
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnPaint();
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_ZPICTURECTRL_H__6C8128EE_71BC_4AF9_81D4_15FCCAE28A5B__INCLUDED_)
