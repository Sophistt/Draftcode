#pragma once


// CSimu �Ի���

#include "UTurn.h"
#include "afxcmn.h"

class CSimu : public CDialog
{
	DECLARE_DYNAMIC(CSimu)

public:
	CSimu(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CSimu();

	

// �Ի�������
	enum { IDD = IDD_SIMU };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	
public:
	afx_msg void OnBnClickedOk();
public:
	afx_msg void OnBnClickedButton1();
public:
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
public:
//	afx_msg void OnNMCustomdrawSlider1(NMHDR *pNMHDR, LRESULT *pResult);
public:
	CSliderCtrl slider;
public:
	int slidernum;
public:
//	afx_msg void OnNMThemeChangedSlider1(NMHDR *pNMHDR, LRESULT *pResult);
public:
//	afx_msg void OnNMCustomdrawSlider1(NMHDR *pNMHDR, LRESULT *pResult);
};
