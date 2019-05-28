#pragma once
#include "afxwin.h"
#include "SocketTransfer.h"

// DataTest 对话框

class DataTest : public CDialog
{
	DECLARE_DYNAMIC(DataTest)

public:
	DataTest(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~DataTest();

// 对话框数据
	enum { IDD = IDD_DATASEND };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	
public:
	int m_Brake;
public:
	int m_Turn;
public:
	int m_Velocity;
public:
	int m_Key;
public:
	afx_msg void OnBnClickedButton1();
public:
	afx_msg void OnBnClickedConsend();
};
