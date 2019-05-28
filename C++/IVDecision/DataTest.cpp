// DataTest.cpp : 实现文件
//

#include "stdafx.h"
#include "IVDecision.h"
#include "DataTest.h"


// DataTest 对话框

IMPLEMENT_DYNAMIC(DataTest, CDialog)

DataTest::DataTest(CWnd* pParent /*=NULL*/)
	: CDialog(DataTest::IDD, pParent)
	, m_Brake(0)
	, m_Turn(0)
	, m_Velocity(0)
	, m_Key(0)
{

}

DataTest::~DataTest()
{
}

void DataTest::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_BRAKE, m_Brake);
	DDX_Text(pDX, IDC_TURN, m_Turn);
	DDX_Text(pDX, IDC_VELOCITY, m_Velocity);
	DDX_Text(pDX, IDC_EDIT1, m_Key);
}


BEGIN_MESSAGE_MAP(DataTest, CDialog)
	ON_BN_CLICKED(IDC_BUTTON1, &DataTest::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_CONSEND, &DataTest::OnBnClickedConsend)
END_MESSAGE_MAP()


// DataTest 消息处理程序

void DataTest::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialog::OnOK();
	CSocketTransfer sock;
//	int  f=1;
	int v=0;
    
	CString str,str1;
	const char *sl = "ok";
//	str1.Format("%f",m_Turn);
	str.Format("%sG100T%dV%dB%dK%d;","%",m_Turn,v,m_Brake,m_Key);
   // const char* sttrr=str1.GetString();
	sock.ClientConnect("192.168.0.106",2222);
	if (sock.SuccSocket)
	sock.ClientSend(sl);
	AfxMessageBox(sl);
}

void DataTest::OnBnClickedConsend()
{
	// TODO: 在此添加控件通知处理程序代码
	CSocketTransfer sock;
	int j=0;
	int t=0;
	int v=0;
	int b=0;
	int k=1;
	int dir_flag=1;
	int bdir_flag=1;
	int i=1;
	while(1)
	{
		sock.ClientConnect("192.168.0.109",3456);
		//if (sock.SuccSocket)
		{
				j=100;
				while(j--)
				{
					if(t>19500*6)
					  {
						  dir_flag =-1;
					  }
					else if(t<-19500*6)
					  {
						  dir_flag = 1;
					  }
			 /*        if(b>3500)
					  {
						  dir_flag =-1;
					  }
					else if(b<-3500)
					  {
						  dir_flag = 1;
					  }*/
					CString str;
					str.Format("%sG100T%dV%dB%dK%di%d;","%",t,v,b,k,i);
					const char* sttrr=str.GetString();
					sock.ClientSend(sttrr);
					t=t+(dir_flag * 5000);
					i++;
					Sleep(200);
	 //       b=b+(bdir_flag * 500);
		        }
		}
	}
}
