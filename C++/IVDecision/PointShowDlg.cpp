// PointShowDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "IVDecision.h"
#include "PointShowDlg.h"


// PointShowDlg 对话框

IMPLEMENT_DYNAMIC(PointShowDlg, CDialog)

PointShowDlg::PointShowDlg(CWnd* pParent /*=NULL*/)
	: CDialog(PointShowDlg::IDD, pParent)
{

}

PointShowDlg::~PointShowDlg()
{
}

void PointShowDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(PointShowDlg, CDialog)
END_MESSAGE_MAP()

void PointShowDlg::Show(int i,int j)
{
   CClientDC dc(this);
   int count=256/i;
   for (int k=0; k<i; k++)
   {	
	 /*  dc.MoveTo(PointSets[0].GetDimension()[0],PointSets[0].GetDimension()[1]);*/
	  /* Pen.CreatePen(PS_SOLID,2,RGB(k*10,k*10,k*10)); */
	   for (int h=0; h<j; h++)
	   {	
		   if (mpoint[h].clusterID==k)
		   {
			   dc.SetPixel(mpoint[h].x,mpoint[h].y,RGB(k*count,k*count,k*count));
		   }
	   }
   }
}

