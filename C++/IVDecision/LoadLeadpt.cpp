#include "StdAfx.h"
#include "LoadLeadpt.h"



CLoadLeadpt::CLoadLeadpt(void)
{
}

CLoadLeadpt::~CLoadLeadpt(void)
{
}

void CLoadLeadpt::LoadLead()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	FILE    *m_pLoadRNDF;
	char    Buf[128];
	memset(Buf,0,128);
	int     i = 0;
	CString strTemp;
	char    *strtest = NULL;
	state_struct node_state;

	fopen_s(&m_pLoadRNDF,"Points.txt","r");
	if( fseek(m_pLoadRNDF,0L,SEEK_SET))
		AfxMessageBox(_T("��ȡ�ļ�ʧ��"));
	else
	{
		while(m_result.GetCount())
		{
			m_result.DeleteString(0);
		}

		while(NULL != fgets(Buf,128,m_pLoadRNDF))
		{
			strTemp = strtok_s(Buf,"	,\t\n",&strtest);
			i = atoi(strTemp);
			strTemp = strtok_s(NULL,"	,\t\n",&strtest);
			node_state.lat = atof(strTemp);
			strTemp = strtok_s(NULL,"	,\t\n",&strtest);
			node_state.lng = atof(strTemp);
			strTemp = strtok_s(NULL,"	,\t\n",&strtest);
			node_state.height = atof(strTemp);
			strTemp = strtok_s(NULL,"	,\t\n",&strtest);
			node_state.param1 = atoi(strTemp);
			strTemp = strtok_s(NULL,"	,\t\n",&strtest);
			node_state.param2 = atoi(strTemp);	
			
			CString result_str, result_str1, result_str2, result_str3;
			result_str1.Format("%d--����:%.7f--γ��:%.7f--�߶�:%.3f--",i,node_state.lng,node_state.lat,node_state.height);
			switch(node_state.param1)
			{
			case 0:
				result_str2.Format("����1:���--");
				break;
			case 1:
				result_str2.Format("����1:��������--");
				break;
			case 2:
				result_str2.Format("����1:����ڳ���--");
				break;
			case 3:
				result_str2.Format("����1:�յ�--");
				break;
			default:
				break;
			}
			switch(node_state.param2)
			{
			case 0:
				result_str3.Format("����2:δ֪\n");
				break;
			case 1:
				result_str3.Format("����2:ֱ��\n");
				break;
			case 2:
				result_str3.Format("����2:��ת\n");
				break;
			case 3:
				result_str3.Format("����2:��ת\n");
				break;
			case 4:
				result_str3.Format("����2:��־��\n");
				break;
			default:
				break;
			}
			result_str.Format("%s%s%s",result_str1,result_str2,result_str3);
			m_result.AddString(result_str);
			m_result.AddString("\n");
		}
	}
	fclose(m_pLoadRNDF);
}
