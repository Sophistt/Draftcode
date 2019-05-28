// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�

#pragma once

#ifndef _SECURE_ATL
#define _SECURE_ATL 1
#endif

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN		// �� Windows ͷ���ų�����ʹ�õ�����
#endif

// ���������ʹ��������ָ����ƽ̨֮ǰ��ƽ̨�����޸�����Ķ��塣
// �йز�ͬƽ̨����Ӧֵ��������Ϣ����ο� MSDN��
#ifndef WINVER				// ����ʹ���ض��� Windows XP ����߰汾�Ĺ��ܡ�
#define WINVER 0x0501		// ����ֵ����Ϊ��Ӧ��ֵ���������� Windows �������汾��
#endif

#ifndef _WIN32_WINNT		// ����ʹ���ض��� Windows XP ����߰汾�Ĺ��ܡ�
#define _WIN32_WINNT 0x0501	// ����ֵ����Ϊ��Ӧ��ֵ���������� Windows �������汾��
#endif						

#ifndef _WIN32_WINDOWS		// ����ʹ���ض��� Windows 98 ����߰汾�Ĺ��ܡ�
#define _WIN32_WINDOWS 0x0410 // ��������Ϊ�ʺ� Windows Me ����߰汾����Ӧֵ��
#endif

#ifndef _WIN32_IE			// ����ʹ���ض��� IE 6.0 ����߰汾�Ĺ��ܡ�
#define _WIN32_IE 0x0600	// ����ֵ����Ϊ��Ӧ��ֵ���������� IE �������汾��ֵ��
#endif

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS	// ĳЩ CString ���캯��������ʽ��

// �ر� MFC ��ĳЩ�����������ɷ��ĺ��Եľ�����Ϣ������
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC ��������ͱ�׼���
#include <afxext.h>         // MFC ��չ
#include <windows.h>

#include <afxdisp.h>        // MFC �Զ�����
#include "SkinMagicLib.h"
#pragma comment(lib,"SkinMagicTrial.lib")

//#include "../ui/source/XTToolkitPro.h"   // Codejock Software Components
//#if DEBUG
//#pragma comment(lib, "../../ui/vc80/ToolkitPro1200vc80SD.lib")
//#else
//#pragma comment(lib, "../../ui/vc80/ToolkitPro1200vc80S.lib")
//#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#define AUTODRIVEREQ WM_USER+8001	//�Զ���ʻ����
#define MOTIONPLANREQ WM_USER+8002	//�˶��滮�������
#define IBEOREQ WM_USER+8003		//IBEO����
#define HDLREQ WM_USER+8004			//HDL����
#define LANLINEREQ WM_USER+8005		//������ʶ��������
#define TRAFFLIGHTREQ WM_USER+8006	//�źŵ�ʶ��������
#define TRAFFSIGNREQ WM_USER+8007	//��ʶ��ʶ��������
#define INFRAREDREQ WM_USER+8008	//�������ʶ��������
#define GPSENCREQ WM_USER+8009		//GPS��������
#define RETRMSGREQ WM_USER+8010		//ADASIS�����ط�����
#define NEWRPRCV WM_USER+8011		//���ݸ�����Ϣ
#define ADASISMSGREQ WM_USER+8012	//ADASIS���ݷ�������
#define STOPAUTODRIVING WM_USER+8013	//ֹͣ�Զ���ʻ����


#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>		// MFC �� Internet Explorer 4 �����ؼ���֧��
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>			// MFC �� Windows �����ؼ���֧��
#endif // _AFX_NO_AFXCMN_SUPPORT









#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif


