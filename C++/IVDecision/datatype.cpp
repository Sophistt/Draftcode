/*----------------------------------------------------------
			
			  本程序用于定义自定义的功能函数

----------------------------------------------------------*/
#include "stdafx.h"
#include "datatype.h"

////////////////////////////////////////////////////////
//按字复制数据
////////////////////////////////////////////////////////
void CopyWORD(WORD* wp1,WORD* wp2)
{
    *wp2 = *wp1;
}
////////////////////////////////////////////////////////
//按双字复制数据
////////////////////////////////////////////////////////
void CopyDWORD(DWORD* wp1,DWORD* wp2)
{
    *wp2 = *wp1;
}