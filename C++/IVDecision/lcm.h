#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/select.h>
#endif

#include <fcntl.h>

#include <lcm/lcm.h>
#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif
#include "lcm.h"
#include<fstream>
using namespace std;
	 ofstream out3("map111.txt");

	 
