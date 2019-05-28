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
	#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif
#include "lcm.h"
#include<fstream>
using namespace std;
	 ofstream out3("map111.txt");

	 
