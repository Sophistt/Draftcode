# include <math.h>
# include <stdlib.h>
# include "iostream"
#include "LoadRNDF.h"
#pragma once
#include "afxdtctl.h"
using namespace std;


class CAstar
{
public:
	CAstar(void);
public:
	~CAstar(void);
public:
struct MAPPOINT *open,*close;
//CLoadRNDF rndf;
 float solvedistance(MAPPOINT p,MAPPOINT *q);
 float solveheuristic(MAPPOINT *p,MAPPOINT q);
void addopen(struct MAPPOINT *pno);
int deleteopen(struct MAPPOINT *pyes);
void addclose(struct MAPPOINT *cno);
struct MAPPOINT *SearchBest();
void Sort() ;
bool existopen(struct MAPPOINT *Eo,struct MAPPOINT *noop);
bool existclose(struct MAPPOINT *Ec,struct MAPPOINT *nocl);
//CString VariantToString(VARIANT var);
};