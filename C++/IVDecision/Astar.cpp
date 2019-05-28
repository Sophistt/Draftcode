#include "StdAfx.h"
#include "Astar.h"
# define M 1000;
# define I 999999

CAstar::CAstar(void)
{
}

CAstar::~CAstar(void)
{
}

float CAstar::solvedistance(MAPPOINT p,MAPPOINT *q)//定义g，此距离是用来作为A星算法中的g值的，不是实际距离，所以直接用经纬度求欧氏距离是可以的
{
	float distance_ludian;
	if(q!=NULL)
	{
		distance_ludian = sqrt(pow((p.x - (q->x)),2) + pow((p.y - (q->y)),2));
	}
	else
	{
		distance_ludian = I;
	}
	return distance_ludian;
}

float CAstar::solveheuristic(MAPPOINT *p,MAPPOINT q)//定义h为曼哈顿距离
{
	float heuristicvalue;
	if(p!=NULL)
	{
		heuristicvalue = fabs((p->x) - q.x) + fabs((p->y) - q.y);
	}
	else
	{
		heuristicvalue = I;
	}
	return heuristicvalue;
}


void CAstar::addopen(struct MAPPOINT *pno)
{
	if (pno!=NULL)
	{
	pno->next = open->next;
	open->next = pno;
	}
}

int CAstar::deleteopen(struct MAPPOINT *pyes)
{
	struct MAPPOINT *Delo;
	Delo = open->next;
	if(Delo == pyes)
	{
		open->next = Delo->next;
		return 1;
	}
	else
	{
		while(Delo!=NULL)
		{
			if (Delo->next == pyes)
			{
				Delo->next = pyes->next;
				pyes->next = NULL;
				return 1;
			}
			Delo = Delo->next;
		}
	}
}

void CAstar::addclose(struct MAPPOINT *cno)
{
	if (cno!=NULL)
	cno->next = close->next;
	close->next = cno;
}

struct MAPPOINT* CAstar::SearchBest() 
{
	struct MAPPOINT *tp;
	if(open->next!=NULL)  	
	{  		
		tp=open->next;  		
		open->next=tp->next;  		
		return tp;  	
	}  	
	else  		
		exit(0);  
} 

void CAstar::Sort()  
{  	
	struct MAPPOINT *temp1,*temp2,*temp3,*temp4;    	
	struct MAPPOINT tempv;  	
	struct MAPPOINT *temp= &tempv;
	for(temp1=open->next;temp1!=NULL && temp1->next!=NULL;temp1=temp1->next)	
	{  
		for(temp2=open;temp2->next!=NULL && temp2->next->next!=NULL;temp2=temp2->next)
		{
			temp3=temp2->next;
			temp4=temp3->next;
			if((temp3->f)>(temp4->f))  
			{
				temp=temp4->next;
				temp2->next = temp4;
				temp4->next = temp3;
				temp3->next = temp;
			}
		}
	}    
} 

bool CAstar::existopen(struct MAPPOINT *Eo,struct MAPPOINT *noop)
{
	while(Eo->next!=NULL)
	{
		if(Eo->next == noop)
			return true;
		Eo = Eo->next;
	}
	return false;
}

bool CAstar::existclose(struct MAPPOINT *Ec,struct MAPPOINT *nocl)
{
	while(Ec->next!=NULL)
	{
		if(Ec->next == nocl)
			return true;
		Ec = Ec->next;
	}
	return false;
}
/*CString CAstar::VariantToString(VARIANT var)
{
	
	CString strValue;
	_variant_t var_t;
	_bstr_t bstr_t;
	CTime time_value;
	COleCurrency var_currency;
	switch(var.vt)
	{
	case VT_EMPTY:
	case VT_NULL:strValue=_T("");break;
	case VT_UI1:strValue.Format(_T("%d"),var.bVal);break;
	case VT_I2:strValue.Format(_T("%d"),var.iVal);break;
	case VT_I4:strValue.Format(_T("%d"),var.lVal);break;
	case VT_R4:strValue.Format(_T("%f"),var.fltVal);break;
	case VT_R8:strValue.Format(_T("%f"),var.dblVal);break;
	case VT_CY:
		var_t=var;
		bstr_t=var_t;
		strValue = (LPCSTR) bstr_t;
		break;
	case VT_BSTR:
		var_t =var;
		bstr_t=var_t;
		strValue.Format(_T("%s"),(const TCHAR *)bstr_t);break;
	case VT_DATE:
		var_t=var;
		bstr_t=var_t;
		strValue = (LPCSTR) bstr_t;
		break;
	case VT_BOOL:strValue.Format(_T("%d"),var.boolVal);break;
	default:strValue=_T("");break;
	}
	return strValue;
}*/