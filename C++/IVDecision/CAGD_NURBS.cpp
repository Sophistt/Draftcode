#include "StdAfx.h"
#include "CAGD_NURBS.h"
#include "cmath"

CCAGD_NURBS::CCAGD_NURBS(void)
{
	BSpline_k = 3;
	BSpline_Type = ZJYB;
	Knot_Mark = true;
	Deboor_Point_Num = 100;
	Base_Count = 100;
	Deboor_Point_x = NULL;
	Deboor_Point_y = NULL;
}

CCAGD_NURBS::~CCAGD_NURBS(void)
{
	delete []Control_Point_x;
	delete []Control_Point_y;
	delete []Knot_Dif_Array;
	delete []Knot_Mul_Array;
	if(Deboor_Point_x)
		delete []Deboor_Point_x;
	if(Deboor_Point_y)
		delete []Deboor_Point_y;
}

void CCAGD_NURBS::Set_Control_Point_Num(int n)//输入控制顶点数
{
	Control_Point_Num = n;
}

int CCAGD_NURBS::Get_Control_Point_Num()//输出控制顶点数
{
	return Control_Point_Num;
}

void CCAGD_NURBS::Set_Knot_Dif_Num()//得到某种曲线类型下相异节点数
{
	switch (BSpline_Type)
	{
	case JYB:
		Knot_Dif_Num = Control_Point_Num + BSpline_k + 1;//均匀B样条曲线相异节点数为：n + k + 1，这里为数组上限，n从1开始，与Control_Point_Num一致
		break;
	case ZJYB:
		Knot_Dif_Num = Control_Point_Num - BSpline_k + 1;//准均匀B样条曲线相异节点数为：n + k + 1 - k - k = n - k + 1
		break;
	case Bezier:
		Knot_Dif_Num = (Control_Point_Num - 1) / BSpline_k + 1;//分段贝齐尔曲线相异节点数可以通过如下方程求得：2 * (k + 1) + (x - 2) * k = n + k + 1 => x = (n - 1) / k + 1 
		break;
	case FJYB:
		Knot_Dif_Num = Control_Point_Num - BSpline_k + 1;//非均匀B样条曲线相异节点数与准均匀B样条一样：n - k + 1
	default:
		break;
	}
	Knot_Dif_Array = new double[Knot_Dif_Num];//实例化相异节点数组
	Knot_Mul_Array = new int[Knot_Dif_Num];//实例化节点重复度数组
}

void CCAGD_NURBS::Set_Knot()//确定某种曲线类型下的节点矢量
{
	Set_Knot_Dif_Num();//首先得到某种曲线类型下相异节点数  Knot_Dif_Num
	switch (BSpline_Type)
	{
	case JYB:
		for (int i = 0; i < Knot_Dif_Num; i++)
		{
			Knot_Dif_Array[i] = (double)i / (Knot_Dif_Num - 1); //均匀参数化，[0, 1]
			Knot_Mul_Array[i] = 1; //每个节点的重复度为1
		}
		break;
	case ZJYB:
		Knot_Dif_Array[0] = 0; //第一个节点为0
		Knot_Mul_Array[0] = BSpline_k + 1;//第一个节点重复度为 k + 1
		for (int i = 1; i < Knot_Dif_Num - 1; i++)//中间每一个节点
		{
			Knot_Dif_Array[i] = (double)i / (Knot_Dif_Num - 1);
			Knot_Mul_Array[i] = 1;//重复度为1
		}
		Knot_Dif_Array[Knot_Dif_Num-1] = 1;//最后一个节点为1
		Knot_Mul_Array[Knot_Dif_Num-1] = BSpline_k + 1;//最后一个节点重复度为k + 1
		break;
	case Bezier:
		Knot_Dif_Array[0] = 0;//第一个节点为0
		Knot_Mul_Array[0] = BSpline_k + 1;//第一个节点重复度为 k + 1
		for (int i = 1; i < Knot_Dif_Num - 1; i++)
		{
			Knot_Dif_Array[i] = (double)i / (Knot_Dif_Num - 1);
			Knot_Mul_Array[i] = BSpline_k;//中间节点重复度为 k
		}
		Knot_Dif_Array[Knot_Dif_Num-1] = 1;
		Knot_Mul_Array[Knot_Dif_Num-1] = BSpline_k + 1;
		break;
	case FJYB:					//非均匀B样条曲线节点矢量的确定采用Hartley-Judd方法
		double *L;
		double *LL;
		double Sum,temp;
		int n;
		int k;
		int i,j;
		Sum = temp = 0;
		n = Control_Point_Num - 1;
		k = BSpline_k;
		L = new double[n+1];//记录相邻两个控制顶点之间的距离
		LL = new double[n+2];
		for( i = 1; i < n + 1; i++)
		{
			L[i] = sqrt(pow((Control_Point_x[i] - Control_Point_x[i-1]), 2) + pow((Control_Point_y[i] - Control_Point_y[i-1]), 2));//求距离
		}
		for ( i = k + 1; i < n + 2; i++)//分母项
		{
			LL[i] = 0;
			for(j = i - k; j < i; j++)//分子项
			{
				LL[i] += L[j];
			}
			Sum += LL[i];
		}

		Knot_Dif_Array[0] = 0;//第一个节点还是0
		Knot_Mul_Array[0] = BSpline_k + 1;//重复度 k + 1
		for (i = 1; i < Knot_Dif_Num - 1; i++)
		{
			for (j = i + k - k; j <= i + k - 1; j++)
			{
				temp = temp + L[j];
			}
			Knot_Dif_Array[i] = temp / Sum;
			Knot_Mul_Array[i] = 1;
		}
		Knot_Dif_Array[Knot_Dif_Num-1] = 1;//最后一个节点还是1
		Knot_Mul_Array[Knot_Dif_Num-1] = BSpline_k + 1;//重复度 k + 1
		break;
	default: 
		break;
	}
}

double CCAGD_NURBS::Get_Knot_Value(int j)//得到节点矢量中下标值为j的节点值
{
	double u;
	int index = 0;
	if (Knot_Mark == true)//初始为true
	{
		Set_Knot();//设置好节点矢量
		Knot_Mark = false;//设置完成之后，将标致置反
	}
	for (int i = 0; i < Knot_Dif_Num; i++)
	{
		index = index + Knot_Mul_Array[i];
		if ((index - 1) >= j)
		{
			u = Knot_Dif_Array[i];
			break;
		}
	}
	return u;
}

void CCAGD_NURBS::Get_Control_Point(int num,double *x, double *y)//得到控制顶点坐标
{
	Control_Point_Num = num;
	int temp;
	temp = Control_Point_Num;
	Control_Point_x = new double[temp];
	Control_Point_y = new double[temp];
	for (int i = 0; i < Control_Point_Num; i++)
	{
		Control_Point_x[i] = x[i];
		Control_Point_y[i] = y[i];
	}
}
double CCAGD_NURBS::LineSect(double u0,double u1,double t)
{
	t = (t-u0)/(u1-u0);

	if (t>=0 && t<1.0/3.0)
	{
		return t*t*t*9.0/2.0;
	}
	else 	if (t>=1.0/3.0 && t<2.0/3.0)
	{
		return (t*t*t - 3.0*(t-1.0/3.0)*(t-1.0/3.0)*(t-1.0/3.0))*9.0/2.0;
	} 
	else 	if (t>=2.0/3.0 && t<=1.0)
	{
		return 1-(1-t)*(1-t)*(1-t)*9.0/2.0;
	}
	else 
		return 0;
}
void CCAGD_NURBS::DeBoor(double u,double &point_x,double &point_y)//deboor算法，该算法确定参数u下经过k级递推之后的deboor点
{
	int k0, j, i;//k0用来标记当前u所在的节点区间
	int index = 0;
	double denom, alpha;
	double *DeBoor_Px, *DeBoor_Py;

	DeBoor_Px = new double[BSpline_k+1];
	DeBoor_Py = new double[BSpline_k+1];

	//在定义域[Uk, U(n+1)]先确定u所在的节点区间	
	for (i = BSpline_k; i < Control_Point_Num + 1; i++)
	{
		if (Get_Knot_Value(i) <= u)
		{
			if (Get_Knot_Value(i + 1) >= u)
			{
				k0 = i;
				break;
			}
		}
	}

	//这里涉及到在参数u下确定某一个deboor点所需要的k + 1个控制顶点，从D(i-k)到Di共k+1个控制顶点先存储在DeBoor数组中
	for (i = 0; i < BSpline_k + 1; i++)
	{
		DeBoor_Px[i] = Control_Point_x[k0-BSpline_k+i];
		DeBoor_Py[i] = Control_Point_y[k0-BSpline_k+i];
	}

	//Deboor递推过程
	for (i = 1; i <= BSpline_k; i++)//最外层循环控制递推级数，递推级数不大于次数k
	{
		for (j = 0; j <= BSpline_k - i; j++)
		{
			denom = Get_Knot_Value(BSpline_k + j + 1 + k0 - BSpline_k) - Get_Knot_Value( j + i + k0 - BSpline_k);
			if (fabs(denom) <= 0.00005)
			{
				alpha = 0;
			}
			else
			{
				alpha = (u - Get_Knot_Value(j + i + k0 - BSpline_k)) / denom;
			}
			DeBoor_Px[j] = (1 - alpha) * DeBoor_Px[j] + alpha * DeBoor_Px[j+1];
			DeBoor_Py[j] = (1 - alpha) * DeBoor_Py[j] + alpha * DeBoor_Py[j+1];
		}
	}
	//最终得到经过k级递推之后的deboor点
	point_x = DeBoor_Px[0];
	point_y = DeBoor_Py[0];

	delete []DeBoor_Px;
	delete []DeBoor_Py;
}

void CCAGD_NURBS::Get_Draw_Point(int Point_Num)
{
	int i,j;
	int All_Point;
	double delta;
	//Point_Num为每一段B样条曲线上绘制点的个数，All_Point计算了所有曲线段上的绘制电个数
	All_Point = (Control_Point_Num - BSpline_k) * Point_Num + 1;
	//用于存储绘制Deboor点
	Deboor_Point_Num = All_Point;
	Deboor_Point_x = new double[All_Point];
	Deboor_Point_y = new double[All_Point];

	for (i = BSpline_k; i <= Control_Point_Num - 1; i++)//遍历整个定义域
	{
		delta = (Get_Knot_Value(i + 1) - Get_Knot_Value(i)) / Point_Num;//delta为步长
		for (j = 0; j < Point_Num; j++)
		{
			//利用deboor算法来对绘制点进行赋值
			//DeBoor(Get_Knot_Value(i) + j * delta, Deboor_Point_x[(i - BSpline_k) * Point_Num + j], Deboor_Point_y[(i - BSpline_k) * Point_Num + j]);
			//增加奇异混合插值
			DeBoor(Get_Knot_Value(i) + j * delta, Deboor_Point_x[(i - BSpline_k) * Point_Num + j], Deboor_Point_y[(i - BSpline_k) * Point_Num + j]);
			
		}
	}
	DeBoor(Get_Knot_Value(Control_Point_Num), Deboor_Point_x[All_Point-1], Deboor_Point_y[All_Point-1]);
}

void CCAGD_NURBS::Get_Insert_Point(double shape_arfa,int Point_Num,int All_Point,double *Curve_Point_x,double *Curve_Point_y)
{
	int i,j;
	double delta;
	//Point_Num为每一段B样条曲线上绘制点的个数，All_Point计算了所有曲线段上的绘制电个数
	//All_Point = (Control_Point_Num - BSpline_k) * Point_Num + 1;
	//用于存储绘制Deboor点
	Deboor_Point_Num = All_Point;
	double *temp_Curve_Point_x = new double[All_Point];
	double *temp_Curve_Point_y = new double[All_Point];
	double arfa;
	double *cpntx = new double[Control_Point_Num - BSpline_k+1];
	double *cpnty = new double[Control_Point_Num - BSpline_k+1];

	for (i = BSpline_k; i <= Control_Point_Num - 1; i++)//遍历整个定义域
	{
		delta = (Get_Knot_Value(i + 1) - Get_Knot_Value(i)) / Point_Num;//delta为步长
		for (j = 0; j < Point_Num; j++)
		{
			//利用deboor算法来对绘制点进行赋值
			DeBoor(Get_Knot_Value(i) + j * delta, Curve_Point_x[(i - BSpline_k) * Point_Num + j], Curve_Point_y[(i - BSpline_k) * Point_Num + j]);
		}
		cpntx[i - BSpline_k] = Curve_Point_x[(i - BSpline_k) * Point_Num + 0];
		cpnty[i - BSpline_k] = Curve_Point_y[(i - BSpline_k) * Point_Num + 0];
	}
	DeBoor(Get_Knot_Value(Control_Point_Num), Curve_Point_x[All_Point-1], Curve_Point_y[All_Point-1]);
	cpntx[i - BSpline_k] = Curve_Point_x[All_Point-1];
	cpnty[i - BSpline_k] = Curve_Point_y[All_Point-1];

	double s,p1,p2,cpnt,cpntx0,cpntx1,cpnty0,cpnty1;
	for (i = BSpline_k; i <= Control_Point_Num - 1; i++)//遍历整个定义域
	{
		cpntx0 =  cpntx[(i - BSpline_k)];
		cpntx1 =  cpntx[(i - BSpline_k)+1];
		cpnty0 =  cpnty[(i - BSpline_k)];;
		cpnty1 =  cpnty[(i - BSpline_k)+1];

		delta = (Get_Knot_Value(i + 1) - Get_Knot_Value(i)) / Point_Num;//delta为步长
		double oldx,oldy,newx,newy;
		double firstangle,secondangle;
		double maxdiffangle = 0;
		BOOL b_yichang = FALSE;
		arfa = shape_arfa;
		for (j = 0; j < Point_Num; j++)
		{
			s = LineSect(Get_Knot_Value(i),Get_Knot_Value(i+1),Get_Knot_Value(i) + j * delta);
			cpnt = Curve_Point_x[(i - BSpline_k) * Point_Num + j];

			p1 = Control_Point_x[i - BSpline_k+1];
			p2 = Control_Point_x[i - BSpline_k+2];
			temp_Curve_Point_x[(i - BSpline_k) * Point_Num + j] = (1-arfa)*cpnt + (1-s)*(p1-(1-arfa)*cpntx0)+s*(p2-(1-arfa)*cpntx1);

			cpnt = Curve_Point_y[(i - BSpline_k) * Point_Num + j];
			p1 = Control_Point_y[i - BSpline_k+1];
			p2 = Control_Point_y[i - BSpline_k+2];
			temp_Curve_Point_y[(i - BSpline_k) * Point_Num + j] = (1-arfa)*cpnt + (1-s)*(p1-(1-arfa)*cpnty0)+s*(p2-(1-arfa)*cpnty1);

			if (j==1)//计算起点处的切线方向
			{
				double dx = temp_Curve_Point_x[(i - BSpline_k) * Point_Num + j]-temp_Curve_Point_x[(i - BSpline_k) * Point_Num+j-1];
				double dy = temp_Curve_Point_y[(i - BSpline_k) * Point_Num + j]-temp_Curve_Point_y[(i - BSpline_k) * Point_Num+j-1];
				firstangle = atan2(dy,dx);
			}
			else if (j>1)
			{
				double dx = temp_Curve_Point_x[(i - BSpline_k) * Point_Num + j]-temp_Curve_Point_x[(i - BSpline_k) * Point_Num+j-1];
				double dy = temp_Curve_Point_y[(i - BSpline_k) * Point_Num + j]-temp_Curve_Point_y[(i - BSpline_k) * Point_Num+j-1];
				secondangle = atan2(dy,dx);

				if (fabs(secondangle-firstangle)>maxdiffangle)
				{
					maxdiffangle = fabs(secondangle-firstangle);
				}
				if (fabs(secondangle-firstangle)>3.1415/2.0)
				{
					b_yichang = TRUE;
					break;
				}
			}
		}
		if (b_yichang)
		{
			arfa = 1.0;
			for (j = 0; j < Point_Num; j++)
			{
				s = LineSect(Get_Knot_Value(i),Get_Knot_Value(i+1),Get_Knot_Value(i) + j * delta);
				cpnt = Curve_Point_x[(i - BSpline_k) * Point_Num + j];

				p1 = Control_Point_x[i - BSpline_k+1];
				p2 = Control_Point_x[i - BSpline_k+2];
				temp_Curve_Point_x[(i - BSpline_k) * Point_Num + j] = (1-arfa)*cpnt + (1-s)*(p1-(1-arfa)*cpntx0)+s*(p2-(1-arfa)*cpntx1);

				cpnt = Curve_Point_y[(i - BSpline_k) * Point_Num + j];
				p1 = Control_Point_y[i - BSpline_k+1];
				p2 = Control_Point_y[i - BSpline_k+2];
				temp_Curve_Point_y[(i - BSpline_k) * Point_Num + j] = (1-arfa)*cpnt + (1-s)*(p1-(1-arfa)*cpnty0)+s*(p2-(1-arfa)*cpnty1);
			}
		}
		for (j = 0; j < Point_Num; j++)
		{
			Curve_Point_x[(i - BSpline_k) * Point_Num + j] = temp_Curve_Point_x[(i - BSpline_k) * Point_Num + j];
			Curve_Point_y[(i - BSpline_k) * Point_Num + j] = temp_Curve_Point_y[(i - BSpline_k) * Point_Num + j];
		}
	}

	delete[] temp_Curve_Point_x;
	delete[] temp_Curve_Point_y;
	delete[] cpntx;
	delete[] cpnty;
}

void CCAGD_NURBS::Display_Control_Polygon(CClientDC *dc, double point_size, COLORREF point_color, COLORREF line_color)//显示控制多边形
{
	CPen NewPen;
	CPen *pOldPen;
	CBrush *pBrush = new CBrush(point_color);
	dc->SelectObject(pBrush);
	for(int i = 0; i < Control_Point_Num; i++)
	{
		dc->Ellipse(Control_Point_x[i] - point_size, Control_Point_y[i] - point_size, Control_Point_x[i] + point_size, Control_Point_y[i] + point_size);
	}
	NewPen.CreatePen(PS_DASH, 1, line_color);
	pOldPen = dc->SelectObject(&NewPen);
	dc->MoveTo(Control_Point_x[0], Control_Point_y[0]);
	for(int i = 1; i < Control_Point_Num; i++)
	{
		dc->LineTo(Control_Point_x[i], Control_Point_y[i]);
	}
	dc->SelectObject(pOldPen);
	NewPen.DeleteObject();
	delete pBrush;
}

void CCAGD_NURBS::Display_BSpline(CClientDC *dc, double line_size, COLORREF line_color)//显示B样条曲线
{
	CPen NewPen,*pOldPen;
	NewPen.CreatePen(PS_SOLID, line_size, line_color);
	pOldPen = dc->SelectObject(&NewPen);
	dc->MoveTo(Deboor_Point_x[0], Deboor_Point_y[0]);
	for(int i = 1; i < Deboor_Point_Num; i++)
	{
		
		dc->LineTo(Deboor_Point_x[i],Deboor_Point_y[i]);
	}
	dc->SelectObject(pOldPen);
	NewPen.DeleteObject();
}

double CCAGD_NURBS::BaseFunc_N(double u, int i, int k)//计算基函数，返回参数u对应的基函数值
{
	//如果是0次，对应基函数节点区间跨度为1，如果在该区间内，返回函数值1，否则返回0
	if(k == 0)
	{
		if ((u >= Get_Knot_Value(i)) && (u < Get_Knot_Value(i + 1)))
			return 1;
		else 
			return 0;
	}
	//下面用到了B样条基函数的递推公式
	double denom1, denom2;
	denom1 = Get_Knot_Value(i + k) - Get_Knot_Value(i);// U（i+k） - U(i)
	denom2 = Get_Knot_Value(i + k + 1) - Get_Knot_Value(i + 1);//U(i+k+1) - U(i+1)
	if (fabs(denom1) <= 0.00005)//防止除0错误
	{
		denom1 = 0;
	}
	else
	{
		denom1 = (u - Get_Knot_Value(i)) / denom1;// [U - U(i)] / [U（i+k） - U(i)]
	}

	if (fabs(denom2) <= 0.00005)
	{
		denom2 = 0;
	}
	else
	{
		denom2 = (Get_Knot_Value(i + k + 1) - u) / denom2;//[U(i+k+1) - U] / [U(i+k+1) - U(i+1)]
	}
	//计算返回函数值，递归算法
	if (denom1 == 0)
	{
		return denom2 * BaseFunc_N(u, i + 1, k - 1);
	}
	else if (denom2 == 0)
	{
		return denom1 * BaseFunc_N(u, i, k - 1);
	}
	else
	{
		return denom1 * BaseFunc_N(u, i, k - 1) + denom2 * BaseFunc_N(u, i + 1, k - 1);//书上的递推公式在这里体现出来
	}
}

void CCAGD_NURBS::Display_BaseFunc(CClientDC *pdc, double x, double y, double ratio)//显示基函数
{
	int All_Point;
	int i, m;
	double delta;
	double u;
	double *base_pointx;
	double *base_pointy;
	
	All_Point = Control_Point_Num * (Base_Count+1);
	base_pointx = new double[All_Point];
	base_pointy = new double[All_Point];
	
	CPen NewPen,*pOldPen;
	NewPen.CreatePen(PS_SOLID, 2, RGB(0,0,0));
	pOldPen = pdc->SelectObject(&NewPen);
	for (i = 0; i < Control_Point_Num; i++)//首先遍历所有节点区间段
	{
		delta = (Get_Knot_Value(i + BSpline_k + 1) - Get_Knot_Value(i)) / Base_Count;// [U（i+k+1）- U(i)] / Cnt 表示每个基函数对应的支撑区间上两个相邻绘制点之间的距离，也就是步长
		u = Get_Knot_Value(i);//u的初始化
		for (m = 0; m <= Base_Count; m++)//对每个节点区间段上的基函数绘制点进行遍历
		{
			base_pointx[i*(Base_Count+1)+m] = x + u * ratio;
			if ((m == Base_Count) && (i == Control_Point_Num - 1))
			{
				base_pointy[i*(Base_Count+1)+m] = y - ratio * BaseFunc_N(Get_Knot_Value(i+BSpline_k+1), i, BSpline_k) / 4;
			}
			else
			{
				base_pointy[i*(Base_Count+1)+m] = y - ratio * BaseFunc_N(u, i, BSpline_k) / 4;
			}
			if (m == 0)
			{
				pdc->MoveTo(base_pointx[i*(Base_Count+1)], base_pointy[i*(Base_Count+1)]);
			}
			else
			{
				pdc->LineTo(base_pointx[i*(Base_Count+1)+m], base_pointy[i*(Base_Count+1) + m]);
			}
			u = u + delta;

		}
	}
	pdc->SelectObject(pOldPen);
	NewPen.DeleteObject();
	/*NewPen.CreatePen(PS_SOLID,2,RGB(0,0,0));
	pOldPen = pdc->SelectObject(&NewPen);
	pdc->MoveTo(x,y);
	pdc->LineTo(x+ratio,y);
	pdc->MoveTo(x,y);
	pdc->LineTo(x,y-ratio/4);
	pdc->MoveTo(x+ratio,y);
	pdc->LineTo(x+ratio,y-ratio/4);
	pdc->SelectObject(pOldPen);
	NewPen.DeleteObject();*/

	NewPen.CreatePen(PS_DASH,1,RGB(0,0,0));
	pOldPen=pdc->SelectObject(&NewPen);
	CString str;
	for(i=1;i<Knot_Dif_Num-1;i++)
	{
		if(i==1)
		{
			str.Format(_T("%.2lf"),Knot_Dif_Array[i-1]);
			pdc->TextOut(x-15+Knot_Dif_Array[i-1]*ratio,y+3,str,str.GetLength());
		}
		if(i==Knot_Dif_Num-2)
		{
			str.Format(_T("%.2lf"),Knot_Dif_Array[i+1]);
			pdc->TextOut(x-15+Knot_Dif_Array[i+1]*ratio,y+3,str,str.GetLength());
		}
		str.Format(_T("%.2lf"),Knot_Dif_Array[i]);
		pdc->TextOut(x-15+Knot_Dif_Array[i]*ratio,y+3,str,str.GetLength());
		pdc->MoveTo(x+Knot_Dif_Array[i]*ratio,y);
		pdc->LineTo(x+Knot_Dif_Array[i]*ratio,y-ratio/4);

	}

	pdc->SelectObject(pOldPen);
	NewPen.DeleteObject();
	delete []base_pointx;
	delete []base_pointy;

}





