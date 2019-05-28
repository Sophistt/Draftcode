#pragma once


//定义B样条曲线类型
const int JYB = 0;
const int ZJYB = 1;
const int Bezier = 2;
const int FJYB = 3;

class CCAGD_NURBS
{
public:
	CCAGD_NURBS(void);
	~CCAGD_NURBS(void);

public:
	int		Control_Point_Num;		//控制定点数
	double	*Control_Point_x;		//控制顶点x坐标
	double	*Control_Point_y;		//控制顶点y坐标
	int		Knot_Dif_Num;			//相异节点数
	double	*Knot_Dif_Array;		//相异节点数组
	int		*Knot_Mul_Array;		//节点重复度数组
	int		BSpline_k;				//B样条次数
	int		BSpline_Type;			//B样条类型
	bool	Knot_Mark;				//节点矢量标志
	int		Deboor_Point_Num;		//Deboor点的个数
	double	*Deboor_Point_x;
	double	*Deboor_Point_y;
	int Base_Count;					//支撑区间点数

	//CCAGD_NURBS的各种方法
public:
	void Set_Control_Point_Num(int n);				//输入控制顶点数
	int Get_Control_Point_Num(void);				//输出控制顶点数
	void Set_k(int k);								//输入曲线的次数
	int Get_k(void);								//输出曲线的次数
	void Set_Type(int Type);						//输入曲线的类型
	int Get_Type(void);								//输出曲线的类型
	void Set_Knot_Dif_Num(void);					//确定某种曲线类型下相异节点个数
	void Set_Knot(void);							//确定某种曲线类型下的节点矢量
	double Get_Knot_Value(int j);					//得到节点矢量中下标值为j的节点值
	void Get_Control_Point(int num,double *x, double *y);	//获得曲线的控制点
	void DeBoor(double u,double &point_x, double &point_y);			//deboor算法
	void Get_Draw_Point(int Point_Num);				//得到曲线上的Deboor点
	void Display_Control_Polygon(CClientDC *dc, double point_size, COLORREF point_color, COLORREF line_color);//显示控制多边形
	void Display_BSpline(CClientDC *dc, double line_size, COLORREF line_color);//显示B样条曲线
	double BaseFunc_N(double u, int i, int k);		//得到基函数的N_i_k(u)的值
	void Display_BaseFunc(CClientDC *pdc, double x, double y, double ratio);//显示基函数
	void Get_Insert_Point(double shape_arfa,int Point_Num,int All_Point,double *Deboor_Point_x,double *Deboor_Point_y);
	double LineSect(double u0,double u1,double t);
};
