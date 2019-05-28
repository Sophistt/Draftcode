#pragma once


//����B������������
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
	int		Control_Point_Num;		//���ƶ�����
	double	*Control_Point_x;		//���ƶ���x����
	double	*Control_Point_y;		//���ƶ���y����
	int		Knot_Dif_Num;			//����ڵ���
	double	*Knot_Dif_Array;		//����ڵ�����
	int		*Knot_Mul_Array;		//�ڵ��ظ�������
	int		BSpline_k;				//B��������
	int		BSpline_Type;			//B��������
	bool	Knot_Mark;				//�ڵ�ʸ����־
	int		Deboor_Point_Num;		//Deboor��ĸ���
	double	*Deboor_Point_x;
	double	*Deboor_Point_y;
	int Base_Count;					//֧���������

	//CCAGD_NURBS�ĸ��ַ���
public:
	void Set_Control_Point_Num(int n);				//������ƶ�����
	int Get_Control_Point_Num(void);				//������ƶ�����
	void Set_k(int k);								//�������ߵĴ���
	int Get_k(void);								//������ߵĴ���
	void Set_Type(int Type);						//�������ߵ�����
	int Get_Type(void);								//������ߵ�����
	void Set_Knot_Dif_Num(void);					//ȷ��ĳ����������������ڵ����
	void Set_Knot(void);							//ȷ��ĳ�����������µĽڵ�ʸ��
	double Get_Knot_Value(int j);					//�õ��ڵ�ʸ�����±�ֵΪj�Ľڵ�ֵ
	void Get_Control_Point(int num,double *x, double *y);	//������ߵĿ��Ƶ�
	void DeBoor(double u,double &point_x, double &point_y);			//deboor�㷨
	void Get_Draw_Point(int Point_Num);				//�õ������ϵ�Deboor��
	void Display_Control_Polygon(CClientDC *dc, double point_size, COLORREF point_color, COLORREF line_color);//��ʾ���ƶ����
	void Display_BSpline(CClientDC *dc, double line_size, COLORREF line_color);//��ʾB��������
	double BaseFunc_N(double u, int i, int k);		//�õ���������N_i_k(u)��ֵ
	void Display_BaseFunc(CClientDC *pdc, double x, double y, double ratio);//��ʾ������
	void Get_Insert_Point(double shape_arfa,int Point_Num,int All_Point,double *Deboor_Point_x,double *Deboor_Point_y);
	double LineSect(double u0,double u1,double t);
};
