#include "StdAfx.h"
#include "KalmanFilter.h"
#define T 0.1

KalmanFilter::KalmanFilter(void)
{

	m_count = 0;
    double mmm = 0;
	MatrixP<double> m(4,4,mmm);
	A = m;
	A.M[0][0] = 1;
	A.M[0][1] = 0;
	A.M[0][2] = T;
	A.M[0][3] = 0;

	A.M[1][0] = 0;
	A.M[1][1] = 1;
	A.M[1][2] = 0;
	A.M[1][3] = T;

	A.M[2][0] = 0;
	A.M[2][1] = 0;
	A.M[2][2] = 1;
	A.M[2][3] = 0;

	A.M[3][0] = 0;
	A.M[3][1] = 0;
	A.M[3][2] = 0;
	A.M[3][3] = 1;

	MatrixP<double> n(2,4,mmm);
	H = n;
	H.M[0][0] = 1;
	H.M[0][1] = 0;
	H.M[0][2] = 0;
	H.M[0][3] = 0;

	H.M[1][0] = 0;
	H.M[1][1] = 1;
	H.M[1][2] = 0;
	H.M[1][3] = 0;

	Q=m;
	Q.M[0][0] = 0.00000000002;
	Q.M[0][1] = 0;
	Q.M[0][2] = 0;
	Q.M[0][3] = 0;

	Q.M[1][0] = 0;
	Q.M[1][1] = 0.0000000002;
	Q.M[1][2] = 0;
	Q.M[1][3] = 0;

	Q.M[2][0] = 0;
	Q.M[2][1] = 0;
	Q.M[2][2] = 0.0000000004;
	Q.M[2][3] = 0;

	Q.M[3][0] = 0;
	Q.M[3][1] = 0;
	Q.M[3][2] = 0;
	Q.M[3][3] = 0.0000000004;

	MatrixP<double> o(2,2,mmm);
	R=o;
	R.M[0][0] = 0.0000000001;
	R.M[0][1] = 0;
	R.M[1][0] = 0;
	R.M[1][1] = 0.0000000001;

	P = m;
	P.M[0][0] = 0.000000000001;
	P.M[1][1] = 0.000000000001;
	P.M[2][2] = 0.000000000001;
	P.M[3][3] = 0.000000000001;
	
    MatrixP<double> mat1(4,1,mmm);
	X = mat1;

	MatrixP<double> mat2(2,1,mmm);
	Z = mat2;
}

int KalmanFilter::Filter(double x,double y,double x_,double y_)
{
		X.M[0][0] = x;
		X.M[1][0] = y;
		X.M[2][0] = x_;
		X.M[3][0] = y_;
		m_x = x;
		m_y = y;
		m_x_ = x_;
		m_y_ = y_;
		m_count++;
		return 0;
}

int KalmanFilter::Filter(double x,double y)
{
	Z.M[0][0] = x;
	Z.M[1][0] = y;
	X = A*X;  //　　X(k|k-1)=A X(k-1|k-1)+B U(k) ……….. (1)……(预测结果)
	P = A*P*(A.t())+Q;  //　　P(k|k-1)=A P(k-1|k-1) A’+Q ……… (2) ……(预测系数)

	MatrixP<double> HT = H.t();
	K = P*(HT)/(H*P*(HT)+R); //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R) ……… (4)  ……(增益系数)
	X = X+K*(Z-H*X);//……… (3) ……(预测与测量组合结果)

	P = P-K*H*P;  //　　……(更新预测系数)
	m_x = X.M[0][0];
	m_y = X.M[1][0];
	m_x_ = X.M[2][0];
	m_y_ = X.M[3][0];
	m_count++;
	ASSERT(m_x_<0.001&&m_y_<0.001);
	GetVdir();
	return 0;
}

double KalmanFilter::GetX()
{
	return m_x;
}

double KalmanFilter::GetY()
{
	return m_y;
}

double KalmanFilter::GetX_()
{
	return m_x_*6378137*3.1415926/180;    //纬度方向速度
}

double KalmanFilter::GetY_()
{
	return m_y_*6378137*3.1415926/180;    //经度方向速度
}


KalmanFilter::~KalmanFilter(void)
{
}

double KalmanFilter::GetV()
{
	
	return m_v;

}

void KalmanFilter::GetVdir()
{
	double v=sqrt(m_x_*m_x_+m_y_*m_y_)*6378137*3.1415926/180;
	v_change=v-m_v;
	m_v=v;

	double v_dir=atan2(m_y_,m_x_)*180/3.1415926;
	if (v_dir<0)
	{
		v_dir+=360;
	}
	m_vdirchange=(m_vdir-v_dir);
	if (abs(m_vdirchange)<10)
	{
		keep_count++;
	}
	else
	{
		keep_count=0;
	}
	m_vdir=v_dir;

}

double KalmanFilter::ReturVdir()
{
	return m_vdir;
	

}

double KalmanFilter::ReVdirChange()
{
	return m_vdirchange;
}
