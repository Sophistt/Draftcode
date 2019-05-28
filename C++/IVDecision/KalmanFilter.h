#pragma once
#include "Matrix.h"


class KalmanFilter
{
private:
	double m_x; 
	double m_y;
	double m_x_;
	double m_y_;
	double m_vdir;
	double m_v;
	
	

public:
	KalmanFilter(void);
	
	MatrixP<double> X;     //�洢�˶�״̬

	MatrixP<double> Z;     //�洢�۲�ֵ

	MatrixP<double> A;     //�洢ת������
	
	MatrixP<double> Y;     //�洢�۲�ֵ
	
	MatrixP<double> H;     //�洢�۲����
	
	MatrixP<double> Q;     //�����Ŷ�
	
	MatrixP<double> R;     //�۲��Ŷ�
	
	MatrixP<double> P;     //�����¶�ƫ���Э����
	
	MatrixP<double> K;;    //kalman����

	int m_count;
	int keep_count;
	double m_vdirchange;
	double v_change;

	int Filter(double x,double y);
	int Filter(double x,double y,double x_,double y_);
	double GetX();
	double GetY();
	double GetX_();
	double GetY_();
	double GetV();
	void GetVdir();
	double ReturVdir();
	double ReVdirChange();
	KalmanFilter &operator=(const KalmanFilter &Tkalman)
	{
		X = Tkalman.X;
		Z = Tkalman.Z;
		A = Tkalman.A;
		Y = Tkalman.Y;
		H = Tkalman.H;
		Q = Tkalman.Q;
		R = Tkalman.R;
		P = Tkalman.P;
		K = Tkalman.K;
		m_count = Tkalman.m_count;
		m_x = Tkalman.m_x;
		m_y = Tkalman.m_y;
		m_x_ = Tkalman.m_x_;
		m_y_ = Tkalman.m_y_;
		m_vdir=Tkalman.m_vdir;
		keep_count=Tkalman.keep_count;
		m_vdirchange=Tkalman.m_vdirchange;
		v_change=Tkalman.v_change;
		m_v=Tkalman.m_v;

		return *this;
	}
public:
	~KalmanFilter(void);
};