#pragma once
#include <math.h>
#include <iomanip>
#include <string>
#include <string.h>
#include <Eigen/Dense> 
using namespace Eigen; 
using Eigen::MatrixXd; 
using namespace std;
template <class T>
class MatrixP
{
private:
	int row,column;

	

public:
	MatrixP(void);
	//MatrixP(int, int, T **);//一般矩阵
	MatrixP(int, int, T);   //全m矩阵
	MatrixP(const MatrixP<T> &Mat)     //拷贝构造函数
	{
		row = Mat.row;
		column = Mat.column;
		int i,j;
		M = new T *[Mat.row];
		for (i = 0; i<Mat.row; i++)
		{
			M[i] = new T[Mat.column]; 
		}
		for (i = 0; i<Mat.row; i++)
		{
			for (j = 0; j<Mat.column; j++)
			{
				M[i][j] = Mat.M[i][j];
			}
		}
	}
	MatrixP<T> t()         //转置
	{
		double KKKK = 0;
		MatrixP<T> Mnew(column,row,KKKK);
		int i,j;
		for (i = 0; i<column; i++)
		{
			for (int j = 0; j<row; j++)
			{
				Mnew.M[i][j] = M[j][i];
			}
		}

		return Mnew;
	}
	MatrixP<T> inv()
	{
		ASSERT(row == column);
		MatrixP<T> Mnew = *this;
		int n = row;

		MatrixXd tem_M = MatrixXd::Random(n,n); 
		for (int i = 0 ; i<n;i++)
		{
			for (int j = 0; j<n; j++)
			{
				tem_M(i,j) = Mnew.M[i][j];
			}
		}
		MatrixXd s=tem_M.inverse(); 

		for (int i = 0 ; i<n;i++)
		{
			for (int j = 0; j<n; j++)
			{
				Mnew.M[i][j]=s(i,j) ;
			}
		}

		return Mnew;
	}

	T **M;



	/************************************************************************/
	/* 符号重载                                                                     */
	/************************************************************************/
	template <class T> friend MatrixP<T> operator+(const MatrixP<T>&,const MatrixP<T>&);
	template <class T> friend MatrixP<T> operator-(const MatrixP<T>&,const MatrixP<T>&);
	template <class T> friend MatrixP<T> operator*(const MatrixP<T>&,const MatrixP<T>&);
	template <class T> friend MatrixP<T> operator*(const T &,const MatrixP<T>&);
	template <class T> friend MatrixP<T> operator*(const MatrixP<T>&,const T &);
	template <class T> friend MatrixP<T> operator/(const MatrixP<T>&,const MatrixP<T>&);
	template <class T> friend MatrixP<T> operator/(const MatrixP<T>&,const T &);
    MatrixP<T> operator=(const MatrixP<T>&);

public:
	~MatrixP(void);
};

template <class T>
MatrixP<T>::MatrixP(void)
{
	row = 1;
	column = 1;
	int i,j;
	M = new T*[row];
	for (i = 0; i<row; i++)
	{
		M[i] = new T [column];
	}
	for (i = 0; i<row; i++)
	{
		for (j = 0; j<column; j++)
		{
			M[i][j] = T(0.0);
		}
	}
}


template <class T>
MatrixP<T>::MatrixP(int r, int c, T m)
{
	row = r; 
	column = c;
	int i,j;
	M = new T *[row];
	for (i = 0; i<row; i++)
	{
		M[i] = new T [column];
	}
	for (i = 0; i<row; i++)
	{
		for (j = 0; j<column; j++)
		{
			M[i][j] = m;
		}
	}
}



//template<class T>
//MatrixP<T>::MatrixP(int r, int c, T **m)
//{
//	row = r;
//	column = c;
//	int i,j;
//	M = new T *[row];
//	for (i = 0; i<row; i++)
//	{
//		M[i] = new T[column];
//	}
//	for (i = 0; i<row; i++)
//	{
//		for (j = 0; j<column; j++)
//		{
//			M[i][j] = m[i][j];
//		}
//	}
//
//}

//template<class T>
//MatrixP<T>::MatrixP(const MatrixP<T> &Mat)
//
//
// template <class T>
// MatrixP<T> MatrixP<T>::t()
//
//
//template <class T>
//MatrixP<T> MatrixP<T>::inv()


template <class T>
MatrixP<T> operator + (const MatrixP<T>& Mat1,const MatrixP<T>& Mat2)
{
	if (Mat1.row!=Mat2.row||Mat1.column!=Mat2.column)
	{
		AfxMessageBox("两个矩阵阶数不等!");
		exit(1);
	}

	else
	{
		int kkkk = 0;
		MatrixP<T> Mnew(Mat1.row,Mat1.column,kkkk);
		int i,j;
		for (i = 0; i<Mat1.row; i++)
		{
			for (j = 0; j<Mat1.column; j++)
			{
				Mnew.M[i][j] = Mat1.M[i][j]+Mat2.M[i][j];
			}
		}

		return Mnew;
	}
}

template <class T>
MatrixP<T> operator - (const MatrixP<T>& Mat1,const MatrixP<T>& Mat2)
{
	if (Mat1.row!=Mat2.row||Mat1.column!=Mat2.column)
	{
		AfxMessageBox("两个矩阵阶数不等!");
		exit(1);
	}

	else
	{
		MatrixP<T> Mnew(Mat1.row,Mat1.column,'0');
		int i,j;
		for (i = 0; i<Mat1.row; i++)
		{
			for (j = 0; j<Mat1.column; j++)
			{
				Mnew.M[i][j] = Mat1.M[i][j]-Mat2.M[i][j];
			}
		}

		return Mnew;
	}
}

template <class T>
MatrixP<T> operator * (const MatrixP<T>& Mat,const T &x)
{
	MatrixP<T>Mnew(Mat.row,Mat.column,'0');
	int i,j;
	for (i = 0; i<Mat.row; i++)
	{
		for (j = 0; j<Mat.column; j++)
		{
			Mnew.M[i][j] = Mat.M[i][j]*x;
		}
	}
	return Mnew;
}

template<class T>
MatrixP<T> operator * (const T &x,const MatrixP<T>& Mat)
{
	MatrixP<T>Mnew(Mat.row,Mat.column,'0');
	int i,j;
	for (i = 0; i<Mat.row; i++)
	{
		for (j = 0; j<Mat.column; j++)
		{
			Mnew.M[i][j] = Mat.M[i][j]*x;
		}
	}
	return Mnew;
}


template <class T>
MatrixP<T> operator * (const MatrixP<T>& Mat1,const MatrixP<T>& Mat2)
{
	ASSERT(Mat1.column==Mat2.row);
	if (Mat1.column!=Mat2.row)
	{
		AfxMessageBox("矩阵行列不匹配无法相乘!");
		exit(1);
	}
	else
	{
		MatrixP<T>Mnew(Mat1.row,Mat2.column,'0');
		int i,j,k;
		T d;
		for (i = 0; i<Mat1.row; i++)
		{
			for (j = 0; j<Mat2.column; j++)
			{
				d = T(0.0);
				for (k = 0; k<Mat1.column; k++)
				{
					d+=Mat1.M[i][k]*Mat2.M[k][j];
				}
				Mnew.M[i][j] = d;
			}
		}
		return Mnew;
	}
}


template <class T>
MatrixP<T> operator / (const MatrixP<T>& Mat1,const MatrixP<T>& Mat2)
{
	MatrixP<T> Mat = Mat2;
	return Mat1*(Mat.inv());
}

template <class T>
MatrixP<T> operator / (const MatrixP<T>& Mat,const T &x)
{

	return Mat*(T(1.0)/x);
}


template<class T>
MatrixP<T>  MatrixP<T>::operator=(const MatrixP<T>&Mat)
{
	row = Mat.row;
	column = Mat.column;
	int i,j;
	M=new T *[row];
	for (i = 0; i<row; i++)
	{
		M[i]=new T [column];
	}
	for (i = 0; i<row; i++)
	{
		for (j = 0; j<column; j++)
		{
			M[i][j] = Mat.M[i][j];
		}
	}

	return *this;


}



template <class T>
MatrixP<T>::~MatrixP(void)
{
	int i;
	for (i = 0; i<row; i++)
	{
		delete []M[i];
	}
	delete []M;
}
