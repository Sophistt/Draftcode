#pragma once

#include "LoadRNDF.h"

//#include "hfv.h"
//#include "CHfvTypes.h"
////#include "hfxcore.h"
//#include "hfvgui.h"
//
////#include "HfvImage.h"
//
////#include "HfvRegionArray.h"
//
//#include "hfvlib.h"

// CVisionCK ����Ŀ��

class CVisionCK : public CObject
{
public:
	CVisionCK();
	virtual ~CVisionCK();
public:
	IplImage *pMainImage;//��ͼ��
	CvRect m_ckROI;//����ⷶΧ

public:
	void zLoadParam();
	void zSaveParam();
	void zLocationTest(IplImage *src, IplImage *dst);
	void zLocationCheck();

protected:

};
//ͼ����ת
void zRotateImage(IplImage *src, IplImage *dst, float angle);

void zCreatePyrRotTemplate( IplImage* src, IplImage* dst, double low_thresh, double high_thresh, int aperture_size, 
						   CvSeq* ptemplate, double start_angle, double end_angle, double angle_step, int n_pyr);

void zCalImagePrySimilar( IplImage *src, CvSeq* pedge, double simithreshold, double &fsimilar, CvPoint2D32f &pos, double &fangle );

//GPS��·���ļ�����
double distance(double wd1, double jd1, double wd2, double jd2);// ���ݾ�γ���������ʵ�ʾ���

//����·���ļ���ͼ����
void CvtRNDF2ImgMap( CLoadRNDF RNDF, IplImage *dst, double &centerx, double &centery, int b_GPS, int b_charpoint);

void CvtGPATH2ImgMap( CLoadRNDF RNDF, IplImage *dst,  struct MAPPOINT *path, double &centerx, double &centery, int b_GPS, int b_charpoint);

//·�ζ�λ�����ݵ�ǰGPS��ȷ����ǰ·�Σ��Լ�����·���ļ���GPS·��.·�α�Ŵ�0��ʼ
int GetSegmentNumberByGPS( CLoadRNDF RNDF, double wd, double jd, int &n_seg );

//��õ�ǰִ��·��
CString  ReturnPath();

void GetXYFromGPS(double wd1, double jd1, double wd2, double jd2, double &X, double &Y, double &angle, double &dist);
double lineSpace(double x1, double y1, double x2, double y2) ;
int GetMapSize(CLoadRNDF RNDF);
int GetMapSize(CLoadRNDF RNDF,double &centerlat, double &centerlng);