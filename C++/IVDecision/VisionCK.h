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

// CVisionCK 命令目标

class CVisionCK : public CObject
{
public:
	CVisionCK();
	virtual ~CVisionCK();
public:
	IplImage *pMainImage;//主图像
	CvRect m_ckROI;//主检测范围

public:
	void zLoadParam();
	void zSaveParam();
	void zLocationTest(IplImage *src, IplImage *dst);
	void zLocationCheck();

protected:

};
//图像旋转
void zRotateImage(IplImage *src, IplImage *dst, float angle);

void zCreatePyrRotTemplate( IplImage* src, IplImage* dst, double low_thresh, double high_thresh, int aperture_size, 
						   CvSeq* ptemplate, double start_angle, double end_angle, double angle_step, int n_pyr);

void zCalImagePrySimilar( IplImage *src, CvSeq* pedge, double simithreshold, double &fsimilar, CvPoint2D32f &pos, double &fangle );

//GPS及路网文件处理
double distance(double wd1, double jd1, double wd2, double jd2);// 根据经纬度坐标计算实际距离

//加载路网文件到图像中
void CvtRNDF2ImgMap( CLoadRNDF RNDF, IplImage *dst, double &centerx, double &centery, int b_GPS, int b_charpoint);

void CvtGPATH2ImgMap( CLoadRNDF RNDF, IplImage *dst,  struct MAPPOINT *path, double &centerx, double &centery, int b_GPS, int b_charpoint);

//路段定位。根据当前GPS点确定当前路段，以及方向。路网文件是GPS路网.路段编号从0开始
int GetSegmentNumberByGPS( CLoadRNDF RNDF, double wd, double jd, int &n_seg );

//获得当前执行路径
CString  ReturnPath();

void GetXYFromGPS(double wd1, double jd1, double wd2, double jd2, double &X, double &Y, double &angle, double &dist);
double lineSpace(double x1, double y1, double x2, double y2) ;
int GetMapSize(CLoadRNDF RNDF);
int GetMapSize(CLoadRNDF RNDF,double &centerlat, double &centerlng);