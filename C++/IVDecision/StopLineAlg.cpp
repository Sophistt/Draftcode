#include "StdAfx.h"
#include "StopLineAlg.h"

CStopLineAlg::CStopLineAlg(void)
{
}

CStopLineAlg::~CStopLineAlg(void)
{
}


int CStopLineAlg::GetStopCalib(IplImage *src, double m_distance, CvRect m_rect, float line[4])
{
	IplImage *tmp = cvCreateImage(cvGetSize(src), 8, 1);
	if( src->nChannels > 1)  cvCvtColor(src, tmp, CV_BGR2GRAY);
	else cvCopy(src, tmp);

	IplImage *tmproi = cvCreateImage(cvSize(m_rect.width,m_rect.height), 8, 1);

	cvSetImageROI(tmp, m_rect);
	cvCopy(tmp, tmproi);
	cvResetImageROI(tmp);

	cvSmooth(tmproi, tmproi);
	cvThreshold(tmproi, tmproi, 100, 255, CV_THRESH_OTSU);


	IplConvKernel* kernel = cvCreateStructuringElementEx( 31, 1, 15, 0, CV_SHAPE_RECT);
	cvErode(tmproi,tmproi,kernel);
	cvReleaseStructuringElement( &kernel );


	CvMemStorage* storage = 0;
	storage = cvCreateMemStorage(0);         //开辟默认大小的空间
	CvSeq* contour = 0;
	cvFindContours( tmproi, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );              // 查找外边缘

	int b_line = 0;
	for( ; contour != 0; contour = contour->h_next )
	{
		CvRect rect = cvBoundingRect(contour);
		if(rect.width > m_rect.width-10)//找到停止线
		{
			cvFitLine(contour, CV_DIST_FAIR, 0, 0.01, 0.01, line); 
			line[2]+=m_rect.x;
			line[3]+=m_rect.y;
			b_line = 1;

			m_stopdistance = m_distance;
			m_stoprect = m_rect;
			m_stoprect.y = (rect.y+rect.height)/2 - m_stoprect.height/2;

			break;
		}
	}

	cvReleaseImage(&tmp);
	cvReleaseImage(&tmproi);
	cvReleaseMemStorage(&storage);

	return b_line;
}

int CStopLineAlg::FindStopLine(IplImage *src, double m_distance, CvRect m_rect, float line[4])
{
	IplImage *tmp = cvCreateImage(cvGetSize(src), 8, 1);
	if( src->nChannels > 1)  cvCvtColor(src, tmp, CV_BGR2GRAY);
	else cvCopy(src, tmp);

	IplImage *tmproi = cvCreateImage(cvSize(m_rect.width,m_rect.height), 8, 1);

	cvSetImageROI(tmp, m_rect);
	cvCopy(tmp, tmproi);
	cvResetImageROI(tmp);

	cvSmooth(tmproi, tmproi);
	cvThreshold(tmproi, tmproi, 100, 255, CV_THRESH_OTSU);

	IplConvKernel* kernel = cvCreateStructuringElementEx( 31, 1, 15, 0, CV_SHAPE_RECT);
	cvErode(tmproi,tmproi,kernel);
	cvReleaseStructuringElement( &kernel );

	CvMemStorage* storage = 0;
	storage = cvCreateMemStorage(0);         //开辟默认大小的空间
	CvSeq* contour = 0;
	cvFindContours( tmproi, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );              // 查找外边缘

	int b_line = 0;
	for( ; contour != 0; contour = contour->h_next )
	{
		CvRect rect = cvBoundingRect(contour);
		if(rect.width > m_rect.width-10)//找到停止线
		{
			cvFitLine(contour, CV_DIST_FAIR, 0, 0.01, 0.01, line); 
			line[2]+=m_rect.x;
			line[3]+=m_rect.y;
			b_line = 1;

		}
	}

	cvReleaseImage(&tmp);
	cvReleaseImage(&tmproi);
	cvReleaseMemStorage(&storage);

	return b_line;
}

int CStopLineAlg::FindSideLine(IplImage *src, CvRect m_rect, double &cx, double &cy)
{
	IplImage *tmp = cvCreateImage(cvGetSize(src), 8, 1);
	if( src->nChannels > 1)  cvCvtColor(src, tmp, CV_BGR2GRAY);
	else cvCopy(src, tmp);

	IplImage *tmproi = cvCreateImage(cvSize(m_rect.width,m_rect.height), 8, 1);

	cvSetImageROI(tmp, m_rect);
	cvCopy(tmp, tmproi);
	cvResetImageROI(tmp);

	cvSmooth(tmproi, tmproi);
	cvThreshold(tmproi, tmproi, 100, 255, CV_THRESH_OTSU);

	//IplConvKernel* kernel = cvCreateStructuringElementEx( 7, 3, 3, 1, CV_SHAPE_ELLIPSE);
	//cvErode(tmproi,tmproi,kernel);
	//cvReleaseStructuringElement( &kernel );

	cvErode(tmproi,tmproi);

	CvMemStorage* storage = 0;
	storage = cvCreateMemStorage(0);         //开辟默认大小的空间
	CvSeq* contour = 0;
	cvFindContours( tmproi, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );              // 查找外边缘

	IplImage *sidetmp =  cvCreateImage(cvSize(m_rect.width,m_rect.height), 8, 1);
	int max_area = 0;
	for( ; contour != 0; contour = contour->h_next )
	{

		//CvRect rect = cvBoundingRect(contour);
		//if(rect.height > m_rect.height-4)//找到线
		//{
			double area = cvContourArea( contour );
			if((fabs(area) > 30) && (fabs(area) > max_area))
			{
				max_area = fabs(area);
				cvZero(sidetmp);
				cvDrawContours(sidetmp, contour, cvScalarAll(255), cvScalarAll(255), 1, CV_FILLED);

				//找中心点
				int start = -1;
				int istart = -1;
				int ilen = 0;//找最长段
				for(int i=3; i<sidetmp->width-3; i++)
				{
					uchar a = ((uchar*)(sidetmp->imageData + sidetmp->widthStep*(sidetmp->height)/2))[i];
					if(abs(a) > 0)
					{
						if(start == -1)
						{
							istart = i;
							start = 1;
						}
					}
					else
					{
						if( start != -1)
						{
							start = -1;

							if( i-istart < ilen ) continue;

							ilen = i-istart;
							istart = (i+istart)/2;
							if( i-istart < 4 ) istart = -1;
						}
					}
				}
				cx = (double)istart + m_rect.x;
				if ( istart == -1 )  cx = -1;
				cy = (double)(sidetmp->height)/2 + m_rect.y;

			}
		//}
	}

	cvReleaseImage(&sidetmp);
	cvReleaseImage(&tmp);
	cvReleaseImage(&tmproi);
	cvReleaseMemStorage(&storage);

	if(cx < 0) return 0;
	return 1;
}