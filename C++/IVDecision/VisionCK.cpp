// VisionCK.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "VisionCK.h"


// CVisionCK

CVisionCK::CVisionCK()
{
}

CVisionCK::~CVisionCK()
{
}


// CVisionCK ��Ա����


void CVisionCK::zLocationTest(IplImage *src, IplImage *dst)
{
	if(!src) return;
	cvCanny(src, dst, 30, 120);
}

//.......................................��״ƥ��...................................//

/**************************************************************
�������ƶ�
���룺
pedge  ��Ե��,ģ���б�Ե������, CvPoint2D32f
pVectorDst  ƥ��Ŀ��ͼ���е�������CvPoint2D32f
size  ͼ���С
rect  ͼ���е�����
threshold ���ƶ���ֵ,>threshold
�����
fsimilar  ���ƶ�
**************************************************************/
void zCalShapeSimilar( CvSeq* pedge, CvSeq *pVectorDst, CvSize size, CvRect rect, double threshold, double &fsimilar )
{
	TRACE0("Start zCalShapeSimilar\n");
	int num = pedge->total/2;
	double n_total = 0;

	for (int i=0; i<num; i++)
	{
		CvPoint2D32f pt = *CV_GET_SEQ_ELEM( CvPoint2D32f, pedge, i*2 );
		int id = int(size.width*(rect.y+pt.y) + rect.x + pt.x);

		CvPoint2D32f D = *CV_GET_SEQ_ELEM( CvPoint2D32f, pedge, i*2+1 );
		CvPoint2D32f E = *CV_GET_SEQ_ELEM( CvPoint2D32f, pVectorDst, id );

		double a = D.x*E.x + D.y*E.y;
		double b = cvSqrt(D.x*D.x + D.y*D.y)*cvSqrt(E.x*E.x + E.y*E.y);

		if ( b == 0 ) continue;
		n_total += a/b;

		double pre = fabs(n_total/num)+(double)(num-i)/num;
		if (pre < threshold) break;
	}
	fsimilar = fabs(n_total/num);

	TRACE0("End zCalShapeSimilar\n");
}

/**************************************************************
ͼ����ÿ��ķ�����Ϣ
���룺
srcImage  ԭʼͼ�񣨻Ҷ�ͼ��
aperture_size  �ݶ������С��3
�����
pVector  ������Ϣ���У���float x, float y ��
***************************************************************/
void zGetGradVector( IplImage *srcImage, int aperture_size, CvSeq *pVector )
{
	TRACE0("Start zGetGradVector\n");
	int i,j;
	CvMat *dx = 0, *dy = 0;
	CvMat srcstub, *src = (CvMat*)srcImage;
	src = cvGetMat( src, &srcstub );

	CvSize size = cvSize( src->width, src->height );

	dx = cvCreateMat( size.height, size.width, CV_16SC1 );
	dy = cvCreateMat( size.height, size.width, CV_16SC1 );
	cvSobel( src, dx, 1, 0, aperture_size );
	cvSobel( src, dy, 0, 1, aperture_size );

	for( i = 0; i < size.height; i++ )
	{
		const short* _dx = (short*)(dx->data.ptr + dx->step*i);
		const short* _dy = (short*)(dy->data.ptr + dy->step*i);
		for( j = 0; j < size.width; j++ )
		{
			CvPoint2D32f pv= cvPoint2D32f(double(_dx[j]), double(_dy[j]));
			cvSeqPush( pVector, &pv );
		}
	}

	cvReleaseMat( &dx );
	cvReleaseMat( &dy );
	TRACE0("End zGetGradVector\n");
}


//ģ��
void zGetEdgePointAndVector( const void* srcarr, void* dstarr,
							double low_thresh, double high_thresh, int aperture_size, CvSeq* pedge, void* maskarr )
{

	TRACE0("Start zGetEdgePointAndVector\n");
	static const int sec_tab[] = { 1, 3, 0, 0, 2, 2, 2, 2 };
	CvMat *dx = 0, *dy = 0;
	void *buffer = 0;
	uchar **stack_top, **stack_bottom = 0;

	CvMat *matVx = 0, *matVy = 0;

	CV_FUNCNAME( "cvCanny" );

	__BEGIN__;

	//if(maskarr)
	//{
	CvMat maskstub, *mask = (CvMat*)maskarr;
	//}
	CvMat srcstub, *src = (CvMat*)srcarr;
	CvMat dststub, *dst = (CvMat*)dstarr;
	CvSize size;
	int flags = aperture_size;
	int low, high;
	int* mag_buf[3];
	uchar* map;
	int mapstep, maxsize;
	int i, j;
	CvMat mag_row;

	if(maskarr)
	{
		CV_CALL( mask = cvGetMat( mask, &maskstub ));
	}
	CV_CALL( src = cvGetMat( src, &srcstub ));
	CV_CALL( dst = cvGetMat( dst, &dststub ));

	if( CV_MAT_TYPE( src->type ) != CV_8UC1 ||
		CV_MAT_TYPE( dst->type ) != CV_8UC1 )
		CV_ERROR( CV_StsUnsupportedFormat, "" );

	if( !CV_ARE_SIZES_EQ( src, dst ))
		CV_ERROR( CV_StsUnmatchedSizes, "" );

	if( low_thresh > high_thresh )
	{
		double t;
		CV_SWAP( low_thresh, high_thresh, t );
	}

	aperture_size &= INT_MAX;
	if( (aperture_size & 1) == 0 || aperture_size < 3 || aperture_size > 7 )
		CV_ERROR( CV_StsBadFlag, "" );

	//size = cvGetMatSize( src );
	size = cvSize( src->width, src->height );

	dx = cvCreateMat( size.height, size.width, CV_16SC1 );
	dy = cvCreateMat( size.height, size.width, CV_16SC1 );
	cvSobel( src, dx, 1, 0, aperture_size );
	cvSobel( src, dy, 0, 1, aperture_size );

	matVx = cvCloneMat( dx );
	matVy = cvCloneMat( dy );

	if( flags & CV_CANNY_L2_GRADIENT )
	{
		Cv32suf ul, uh;
		ul.f = (float)low_thresh;
		uh.f = (float)high_thresh;

		low = ul.i;
		high = uh.i;
	}
	else
	{
		low = cvFloor( low_thresh );
		high = cvFloor( high_thresh );
	}

	CV_CALL( buffer = cvAlloc( (size.width+2)*(size.height+2) +
		(size.width+2)*3*sizeof(int)) );

	mag_buf[0] = (int*)buffer;
	mag_buf[1] = mag_buf[0] + size.width + 2;
	mag_buf[2] = mag_buf[1] + size.width + 2;
	map = (uchar*)(mag_buf[2] + size.width + 2);
	mapstep = size.width + 2;

	maxsize = MAX( 1 << 10, size.width*size.height/10 );
	CV_CALL( stack_top = stack_bottom = (uchar**)cvAlloc( maxsize*sizeof(stack_top[0]) ));

	memset( mag_buf[0], 0, (size.width+2)*sizeof(int) );
	memset( map, 1, mapstep );
	memset( map + mapstep*(size.height + 1), 1, mapstep );

	/* sector numbers 
	(Top-Left Origin)

	1   2   3
	*  *  * 
	* * *  
	0*******0
	* * *  
	*  *  * 
	3   2   1
	*/

#define CANNY_PUSH(d)    *(d) = (uchar)2, *stack_top++ = (d)
#define CANNY_POP(d)     (d) = *--stack_top

	mag_row = cvMat( 1, size.width, CV_32F );

	// calculate magnitude and angle of gradient, perform non-maxima supression.
	// fill the map with one of the following values:
	//   0 - the pixel might belong to an edge
	//   1 - the pixel can not belong to an edge
	//   2 - the pixel does belong to an edge
	for( i = 0; i <= size.height; i++ )
	{
		int* _mag = mag_buf[(i > 0) + 1] + 1;
		float* _magf = (float*)_mag;
		const short* _dx = (short*)(dx->data.ptr + dx->step*i);
		const short* _dy = (short*)(dy->data.ptr + dy->step*i);
		uchar* _map;
		int x, y;
		int magstep1, magstep2;
		int prev_flag = 0;

		if( i < size.height )
		{
			_mag[-1] = _mag[size.width] = 0;

			if( !(flags & CV_CANNY_L2_GRADIENT) )
				for( j = 0; j < size.width; j++ )
					_mag[j] = abs(_dx[j]) + abs(_dy[j]);
			else
			{
				for( j = 0; j < size.width; j++ )
				{
					x = _dx[j]; y = _dy[j];
					_magf[j] = (float)sqrt((double)x*x + (double)y*y);
				}
			}
		}
		else
			memset( _mag-1, 0, (size.width + 2)*sizeof(int) );

		// at the very beginning we do not have a complete ring
		// buffer of 3 magnitude rows for non-maxima suppression
		if( i == 0 )
			continue;

		_map = map + mapstep*i + 1;
		_map[-1] = _map[size.width] = 1;

		_mag = mag_buf[1] + 1; // take the central row
		_dx = (short*)(dx->data.ptr + dx->step*(i-1));
		_dy = (short*)(dy->data.ptr + dy->step*(i-1));

		magstep1 = (int)(mag_buf[2] - mag_buf[1]);
		magstep2 = (int)(mag_buf[0] - mag_buf[1]);

		if( (stack_top - stack_bottom) + size.width > maxsize )
		{
			uchar** new_stack_bottom;
			maxsize = MAX( maxsize * 3/2, maxsize + size.width );
			CV_CALL( new_stack_bottom = (uchar**)cvAlloc( maxsize * sizeof(stack_top[0])) );
			memcpy( new_stack_bottom, stack_bottom, (stack_top - stack_bottom)*sizeof(stack_top[0]) );
			stack_top = new_stack_bottom + (stack_top - stack_bottom);
			cvFree( &stack_bottom );
			stack_bottom = new_stack_bottom;
		}

		for( j = 0; j < size.width; j++ )
		{
#define CANNY_SHIFT 15
#define TG22  (int)(0.4142135623730950488016887242097*(1<<CANNY_SHIFT) + 0.5)

			x = _dx[j];
			y = _dy[j];
			int s = x ^ y;
			int m = _mag[j];

			x = abs(x);
			y = abs(y);
			if( m > low )
			{
				int tg22x = x * TG22;
				int tg67x = tg22x + ((x + x) << CANNY_SHIFT);

				y <<= CANNY_SHIFT;

				if( y < tg22x )
				{
					if( m > _mag[j-1] && m >= _mag[j+1] )
					{
						if( m > high && !prev_flag && _map[j-mapstep] != 2 )
						{
							CANNY_PUSH( _map + j );
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
						continue;
					}
				}
				else if( y > tg67x )
				{
					if( m > _mag[j+magstep2] && m >= _mag[j+magstep1] )
					{
						if( m > high && !prev_flag && _map[j-mapstep] != 2 )
						{
							CANNY_PUSH( _map + j );
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
						continue;
					}
				}
				else
				{
					s = s < 0 ? -1 : 1;
					if( m > _mag[j+magstep2-s] && m > _mag[j+magstep1+s] )
					{
						if( m > high && !prev_flag && _map[j-mapstep] != 2 )
						{
							CANNY_PUSH( _map + j );
							prev_flag = 1;
						}
						else
							_map[j] = (uchar)0;
						continue;
					}
				}
			}
			prev_flag = 0;
			_map[j] = (uchar)1;
		}

		// scroll the ring buffer
		_mag = mag_buf[0];
		mag_buf[0] = mag_buf[1];
		mag_buf[1] = mag_buf[2];
		mag_buf[2] = _mag;
	}

	// now track the edges (hysteresis thresholding)
	while( stack_top > stack_bottom )
	{
		uchar* m;
		if( (stack_top - stack_bottom) + 8 > maxsize )
		{
			uchar** new_stack_bottom;
			maxsize = MAX( maxsize * 3/2, maxsize + 8 );
			CV_CALL( new_stack_bottom = (uchar**)cvAlloc( maxsize * sizeof(stack_top[0])) );
			memcpy( new_stack_bottom, stack_bottom, (stack_top - stack_bottom)*sizeof(stack_top[0]) );
			stack_top = new_stack_bottom + (stack_top - stack_bottom);
			cvFree( &stack_bottom );
			stack_bottom = new_stack_bottom;
		}

		CANNY_POP(m);

		if( !m[-1] )
			CANNY_PUSH( m - 1 );
		if( !m[1] )
			CANNY_PUSH( m + 1 );
		if( !m[-mapstep-1] )
			CANNY_PUSH( m - mapstep - 1 );
		if( !m[-mapstep] )
			CANNY_PUSH( m - mapstep );
		if( !m[-mapstep+1] )
			CANNY_PUSH( m - mapstep + 1 );
		if( !m[mapstep-1] )
			CANNY_PUSH( m + mapstep - 1 );
		if( !m[mapstep] )
			CANNY_PUSH( m + mapstep );
		if( !m[mapstep+1] )
			CANNY_PUSH( m + mapstep + 1 );
	}

	// the final pass, form the final image
	for( i = 0; i < size.height; i++ )
	{
		uchar* _mask = NULL;
		if(maskarr)
		{
			_mask = mask->data.ptr + mask->step*i;
		}
		const uchar* _map = map + mapstep*(i+1) + 1;
		uchar* _dst = dst->data.ptr + dst->step*i;

		short* _matVx = (short*)(matVx->data.ptr + matVx->step*i);
		short* _matVy = (short*)(matVy->data.ptr + matVy->step*i);

		for( j = 0; j < size.width; j++ ){
			_dst[j] = (uchar)-(_map[j] >> 1);

			if ( _dst[j] > 0 )
			{
				if(maskarr)
				{
					if(_mask[j] > 0)
					{
						CvPoint2D32f pt = cvPoint2D32f( double(j), double(i) );
						cvSeqPush( pedge, &pt );

						double vx = _matVx[j];
						double vy = _matVy[j];
						CvPoint2D32f pv = cvPoint2D32f( vx, vy );
						cvSeqPush( pedge, &pv );
					}
					else _dst[j] = 0;
				}
				else
				{
					CvPoint2D32f pt = cvPoint2D32f( double(j), double(i) );
					cvSeqPush( pedge, &pt );

					double vx = _matVx[j];
					double vy = _matVy[j];
					CvPoint2D32f pv = cvPoint2D32f( vx, vy );
					cvSeqPush( pedge, &pv );
				}
			}
		}
	}

	__END__;

	cvReleaseMat( &dx );
	cvReleaseMat( &dy );
	cvReleaseMat( &matVx );
	cvReleaseMat( &matVy );
	cvFree( &buffer );
	cvFree( &stack_bottom );

	TRACE0("End zGetEdgePointAndVector\n");
}

/**************************************************************
��ģ����н���������ÿ���Ƕ����ƶȣ�����������ƶȺͽǶ�
**************************************************************/
void zCalRotShapeSimilar( CvSeq* pTemplate, CvSeq *pVectorDst, CvSize size, CvRect rect, double threshold, double &fsimilar, double &fangle )
{
	TRACE0("Start zCalRotShapeSimilar\n");

	CvPoint2D32f pt = *CV_GET_SEQ_ELEM( CvPoint2D32f, pTemplate, 0 );
	double start_angle = pt.x;
	double angle_step = pt.y;
	CvPoint2D32f pt1 = *CV_GET_SEQ_ELEM( CvPoint2D32f, pTemplate, 1 );
	int n_angle = int(pt1.x);

	if(angle_step <= 0) return;

	int points_id = 0;
	double maxsimilar = 0.0;
	for(int i=0; i<=n_angle; i++)
	{
		CvPoint2D32f pt2 = *CV_GET_SEQ_ELEM( CvPoint2D32f, pTemplate, 2+points_id );
		int n_points = int(pt2.x);

		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq *pedges = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );

		pedges = cvSeqSlice(pTemplate, cvSlice(3+points_id, 3+points_id+n_points), storage, 1);
		double similar = 0.0;
		zCalShapeSimilar( pedges, pVectorDst, size, rect, threshold, similar );
		if(similar > maxsimilar)
		{
			maxsimilar = similar;
			fangle = start_angle+angle_step*i;
		}

		cvClearSeq(pedges);
		cvReleaseMemStorage(&storage);

		points_id += n_points+1;
	}
	fsimilar = maxsimilar;

	TRACE0("End zCalRotShapeSimilar\n");
}


/**************************************************************
��ģ����н���������ÿ���Ƕ����ƶȣ�����������ƶȺͽǶ�,�нǶȷ�Χ
**************************************************************/
void zCalRangRotShapeSimilar( CvSeq* pTemplate, CvSeq *pVectorDst, CvSize size, CvRect rect, double threshold, double pre_angle, double &fsimilar, double &fangle )
{
	TRACE0("Start zCalRangRotShapeSimilar\n"); 

	CvPoint2D32f pt = *CV_GET_SEQ_ELEM( CvPoint2D32f, pTemplate, 0 );
	double start_angle0 = pt.x;
	double angle_step0 = pt.y;
	CvPoint2D32f pt1 = *CV_GET_SEQ_ELEM( CvPoint2D32f, pTemplate, 1 );
	int n_angle = int(pt1.x);

	double start_angle = pre_angle - angle_step0*2;
	double end_angle = pre_angle + angle_step0*2;
	if(angle_step0 <= 0) return;
	if(start_angle<start_angle0) start_angle = start_angle0;
	if(end_angle>(start_angle0+angle_step0*n_angle)) end_angle = start_angle0+angle_step0*n_angle;

	int points_id = 0;
	double maxsimilar = 0.0;

	int start_num = (int)((start_angle-start_angle0)/angle_step0);

	for(int i=0; i<start_num; i++)
	{
		CvPoint2D32f pt2 = *CV_GET_SEQ_ELEM( CvPoint2D32f, pTemplate, 2+points_id );
		int n_points = int(pt2.x);

		points_id += n_points+1;
	}

	int end_num = (int)((end_angle-start_angle0-start_num*angle_step0)/angle_step0)+1;

	for(int i=0; i<end_num; i++)
	{
		CvPoint2D32f pt2 = *CV_GET_SEQ_ELEM( CvPoint2D32f, pTemplate, 2+points_id );
		int n_points = int(pt2.x);

		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq *pedges = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );

		pedges = cvSeqSlice(pTemplate, cvSlice(3+points_id, 3+points_id+n_points), storage, 1);
		double similar = 0.0;
		zCalShapeSimilar( pedges, pVectorDst, size, rect, threshold, similar );
		if(similar > maxsimilar)
		{
			maxsimilar = similar;
			fangle = start_angle0+start_num*angle_step0+angle_step0*i;
		}

		cvClearSeq(pedges);
		cvReleaseMemStorage(&storage);

		points_id += n_points+1;
	}
	fsimilar = maxsimilar;

	TRACE0("End zCalRangRotShapeSimilar\n"); 
}



//ͼ����ת
void zRotateImage(IplImage *src, IplImage *dst, float angle)
{
	float m[6];
	// Matrix m looks like:
	//
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]
	// [ m3  m4  m5 ]       [ A21  A22   b2 ]
	//
	CvMat M = cvMat (2, 3, CV_32F, m);
	int w = src->width;
	int h = src->height;
	//if (opt)		// ��ת������
	//	factor = (cos (angle * CV_PI / 180.) + 1.0) * 2;
	//else			//  ������ת
	float factor = 1.0f;
	m[0] = (float) (factor * cos (-angle */* 2 * */CV_PI / 180.));
	m[1] = (float) (factor * sin (-angle */* 2 * */CV_PI / 180.));
	m[3] = -m[1];
	m[4] = m[0];
	// ����ת��������ͼ���м�
	m[2] = w * 0.5f;
	m[5] = h * 0.5f;
	//  dst(x,y) = A * src(x,y) + b

	IplImage *tmp = cvCloneImage(src);
	cvZero (dst);
	cvGetQuadrangleSubPix (tmp, dst, &M);
	cvReleaseImage(&tmp);
}

//ͼ����ת�߽�
void zGetRotateEdge( IplImage *dst, float angle)
{
	float m[6];
	// Matrix m looks like:
	//
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]
	// [ m3  m4  m5 ]       [ A21  A22   b2 ]
	//
	CvMat M = cvMat (2, 3, CV_32F, m);
	int w = dst->width;
	int h = dst->height;
	//if (opt)		// ��ת������
	//	factor = (cos (angle * CV_PI / 180.) + 1.0) * 2;
	//else			//  ������ת
	float factor = 1.0f;
	m[0] = (float) (factor * cos (angle * 2 * CV_PI / 180.));
	m[1] = (float) (factor * sin (angle * 2 * CV_PI / 180.));
	m[3] = -m[1];
	m[4] = m[0];
	// ����ת��������ͼ���м�
	m[2] = w * 0.5f;
	m[5] = h * 0.5f;
	//  dst(x,y) = A * src(x,y) + b

	float x[4], y[4];
	x[0]=-(float)w/2.0f;y[0]=-(float)h/2.0f;
	x[1]=(float)w/2.0f;y[1]=-(float)h/2.0f;
	x[2]=(float)w/2.0f;y[2]=(float)h/2.0f;
	x[3]=-(float)w/2.0f;y[3]=(float)h/2.0f;

	float xd[4], yd[4];
	for(int i=0; i<4; i++)
	{
		xd[i]=m[0]*x[i]+m[1]*y[i]+m[2];
		yd[i]=m[3]*x[i]+m[4]*y[i]+m[5];
	}

	cvZero (dst);

	cvLine(dst, cvPoint(cvRound(xd[0]),cvRound(yd[0])), cvPoint(cvRound(xd[1]),cvRound(yd[1])), cvScalar(255));
	cvLine(dst, cvPoint(cvRound(xd[1]),cvRound(yd[1])), cvPoint(cvRound(xd[2]),cvRound(yd[2])), cvScalar(255));
	cvLine(dst, cvPoint(cvRound(xd[2]),cvRound(yd[2])), cvPoint(cvRound(xd[3]),cvRound(yd[3])), cvScalar(255));
	cvLine(dst, cvPoint(cvRound(xd[3]),cvRound(yd[3])), cvPoint(cvRound(xd[0]),cvRound(yd[0])), cvScalar(255));

	cvFloodFill( dst, cvPoint(w/2,h/2), cvScalar(255));
}

/*****************************************************
//����ģ�壬��ת

******************************************************/
void zCreateRotTemplate( IplImage* src, IplImage* dst, double low_thresh, double high_thresh, int aperture_size, 
						CvSeq* ptemplate, double start_angle, double end_angle, double angle_step, int n_pyr)
{
	TRACE0("Start zCreateRotTemplate\n"); 

	if(!src) return;

	//����Ƕȷֶȵĸ���
	if(start_angle>end_angle)
	{
		double tang = start_angle;
		start_angle = end_angle;
		end_angle = tang;
	}

	int n_angle = 0;
	if (angle_step != 0){
		n_angle = cvRound((end_angle-start_angle)/angle_step);
	}

	//�洢 (��ʼ�Ƕȣ��ǶȲ���)���洢��ptemplate�ĵ�һλ
	CvPoint2D32f store_angle = cvPoint2D32f(start_angle,angle_step);
	cvSeqPush(ptemplate, &store_angle);
	//�洢�Ƕ�ģ��ĸ��� (�Ƕ�ģ�����������������)
	CvPoint2D32f num_angle = cvPoint2D32f(double(n_angle),double(n_pyr));
	cvSeqPush(ptemplate, &num_angle);

	for(int i=0; i<=n_angle; i++)
	{
		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq *ptmpseq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );

		IplImage *srctmp = cvCloneImage(src);

		double tmp_angle = start_angle+angle_step*i;

		zRotateImage(srctmp, srctmp, (float)tmp_angle);

		IplImage *mask = cvCreateImage(cvGetSize(srctmp), 8, 1);
		zGetRotateEdge( mask, (float)tmp_angle);

		IplImage *dsttmp = cvCreateImage(cvGetSize(srctmp), 8, 1);
		zGetEdgePointAndVector( srctmp, dsttmp, low_thresh, high_thresh, aperture_size, ptmpseq, mask );

		if(i == n_angle/2)
		{
			cvCopy(dsttmp, dst);
		}

		//�洢���� (������0)
		CvPoint2D32f num_points = cvPoint2D32f(double(ptmpseq->total),0.0);
		cvSeqPush(ptemplate, &num_points);
		//�洢ģ������

		for(int k=0; k<ptmpseq->total; k++)
		{
			CvPoint2D32f pp = *CV_GET_SEQ_ELEM( CvPoint2D32f, ptmpseq, k );
			cvSeqPush(ptemplate, &pp);
		}

		cvReleaseImage(&dsttmp);
		cvReleaseImage(&mask);
		cvReleaseImage(&srctmp);

		cvClearSeq(ptmpseq);
		cvReleaseMemStorage(&storage);
	}

	TRACE0("End zCreateRotTemplate\n"); 
}


/*****************************************************
//����ģ�壬����������ת
���룺
src  ԭʼͼ��
dst  ��Եͼ����ʾ�ã�
low_thresh  ��Ե��ֵ
high_thresh  ��Ե��ֵ
aperture_size  ���ڴ�С
start_angle ��ʼ�Ƕ�
end_angle ��ֹ�Ƕ�
angle_step �ǶȲ���
n_pyr ������������0��ԭʼͼ��1��3 �������
�����
ptemplate  ģ��
******************************************************/
void zCreatePyrRotTemplate( IplImage* src, IplImage* dst, double low_thresh, double high_thresh, int aperture_size, 
						   CvSeq* ptemplate, double start_angle, double end_angle, double angle_step, int n_pyr)
{
	TRACE0("Start zCreatePyrRotTemplate\n"); 
	if(!src) return;

	//������������,0--ԭʼ
	if(n_pyr > 3) return;
	if(n_pyr < 0) return;

	int ipry;

	IplImage *srcPryDownImage[4];
	for(ipry=0; ipry<4; ipry++)
	{
		srcPryDownImage[ipry] = cvCreateImage(cvSize(src->width/int(pow(2.0f,ipry)),src->height/int(pow(2.0f,ipry))), 8, 1);
	}

	cvCopy(src, srcPryDownImage[0]);
	for(ipry=1; ipry<=n_pyr; ipry++)
	{
		cvPyrDown( srcPryDownImage[ipry-1], srcPryDownImage[ipry] );
	}

	double start_angle_pry=start_angle;
	double end_angle_pry=end_angle;
	double angle_step_pry=angle_step;

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq *ptmpseq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );

	//�洢 (ģ���ģ���)���洢��ptemplate�ĵ�0λ
	CvPoint2D32f store_size = cvPoint2D32f((double)src->width,double(src->height));
	cvSeqPush(ptemplate, &store_size);
	//�洢 (�������ܲ���0��3����ǰ��0��3)���洢��ptemplate�ĵ�һλ
	CvPoint2D32f store_pry = cvPoint2D32f(n_pyr,n_pyr);
	cvSeqPush(ptemplate, &store_pry);

	for(ipry=n_pyr; ipry>=0; ipry--)
	{
		//ÿ���Ĳ�������
		angle_step_pry = angle_step*pow(2.0f, ipry);

		IplImage *top1img = cvCreateImage(cvGetSize(srcPryDownImage[ipry]),8,1);
		zCreateRotTemplate( srcPryDownImage[ipry], top1img, low_thresh, high_thresh, aperture_size, ptmpseq, start_angle_pry, end_angle_pry, angle_step_pry, ipry);

		if(ipry == 0)  cvCopy(top1img, dst);
		cvReleaseImage(&top1img);
		//�洢 (������������ǰ��0��3)���洢��ptemplate�ĵڶ�λ
		CvPoint2D32f store_info = cvPoint2D32f(ptmpseq->total,ipry);
		cvSeqPush(ptemplate, &store_info);

		//�洢ģ������

		for(int k=0; k<ptmpseq->total; k++)
		{
			CvPoint2D32f pp = *CV_GET_SEQ_ELEM( CvPoint2D32f, ptmpseq, k );
			cvSeqPush(ptemplate, &pp);
		}

		cvClearSeq(ptmpseq);
	}

	cvReleaseMemStorage(&storage);

	for(ipry=0; ipry<4; ipry++)
	{
		cvReleaseImage(&srcPryDownImage[ipry]);
	}

	TRACE0("End zCreatePyrRotTemplate\n"); 
}

/**************************************************************
�ý�������������һ����ͼ���е����ƶ���Ƕ�
���룺
src  ԭʼͼ��
pedge  ģ���Ե��0, ģ���б�Ե������1   CvPoint2D32f
simithreshold ���ƶ���ֵ�����ƶ�>simithreshold��С�������ֵ����ֹ����
�����
fsimilar  ���ƶ�
pos λ��
fangle ��ת�Ƕ�
**************************************************************/
void zCalImagePrySimilar( IplImage *src, CvSeq* pedge, double simithreshold, double &fsimilar, CvPoint2D32f &pos, double &fangle )
{
	TRACE0("Start zCalImagePrySimilar\n"); 

	//��ģ����н���
	CvPoint2D32f store_size = *CV_GET_SEQ_ELEM( CvPoint2D32f, pedge, 0 );//ģ��ߴ�
	CvSize tempsize = cvSize((int)store_size.x,(int)store_size.y);
	CvSize imgSize = cvGetSize(src);
	if ((imgSize.width<tempsize.width) || (imgSize.height<tempsize.height)) return;

	//��ģ����н���
	CvPoint2D32f store_pry = *CV_GET_SEQ_ELEM( CvPoint2D32f, pedge, 1 );//����������
	int n_pry = (int)store_pry.x;//����

	CvPoint2D32f store_top1 = *CV_GET_SEQ_ELEM( CvPoint2D32f, pedge, 2 );//��߲�
	int num_seq = (int)store_top1.x;//������
	int cur_pry = (int)store_top1.y;//��ǰ���������(3,2,1,0)

	///////����ÿ��ͼ��
	int ipry;

	IplImage *srcPryDownImage[4];
	for(ipry=0; ipry<4; ipry++)
	{
		srcPryDownImage[ipry] = cvCreateImage(cvSize(src->width/int(pow(2.0f,ipry)),src->height/int(pow(2.0f,ipry))), 8, 1);
	}

	cvCopy(src, srcPryDownImage[0]);
	for(ipry=1; ipry<=n_pry; ipry++)
	{
		cvPyrDown( srcPryDownImage[ipry-1], srcPryDownImage[ipry] );
	}

	//��߲㿪ʼ����
	int i,j;

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq *pVector = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
	zGetGradVector( srcPryDownImage[n_pry], 3, pVector );

	//��ʱģ��
	CvSeq *pTmpTeplate = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
	pTmpTeplate = cvSeqSlice(pedge, cvSlice(3, 3+num_seq), storage, 1);

	//������Χ
	CvSize pyrimgSize = cvSize(imgSize.width/int(pow(2.0f,n_pry)),imgSize.height/int(pow(2.0f,n_pry)));
	CvSize pyrtempSize = cvSize(tempsize.width/int(pow(2.0f,n_pry)),tempsize.height/int(pow(2.0f,n_pry)));
	int w=pyrimgSize.width-pyrtempSize.width;
	int h=pyrimgSize.height-pyrtempSize.height;

	double maxsimilar = 0.0;
	double maxangle = 0.0;
	for (i=0; i<w; i++)
	{
		for (j=0; j<h; j++)
		{
			CvRect tempRect = cvRect(i,j,pyrtempSize.width,pyrtempSize.height);

			double similar = 0.0;
			double angle = 0.0;
			//zCalShapeSimilar( pedge, pVector, imgSize, tempRect, simithreshold, similar );
			//����ÿ���Ƕȵ����ƶ�
			zCalRotShapeSimilar( pTmpTeplate, pVector, pyrimgSize, tempRect, simithreshold, similar, angle );
			//zCalRangRotShapeSimilar( pTmpTeplate, pVector, pyrimgSize, tempRect, simithreshold, 0, similar, angle );
			//.......
			//((float*)(similarmap->imageData + similarmap->widthStep*j))[i] = (float)similar;
			//.......

			if (maxsimilar < similar)
			{
				maxsimilar = similar;
				maxangle = angle;
				pos.x = (float)i + pyrtempSize.width * 0.5f;//�Ƶ�ͼ������
				pos.y = (float)j + pyrtempSize.height * 0.5f;
			}
		}
	}
	cvClearSeq(pVector);
	cvClearSeq(pTmpTeplate);

	for(ipry=n_pry-1; ipry>=0; ipry--)
	{
		int pre_total = num_seq;
		double pre_angle = maxangle;
		store_top1 = *CV_GET_SEQ_ELEM( CvPoint2D32f, pedge, 2+pre_total+1 );//��߲�
		num_seq = (int)store_top1.x;//������

		//��ʱģ��
		pTmpTeplate = cvSeqSlice(pedge, cvSlice(3+pre_total+1, 3+pre_total+1+num_seq), storage, 1);

		num_seq += pre_total+1;

		//�������
		//�Ƕ�maxsimilar
		//��Χ,��ʼ��
		float sx = (pos.x - pyrtempSize.width * 0.5f - 1)*2;
		float sy = (pos.y - pyrtempSize.height * 0.5f - 1)*2;

		//������Χ
		//pyrimgSize = cvSize(imgSize.width/int(pow(2.0f,ipry)),imgSize.height/int(pow(2.0f,ipry)));
		pyrtempSize = cvSize(tempsize.width/int(pow(2.0f,ipry)),tempsize.height/int(pow(2.0f,ipry)));
		pyrimgSize = cvSize(pyrtempSize.width+6,pyrtempSize.height+6);

		if (sx < 0) sx=0;
		if (sy < 0) sy=0;
		if (sx+pyrimgSize.width > srcPryDownImage[ipry]->width)  pyrimgSize.width = (int)(srcPryDownImage[ipry]->width-sx);
		if (sy+pyrimgSize.height > srcPryDownImage[ipry]->height)  pyrimgSize.height = (int)(srcPryDownImage[ipry]->height-sy);

		//w=16;
		//h=16;
		w=pyrimgSize.width-pyrtempSize.width;
		h=pyrimgSize.height-pyrtempSize.height;

		CvRect pyrrect = cvRect((int)sx, (int)sy, pyrimgSize.width, pyrimgSize.height);
		//CvRect pyrrect = cvRect(0, 0, pyrimgSize.width, pyrimgSize.height);
		IplImage* spyrimg = cvCreateImage(pyrimgSize, 8, 1);
		cvSetImageROI( srcPryDownImage[ipry], pyrrect );
		cvCopy(srcPryDownImage[ipry], spyrimg);
		cvResetImageROI(srcPryDownImage[ipry]);
		zGetGradVector( spyrimg, 3, pVector );
		cvReleaseImage(&spyrimg);

		maxsimilar = 0.0;
		maxangle = 0.0;
		for (i=0; i<w; i++)
		{
			for (j=0; j<h; j++)
			{
				CvRect tempRect = cvRect(i,j,pyrtempSize.width,pyrtempSize.height);

				double similar = 0.0;
				double angle = 0.0;
				//zCalShapeSimilar( pedge, pVector, imgSize, tempRect, simithreshold, similar );
				//����ÿ���Ƕȵ����ƶ�
				zCalRangRotShapeSimilar( pTmpTeplate, pVector, pyrimgSize, tempRect, simithreshold, pre_angle, similar, angle );
				//((float*)(similarmap->imageData + similarmap->widthStep*j))[i] = (float)similar;

				if (maxsimilar < similar)
				{
					maxsimilar = similar;
					maxangle = angle;
					pos.x = (float)i + pyrtempSize.width * 0.5f +sx;//�Ƶ�ͼ������
					pos.y = (float)j + pyrtempSize.height * 0.5f +sy;
				}
			}
		}
		cvClearSeq(pVector);
		cvClearSeq(pTmpTeplate);

	}

	for(ipry=0; ipry<4; ipry++)
	{
		cvReleaseImage(&srcPryDownImage[ipry]);
	}

	cvReleaseMemStorage(&storage);

	fsimilar = maxsimilar;
	fangle = maxangle;

	TRACE0("End zCalImagePrySimilar\n"); 
}

// ���ݾ�γ���������ʵ�ʾ���
/*double hypot_gps(double x, double y) {
	return sqrt(x * x + y * y);
}

double distance(double wd1, double jd1, double wd2, double jd2) {// ���ݾ�γ���������ʵ�ʾ���
	double x, y, out;
	double PI = 3.1415926535898;
	double R = 6.371229 * 1e6;
	x = (jd2 - jd1) * PI * R * cos( ( (wd1 + wd2) / 2) * PI / 180) / 180;
	y = (wd2 - wd1) * PI * R / 180;
	out = hypot_gps(x, y);
	return out;
}*/


/*********************************************************************************************
���ݾ�γ���������ʵ��������롢�Ƕȡ�����
���룺
	wd1,jd1:��һ��γ�ȡ�����
	wd2,jd2:�ڶ���γ�ȡ�����
�����
	X,Y:Y����Ϊ����X����Ϊ����������ֵ
	angle������������ļнǣ�˳ʱ����
	dist������ľ���
********************************************************************************************/
void GetXYFromGPS(double wd1, double jd1, double wd2, double jd2, double &X, double &Y, double &angle, double &dist)
{
	double PI = 3.1415926535898;
	dist = distance( wd1, jd1, wd2, jd2);

	//�뱱�н�
	double radLat1 = (wd1 * PI / 180.0);
	double radLat2 = (wd2 * PI / 180.0);
	double a = radLat2 - radLat1;
	double b = (jd2 * PI / 180.0) - (jd1 * PI / 180.0);
	double t=atan(b/a);
	if (a<0)
	{
		if(b > 0)
			t=t+PI;
		else t=t-PI;
	}
	//t=PI/2-t;

	angle = t * 180 / PI;

	X = dist * sin(t);
	Y = -dist * cos(t);

}

/*********************************************************************************************
���ݾ�γ���������ʵ���������
���룺
    wd1,jd1:��һ��γ�ȡ�����
	wd2,jd2:�ڶ���γ�ȡ�����
�����
    �����������
********************************************************************************************/
double distance(double wd1, double jd1, double wd2, double jd2) {// ���ݾ�γ���������ʵ�ʾ���

	double PI = 3.1415926535898;
	double radLat1 = (wd1 * PI / 180.0);
	double radLat2 = (wd2 * PI / 180.0);
	double a = radLat1 - radLat2;
	double b = (jd1 * PI / 180.0) - (jd2 * PI / 180.0);
	double s = 2 * asin(sqrt(pow(sin(a/2),2) + 
		cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
	s = s * 6378.137*1000;
	//s = round(s * 10000) / 10000;
	return s;

	//main code inside the class
/*	double dlat1=jd1*(PI/180);

	double dlong1=wd1*(PI/180);
	double dlat2=jd2*(PI/180);
	double dlong2=wd2*(PI/180);

	double dLong=dlong1-dlong2;
	double dLat=dlat1-dlat2;

	double aHarv= pow(sin(dLat/2.0),2.0)+cos(dlat1)*cos(dlat2)*pow(sin(dLong/2),2);
	double cHarv=2*atan2(sqrt(aHarv),sqrt(1.0-aHarv));
	//earth's radius from wikipedia varies between 6,356.750 km = 6,378.135 km (3,949.901  3,963.189 miles)
	//The IUGG value for the equatorial radius of the Earth is 6378.137 km (3963.19 mile)
	const double earth=6378.137*1000;//I am doing miles, just change this to radius in kilometers to get distances in km
	double distance=earth*cHarv;

	return distance;*/
}

/*************************************************************************************************
/////////////////////////////////////////////////////////////////////////////////////////////
//����·���ļ���ͼ����
//���ɻҶ�ͼ����·���ļ�ƥ����
���룺
    RNDF:����·���ļ�
	b_GPS:��GPS���ݣ�1�����ǵѿ����������ݣ�0��
	b_charpoint:��������1
�����
    dst:·��ͼ�񣬴�С4000*4000
	centerx:���ĵ�����x��GPS��ѿ������꣩
	centery:���ĵ�����y��GPS��ѿ������꣩
	**********************************************************************************************/
void CvtRNDF2ImgMap( CLoadRNDF RNDF, IplImage *dst, double &centerx, double &centery, int b_GPS, int b_charpoint )
{
	//4000�ס�4000�׵�ͼ��ÿ�����ش���1�ף�/*����Ϊ��γ�ȵ�С�����5λ*/
	//�����ͼ�ߴ�
	int DIM_M = GetMapSize(RNDF,centerx,centery)+200;
	
	//int DIM_M = 4000;//��ͼ�ߴ� DIM_M*DIM_M
	//cvSet(dst, CV_RGB(255,255,255));
	cvZero(dst);
	int nsegment = RNDF.m_mapInfo.segment_num;

	//��һ�εĵ�һ����Ϊ��ʼ�㣬��ͼ�����ģ�(2000,2000)
	//centerx = 0;
	//centery = 0;

	for(int i=0; i<nsegment; i++)//ÿ��
	{
		int nlane = RNDF.m_mapInfo.pSegment[i].lane_num;
		for(int j=0; j<nlane; j++)//ÿ��·
		{
			int nwaypoint = RNDF.m_mapInfo.pSegment[i].pLane[j].waypoints_num;//·��
			int ncharpoint = RNDF.m_mapInfo.pSegment[i].pLane[j].charpoints_num;//������
			int nexit = RNDF.m_mapInfo.pSegment[i].pLane[j].exit_num;//���ڵ�,����
			int exitid = 0;
//			

			for (int k1=0; k1<nwaypoint; k1++)
			{
				double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].x;
				double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].y;

				if(i==0 && j==0 && k1==0)//��һ����Ϊ���ĵ�
				{
					//centerx = wx;
					//centery = wy;

					int px = /*cvRound((wx-centerx)*200000)+*/DIM_M/2;
					int py = /*cvRound((wy-centery)*200000)+*/DIM_M/2;

					cvCircle(dst, cvPoint(px,py), 1, cvScalarAll(255), CV_FILLED);

					//continue;
				}

				//����ĵ������ĵ��������
				int px ;//= distance(centerx, centery, centerx, wy)+5000;
				int py ;//= distance(centerx, centery, wx, centery)+5000;
				double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
				double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
				double angle;
				double dist;

				if(b_GPS)
				{
					GetXYFromGPS( centerx, centery, wx, wy, pxx, pyy, angle, dist);
					px = (int)(pxx)+DIM_M/2;
					py = (int)(pyy)+DIM_M/2;
				}
				else
				{
					px = (int)wx - (int)centerx + DIM_M/2;
					py = (int)wy - (int)centery + DIM_M/2;
				}
				cvCircle(dst, cvPoint(px,py), 1, cvScalarAll(255), CV_FILLED);

				//connect����
				int n_con = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].con_num;
				for(int k2=0; k2<n_con; k2++)
				{
					int m = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].connect[k2].m-1;
					int n = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].connect[k2].n-1;
					int p = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].connect[k2].p-1;

					double wx0 = RNDF.m_mapInfo.pSegment[m].pLane[n].pPoint[p].x;
					double wy0 = RNDF.m_mapInfo.pSegment[m].pLane[n].pPoint[p].y;

					int px0, py0;
					double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
					double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
					double angle;
					double dist;

					if(b_GPS)
					{
						GetXYFromGPS( centerx, centery, wx0, wy0, pxx, pyy, angle, dist);
						px0 = (int)(pxx)+DIM_M/2;
						py0 = (int)(pyy)+DIM_M/2;
					}
					else
					{
						px0 = (int)wx0 - (int)centerx + DIM_M/2;
						py0 = (int)wy0 - (int)centery + DIM_M/2;
					}

					cvLine(dst, cvPoint(px,py), cvPoint(px0,py0), CV_RGB(0,0,255), 2);

				}
				//ͬһ·��������
				if(k1 > 0)
				{
					double wx0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1-1].x;
					double wy0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1-1].y;

					int px0, py0;
					double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
					double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
					double angle;
					double dist;

					if(b_GPS)
					{
						GetXYFromGPS( centerx, centery, wx0, wy0, pxx, pyy, angle, dist);
						px0 = (int)(pxx)+DIM_M/2;
						py0 = (int)(pyy)+DIM_M/2;
					}
					else
					{
						px0 = (int)wx0 - (int)centerx + DIM_M/2;
						py0 = (int)wy0 - (int)centery + DIM_M/2;
					}
					cvLine(dst, cvPoint(px,py), cvPoint(px0,py0), CV_RGB(0,0,255), 2);
				}

				//����
				CString str;
				str.Format("%d.%d.%d", i+1, j+1, k1+1);

				CvFont font;

				double hscale = 0.7;
				double vscale = 0.7;
				int linewidth = 2;
				cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);
				if(i == 3)
				{
					if(j == 0)
						cvPutText(dst, str, cvPoint(px,py+20+j*30), &font,CV_RGB(30, 200, 100));
					if(j == 1)
						cvPutText(dst, str, cvPoint(px,py-30+j*30), &font,CV_RGB(30, 200, 100));
				}
				
				else
				cvPutText(dst, str, cvPoint(px,py-15+j*30), &font,CV_RGB(30, 200, 100));
			}

			if(b_charpoint)
			{
				for (int k2=0; k2<ncharpoint; k2++)
				{
					double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pCharpoint[k2].x;
					double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pCharpoint[k2].y;

					int px, py;
					double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
					double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
					double angle;
					double dist;

					if(b_GPS)
					{
						GetXYFromGPS( centerx, centery, wx, wy, pxx, pyy, angle, dist);
						px = (int)(pxx)+DIM_M/2;
						py = (int)(pyy)+DIM_M/2;
					}
					else
					{
						px = (int)wx - (int)centerx + DIM_M/2;
						py = (int)wy - (int)centery + DIM_M/2;
					}
					cvCircle(dst, cvPoint(px,py), 5, CV_RGB(255,0,0), CV_FILLED);
				}

			}

			double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
			double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
			double angle;
			double dist;
			//����



			for (int k3=0; k3<nexit; k3++)
			{

			int m = RNDF.m_mapInfo.pSegment[i].pLane[j].pExit[k3].m;
			int n = RNDF.m_mapInfo.pSegment[i].pLane[j].pExit[k3].n;
			int p = RNDF.m_mapInfo.pSegment[i].pLane[j].pExit[k3].p;

			if(m<1 || n<1 || p<1) continue;
			double wx0 = RNDF.m_mapInfo.pSegment[m-1].pLane[n-1].pPoint[p-1].x;
			double wy0 = RNDF.m_mapInfo.pSegment[m-1].pLane[n-1].pPoint[p-1].y;
			GetXYFromGPS( centerx, centery, wx0, wy0, pxx, pyy, angle, dist);
			int px0 = (int)(pxx)+DIM_M/2;
			int py0 = (int)(pyy)+DIM_M/2;

			exitid = RNDF.m_mapInfo.pSegment[i].pLane[j].pExit[k3].exit_Id;
			double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[exitid-1].x;
			double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[exitid-1].y;
			GetXYFromGPS( centerx, centery, wx, wy, pxx, pyy, angle, dist);
			int px = (int)(pxx)+DIM_M/2;
			int py = (int)(pyy)+DIM_M/2;

			cvLine(dst, cvPoint(px,py), cvPoint(px0,py0), CV_RGB(0,0,255), 2);
			}
		}
	}

}

void CvtGPATH2ImgMap( CLoadRNDF RNDF, IplImage *dst,  struct MAPPOINT *path, double &centerx, double &centery, int b_GPS, int b_charpoint )
{
	//4000�ס�4000�׵�ͼ��ÿ�����ش���1�ף�/*����Ϊ��γ�ȵ�С�����5λ*/
	//�����ͼ�ߴ�
	int DIM_M = GetMapSize(RNDF,centerx,centery)+200;

	//cvZero(dst);
	double x1 = 0;
	double y1 = 0;

	double x2 = 0;
	double y2 = 0;

	for(;path!=NULL;)
	{
		x1 = path->next->x;
		y1 = path->next->y;
		path = path->next;
		if (path->next == NULL)
		{
			break;
		}
		x2 = path->next->x;
		y2 = path->next->y;
		
		int px1,px2 ;//= distance(centerx, centery, centerx, wy)+5000;
		int py1,py2 ;//= distance(centerx, centery, wx, centery)+5000;
		double pxx;// = (int)distance(centerx, centery, centerx, py)+DIM_M/2;
		double pyy;// = (int)distance(centerx, centery, px, centery)+DIM_M/2;
		double angle;
		double dist;

		GetXYFromGPS( centerx, centery, x1, y1, pxx, pyy, angle, dist);
		px1 = (int)(pxx)+DIM_M/2;
		py1 = (int)(pyy)+DIM_M/2;
	
		GetXYFromGPS( centerx, centery, x2, y2, pxx, pyy, angle, dist);
		px2 = (int)(pxx)+DIM_M/2;
		py2 = (int)(pyy)+DIM_M/2;

		cvLine(dst, cvPoint(px1,py1), cvPoint(px2,py2), CV_RGB(255,0,0), 7);
	}



			/*	CString str;
				str.Format("%d.%d.%d", i+1, j+1, k1+1);

				CvFont font;

				double hscale = 0.7;
				double vscale = 0.7;
				int linewidth = 2;
				cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);
				cvPutText(dst, str, cvPoint(px,py-15+j*30), &font,CV_RGB(30, 200, 100));*/
		
}


/*********************************************************************************
//·�ζ�λ�����ݵ�ǰGPS��ȷ����ǰ·�Σ��Լ�����·���ļ���GPS·��.·�α�Ŵ�0��ʼ
���룺
    RNDF:·���ļ�
	wd:��ǰγ��
	jd:��ǰ����
�����
    n_seg:����·�α�ţ���0��ʼ
**********************************************************************************/
int GetSegmentNumberByGPS( CLoadRNDF RNDF, double wd, double jd, int &n_seg )
{
	int nsegment = RNDF.m_mapInfo.segment_num;

	double *dist_segment;
	dist_segment = (double *)calloc(nsegment, sizeof(double));

	for(int i=0; i<nsegment; i++)//ÿ��
	{
		int nlane = RNDF.m_mapInfo.pSegment[i].lane_num;

		//ÿ��·��ȡ���ֵ
		//�����ʽ��area=������s(s-a)(s-b)(s-c).����s=(a+b+c)/2
		//���߳��ȣ�area/c/2;
		double min_dist = 999999;
		for(int j=0; j<nlane; j++)//ÿ��·
		{
			int nwaypoint = RNDF.m_mapInfo.pSegment[i].pLane[j].waypoints_num;//·��
			int ncharpoint = RNDF.m_mapInfo.pSegment[i].pLane[j].charpoints_num;//������
			int nexit = RNDF.m_mapInfo.pSegment[i].pLane[j].exit_num;//���ڵ�,����
			int exitid = 0;
//			exitid = RNDF.m_mapInfo.pSegment[i].pLane[j].exit_id;

			//ÿ��·�ϵ������������ߣ�����ǰ��ľ���
			//�������߱߳�
			for (int k1=0; k1<nwaypoint-1; k1++)
			{
				double wx0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].x;
				double wy0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].y;

				double wx1 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1+1].x;
				double wy1 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1+1].y;

				//����㵽·�ξ���
				//�������߱߳�
				double a = distance(wd, jd, wx0, wy0);
				double b = distance(wd, jd, wx1, wy1);
				double c = distance(wx0, wy0, wx1, wy1);
				a = fabs(a);
				b = fabs(b);
				c = fabs(c);

				double s = (a + b + c)/2;
				double area = sqrt(s*(s-a)*(s-b)*(s-c));
				double dd = area/(c*2);

				if(min_dist > dd)
				{
					min_dist = dd;
				}

			}

		}
		dist_segment[i] = min_dist;
	}

	int min_n_seg=0;
	double min_dist = 999999;
	for(int i=0; i<nsegment; i++)//ÿ��
	{
		if( dist_segment[i] < min_dist )
		{
			min_dist = dist_segment[i];
			min_n_seg = i;
		}
	}

	delete []dist_segment;

	n_seg = min_n_seg;

	return min_n_seg;
}

//��õ�ǰִ��·��
CString  ReturnPath() 
{
	CString    sPath;
	GetModuleFileName(NULL,sPath.GetBufferSetLength(MAX_PATH+1),MAX_PATH);
	sPath.ReleaseBuffer();
	int    nPos;
	nPos=sPath.ReverseFind('\\');
	sPath=sPath.Left(nPos);
	return    sPath;
} 

//-----------------------------------���治��----------------------------------
//GPS����ת����
//�ѿ�������ϵ
typedef struct tagCRDCARTESIAN{
	double x;
	double y;
	double z;
}CRDCARTESIAN,*PCRDCARTESIAN;
//typedef CRDCARTESIAN *PCRDCARTESIAN;

//�������ϵ
typedef struct tagCRDGEODETIC{
	double longitude; //����
	double latitude;  //γ�� 
	double height;    //��ظ�,����Ϊ0
}CRDGEODETIC;
typedef CRDGEODETIC *PCRDGEODETIC;


void GeodeticToCartesian (PCRDCARTESIAN pcc, PCRDGEODETIC pcg,
									   double dSemiMajorAxis, double dFlattening)
{
	double B;    //γ�ȶ���
	double L;    //���ȶ���
	double L0;    //���뾭�߶���
	double l;    //L-L0
	double t;    //tanB
	double m;    //ltanB
	double N;    //î��Ȧ���ʰ뾶 
	double q2;
	double x;    //��˹ƽ��������
	double y;    //��˹ƽ�������
	double s;    //�����γ��B�ľ��߻���
	double f;    //�ο����������
	double e2;    //�����һƫ����
	double a;    //�ο������峤����
	//double b;    //�ο�������̰���
	double a1;
	double a2;
	double a3;
	double a4;
	double b1;
	double b2;
	double b3;
	double b4;
	double c0;
	double c1;
	double c2;
	double c3;
	int Datum=84;    //ͶӰ��׼�����ͣ�����54��׼��Ϊ54������80��׼��Ϊ80��WGS84��׼��Ϊ84
	int prjno=0;    //ͶӰ����
	int zonewide=3;    
	double IPI=0.0174532925199433333333;    //3.1415926535898/180.0
	B=pcg->latitude ; //γ��
	L=pcg->longitude ; //����
	if (zonewide==6) 
	{
		prjno=(int)(L/zonewide)+1;
		L0=prjno*zonewide-3;
	}
	else
	{
		prjno=(int)((L-1.5)/3)+1;
		L0=prjno*3;
	}

	if(Datum==54)
	{
		a=6378245;
		f=1/298.3;
	}    
	else if(Datum==84)
	{
		a=6378137;
		f=1/298.257223563;
	}
	L0=L0*IPI;
	L=L*IPI;
	B=B*IPI;

	e2=2*f-f*f;//(a*a-b*b)/(a*a);
	l=L-L0;
	t=tan(B);
	m=l * cos(B);
	N=a/sqrt(1-e2* sin(B) * sin(B));
	q2=e2/(1-e2)* cos(B)* cos(B);
	a1=1+(double)3/4*e2+(double)45/64*e2*e2+(double)175/256*e2*e2*e2+(double)11025/16384*e2*e2*e2*e2+(double)43659/65536*e2*e2*e2*e2*e2;
	a2=(double)3/4*e2+(double)15/16*e2*e2+(double)525/512*e2*e2*e2+(double)2205/2048*e2*e2*e2*e2+(double)72765/65536*e2*e2*e2*e2*e2;
	a3=(double)15/64*e2*e2+(double)105/256*e2*e2*e2+(double)2205/4096*e2*e2*e2*e2+(double)10359/16384*e2*e2*e2*e2*e2;
	a4=(double)35/512*e2*e2*e2+(double)315/2048*e2*e2*e2*e2+(double)31185/13072*e2*e2*e2*e2*e2;
	b1=a1*a*(1-e2);
	b2=(double)-1/2*a2*a*(1-e2);
	b3=(double)1/4*a3*a*(1-e2);
	b4=(double)-1/6*a4*a*(1-e2);
	c0=b1;
	c1=2*b2+4*b3+6*b4;
	c2=-(8*b3+32*b4);
	c3=32*b4;
	s=c0*B+cos(B)*(c1*sin(B)+c2*sin(B)*sin(B)*sin(B)+c3*sin(B)*sin(B)*sin(B)*sin(B)*sin(B));
	x=s+(double)1/2*N*t*m*m+(double)1/24*(5-t*t+9*q2+4*q2*q2)*N*t*m*m*m*m+(double)1/720*(61-58*t*t+t*t*t*t)*N*t*m*m*m*m*m*m;
	y=N*m+(double)1/6*(1-t*t+q2)*N*m*m*m+(double)1/120*(5-18*t*t+t*t*t*t-14*q2-58*q2*t*t)*N*m*m*m*m*m;
	y=y+1000000*prjno+500000;
	pcc->x=x;
	pcc->y=y-38000000;
	pcc->z=0;

}
double lineSpace(double x1, double y1, double x2, double y2) 
{
	double lineLength = 0;
	lineLength = sqrt((double)(x1 - x2) * (x1 - x2) + (double)(y1 - y2) * (y1 - y2));
	return lineLength;
}
int GetMapSize(CLoadRNDF RNDF)
{
	double lat_min,lat_max,lng_min,lng_max;
	
	lat_min = 200;
	lat_max = 0;
	lng_min = 200;
	lng_max = 0;

	int nsegment = RNDF.m_mapInfo.segment_num;
	for(int i=0; i<nsegment; i++)//ÿ��
	{
		int nlane = RNDF.m_mapInfo.pSegment[i].lane_num;
		for(int j=0; j<nlane; j++)//ÿ��·
		{
			int nwaypoint = RNDF.m_mapInfo.pSegment[i].pLane[j].waypoints_num;//·��
			
			for (int k1=0; k1<nwaypoint; k1++)
			{
				double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].x;
				double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].y;
				
				if(lat_min > wx) lat_min = wx;
				if(lat_max < wx) lat_max = wx;
				if(lng_min > wy) lng_min = wy;
				if(lng_max < wy) lng_max = wy;
			}
		}
	}

	int dist1 = distance(lat_min,lng_min,lat_max,lng_min);
	int dist2 = distance(lat_min,lng_max,lat_min,lng_min);

	if(dist1>dist2)return dist1;
	return dist2;

}
int GetMapSize(CLoadRNDF RNDF,double &centerlat, double &centerlng)
{
	double lat_min,lat_max,lng_min,lng_max;
	
	lat_min = 200;
	lat_max = 0;
	lng_min = 200;
	lng_max = 0;

	int nsegment = RNDF.m_mapInfo.segment_num;
	for(int i=0; i<nsegment; i++)//ÿ��
	{
		int nlane = RNDF.m_mapInfo.pSegment[i].lane_num;
		for(int j=0; j<nlane; j++)//ÿ��·
		{
			int nwaypoint = RNDF.m_mapInfo.pSegment[i].pLane[j].waypoints_num;//·��
			
			for (int k1=0; k1<nwaypoint; k1++)
			{
				double wx = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].x;
				double wy = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].y;
				
				if(lat_min > wx) lat_min = wx;
				if(lat_max < wx) lat_max = wx;
				if(lng_min > wy) lng_min = wy;
				if(lng_max < wy) lng_max = wy;
			}
		}
	}

	int dist1 = distance(lat_min,lng_min,lat_max,lng_min);
	int dist2 = distance(lat_min,lng_max,lat_min,lng_min);

	centerlat = (lat_min + lat_max)/2;
	centerlng = (lng_min + lng_max)/2;

	if(dist1>dist2)return dist1;
	return dist2;

}