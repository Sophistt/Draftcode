#pragma once

class CStopLineAlg
{
public:
	CStopLineAlg(void);
public:
	~CStopLineAlg(void);

public://Õ£÷πœﬂæ‡¿Î±Í∂®
	double m_stopdistance;
	CvRect m_stoprect;
	int GetStopCalib(IplImage *src, double m_distance, CvRect m_rect, float line[4]);

public://’“Õ£÷πœﬂ
	int FindStopLine(IplImage *src, double m_distance, CvRect m_rect, float line[4]);
	int FindSideLine(IplImage *src, CvRect m_rect, double &cx, double &cy);
};
