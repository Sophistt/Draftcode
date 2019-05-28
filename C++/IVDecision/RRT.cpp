#include "stdafx.h"
#include "RRT.h"


RRT::RRT(void)
{
}
RRT::~RRT(void)
{
}
void RRT::find_rrt()
{
	rPath.clear();
	int cnt=0;//随机采样次数
	rPath.push_back(begin);
	CvPoint2D64f heading = Heading1(begin,theta);
	CvPoint2D64f temp = heading;
	CvPoint2D64f ending = Heading2(end,beita);

	srand((unsigned)time(NULL));//srand()函数产生一个以当前时间开始的随机种子， time的值每时每刻都不同。所以种子不同，所以，产生的随机数也不同

	while(getDis(temp,ending)>minPath)
	{
		CvPoint2D64f aPos = getNextPos(temp,ending);

		bool flag = true;
		for(size_t i=0;i<data.size();i++)
		{
			if(getDis(aPos,data[i])<0.455*minDis)//这个距离需要调试
			{
				flag = false;
				cnt++;
				break;
			}
		}
		if(cnt>100000)break;
		if(flag)
		{
			rPath.push_back(temp);
			temp = aPos;
		}
	}
	rPath.push_back(temp);

	if(cnt<=100000)
		rPath.push_back(ending);

	rPath.push_back(end);
}

CvPoint2D64f RRT::Heading1(CvPoint2D64f a, double b)
{
	CvPoint2D64f c;
	//c.x = a.x + 3.5*minDis*cos(b);
	//c.y = a.y - 3.5*minDis*sin(b);
	c.x = a.x + /*2.0**/0.5*minDis*cos(b*PI/180.0);
	c.y = a.y - /*2.0**/0.5*minDis*sin(b*PI/180.0);
	return c;
}

CvPoint2D64f RRT::Heading2(CvPoint2D64f a, double b)
{
	CvPoint2D64f c;
	c.x = a.x - /*3.0*/2.05*minDis*cos(b*PI/180.0);
	c.y = a.y + /*3.0*/2.05*minDis*sin(b*PI/180.0);

	return c;
}

double RRT::getDis(CvPoint2D64f a, CvPoint2D64f b)
{
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

CvPoint2D64f RRT::getNextPos(CvPoint2D64f a,CvPoint2D64f b)
{
	double rnd=rand()%361*1.0;//产生一个0至361-1之间随机的一个数
	double k=rand()%100;//产生0至99之间的随机数
	double xDet=cos(rnd*PI/180.0)*minPath;//随机数rand确定了搜索的方向
	double yDet=sin(rnd*PI/180.0)*minPath;
	double len=getDis(a,b);
	CvPoint2D64f temp;
	temp.x=(b.x-a.x)*minPath/len;
	temp.y=(b.y-a.y)*minPath/len;
	a.x+=(xDet*(100-k-50)+temp.x*(k+50))/100;
	a.y+=(yDet*(100-k-50)+temp.y*(k+50))/100;

	return a;
}

void RRT::prune() 
{
	rPath5.clear();
	CvPoint2D64f ending = Heading2(end,beita);
	rPath5.push_back(rPath[0]);
	//rPath5.push_back(rPath[0]);
	CvPoint2D64f current = rPath[0];
	int xiabiao = 0;
	CvPoint2D64f last = rPath5.back();
	
	while((last.x != end.x)&&(last.y != end.y))
	{
		bool flag1 = 0;
		for(size_t i=xiabiao+1;i<rPath.size();i++)
		{

			for(size_t j=0;j<data.size();j++)
			{
				if((i == rPath.size()-2)||(i == rPath.size()-1))
				{
					rPath5.push_back(rPath[i]);
					flag1 = 1;
					break;
				}
				if (check_free(current,rPath[i],data[j])==0)
				{
					rPath5.push_back(rPath[i-1]);
					current = rPath[i-1];
					xiabiao = i-1;
					flag1 = 1;
					break;
				}
			}
		}

		if (flag1)
		{
			last = rPath5.back();
			break;
		}
		if (flag1==0)
		{
			rPath5.push_back(end);
			last = rPath5.back();
			break;
		}
	}
	rPath5.pop_back();
	CvPoint2D64f temp,temp1;
	temp.x = (end.x+ending.x)/2.0;
	temp.y =(end.y+ending.y)/2.0;
	temp1.x = (temp.x + ending.x)/2.0;
	temp1.y = (temp.y + ending.y)/2.0;
	rPath5.push_back(ending);
	rPath5.push_back(temp1);
	rPath5.push_back(temp);
	rPath5.push_back(end);
	CvPoint2D64f temp2,temp3,temp4;
	temp2.x = 244; temp2.y = 240;
	temp3.x = 245; temp3.y = 230;
	temp4.x = 245; temp4.y = 220;
	/*rPath5.push_back(temp2);*/
	//rPath5.push_back(temp3);
	//rPath5.push_back(temp4);
	//********10.9*******//
	//CvPoint2D64f end1, end2,end3;
	//end1.x = 262;end1.y=232;
	//end2.x = 261; end2.y=222;
	//rPath5.push_back(end1);
	//rPath5.push_back(end2);
	//*******10.9********//
	//rPath5.clear();
	//CvPoint2D64f N1,N2,N3,N4,N5,N6,N7,N8,N9,N10,N11;
	//N1.x=264;N1.y=397;
	//N2.x=265;N2.y=392;
	//N3.x=266;N3.y=387;
	//N4.x=267;N4.y=382;
	//N5.x=253;N5.y=312;
	//N6.x=253;N6.y=307;
	//N7.x=253;N7.y=302;
	//N8.x=262;N8.y=242;
	//N9.x=263;N9.y=237;
	//N10.x=263;N10.y=232;
	//rPath5.push_back(begin);
	//rPath5.push_back(N1);
	//rPath5.push_back(N2);
	//rPath5.push_back(N3);
	//rPath5.push_back(N4);
	//rPath5.push_back(N5);
	//rPath5.push_back(N6);
	//rPath5.push_back(N7);
	//rPath5.push_back(N8);
	//rPath5.push_back(N9);
	//rPath5.push_back(N10);
	//**********//
}

bool RRT::check_free(CvPoint2D64f a,CvPoint2D64f b,CvPoint2D64f c)
{
	if ((minPath<sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)))&&(sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))<10*minDis))
	{
		if((b.x>a.x)&&(c.x > a.x)&&(c.x < b.x))
		{
			double k = (b.y -a.y)/(b.x - a.x);
			double d = abs(k*c.x - k*a.x + a.y - c.y)/(sqrt(1+k*k));
			if(d > /*0.5**/minDis)
				return true;
			else
				return false;
		}
		if((b.x<a.x)&&(c.x < a.x)&&(c.x > b.x))
		{
			double k = (b.y -a.y)/(b.x - a.x);
			double d = abs(k*c.x - k*a.x + a.y - c.y)/(sqrt(1+k*k));
			if(d > /*0.5**/minDis)
				return true;
			else
				return false;
		}
	}
	else if(sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))<=minPath)
		return true;
	else
		return false;
}
void RRT::B_Spline() 
{
	vector<CvPoint2D64f> rPath7;
	//rPath7.clear();
	//rPath8.clear();
	//double N03,N13,N23,N33;
	//CvPoint2D64f L;
	//int n = (int)rPath5.size();
	//for(int i=0;i<=2;i++)
	//	rPath7.push_back(rPath5[0]);
	//for(int i=1;i<n-1;i++)
	//	rPath7.push_back(rPath5[i]);
	//for(int i=0;i<=2;i++)
	//	rPath7.push_back(rPath5[n-1]);
	//int m = (int)rPath7.size();
	//for(int i=1;i<m-2;i++)
	//{
	//	for(double t=0;t<=1;t+=(double)(1.0/(3+m)))
	//	{
	//		N03 = (-t*t*t+3*t*t-3*t+1)/6;
	//		N13 = (3*t*t*t-6*t*t+4)/6;
	//		N23 = (-3*t*t*t+3*t*t+3*t+1)/6;
	//		N33 = t*t*t/6;
	//		L.x = rPath7[i-1].x * N03+rPath7[i].x * N13+rPath7[i+1].x * N23+rPath7[i+2].x * N33;
	//		L.y = rPath7[i-1].y * N03+rPath7[i].y * N13+rPath7[i+1].y * N23+rPath7[i+2].y * N33;
	//		rPath8.push_back(L);
	//	}
	//}
	//rPath5.clear();
	//修改的：贝塞尔后用B样条
	rPath7.clear();
	rPath8.clear();
	double N03,N13,N23,N33;
	CvPoint2D64f L;
	int n = (int)rPath4.size();
	for(int i=0;i<=2;i++)
		rPath7.push_back(rPath4[0]);
	for(int i=1;i<n-1;i++)
		rPath7.push_back(rPath4[i]);
	for(int i=0;i<=2;i++)
		rPath7.push_back(rPath4[n-1]);
	int m = (int)rPath7.size();
	for(int i=1;i<m-2;i++)
	{
		for(double t=0;t<=1;t+=(double)(1.0/(3+m)))
		{
			N03 = (-t*t*t+3*t*t-3*t+1)/6;
			N13 = (3*t*t*t-6*t*t+4)/6;
			N23 = (-3*t*t*t+3*t*t+3*t+1)/6;
			N33 = t*t*t/6;
			L.x = rPath7[i-1].x * N03+rPath7[i].x * N13+rPath7[i+1].x * N23+rPath7[i+2].x * N33;
			L.y = rPath7[i-1].y * N03+rPath7[i].y * N13+rPath7[i+1].y * N23+rPath7[i+2].y * N33;
			rPath8.push_back(L);
		}
	}
	rPath4.clear();

}

void RRT::Clear()
{
	rPath.clear();
	rPath5.clear();
	data.clear();
}
void RRT::Bezier()
{
	rPath4.clear();
	int n = (int)rPath5.size();
	CvPoint2D64f *pc = new CvPoint2D64f[n+1];
	int i,r;
	float u;
	int count = 0;
	for(int k = 0;k<200;k++)
	{   
		u=static_cast<float>(k/199.0);
		for(i=0;i<n;i++)
			pc[i]=rPath5[i];   

		for(r=1;r<=n;r++)
		{   
			for(i=0;i<n-r;i++)
			{   
				pc[i].x=(1-u)*pc[i].x+u*pc[i+1].x;   
				pc[i].y=(1-u)*pc[i].y+u*pc[i+1].y;   
			}   
		}   
		rPath4.push_back(pc[0]);
	}   
	delete [] pc;  
}