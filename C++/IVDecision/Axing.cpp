//Axing.cpp
//A星算法的实现
#include "stdafx.h" //预编译头文件要放在最前面！
#include "Axing.h"
#include "stdlib.h"

Axing::Axing(void)
{
   ditu = new I_Map;
}
Axing::~Axing(void)
{
	delete ditu;
}
void Axing::Axingpath(int sx,int sy,int dx,int dy,double theta1,double theta2)
{
//************结合航向做得变动*******//
	CvPoint2D64f mubiao,qishi;
	mubiao.x = dx;
	mubiao.y = dy;
	//path1.push_back(mubiao);

	//Dx=static_cast<int>(dx-stepsize*cos(theta2*PI/180.0));				
	//Dy=static_cast<int>(dy+stepsize*sin(theta2*PI/180.0));
	Dx = dx;
	Dy = dy;

	int sx1,sy1;
	//sx1 = static_cast<int>(sx + stepsize*cos(theta1*PI/180.0));
	//sy1 = static_cast<int>(sy - stepsize*sin(theta1*PI/180.0));
	//qishi.x = sx1;
	//qishi.y = sy1;
	sx1 = sx;
	sy1 = sy;
	//******************//

	struct NodeAxing *node;
	struct NodeAxing *Min;

	open=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));//在内存的动态存储区中分配1个长度为sizeof(struct NodeAxing)的连续空间，函数返回一个指向分配起始地址的指针。
	close=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	node=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));

	//初始化
	node->x=sx1;
	node->y=sy1;
	node->g=0;
	node->h=sqrt(static_cast<double>((Dx-sx1)*(Dx-sx1)+(Dy-sy1)*(Dy-sy1)));
	/*node->h=static_cast<int>(sqrt(0.01*node->h)*100);*///h=10倍的欧式距离
	node->f=node->g + node->h;
	node->par=NULL;//初始化父节点为空
	node->next=NULL;//
	node->closeexist=false;

	open->next=NULL;


	Addopen(node);////把起始点 添加到open列表中，此时open表中只有起始点
	//open->next就是起始点

	while((node->x!=Dx||node->y!=Dy)&&open->next!=NULL)
	{
		Min=SearchMin(open);
		node=Min;
		Addnearnode(Min);
		deleteopen(Min);
		Addclose(Min);

	}
	if(node->x==Dx && node->y==Dy)
	{
		ShowPath(node);
	}

	//path1.push_back(qishi);
	free(node);
	free(close);
	free(open);

}



NodeAxing* Axing::SearchMin(struct NodeAxing* t)
{
	struct NodeAxing* node=t->next;
	struct NodeAxing* node1=t;
	if(t->next==NULL)
		exit(0);
	node1=node1->next;
	while(node1!=NULL)
	{
		if(node->f > node1->f)
			node=node1;
		node1=node1->next;
	}

	return node;
}

void Axing::Addnearnode(struct NodeAxing* t)
{
	struct NodeAxing* nodefirst=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	struct NodeAxing* nodesecond=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	struct NodeAxing* nodethird=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	struct NodeAxing* nodefourth=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	struct NodeAxing* nodefifth=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	struct NodeAxing* nodesixth=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	struct NodeAxing* nodeseventh=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));
	struct NodeAxing* nodeeighth=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));

	struct NodeAxing* q=(struct NodeAxing*)calloc(1,sizeof(struct NodeAxing));

	//struct NodeAxing* nodefirst 左

	nodefirst->x=t->x-stepsize;
	nodefirst->y=t->y;
	if(nodefirst->x >=0 && (ditu->MapPoint[nodefirst->y][nodefirst->x]!=1))//node 左侧的节点在map图上，且不是障碍物
	{
		q=existopen(nodefirst);//根据返回值判断是否在open表中
		nodefirst->closeexist=existclose(nodefirst);//判断是否在close表中

		if((existopen(nodefirst)==NULL) && !nodefirst->closeexist)//不在open表中，也不在close表中
		{
			nodefirst->g = t->g + XX;
			nodefirst->h = sqrt(static_cast<double>((Dx-nodefirst->x)*(Dx-nodefirst->x)+(Dy-nodefirst->y)*(Dy-nodefirst->y)));
			/*nodefirst->h = static_cast<int>(sqrt(0.01*nodefirst->h)*100);*/
			nodefirst->f = nodefirst->g + nodefirst->h;
			nodefirst->par = t;
			Addopen(nodefirst); //加到open表中

		}
		else
		{
			int gvalue = t->g + XX;
			if( q!=NULL)//在open表中
			{
				if(gvalue < q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}

			}
		}
	}


	//struct Node *nodesecond 右
	nodesecond->x = t->x + stepsize;
	nodesecond->y = t->y;
	if(nodesecond->x < kuan && (ditu->MapPoint[nodesecond->y][nodesecond->x]!=1))
	{
		q=existopen(nodesecond);
		nodesecond->closeexist=existclose(nodesecond);
		if(existopen(nodesecond)==NULL && !nodesecond->closeexist)
		{
			nodesecond->g = t->g + XX;
			nodesecond->h = sqrt(static_cast<double>((Dx-nodesecond->x)*(Dx-nodesecond->x)+(Dy-nodesecond->y)*(Dy-nodesecond->y)));
			/*nodesecond->h = static_cast<int>(sqrt(0.01*nodesecond->h)*100);*/
			nodesecond->f = nodesecond->g + nodesecond->h;
			nodesecond->par = t;
			Addopen(nodesecond);

		}
		else
		{
			int gvalue = t->g + XX;
			if( q!=NULL)//在open表中
			{
				if(gvalue < q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}

			}
		}
	}


	//struct Node *nodethird 上
	nodethird->x = t->x;
	nodethird->y = t->y - stepsize;
	if(nodethird->y >= 0 && (ditu->MapPoint[nodethird->y][nodethird->x]!=1))
	{
		q = existopen(nodethird);
		nodethird->closeexist = existclose(nodethird);
		if(existopen(nodethird)==NULL && !nodethird->closeexist)
		{
			nodethird->g = t->g + XX;
			nodethird->h = sqrt(static_cast<double>((Dx-nodethird->x)*(Dx-nodethird->x)+(Dy-nodethird->y)*(Dy-nodethird->y)));
			/*nodethird->h = static_cast<int>(sqrt(0.01*nodethird->h)*100);*/
			nodethird->f = nodethird->g + nodethird->h;
			nodethird->par = t;
			Addopen(nodethird);

		}
		else
		{
			int gvalue = t->g + XX;
			if(q!=NULL)
			{
				if(gvalue < q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}
			}
		}


	}

	//struct Node* nodefourth;下
	nodefourth->x = t->x;
	nodefourth->y = t->y + stepsize;
	if(nodefourth-> y < chang && (ditu->MapPoint[nodefourth->y][nodefourth->x]!=1))
	{
		q = existopen(nodefourth);
		nodefourth->closeexist = existclose(nodefourth);
		if(existopen(nodefourth)==NULL && !nodefourth->closeexist)
		{
			nodefourth->g = t->g + XX;
			nodefourth->h = sqrt(static_cast<double>((Dx-nodefourth->x)*(Dx-nodefourth->x)+(Dy-nodefourth->y)*(Dy-nodefourth->y)));
			/*nodefourth->h = static_cast<int>(sqrt(0.01*nodefourth->h)*100);*/
			nodefourth->f = nodefourth->g + nodefourth->h;
			nodefourth->par = t;
			Addopen(nodefourth);

		}
		else
		{
			int gvalue = t->g + XX;
			if(q!=NULL)
			{
				if(gvalue < q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}
			}

		}
	}
	//struct Node* nodefifth 左上

	nodefifth->x = t->x - stepsize;
	nodefifth->y = t->y - stepsize;
	if(nodefifth->x >= 0 && nodefifth->y >=0 && ditu->MapPoint[nodefifth->y][nodefifth->x]!=1)
	{
		q = existopen(nodefifth);
		nodefifth->closeexist = existclose(nodefifth);
		if(existopen(nodefifth)==NULL && !nodefifth->closeexist)
		{
			nodefifth->g = t->g + YY;
			nodefifth->h = sqrt(static_cast<double>((Dx-nodefifth->x)*(Dx-nodefifth->x)+(Dy-nodefifth->y)*(Dy-nodefifth->y)));
			/*nodefifth->h = static_cast<int>(sqrt(0.01*nodefifth->h)*100);*/
			nodefifth->f = nodefifth->g + nodefifth->h;
			nodefifth->par = t;
			Addopen(nodefifth);

		}
		else
		{
			int gvalue = t->g + YY;
			if(q!=NULL)
			{
				if(gvalue < q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}
			}

		}
	}


	//struct Node* nodesixth 右上
	nodesixth->x = t->x + stepsize;
	nodesixth->y = t->y - stepsize;
	if(nodesixth->x < kuan && nodesixth->y >=0 && (ditu->MapPoint[nodesixth->y][nodesixth->x]!=1))
	{
		q = existopen(nodesixth);
		nodesixth->closeexist = existclose(nodesixth);
		if(existopen(nodesixth)==NULL && !nodesixth->closeexist)
		{
			nodesixth->g = t->g + YY;
			nodesixth->h = sqrt(static_cast<double>((Dx-nodesixth->x)*(Dx-nodesixth->x)+(Dy-nodesixth->y)*(Dy-nodesixth->y)));
			/*nodesixth->h = static_cast<int>(sqrt(0.01*nodesixth->h)*100);*/
			nodesixth->f = nodesixth->g + nodesixth->h;
			nodesixth->par = t;
			Addopen(nodesixth);

		}
		else
		{
			int gvalue = t->g + YY;
			if(q!=NULL)
			{
				if(gvalue< q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}
			}
		}
	}

	//struct Node* nodeseventh 左下
	nodeseventh->x = t->x - stepsize;
	nodeseventh->y = t->y +stepsize;
	if(nodeseventh-> x >=0 && nodeseventh->y < chang && (ditu->MapPoint[nodeseventh->y][nodeseventh->x]!=1))
	{
		q = existopen(nodeseventh);
		nodeseventh->closeexist = existclose(nodeseventh);
		if(existopen(nodeseventh)==NULL && !nodeseventh->closeexist)
		{
			nodeseventh->g = t->g + YY;
			nodeseventh->h = sqrt(static_cast<double>((Dx-nodeseventh->x)*(Dx-nodeseventh->x)+(Dy-nodeseventh->y)*(Dy-nodeseventh->y)));
			/*nodeseventh->h = static_cast<int>(sqrt(0.01*nodeseventh->h)*100);*/
			nodeseventh->f = nodeseventh->g + nodeseventh->h;
			nodeseventh->par = t;
			Addopen(nodeseventh);

		}
		else
		{
			int gvalue = t->g + YY;
			if(q!=NULL)
			{
				if(gvalue < q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}
			}
		}
	}

	//struct Node* nodeeighth 右下
	nodeeighth->x = t->x + stepsize;
	nodeeighth->y = t->y + stepsize;
	if(nodeeighth->x <kuan && nodeeighth->y <chang && (ditu->MapPoint[nodeeighth->y][nodeeighth->x]!=1))
	{
		q = existopen(nodeeighth);
		nodeeighth->closeexist = existclose(nodeeighth);
		if(existopen(nodeeighth)==NULL && !nodeeighth->closeexist)
		{
			nodeeighth->g = t->g + YY;
			nodeeighth->h = sqrt(static_cast<double>((Dx-nodeeighth->x)*(Dx-nodeeighth->x)+(Dy-nodeeighth->y)*(Dy-nodeeighth->y)));
			/*nodeeighth->h = static_cast<int>(sqrt(0.01*nodeeighth->h)*100);*/
			nodeeighth->f = nodeeighth->g + nodeeighth->h;
			nodeeighth->par = t;
			Addopen(nodeeighth);
		}
		else
		{
			int gvalue = t->g + YY;
			if(q!=NULL)
			{
				if(gvalue < q->g)
				{
					q->g = gvalue;
					q->f = q->g + q->h;
					q->par = t;
				}
			}
		}
	}
	//free(nodefirst);
	//free(nodesecond);
	//free(nodethird);
	//free(nodefourth);
	//free(nodefifth);
	//free(nodesixth);
	//free(nodeseventh);
	//free(nodeeighth);
}

struct NodeAxing* Axing::existopen(struct NodeAxing *t)
{
	struct NodeAxing* p;
	//p=t->next;
	p=open->next;
	while(p!=NULL)
	{
		if(p->x==t->x && p->y==t->y)
			return p;
		p=p->next;
	}
	return NULL;

}

void Axing::Addopen(struct NodeAxing *t)
{
	t->next = open->next;
	open->next = t;
}

bool Axing::existclose(struct NodeAxing *t)
{
	struct NodeAxing* p;
	p=close->next;
	while(p!=NULL)
	{
		if(p->x==t->x && p->y==t->y)
			return true;
		p=p->next;
	}
	return false;

}

void Axing::Addclose(struct NodeAxing *t)
{
	t->next=close->next;
	close->next=t;
}

void Axing::deleteopen(struct NodeAxing *t)
{
	struct NodeAxing* p;
	p=open->next;
	if(p==t)
	{
		open->next=p->next;
		return;
	}
	else
	{
		while(p!=NULL)
		{
			if(p->next==t)
			{
				p->next = t->next;	
				return;
			}

			p=p->next;
		}

	}

}

void Axing::ShowPath(struct NodeAxing* t)
{
	while(t!=NULL)
	{
		CvPoint2D64f temp;
		temp.x = t->x;
		temp.y = t->y;
		path1.push_back(temp);//得到的是从goal 到start的路径点
		t = t->par;
	}
}

void Axing::Bezier()
{
	path2.clear();
	int n = (int)path1.size();
	CvPoint2D64f *pc = new CvPoint2D64f[n+1];
	int i,r;
	float u;
	int count = 0;
	for (int k=0;k<200;k++)
	{
		u = static_cast<float>(k/199.0);
		for (i=0;i<n;i++)
			pc[i]=path1[i];
		for (r=1;r<=n;r++)
		{
			for (i=0;i<n-r;i++)
			{
				pc[i].x=(1-u)*pc[i].x+u*pc[i+1].x;   
				pc[i].y=(1-u)*pc[i].y+u*pc[i+1].y;   
			}
		}
		path2.push_back(pc[0]);
	}
	delete [] pc;
}

void Axing::quzheng()
{
	for (size_t i=1;i<path1.size()-1;i++)
	{
		CvPoint2D64f temp;
		temp.x = ((int)(path1[i].x/10))*10;
		temp.y = ((int)(path1[i].y/10))*10;
		path3.push_back(temp);
	}
}
