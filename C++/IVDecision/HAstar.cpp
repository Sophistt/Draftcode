#include "StdAfx.h"
#include "HAstar.h"

HAstar::HAstar(int sx,int sy,int dx,int dy, I_Map *Amap)
{
	sx=sx/5;
	sy=sy/5;
	int countnum=0;
	for (int i=0; i<500; i++)
	{
		for (int j=0; j<500; j++)
		{
			if (Amap->MapPoint[i][j]==18)
			{
				countnum++;
			}

		}
	}
	for (int i=0; i<500; i++)
	{
		for (int j=0; j<500; j++)
		{
			if (map[j/5][i/5].avalue!=1)
			{
				if (Amap->MapPoint[i][j]==18)
				{
					map[j/5][i/5].avalue=1;
					j=(j/5+1)*5;
				}
			}

			if (map[j/5][i/5].avalue==1)
			{
				j=(j/5+1)*5;
				continue;
			}

		}
	}
	int count=0;
	for (int i=0 ;i<100; i++)
	{
		for (int j=0; j<100; j++)
		{
			if (map[j][i].avalue==1)
			{
				count++;
			}
		}
	}

	Dx=dx/5;
	Dy=dy/5;


	struct Node *node,*best;
	open=(struct Node*)calloc(1,sizeof(struct Node));//分配一个空间给open
	close=(struct Node*)calloc(1,sizeof(struct Node));
	node=(struct Node*)calloc(1,sizeof(struct Node));
	node->x=sx;
	node->y=sy;
	node->g=0;
	node->h=(dx-sx)*(dx-sx)+(dy-sy)*(dy-sy);
	node->h=(sqrt(0.01*node->h))*100;
	node->f=node->g+node->h;
	node->par=NULL;
	node->next=NULL;
	open->next=NULL;

	node->closeexist=false;
	Addopen(node);
	while (node->x!=dx||node->y!=dy&&open->next!=NULL)
	{
		best=SearchBest(open);
		node=best;
		Addnearnode(best);
		deleteopen(best);
		Addclose(best);
	}

	if (node->x==dx&&node->y==dy)
	{
		ShowPath(node);
	}
}

HAstar::~HAstar(void)
{
}



Node* HAstar::SearchBest(struct Node* t)
{
	struct Node *node=t->next;
	struct Node *node1=t;
	if (t->next==NULL)
	{
		exit(0);    //程序正常结束
	}

	node1=node1->next;
	while (node1!=NULL)
	{
		if (node->f>node1->f)
		{
			node=node1;
		}

		node1=node1->next;
	}
	return node;
}

void HAstar::Addnearnode(struct Node* t)
{
	struct Node *nodefirst=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *nodesecond=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *nodethird=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *nodefourth=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *nodefifth=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *nodesixth=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *nodeseventh=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *nodeeighth=(struct Node*)calloc(1,sizeof(struct Node));
	struct Node *q=(struct Node*)calloc(1,sizeof(struct Node));
//struct Node* nodefirst; 左
	nodefirst->x=t->x-1;
	nodefirst->y=t->y;

	if (nodefirst->x>=0&&(map[nodefirst->x][nodefirst->y].avalue!=1))
	{
		q=existopen(nodefirst);
		nodefirst->closeexist=existclose(nodefirst);
		if (existopen(nodefirst)==NULL&&!nodefirst->closeexist)       //如果这个点第一次出现
		{
			nodefirst->g=t->g+X;
			nodefirst->h=(Dx-nodefirst->x)*(Dx-nodefirst->x)+(Dy-nodefirst->y)*(Dy-nodefirst->y);
			nodefirst->h=sqrt(0.01*nodefirst->h)*100;
			nodefirst->f=nodefirst->g+nodefirst->h;
			nodefirst->par=t;
			Addopen(nodefirst);
		}

		else
		{
			int gvalue=t->g+X;
			if (q!=NULL)                  //如果这个点存在于open表中，更新此点的信息
			{
				if (gvalue<q->g)
				{
					q->g=gvalue;
					q->f=q->g+q->h;
					q->par=t;
				}
			}
		}
	}


	//struct Node* nodesecond 右

	nodesecond->x=t->x+1;
	nodesecond->y=t->y;
	if (nodesecond->x<W&&(map[nodesecond->x][nodesecond->y].avalue!=1))
	{
		q=existopen(nodesecond);
		nodesecond->closeexist=existclose(nodesecond);
		if ((existopen(nodesecond)==NULL)&&!nodesecond->closeexist)
		{
			nodesecond->g=t->g+X;
			nodesecond->h=(Dx-nodesecond->x)*(Dx-nodesecond->x)+(Dy-nodesecond->y)*(Dy-nodesecond->y);
			nodesecond->h=sqrt(0.01*nodesecond->h)*100;
			nodesecond->par=t;
			Addopen(nodesecond);
		}

		else
		{
			int gvalue=t->g+X;
			if (q!=NULL)
			{
				if (gvalue<q->g)
				{
					q->g=gvalue;
					q->f=q->g+q->h;
					q->par=t;
				}
			}
		}
	}


	//struct Node* nodethird; 上

	nodethird->x=t->x;
	nodethird->y=t->y;
	if (nodethird->y>=0&&(map[nodethird->x][nodethird->y].avalue!=1))
	{
		q=existopen(nodethird);
		nodethird->closeexist=existclose(nodethird);
		if ((existclose(nodethird)==NULL)&&!nodeeighth->closeexist)
		{
			nodethird->g=t->g+X;
			nodethird->h=(Dx-nodethird->x)*(Dx-nodethird->x)+(Dy-nodethird->y)*(Dy-nodethird->y);
			nodethird->h=sqrt(0.01*nodethird->h)*100;
			nodethird->f=nodethird->g+nodethird->h;
			nodethird->par=t;
			Addopen(nodethird);
		}
	}

	else
	{
		int gvalue=t->g+X;
		if (q!=NULL)
		{
			if (gvalue<q->g)
			{
				q->g=gvalue;
				q->f=q->g+q->h;
				q->par=t;
			}
		}
	}




	//struct Node* nodefourth 下
	nodefourth->x=t->x;
	nodefourth->y=t->y+1;
	if (nodefourth->y<L&&(map[nodefourth->x][nodefourth->y].avalue!=1))
	{
		q=existopen(nodefourth);
		nodefourth->closeexist=existclose(nodefourth);
		if ((existopen(nodefourth)==NULL)&&!nodefourth->closeexist)
		{
			nodefourth->g=t->g+X;
			nodefourth->h=(Dx-nodefourth->x)*(Dx-nodefourth->x)+(Dy-nodefourth->y)*(Dy-nodefourth->y);
			nodefourth->h=sqrt(0.01*nodefourth->h)*100;
			nodefourth->f=nodefourth->h+nodefourth->g;
			nodefourth->par=t;
			Addopen(nodefourth);
		}
		
		else
		{
			int gvalue=t->g+X;
			if (q!=NULL)
			{
				if (gvalue<q->g)
				{
					q->g=gvalue;
					q->f=q->h+q->g;
					q->par=t;
				}
			}
		}

	}


	//struct Node* nodefifth 左上

	nodefifth->x=t->x-1;
	nodefifth->y=t->y-1;
	if (nodefifth->x>=0&&nodefifth->y>=0&&(map[nodefifth->x][nodefifth->y].avalue!=1)&&!((map[nodefifth->x+1][nodefifth->y].avalue==1)&&(map[nodefifth->x][nodefifth->y+1].avalue==1)))
	{
		q=existopen(nodefifth);
		nodefifth->closeexist=existclose(nodefifth);
		if ((existopen(nodefifth)==NULL)&&nodefifth->closeexist)
		{
			nodefifth->g=t->g+Y;                                //斜向移动一个单位增加的代价
			nodefifth->h=(Dx-nodefifth->x)*(Dx-nodefifth->x)+(Dy-nodefifth->y)*(Dy-nodefifth->y);
			nodefifth->h=sqrt(0.01*nodefifth->h)*100;
			nodefifth->f=nodefifth->g+nodefifth->h;
			nodefifth->par=t;
			Addopen(nodefifth);
		}

		else
		{
			int gvalue=t->g+Y;
			if (q!=NULL)
			{
				if (gvalue<q->g)
				{
					q->g=gvalue;
					q->f=q->h+q->g;
					q->par=t;
				}
			}
		}
	}

//struct Node8 nodesixth 右上

	nodesixth->x=t->x+1;
	nodesixth->y=t->y-1;
	if (nodesixth->x<W&&nodesixth->y>=0&&(map[nodesixth->x][nodesixth->y].avalue!=1)&&!((map[nodesixth->x-1][nodesixth->y].avalue==1)&&(map[nodesixth->x][nodesixth->y+1].avalue==1)))
	{
		q=existopen(nodesixth);
		nodesixth->closeexist=existclose(nodesixth);
		if ((existopen(nodesixth)==NULL)&&!nodesixth->closeexist)
		{
			nodesixth->g=t->g+Y;
			nodesixth->h=(Dx-nodesixth->x)*(Dx-nodesixth->x)+(Dy-nodesixth->y)*(Dy-nodesixth->y);
			nodesixth->h=sqrt(0.01*nodesixth->h)*100;
			nodesixth->f=nodesixth->h+nodesixth->g;
			nodesixth->par=t;
			Addopen(nodesixth);
		}

		else
		{
			int gvalue=t->g+Y;
			if (!q==NULL)
			{
				if (gvalue<q->g)
				{
					q->g=gvalue;
					q->f=q->g+q->h;
					q->par=t;
				}
			}
		}
	}


	//struct Node* nodeseventh;左下
	nodeseventh->x=t->x-1;
	nodeseventh->y=t->y+1;
	if (nodeseventh->x>=0&&nodeseventh->y<L&&(map[nodeseventh->x][nodeseventh->y].avalue!=1)&&!((map[nodeseventh->x][nodeseventh->y-1].avalue==1)&&(map[nodeseventh->x+1][nodeseventh->y].avalue==1)))
	{
		q=existopen(nodeseventh);
		nodeseventh->closeexist=existclose(nodeseventh);
		if ((existopen(nodeseventh)==NULL)&&!nodeseventh->closeexist)
		{
			nodeseventh->g=t->g+Y;
			nodeseventh->h=(Dx-nodeseventh->x)*(Dx-nodeseventh->x)+(Dy-nodeseventh->y)*(Dy-nodeseventh->y);
			nodeseventh->h=sqrt(0.01*nodeseventh->h)*100;
			nodeseventh->f=nodeseventh->h+nodeseventh->g;
			nodeseventh->par=t;
			Addopen(nodeseventh);
		}

		else
		{
			int gvalue=t->g+Y;
			if (q!=NULL)
			{
				if (gvalue<q->g)
				{
					q->g=gvalue;
					q->f=q->g+q->h;
					q->par=t;
				}
			}
		}
	}


	//struct Node* nodeeigth      右下

	nodeeighth->x=t->x+1;
	nodeeighth->y=t->y+1;
	if (nodeeighth->x<W&&nodeeighth->y<L&&(map[nodeeighth->x][nodeeighth->y].avalue!=1)&&!((map[nodeeighth->x-1][nodeeighth->y].avalue==1)&&(map[nodeeighth->x][nodeeighth->y-1].avalue==1)))
	{
		q=existopen(nodeeighth);
		nodeeighth->closeexist=existclose(nodeeighth);
		if ((existopen(nodeeighth)==NULL)&&!nodeeighth->closeexist)
		{
			nodeeighth->g=t->g+Y;
			nodeeighth->h=(Dx-nodeeighth->x)*(Dx-nodeeighth->x)+(Dy-nodeeighth->y)*(Dy-nodeeighth->y);
			nodeeighth->h=sqrt(nodeeighth->h*0.01)*100;
			nodeeighth->f=nodeeighth->g+nodeeighth->h;
		    nodeeighth->par=t;
			Addopen(nodeeighth);
		}

		else
		{
			int gvalue=t->g+Y;
			if (q!=NULL)
			{
				if (gvalue<q->g)
				{
					q->g=gvalue;
					q->f=q->h+q->g;
					q->par=t;
				}
			}
		}
	}
	


}






struct Node* HAstar::existopen(Node * t)
{
	struct Node* p;
	p=open->next;
	while (p!=NULL)
	{
		if (p->x==t->x&&p->y==t->y)
		{
			return p;
		}
		p=p->next;
	}
	return NULL;
}

bool HAstar::existclose(struct Node* t)
{
	struct Node* p;
	p=close->next;
	while (p!=NULL)
	{
		if (p->x==t->x&&p->y==t->y)
		{
			return true;
		}
		p=p->next;
	}
	return false;
}

void HAstar::Addopen(struct Node* k)
{
	k->next=open->next;
	open->next=k;
}

void HAstar::Addclose(struct Node* k)
{
	k->next=close->next;
	close->next=k;
}


void HAstar::deleteopen(struct Node *k)
{
	struct Node* p;
	p=open->next;
	if (p==k)
	{
		open->next=p->next;
		return;
	}
	else
	{
		while (p!=NULL)
		{		
			if (p->next==k)
			{
				p->next=p->next->next;
				return;
			}
			p=p->next;		
		}
	}
}

void HAstar::ShowPath(struct Node*clp)
{
	while(clp!=NULL)
	{
		map[clp->x][clp->y].avalue=2;
		clp=clp->par;
	}
}