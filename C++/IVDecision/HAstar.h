#pragma once
#include <math.h>
#include <iostream>
#include "ComuLcm.h"
#define  W 100
#define  L 100
#define  X 10
#define  Y 14


struct Apoint
{
	int x,y;
	int avalue;
};

struct Node
{
	int x,y;
	int g,f,h;
	struct Node *par;
	struct Node *next;
	bool closeexist;
};


class HAstar
{
public:
	HAstar(int sx,int sy,int dx,int dy,I_Map *Amap);
public:
	~HAstar(void);
	struct Node *SearchBest(struct Node*);
	void Addnearnode(struct Node*);
	void Addopen(struct Node*);
	void Addclose(struct Node*);
	void deleteopen(struct Node*);
	struct Node *existopen(struct Node*);
	bool existclose(struct Node*);
	void ShowPath(struct Node*);
	/*void GetVehicleMap(I_Map *Amap);*/
private:
	struct Node* open,*close;
	int Dx,Dy;
	Apoint map[100][100];
};
