#pragma once
#include "vehicle.h"
class CCheckLight :
	public CVehicle
{
public:
	CCheckLight(void);
	void CCheckLight::CheckLightDrive();
	bool GetSignal(int (&sign)[4]);
private:
	int sign[4];
	//int fx;
public:
	~CCheckLight(void);
};
