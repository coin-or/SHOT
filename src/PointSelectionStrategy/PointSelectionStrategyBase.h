#pragma once
#include "SHOTSettings.h"
#include "../ProcessInfo.h"

class PointSelectionStrategyBase 
{
public:
	PointSelectionStrategyBase();
	~PointSelectionStrategyBase();

	std::vector<double> selectPoint();
	virtual std::vector<std::vector<double>> selectPoints(int maxNumPts) = 0;

protected:
	SHOTSettings::Settings *settings;
	//ProcessInfo *processInfo;
};
