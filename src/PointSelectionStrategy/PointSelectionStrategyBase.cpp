#include "PointSelectionStrategyBase.h"

PointSelectionStrategyBase::PointSelectionStrategyBase()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

PointSelectionStrategyBase::~PointSelectionStrategyBase()
{
}

std::vector<double> PointSelectionStrategyBase::selectPoint()
{
	return selectPoints(1)[0];
}