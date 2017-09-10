#include "PointSelectionStrategyBase.h"

PointSelectionStrategyBase::PointSelectionStrategyBase()
{

}

PointSelectionStrategyBase::~PointSelectionStrategyBase()
{
}

std::vector<double> PointSelectionStrategyBase::selectPoint()
{
	return selectPoints(1)[0];
}
