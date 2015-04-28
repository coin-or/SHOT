#include "PointSelectionStrategyBoundary.h"

PointSelectionStrategyBoundary::PointSelectionStrategyBoundary()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

PointSelectionStrategyBoundary::~PointSelectionStrategyBoundary()
{
}