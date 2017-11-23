#pragma once

#include "IPointSelectionStrategy.h"
#include "PointSelectionStrategyBase.h"

class PointSelectionStrategyBoundary: public IPointSelectionStrategy, PointSelectionStrategyBase
{
	public:
		PointSelectionStrategyBoundary();
		~PointSelectionStrategyBoundary();
};
