#pragma once
#include "IConstraintToleranceUpdateStrategy.h"
#include "ConstraintToleranceUpdateStrategyBase.h"
class ConstraintToleranceUpdateStrategyDecreasing :
	public IConstraintToleranceUpdateStrategy, ConstraintToleranceUpdateStrategyBase
{
public:
	ConstraintToleranceUpdateStrategyDecreasing();
	virtual ~ConstraintToleranceUpdateStrategyDecreasing();

	virtual void calculateNewTolerance();

private:
	double originalTolerance;
	double initialToleranceFactor;
	double solutionLimitDepthFinal;
};

