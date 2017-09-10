#include "ConstraintToleranceUpdateStrategyDecreasing.h"

ConstraintToleranceUpdateStrategyDecreasing::ConstraintToleranceUpdateStrategyDecreasing()
{

	originalTolerance = Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm");
	initialToleranceFactor = Settings::getInstance().getDoubleSetting("ConstrTermTolInitialFactor", "Algorithm");
	solutionLimitDepthFinal = Settings::getInstance().getDoubleSetting("MILPSolLimitEnd", "MILP");
}

ConstraintToleranceUpdateStrategyDecreasing::~ConstraintToleranceUpdateStrategyDecreasing()
{
}

void ConstraintToleranceUpdateStrategyDecreasing::calculateNewTolerance()
{
	Iteration *currIter = ProcessInfo::getInstance().getCurrentIteration();

	// We are at the first iteration
	if (currIter->iterationNumber == 1)
	{
		currIter->usedConstraintTolerance = originalTolerance * initialToleranceFactor;
	}
	else
	{
		Iteration *prevIter = ProcessInfo::getInstance().getPreviousIteration();

		int NL = prevIter->usedMILPSolutionLimit;

		if (NL <= solutionLimitDepthFinal)
		{
			currIter->usedConstraintTolerance = (initialToleranceFactor
					- (initialToleranceFactor - 1) / (1 - solutionLimitDepthFinal)
					+ (initialToleranceFactor - 1) / (1 - solutionLimitDepthFinal) * NL) * originalTolerance;
		}
		else
		{
			currIter->usedConstraintTolerance = originalTolerance;
		}

	}
}
