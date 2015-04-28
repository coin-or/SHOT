#include "ConstraintToleranceUpdateStrategyDecreasing.h"

ConstraintToleranceUpdateStrategyDecreasing::ConstraintToleranceUpdateStrategyDecreasing()
{
	settings = SHOTSettings::Settings::getInstance();
	processInfo = ProcessInfo::getInstance();

	originalTolerance = settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm");
	initialToleranceFactor = settings->getDoubleSetting("ConstrTermTolInitialFactor", "Algorithm");
	solutionLimitDepthFinal = settings->getDoubleSetting("MILPSolLimitEnd", "MILP");
}

ConstraintToleranceUpdateStrategyDecreasing::~ConstraintToleranceUpdateStrategyDecreasing()
{
}

void ConstraintToleranceUpdateStrategyDecreasing::calculateNewTolerance()
{
	Iteration *currIter = processInfo->getCurrentIteration();

	// We are at the first iteration
	if (currIter->iterationNumber == 1)
	{
		currIter->usedConstraintTolerance = originalTolerance * initialToleranceFactor;
	}
	else
	{
		Iteration *prevIter = processInfo->getPreviousIteration();

		int NL = prevIter->usedMILPSolutionLimit;

		//if (prevIter->isMILP() && prevIter->maxDeviation > settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm") && prevIter->maxDeviation < prevIter->usedConstraintTolerance)
		//{
		//	currIter->usedConstraintTolerance = originalTolerance;
		//}s
		//else
		if (NL <= solutionLimitDepthFinal)
		{
			currIter->usedConstraintTolerance = (initialToleranceFactor
					- (initialToleranceFactor - 1) / (1 - solutionLimitDepthFinal)
					+ (initialToleranceFactor - 1) / (1 - solutionLimitDepthFinal) * NL) * originalTolerance;

			//if (currIter->usedConstraintTolerance != prevIter->usedConstraintTolerance)
			//	processInfo->logger.message(2) << "\tNew constraint tolerance: " << currIter->usedConstraintTolerance << CoinMessageEol;
		}
		else
		{
			currIter->usedConstraintTolerance = originalTolerance;
		}

	}
}
