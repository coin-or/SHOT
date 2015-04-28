#include "TaskExecuteSolutionLimitStrategy.h"

TaskExecuteSolutionLimitStrategy::TaskExecuteSolutionLimitStrategy()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	isInitialized = false;

	solutionLimitStrategy = new MILPSolutionLimitStrategyIncrease();
	auto initLim = solutionLimitStrategy->getInitialLimit();
	processInfo->MILPSolver->setSolutionLimit(initLim);

	constrTolUpdateStrategy = new ConstraintToleranceUpdateStrategyDecreasing();
}

TaskExecuteSolutionLimitStrategy::~TaskExecuteSolutionLimitStrategy()
{
	delete solutionLimitStrategy;
}

void TaskExecuteSolutionLimitStrategy::run()
{
	if (!isInitialized)
	{
		isInitialized = true;
	}

	auto currIter = processInfo->getCurrentIteration();

	constrTolUpdateStrategy->calculateNewTolerance();

	currIter->MILPSolutionLimitUpdated = solutionLimitStrategy->updateLimit();

	if (currIter->MILPSolutionLimitUpdated)
	{
		int newLimit = solutionLimitStrategy->getNewLimit();

		if (newLimit != processInfo->getPreviousIteration()->usedMILPSolutionLimit)
		{
			processInfo->MILPSolver->setSolutionLimit(newLimit);
		}
	}

}
