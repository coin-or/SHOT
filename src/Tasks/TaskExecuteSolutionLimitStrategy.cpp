#include "TaskExecuteSolutionLimitStrategy.h"

TaskExecuteSolutionLimitStrategy::TaskExecuteSolutionLimitStrategy()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	isInitialized = false;
	temporaryOptLimitUsed = false;

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

	if (temporaryOptLimitUsed)
	{
		temporaryOptLimitUsed = false;
		processInfo->MILPSolver->setSolutionLimit(previousSolLimit);
	}

	if (currIter->iterationNumber - processInfo->iterLastDualBoundUpdate
			> settings->getIntSetting("MILPSolForceOptimalIter", "MILP") && processInfo->getDualBound() > -OSDBL_MAX)
	{
		previousSolLimit = processInfo->getPreviousIteration()->usedMILPSolutionLimit;
		processInfo->MILPSolver->setSolutionLimit(2100000000);
		temporaryOptLimitUsed = true;
	}
	else
	{
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

}

std::string TaskExecuteSolutionLimitStrategy::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
