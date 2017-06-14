#include "TaskExecuteSolutionLimitStrategy.h"

TaskExecuteSolutionLimitStrategy::TaskExecuteSolutionLimitStrategy(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	isInitialized = false;
	temporaryOptLimitUsed = false;

	solutionLimitStrategy = new MILPSolutionLimitStrategyIncrease(this->MILPSolver);
	auto initLim = solutionLimitStrategy->getInitialLimit();
	MILPSolver->setSolutionLimit(initLim);

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
	auto prevIter = processInfo->getPreviousIteration();

	constrTolUpdateStrategy->calculateNewTolerance();

	if (temporaryOptLimitUsed)
	{
		temporaryOptLimitUsed = false;
		MILPSolver->setSolutionLimit(previousSolLimit);
	}

	if (currIter->iterationNumber - processInfo->iterLastDualBoundUpdate
			> settings->getIntSetting("ForceOptimalIter", "MILP") && processInfo->getDualBound() > -OSDBL_MAX)
	{
		previousSolLimit = prevIter->usedMILPSolutionLimit;
		MILPSolver->setSolutionLimit(2100000000);
		temporaryOptLimitUsed = true;
		currIter->MILPSolutionLimitUpdated = true;
		processInfo->outputInfo("     Forced optimal iteration since too many iterations since last dual bound update");
	}
	else if (processInfo->getElapsedTime("Total") - processInfo->timeLastDualBoundUpdate
			> settings->getDoubleSetting("ForceOptimalTime", "MILP") && processInfo->getDualBound() > -OSDBL_MAX)
	{
		previousSolLimit = prevIter->usedMILPSolutionLimit;
		MILPSolver->setSolutionLimit(2100000000);
		temporaryOptLimitUsed = true;
		currIter->MILPSolutionLimitUpdated = true;
		processInfo->outputAlways("     Forced optimal iteration since too long time since last dual bound update");
	}
	else if (processInfo->getPrimalBound() < OSDBL_MAX
			&& abs(prevIter->objectiveValue - processInfo->getPrimalBound()) < 0.001)
	{
		previousSolLimit = prevIter->usedMILPSolutionLimit + 1;
		MILPSolver->setSolutionLimit(2100000000);
		temporaryOptLimitUsed = true;
		currIter->MILPSolutionLimitUpdated = true;
		processInfo->outputInfo(
				"     Forced optimal iteration since difference between MIP solution and primal is small");
	}
	else
	{
		currIter->MILPSolutionLimitUpdated = solutionLimitStrategy->updateLimit();

		if (currIter->MILPSolutionLimitUpdated)
		{
			int newLimit = solutionLimitStrategy->getNewLimit();

			if (newLimit != processInfo->getPreviousIteration()->usedMILPSolutionLimit)
			{
				MILPSolver->setSolutionLimit(newLimit);
			}
		}
	}
}

std::string TaskExecuteSolutionLimitStrategy::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
