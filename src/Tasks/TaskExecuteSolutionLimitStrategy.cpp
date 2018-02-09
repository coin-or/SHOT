#include "TaskExecuteSolutionLimitStrategy.h"

TaskExecuteSolutionLimitStrategy::TaskExecuteSolutionLimitStrategy(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;

	isInitialized = false;
	temporaryOptLimitUsed = false;

	solutionLimitStrategy = new MIPSolutionLimitStrategyIncrease(this->MIPSolver);
	auto initLim = solutionLimitStrategy->getInitialLimit();
	MIPSolver->setSolutionLimit(initLim);
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

	auto currIter = ProcessInfo::getInstance().getCurrentIteration();
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (temporaryOptLimitUsed)
	{
		temporaryOptLimitUsed = false;
		MIPSolver->setSolutionLimit(previousSolLimit);
	}

	if (currIter->iterationNumber - ProcessInfo::getInstance().iterLastDualBoundUpdate
			> Settings::getInstance().getIntSetting("MIP.SolutionLimit.ForceOptimal.Iteration", "Dual")
			&& ProcessInfo::getInstance().getDualBound() > -OSDBL_MAX)
	{
		previousSolLimit = prevIter->usedMIPSolutionLimit;
		MIPSolver->setSolutionLimit(2100000000);
		temporaryOptLimitUsed = true;
		currIter->MIPSolutionLimitUpdated = true;
		ProcessInfo::getInstance().outputInfo(
				"     Forced optimal iteration since too many iterations since last dual bound update");
	}
	else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().timeLastDualBoundUpdate
			> Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.ForceOptimal.Time", "Dual")
			&& ProcessInfo::getInstance().getDualBound() > -OSDBL_MAX)
	{
		previousSolLimit = prevIter->usedMIPSolutionLimit;
		MIPSolver->setSolutionLimit(2100000000);
		temporaryOptLimitUsed = true;
		currIter->MIPSolutionLimitUpdated = true;
		ProcessInfo::getInstance().outputAlways(
				"     Forced optimal iteration since too long time since last dual bound update");
	}
	else if (ProcessInfo::getInstance().getPrimalBound() < OSDBL_MAX
			&& abs(prevIter->objectiveValue - ProcessInfo::getInstance().getPrimalBound()) < 0.001)
	{
		previousSolLimit = prevIter->usedMIPSolutionLimit + 1;
		MIPSolver->setSolutionLimit(2100000000);
		temporaryOptLimitUsed = true;
		currIter->MIPSolutionLimitUpdated = true;
		ProcessInfo::getInstance().outputInfo(
				"     Forced optimal iteration since difference between MIP solution and primal is small");
	}
	else
	{
		currIter->MIPSolutionLimitUpdated = solutionLimitStrategy->updateLimit();

		if (currIter->MIPSolutionLimitUpdated)
		{
			int newLimit = solutionLimitStrategy->getNewLimit();

			if (newLimit != ProcessInfo::getInstance().getPreviousIteration()->usedMIPSolutionLimit)
			{
				MIPSolver->setSolutionLimit(newLimit);
			}
		}
	}
}

std::string TaskExecuteSolutionLimitStrategy::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
