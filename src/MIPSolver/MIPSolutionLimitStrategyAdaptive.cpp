#include "MIPSolutionLimitStrategyAdaptive.h"

MIPSolutionLimitStrategyAdaptive::MIPSolutionLimitStrategyAdaptive(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;

	//lastIterSolLimIncreased = 1;
	numSolLimIncremented = 1;
	//currentLimit = Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual");
}

MIPSolutionLimitStrategyAdaptive::~MIPSolutionLimitStrategyAdaptive()
{
}

bool MIPSolutionLimitStrategyAdaptive::updateLimit()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (!currIter->isMIP())
	{
		return false;
	}

	/*
	 if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	 {
	 return false;
	 }*/

	if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		return false;
	}
	/*

	 if (prevIter->isMIP()  && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
	 {
	 return true;
	 }*/

	// Solution limit has not been updated in the maximal number of iterations
	if (prevIter->isMIP()
			&& currIter->iterationNumber - lastIterSolLimIncreased
					> Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual"))
	{
		//std::cout << "Force sol lim update" << std::endl;
		return true;
	}

	// We have a feasible MIP solution to the original problem, but not proven optimal by MIP solver
	if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
			&& prevIter->maxDeviation < prevIter->usedConstraintTolerance)
	//	if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
	{
		return true;
	}

	/*
	 // We have a feasible MIP solution to the original problem, but not proven optimal by MIP solver
	 if (prevIter->isMIP() && prevIter->maxDeviation < prevIter->usedConstraintTolerance && (prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit))
	 {
	 return true;
	 }*/

	// The solution fulfills the intermediate epsilon tolerance but not the final one
	/*
	 if (prevIter->maxDeviation > Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination") && prevIter->maxDeviation < prevIter->usedConstraintTolerance)
	 {
	 return true;
	 }*/

	return false;

}

int MIPSolutionLimitStrategyAdaptive::getNewLimit()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	int newLimit;
	//int iterLargeIncrease = Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual");

	newLimit = MIPSolver->getSolutionLimit() + 1;
	lastIterSolLimIncreased = currIter->iterationNumber;
	// Update MIP solution limit
	//if (numSolLimIncremented > Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual")) // Force larger MIP solution limit update
	//{
	//	newLimit = MIPSolver->getSolutionLimit() + 1;
	//	//numSolLimIncremented = 1;

	//}
	//else // Increase by one
	//{
	//	newLimit = MIPSolver->getSolutionLimit() + 1;
	//	numSolLimIncremented = numSolLimIncremented + 1;
	//}

	//lastIterSolLimIncreased = currIter->iterationNumber;

	return newLimit;
}

int MIPSolutionLimitStrategyAdaptive::getInitialLimit()
{
	return Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual");
}
