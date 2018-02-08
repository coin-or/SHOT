#include "MILPSolutionLimitStrategyAdaptive.h"

MILPSolutionLimitStrategyAdaptive::MILPSolutionLimitStrategyAdaptive(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;

	//lastIterSolLimIncreased = 1;
	numSolLimIncremented = 1;
	//currentLimit = Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual");
}

MILPSolutionLimitStrategyAdaptive::~MILPSolutionLimitStrategyAdaptive()
{
}

bool MILPSolutionLimitStrategyAdaptive::updateLimit()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (!currIter->isMILP())
	{
		return false;
	}

	/*
	 if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	 {
	 return false;
	 }*/

	if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		return false;
	}
	/*

	 if (prevIter->isMILP()  && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	 {
	 return true;
	 }*/

	// Solution limit has not been updated in the maximal number of iterations
	if (prevIter->isMILP()
			&& currIter->iterationNumber - lastIterSolLimIncreased
					> Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual"))
	{
		//std::cout << "Force sol lim update" << std::endl;
		return true;
	}

	// We have a feasible MILP solution to the original problem, but not proven optimal by MILP solver
	if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
			&& prevIter->maxDeviation < prevIter->usedConstraintTolerance)
	//	if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	{
		return true;
	}

	/*
	 // We have a feasible MILP solution to the original problem, but not proven optimal by MILP solver
	 if (prevIter->isMILP() && prevIter->maxDeviation < prevIter->usedConstraintTolerance && (prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit))
	 {
	 return true;
	 }*/

	// The solution fulfills the intermediate epsilon tolerance but not the final one
	/*
	 if (prevIter->maxDeviation > Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm") && prevIter->maxDeviation < prevIter->usedConstraintTolerance)
	 {
	 return true;
	 }*/

	return false;

}

int MILPSolutionLimitStrategyAdaptive::getNewLimit()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	int newLimit;
	//int iterLargeIncrease = Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual");

	newLimit = MILPSolver->getSolutionLimit() + 1;
	lastIterSolLimIncreased = currIter->iterationNumber;
	// Update MILP solution limit
	//if (numSolLimIncremented > Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual")) // Force larger MILP solution limit update
	//{
	//	newLimit = MILPSolver->getSolutionLimit() + 1;
	//	//numSolLimIncremented = 1;

	//}
	//else // Increase by one
	//{
	//	newLimit = MILPSolver->getSolutionLimit() + 1;
	//	numSolLimIncremented = numSolLimIncremented + 1;
	//}

	//lastIterSolLimIncreased = currIter->iterationNumber;

	return newLimit;
}

int MILPSolutionLimitStrategyAdaptive::getInitialLimit()
{
	return Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual");
}
