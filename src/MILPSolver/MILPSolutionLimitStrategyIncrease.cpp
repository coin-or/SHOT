#include "MILPSolutionLimitStrategyIncrease.h"

MILPSolutionLimitStrategyIncrease::MILPSolutionLimitStrategyIncrease()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	//MILPSolver = solver;

	lastIterSolLimIncreased = 1;
	numSolLimIncremented = 1;
	//currentLimit = settings->getIntSetting("MILPSolLimitInitial", "MILP");
}

MILPSolutionLimitStrategyIncrease::~MILPSolutionLimitStrategyIncrease()
{
}

bool MILPSolutionLimitStrategyIncrease::updateLimit()
{
	auto currIter = processInfo->getCurrentIteration();
	auto prevIter = processInfo->getPreviousIteration();

	if (!currIter->isMILP())
	{
		lastIterSolLimIncreased = currIter->iterationNumber;
		return false;
	}

	if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		return false;
	}
	/*

	 if (prevIter->isMILP()  && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	 {
	 return true;
	 }*/

	// Solution limit has not been updated in the maximal number of iterations
	if (prevIter->isMILP()
			&& currIter->iterationNumber - lastIterSolLimIncreased
					> settings->getIntSetting("MILPSolIncreaseIter", "MILP"))
	{
		std::cout << " Force solution limit update" << std::endl;
		return true;
	}

	// We have a feasible MILP solution to the original problem, but not proven optimal by MILP solver
	//if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  prevIter->usedConstraintTolerance)
	//TODO use the strategy for updated constraint tolerance
	if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
			&& (prevIter->maxDeviation
					< settings->getDoubleSetting("MILPSolLimitUpdateTol", "MILP") * prevIter->objectiveValue
					|| prevIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm")))
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
	 if (prevIter->maxDeviation > settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm") && prevIter->maxDeviation < prevIter->usedConstraintTolerance)
	 {
	 return true;
	 }*/

	return false;

}

int MILPSolutionLimitStrategyIncrease::getNewLimit()
{
	auto currIter = processInfo->getCurrentIteration();

	int newLimit;
	//int iterLargeIncrease = settings->getIntSetting("MILPSolIncreaseIter", "MILP");

	newLimit = processInfo->MILPSolver->getSolutionLimit() + 1;
	lastIterSolLimIncreased = currIter->iterationNumber;
	// Update MILP solution limit
	//if (numSolLimIncremented > settings->getIntSetting("MILPSolIncreaseIter", "MILP")) // Force larger MILP solution limit update
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

int MILPSolutionLimitStrategyIncrease::getInitialLimit()
{
	return settings->getIntSetting("MILPSolLimitInitial", "MILP");
}
