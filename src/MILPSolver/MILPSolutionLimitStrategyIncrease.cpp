#include "MILPSolutionLimitStrategyIncrease.h"

MILPSolutionLimitStrategyIncrease::MILPSolutionLimitStrategyIncrease(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;

	lastIterSolLimIncreased = 1;
	numSolLimIncremented = 1;
	lastIterOptimal = 1;
	//currentLimit = Settings::getInstance().getIntSetting("MILPSolLimitInitial", "MILP");
}

MILPSolutionLimitStrategyIncrease::~MILPSolutionLimitStrategyIncrease()
{
}

bool MILPSolutionLimitStrategyIncrease::updateLimit()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (!currIter->isMILP())
	{
		lastIterSolLimIncreased = currIter->iterationNumber;
		//lastIterOptimal = prevIter->iterationNumber;
		return (false);
	}

	if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		lastIterOptimal = prevIter->iterationNumber;
		return (false);
	}

	/*

	 if (prevIter->isMILP()  && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	 {
	 return true;
	 }*/

	// Solution limit has not been updated in the maximal number of iterations
	if (prevIter->isMILP()
			&& (currIter->iterationNumber - lastIterSolLimIncreased
					> Settings::getInstance().getIntSetting("MILPSolIncreaseIter", "MILP")
					&& currIter->iterationNumber - lastIterOptimal
							> Settings::getInstance().getIntSetting("MILPSolIncreaseIter", "MILP")))
	{
		ProcessInfo::getInstance().outputInfo("     Force solution limit update.");
		return (true);
	}

	bool useObjectiveLinesearchUpdate = Settings::getInstance().getBoolSetting("UseObjectiveLinesearch", "PrimalBound");

	if (prevIter->maxDeviationConstraint == -1 && useObjectiveLinesearchUpdate)
	{
		return (false);
	}

	// We have a feasible MILP solution to the original problem, but not proven optimal by MILP solver
	//if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  prevIter->usedConstraintTolerance)
	//TODO use the strategy for updated constraint tolerance
	/*if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
	 && (prevIter->maxDeviation
	 < Settings::getInstance().getDoubleSetting("MILPSolLimitUpdateTol", "MILP")
	 * max(1.0, abs(prevIter->objectiveValue))
	 || prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm")))
	 {
	 return (true);
	 }*/

	if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
	{
		if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MILPSolLimitUpdateTol", "MILP")) return (true);

		if (prevIter->maxDeviation < prevIter->usedConstraintTolerance) return (true);

		if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm")) return (true);

		/*std::cout << "test: " << prevIter->maxDeviationConstraint << " == "
		 << ProcessInfo::getInstance().originalProblem->getNonlinearObjectiveConstraintIdx() << ": "
		 << Settings::getInstance().getDoubleSetting("MILPSolLimitUpdateTol", "MILP") * max(1.0, abs(prevIter->objectiveValue))
		 << std::endl;*/

		if (prevIter->maxDeviationConstraint == -1
				&& prevIter->maxDeviation
						< Settings::getInstance().getDoubleSetting("MILPSolLimitUpdateTol", "MILP")
								* max(1.0, abs(prevIter->objectiveValue)))
		{
			/*std::cout << "updated nonlinear sol lim for "
			 << Settings::getInstance().getDoubleSetting("MILPSolLimitUpdateTol", "MILP")
			 * max(1.0, abs(prevIter->objectiveValue)) << std::endl;*/

			return (true);
		}
	}

	/*
	 &&(prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MILPSolLimitUpdateTol", "MILP")
	 || prevIter->maxDeviation < prevIter->usedConstraintTolerance
	 || prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
	 )
	 {
	 return (true);
	 }

	 if (prevIter->isMILP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
	 && (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MILPSolLimitUpdateTol", "MILP")
	 || prevIter->maxDeviation < prevIter->usedConstraintTolerance
	 || prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm")))
	 {
	 return (true);
	 }
	 */
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

	return (false);

}

int MILPSolutionLimitStrategyIncrease::getNewLimit()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	int newLimit;
//int iterLargeIncrease = Settings::getInstance().getIntSetting("MILPSolIncreaseIter", "MILP");

	newLimit = MILPSolver->getSolutionLimit() + 1;
	lastIterSolLimIncreased = currIter->iterationNumber;
// Update MILP solution limit
//if (numSolLimIncremented > Settings::getInstance().getIntSetting("MILPSolIncreaseIter", "MILP")) // Force larger MILP solution limit update
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

	return (newLimit);
}

int MILPSolutionLimitStrategyIncrease::getInitialLimit()
{
	return (Settings::getInstance().getIntSetting("MILPSolLimitInitial", "MILP"));
}
