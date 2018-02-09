#include "MIPSolutionLimitStrategyIncrease.h"

MIPSolutionLimitStrategyIncrease::MIPSolutionLimitStrategyIncrease(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;

	lastIterSolLimIncreased = 1;
	numSolLimIncremented = 1;
	lastIterOptimal = 1;
	//currentLimit = Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual");
}

MIPSolutionLimitStrategyIncrease::~MIPSolutionLimitStrategyIncrease()
{
}

bool MIPSolutionLimitStrategyIncrease::updateLimit()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();
	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (!currIter->isMIP())
	{
		lastIterSolLimIncreased = currIter->iterationNumber;
		//lastIterOptimal = prevIter->iterationNumber;
		return (false);
	}

	if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		lastIterOptimal = prevIter->iterationNumber;
		return (false);
	}

	/*

	 if (prevIter->isMIP()  && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
	 {
	 return true;
	 }*/

	// Solution limit has not been updated in the maximal number of iterations
	if (prevIter->isMIP()
			&& (currIter->iterationNumber - lastIterSolLimIncreased
					> Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual")
					&& currIter->iterationNumber - lastIterOptimal
							> Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual")))
	{
		ProcessInfo::getInstance().outputInfo("     Force solution limit update.");
		return (true);
	}

	bool useObjectiveLinesearchUpdate = Settings::getInstance().getBoolSetting("ObjectiveLinesearch.Use", "Dual");

	if (prevIter->maxDeviationConstraint == -1 && useObjectiveLinesearchUpdate)
	{
		return (false);
	}

	// We have a feasible MIP solution to the original problem, but not proven optimal by MIP solver
	//if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation <  prevIter->usedConstraintTolerance)
	//TODO use the strategy for updated constraint tolerance
	/*if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
	 && (prevIter->maxDeviation
	 < Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual")
	 * max(1.0, abs(prevIter->objectiveValue))
	 || prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination")))
	 {
	 return (true);
	 }*/

	if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
	{
		if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual")) return (true);

		if (prevIter->maxDeviation < prevIter->usedConstraintTolerance) return (true);

		if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination")) return (true);

		/*std::cout << "test: " << prevIter->maxDeviationConstraint << " == "
		 << ProcessInfo::getInstance().originalProblem->getNonlinearObjectiveConstraintIdx() << ": "
		 << Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual") * max(1.0, abs(prevIter->objectiveValue))
		 << std::endl;*/

		if (prevIter->maxDeviationConstraint == -1
				&& prevIter->maxDeviation
						< Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual")
								* max(1.0, abs(prevIter->objectiveValue)))
		{
			/*std::cout << "updated nonlinear sol lim for "
			 << Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual")
			 * max(1.0, abs(prevIter->objectiveValue)) << std::endl;*/

			return (true);
		}
	}

	/*
	 &&(prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual")
	 || prevIter->maxDeviation < prevIter->usedConstraintTolerance
	 || prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
	 )
	 {
	 return (true);
	 }

	 if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
	 && (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual")
	 || prevIter->maxDeviation < prevIter->usedConstraintTolerance
	 || prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination")))
	 {
	 return (true);
	 }
	 */
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

	return (false);

}

int MIPSolutionLimitStrategyIncrease::getNewLimit()
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

	return (newLimit);
}

int MIPSolutionLimitStrategyIncrease::getInitialLimit()
{
	return (Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual"));
}
