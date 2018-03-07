/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolutionLimitStrategyIncrease.h"

MIPSolutionLimitStrategyIncrease::MIPSolutionLimitStrategyIncrease(IMIPSolver *MIPSolver)
{
    this->MIPSolver = MIPSolver;

    lastIterSolLimIncreased = 1;
    numSolLimIncremented = 1;
    lastIterOptimal = 1;
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

    // Solution limit has not been updated in the maximal number of iterations
    if (prevIter->isMIP() && (currIter->iterationNumber - lastIterSolLimIncreased > Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual") && currIter->iterationNumber - lastIterOptimal > Settings::getInstance().getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual")))
    {
        ProcessInfo::getInstance().outputInfo("     Force solution limit update.");
        return (true);
    }

    bool useObjectiveLinesearchUpdate = Settings::getInstance().getBoolSetting("ObjectiveLinesearch.Use", "Dual");

    if (prevIter->maxDeviationConstraint == -1 && useObjectiveLinesearchUpdate)
    {
        return (false);
    }

    if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual"))
            return (true);

        if (prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
            return (true);

        if (prevIter->maxDeviationConstraint == -1 && prevIter->maxDeviation < Settings::getInstance().getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual") * max(1.0, abs(prevIter->objectiveValue)))
        {
            return (true);
        }
    }

    return (false);
}

int MIPSolutionLimitStrategyIncrease::getNewLimit()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    int newLimit;

    newLimit = MIPSolver->getSolutionLimit() + 1;
    lastIterSolLimIncreased = currIter->iterationNumber;

    return (newLimit);
}

int MIPSolutionLimitStrategyIncrease::getInitialLimit()
{
    return (Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual"));
}
