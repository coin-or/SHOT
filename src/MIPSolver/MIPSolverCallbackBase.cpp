/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCallbackBase.h"

bool MIPSolverCallbackBase::checkIterationLimit()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (currIter->iterationNumber >= Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual") + Settings::getInstance().getIntSetting("IterationLimit", "Termination"))
    {
        return (true);
    }

    return (false);
}

bool MIPSolverCallbackBase::checkFixedNLPStrategy(SolutionPoint point)
{
    if (!Settings::getInstance().getBoolSetting("FixedInteger.Use", "Primal"))
    {
        return (false);
    }

    ProcessInfo::getInstance().startTimer("PrimalStrategy");
    ProcessInfo::getInstance().startTimer("PrimalBoundStrategyNLP");

    bool callNLPSolver = false;

    auto userSettingStrategy = Settings::getInstance().getIntSetting("FixedInteger.CallStrategy", "Primal");
    auto userSetting = Settings::getInstance().getIntSetting("FixedInteger.Source", "Primal");

    auto dualBound = ProcessInfo::getInstance().getDualBound();

    if (abs(point.objectiveValue - dualBound) / ((1e-10) + abs(dualBound)) < Settings::getInstance().getDoubleSetting("FixedInteger.DualPointGap.Relative", "Primal"))
    {
        callNLPSolver = true;
    }
    else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
    {
        callNLPSolver = true;
    }
    else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime) || userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
    {
        if (ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithoutNLPCallMIP >= Settings::getInstance().getIntSetting("FixedInteger.Frequency.Iteration", "Primal"))
        {
            Output::getInstance().outputInfo(
                "     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
            callNLPSolver = true;
        }
        else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().solutionStatistics.timeLastFixedNLPCall> Settings::getInstance().getDoubleSetting("FixedInteger.Frequency.Time", "Primal"))
        {
            Output::getInstance().outputInfo(
                "     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
            callNLPSolver = true;
        }
    }

    if (!callNLPSolver)
    {
        ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
    }

    ProcessInfo::getInstance().stopTimer("PrimalBoundStrategyNLP");
    ProcessInfo::getInstance().stopTimer("PrimalStrategy");

    return (callNLPSolver);
}

void MIPSolverCallbackBase::printIterationReport(SolutionPoint solution, std::string threadId, std::string bestBound, std::string openNodes)
{
    if (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber % 50 == 1)
    {
        tPrintIterationHeader->run();
    }

    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    std::stringstream tmpType;
    if (threadId != "")
    {
        tmpType << "CB (th: " << threadId << ")";
    }
    else
    {
        tmpType << "CB";
    }

    Output::getInstance().outputIterationDetail(currIter->iterationNumber,
                                                tmpType.str(),
                                                ProcessInfo::getInstance().getElapsedTime("Total"),
                                                this->lastNumAddedHyperplanes,
                                                currIter->totNumHyperplanes,
                                                ProcessInfo::getInstance().getDualBound(),
                                                ProcessInfo::getInstance().getPrimalBound(),
                                                ProcessInfo::getInstance().getAbsoluteObjectiveGap(),
                                                ProcessInfo::getInstance().getRelativeObjectiveGap(),
                                                solution.objectiveValue,
                                                solution.maxDeviation.idx,
                                                solution.maxDeviation.value);
}

MIPSolverCallbackBase::~MIPSolverCallbackBase()
{
}