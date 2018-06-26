/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"

TaskSelectPrimalFixedNLPPointsFromSolutionPool::TaskSelectPrimalFixedNLPPointsFromSolutionPool()
{
}

TaskSelectPrimalFixedNLPPointsFromSolutionPool::~TaskSelectPrimalFixedNLPPointsFromSolutionPool()
{
}

void TaskSelectPrimalFixedNLPPointsFromSolutionPool::run()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();
    auto allSolutions = ProcessInfo::getInstance().getCurrentIteration()->solutionPoints;

    bool callNLPSolver = false;
    bool useFeasibleSolutionExtra = false;

    if (!currIter->isMIP())
    {
        return;
    }

    if (allSolutions.size() == 0)
    {
        return;
    }

    if (currIter->MIPSolutionLimitUpdated && currIter->solutionStatus != E_ProblemSolutionStatus::Optimal)
    {
        ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return;
    }

    ProcessInfo::getInstance().startTimer("PrimalStrategy");
    ProcessInfo::getInstance().startTimer("PrimalBoundStrategyNLP");

    auto userSettingStrategy = Settings::getInstance().getIntSetting("FixedInteger.CallStrategy", "Primal");
    auto userSetting = Settings::getInstance().getIntSetting("FixedInteger.Source", "Primal");

    auto dualBound = ProcessInfo::getInstance().getDualBound();

    if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal &&
        abs(allSolutions.at(0).objectiveValue - ProcessInfo::getInstance().getDualBound()) / ((1e-10) + abs(dualBound)) < Settings::getInstance().getDoubleSetting("FixedInteger.DualPointGap.Relative", "Primal"))
    {
        callNLPSolver = true;
        useFeasibleSolutionExtra = true;
    }
    else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
    {
        callNLPSolver = true;
    }
    else if (ProcessInfo::getInstance().terminationReason != E_TerminationReason::None)
    {
        callNLPSolver = true; // Call the NLP solver at last iteration.
    }
    else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime) || userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
    {
        if (ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithoutNLPCallMIP >= Settings::getInstance().getIntSetting("FixedInteger.Frequency.Iteration", "Primal"))
        {
            Output::getInstance().outputInfo(
                "     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
            callNLPSolver = true;
        }
        else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().solutionStatistics.timeLastFixedNLPCall > Settings::getInstance().getDoubleSetting("FixedInteger.Frequency.Time", "Primal"))
        {
            Output::getInstance().outputInfo(
                "     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
            callNLPSolver = true;
        }
    }

    if (useFeasibleSolutionExtra)
    {
        auto tmpSol = allSolutions.at(0);
        ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point,
                                                              E_PrimalNLPSource::FirstSolutionNewDualBound, tmpSol.objectiveValue, tmpSol.iterFound,
                                                              tmpSol.maxDeviation);
    }
    else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::SmallestDeviationSolution))
    {
        auto tmpSol = currIter->getSolutionPointWithSmallestDeviation();
        ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point,
                                                              E_PrimalNLPSource::SmallestDeviationSolution, tmpSol.objectiveValue, tmpSol.iterFound,
                                                              tmpSol.maxDeviation);
    }
    else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::FirstSolution))
    {
        auto tmpSol = allSolutions.at(0);
        ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
                                                              tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
    }
    else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions))
    {
        auto tmpSol = allSolutions.at(0);
        ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
                                                              tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

        auto smallestDevSolIdx = currIter->getSolutionPointWithSmallestDeviationIndex();

        if (smallestDevSolIdx != 0)
        {
            tmpSol = allSolutions.at(smallestDevSolIdx);
            ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
                                                                  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
        }
    }
    else if (callNLPSolver && userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
    {
        auto tmpSol = allSolutions.at(0);

        ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
                                                              tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

        for (int i = 1; i < allSolutions.size(); i++)
        {
            auto tmpSol = allSolutions.at(i);

            if (tmpSol.maxDeviation.value <= Settings::getInstance().getDoubleSetting("Tolerance.NonlinearConstraint", "Primal"))
            {
                ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
                                                                      tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
            }
        }
    }
    else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::AllSolutions))
    {
        auto tmpSol = allSolutions.at(0);

        ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
                                                              tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

        for (int i = 1; i < allSolutions.size(); i++)
        {
            tmpSol = allSolutions.at(i);

            if (tmpSol.maxDeviation.value <= Settings::getInstance().getDoubleSetting("Tolerance.NonlinearConstraint", "Primal"))
            {
                ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
                                                                      tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
            }
            else
            {
                ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point,
                                                                      E_PrimalNLPSource::InfeasibleSolution, tmpSol.objectiveValue, tmpSol.iterFound,
                                                                      tmpSol.maxDeviation);
            }
        }
    }
    else
    {
        ProcessInfo::getInstance().stopTimer("PrimalBoundStrategyNLP");
        ProcessInfo::getInstance().stopTimer("PrimalStrategy");
    }
}

std::string TaskSelectPrimalFixedNLPPointsFromSolutionPool::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
