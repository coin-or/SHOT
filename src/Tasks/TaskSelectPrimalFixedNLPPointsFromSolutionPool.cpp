/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"

#include "../Model/Problem.h"

#include "../Iteration.h"
#include "../Results.h"
#include "../PrimalSolver.h"
#include "../Settings.h"
#include "../Timing.h"

namespace SHOT
{

TaskSelectPrimalFixedNLPPointsFromSolutionPool::TaskSelectPrimalFixedNLPPointsFromSolutionPool(
    EnvironmentPtr envPtr, bool isFinalPolish)
    : TaskBase(envPtr), isFinalPolish(isFinalPolish)
{
}

TaskSelectPrimalFixedNLPPointsFromSolutionPool::~TaskSelectPrimalFixedNLPPointsFromSolutionPool() = default;

void TaskSelectPrimalFixedNLPPointsFromSolutionPool::run()
{
    auto currIter = env->results->getCurrentIteration();
    auto sourceIter = currIter;

    // The final dual problem may have no solutions, e.g. if it is infeasible, so the polish then uses the latest
    // iteration with solutions
    if(isFinalPolish && currIter->solutionPoints.empty())
    {
        for(auto it = env->results->iterations.rbegin(); it != env->results->iterations.rend(); ++it)
        {
            if(!(*it)->solutionPoints.empty()
                && ((*it)->isMIP() || !env->reformulatedProblem->properties.isDiscrete))
            {
                sourceIter = *it;
                break;
            }
        }
    }

    auto allSolutions = sourceIter->solutionPoints;

    bool callNLPSolver = false;
    bool useFeasibleSolutionExtra = false;

    if((!sourceIter->isMIP() && env->reformulatedProblem->properties.isDiscrete) || allSolutions.empty())
        return;

    if(!isFinalPolish && currIter->MIPSolutionLimitUpdated
        && currIter->solutionStatus != E_ProblemSolutionStatus::Optimal)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
        return;
    }

    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    auto userSettingStrategy = env->settings->getSetting<int>("Primal.FixedInteger.CallStrategy");
    auto userSetting = env->settings->getSetting<int>("Primal.FixedInteger.Source");

    auto dualBound = env->results->getCurrentDualBound();

    if(isFinalPolish)
    {
        // The polish runs once, after the search, so the iteration- and time-based pacing below does not apply. It is
        // also used without a primal solution, since the NLP solver may find one where the search did not.
        callNLPSolver = true;
        useFeasibleSolutionExtra = true;
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Optimal
        && std::abs(allSolutions.at(0).objectiveValue - env->results->getCurrentDualBound())
                / ((1e-10) + std::abs(dualBound))
            < env->settings->getSetting<double>("Primal.FixedInteger.DualPointGap.Relative"))
    {
        callNLPSolver = true;
        useFeasibleSolutionExtra = true;
    }
    else if(userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
    {
        callNLPSolver = true;
    }
    else if(userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime)
        || userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
    {
        if(env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP
            >= env->settings->getSetting<int>("Primal.FixedInteger.Frequency.Iteration"))
        {
            env->output->outputDebug(
                "        Activating fixed NLP primal strategy since max iterations since last call has been reached.");
            callNLPSolver = true;
        }
        else if(env->timing->getElapsedTime("Total") - env->solutionStatistics.timeLastFixedNLPCall
            > env->settings->getSetting<double>("Primal.FixedInteger.Frequency.Time"))
        {
            env->output->outputDebug(
                "        Activating fixed NLP primal strategy since max time limit since last call has been reached.");
            callNLPSolver = true;
        }
    }

    if(useFeasibleSolutionExtra)
    {
        auto tmpSol = allSolutions.at(0);
        env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolutionNewDualBound,
            tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
    }
    else if(callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::SmallestDeviationSolution))
    {
        auto tmpSol = sourceIter->getSolutionPointWithSmallestDeviation();
        env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::SmallestDeviationSolution,
            tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
    }
    else if(callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::FirstSolution))
    {
        auto tmpSol = allSolutions.at(0);
        env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
            tmpSol.iterFound, tmpSol.maxDeviation);
    }
    else if(callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions))
    {
        auto tmpSol = allSolutions.at(0);
        env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
            tmpSol.iterFound, tmpSol.maxDeviation);

        auto smallestDevSolIdx = sourceIter->getSolutionPointWithSmallestDeviationIndex();

        if(smallestDevSolIdx != 0)
        {
            tmpSol = allSolutions.at(smallestDevSolIdx);
            env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
                tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
        }
    }
    else if(callNLPSolver
        && userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
    {
        auto tmpSol = allSolutions.at(0);

        env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
            tmpSol.iterFound, tmpSol.maxDeviation);

        for(size_t i = 1; i < allSolutions.size(); i++)
        {
            auto tmpSol = allSolutions.at(i);

            if(tmpSol.maxDeviation.value
                <= env->settings->getSetting<double>("Primal.Tolerance.NonlinearConstraint"))
            {
                env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
                    tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
            }
        }
    }
    else if(callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::AllSolutions))
    {
        auto tmpSol = allSolutions.at(0);

        env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
            tmpSol.iterFound, tmpSol.maxDeviation);

        for(size_t i = 1; i < allSolutions.size(); i++)
        {
            tmpSol = allSolutions.at(i);

            if(tmpSol.maxDeviation.value
                <= env->settings->getSetting<double>("Primal.Tolerance.NonlinearConstraint"))
            {
                env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
                    tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
            }
            else
            {
                env->primalSolver->addFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::InfeasibleSolution,
                    tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
            }
        }
    }
    else
    {
        env->timing->stopTimer("PrimalBoundStrategyNLP");
        env->timing->stopTimer("PrimalStrategy");
    }
}

std::string TaskSelectPrimalFixedNLPPointsFromSolutionPool::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT