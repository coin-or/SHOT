/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCallbackBase.h"
#include "../EventHandler.h"
#include "../Iteration.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include <any>

namespace SHOT
{

bool MIPSolverCallbackBase::checkIterationLimit()
{
    if(env->tasks->isTerminated())
        return (true);

    auto mainlimit = env->settings->getSetting<int>("IterationLimit", "Termination");

    if(mainlimit == SHOT_INT_MAX)
        return (false);

    auto currIter = env->results->getCurrentIteration();

    if(currIter->iterationNumber >= mainlimit)
        return (true);

    return (false);
}

bool MIPSolverCallbackBase::checkUserTermination()
{
    env->events->notify(E_EventType::UserTerminationCheck, std::any());

    if(env->tasks->isTerminated())
        return (true);

    return (false);
}

bool MIPSolverCallbackBase::checkFixedNLPStrategy(SolutionPoint point)
{
    if(!env->settings->getSetting<bool>("FixedInteger.Use", "Primal"))
    {
        return (false);
    }

    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    bool callNLPSolver = false;

    auto userSettingStrategy = env->settings->getSetting<int>("FixedInteger.CallStrategy", "Primal");

    auto dualBound = env->results->getCurrentDualBound();

    if(std::abs(point.objectiveValue - dualBound) / ((1e-10) + std::abs(dualBound))
        < env->settings->getSetting<double>("FixedInteger.DualPointGap.Relative", "Primal"))
    {
        callNLPSolver = true;
    }
    else if(userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
    {
        callNLPSolver = true;
    }
    else if(userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime)
        || userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
    {
        if(env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP
            >= env->settings->getSetting<int>("FixedInteger.Frequency.Iteration", "Primal"))
        {
            env->output->outputDebug(
                "        Activating fixed NLP primal strategy since max iterations since last call has been reached.");
            callNLPSolver = true;
        }
        else if(env->timing->getElapsedTime("Total") - env->solutionStatistics.timeLastFixedNLPCall
            > env->settings->getSetting<double>("FixedInteger.Frequency.Time", "Primal"))
        {
            env->output->outputDebug(
                "        Activating fixed NLP primal strategy since max time limit since last call has been reached.");
            callNLPSolver = true;
        }
    }

    if(!callNLPSolver)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
    }

    env->timing->stopTimer("PrimalBoundStrategyNLP");
    env->timing->stopTimer("PrimalStrategy");

    return (callNLPSolver);
}

void MIPSolverCallbackBase::printIterationReport(SolutionPoint solution, std::string threadId)
{
    auto currIter = env->results->getCurrentIteration();

    std::stringstream tmpType;
    if(threadId != "")
    {
        tmpType << "CB (th: " << threadId << ")";
    }
    else
    {
        tmpType << "CB";
    }

    env->report->outputIterationDetail(currIter->iterationNumber, tmpType.str(), env->timing->getElapsedTime("Total"),
        this->lastNumAddedHyperplanes, currIter->totNumHyperplanes, env->results->getCurrentDualBound(),
        env->results->getPrimalBound(), env->results->getAbsoluteGlobalObjectiveGap(),
        env->results->getRelativeGlobalObjectiveGap(), solution.objectiveValue, solution.maxDeviation.index,
        solution.maxDeviation.value, E_IterationLineType::DualCallback);

    this->lastNumAddedHyperplanes = 0;
}

} // namespace SHOT