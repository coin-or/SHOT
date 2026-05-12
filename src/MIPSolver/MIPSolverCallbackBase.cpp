/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCallbackBase.h"
#include "../CallbackData.h"
#include "../EventHandler.h"
#include "../Output.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include <any>
#include <optional>

namespace SHOT
{

bool MIPSolverCallbackBase::checkIterationLimit()
{
    if(env->tasks->isTerminated())
        return (true);

    auto mainlimit = env->settings->getSetting<int>("Termination.IterationLimit");

    if(mainlimit == SHOT_INT_MAX)
        return (false);

    auto currIter = env->results->getCurrentIteration();

    if(currIter->iterationNumber >= mainlimit)
        return (true);

    return (false);
}

bool MIPSolverCallbackBase::checkUserTermination()
{
    // Check if user termination was requested by data provider callbacks
    if(!env->tasks->isTerminated() && env->events->hasDataProvider(E_EventType::UserTerminationCheck))
    {
        // Create termination callback data only when needed
        int iterationNumber = env->results->getNumberOfIterations();
        double timeElapsed = env->timing->getElapsedTime("Total");
        double currentDualBound = env->results->getCurrentDualBound();
        double currentPrimalBound = env->results->getPrimalBound();
        double relativeGap = env->results->getRelativeCurrentObjectiveGap();
        double absoluteGap = env->results->getAbsoluteCurrentObjectiveGap();

        TerminationCallbackData callbackData(iterationNumber, timeElapsed, currentDualBound, currentPrimalBound,
            relativeGap, absoluteGap, env->solutionStatistics);

        auto shouldTerminate = env->events->requestData<bool>(E_EventType::UserTerminationCheck, callbackData);

        if(shouldTerminate.has_value() && *shouldTerminate)
            env->tasks->terminate();
    }

    if(env->tasks->isTerminated())
    {
        return (true);
    }

    return (false);
}

bool MIPSolverCallbackBase::checkFixedNLPStrategy(SolutionPoint point)
{
    if(!env->settings->getSetting<bool>("Primal.FixedInteger.Use"))
    {
        return (false);
    }

    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    bool callNLPSolver = false;

    auto userSettingStrategy = env->settings->getSetting<int>("Primal.FixedInteger.CallStrategy");

    auto dualBound = env->results->getCurrentDualBound();

    if(std::abs(point.objectiveValue - dualBound) / ((1e-10) + std::abs(dualBound))
        < env->settings->getSetting<double>("Primal.FixedInteger.DualPointGap.Relative"))
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

    if(!callNLPSolver)
    {
        env->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
    }

    env->timing->stopTimer("PrimalBoundStrategyNLP");
    env->timing->stopTimer("PrimalStrategy");

    return (callNLPSolver);
}

std::optional<double> MIPSolverCallbackBase::queryAndUpdateExternalDualBound()
{
    if(!env->events->hasDataProvider(E_EventType::ExternalDualBound))
        return std::nullopt;

    bool isMin = env->reformulatedProblem->objectiveFunction->properties.isMinimize;
    double currentDualBound = env->results->getCurrentDualBound();
    double currentPrimalBound = env->results->getPrimalBound();
    int iterationNumber = env->results->getNumberOfIterations();
    double absoluteGap = env->results->getAbsoluteCurrentObjectiveGap();
    double relativeGap = env->results->getRelativeCurrentObjectiveGap();

    DualBoundCallbackData callbackData(isMin, currentDualBound, currentPrimalBound, iterationNumber, relativeGap,
        absoluteGap, env->solutionStatistics);

    auto externalDualBound = env->events->requestData<double>(E_EventType::ExternalDualBound, callbackData);

    if(!externalDualBound.has_value() || std::isnan(*externalDualBound))
        return std::nullopt;

    double newBound = *externalDualBound;

    bool isBoundImproved = isMin ? (std::isnan(currentDualBound) || newBound > currentDualBound)
                                 : (std::isnan(currentDualBound) || newBound < currentDualBound);

    if(!isBoundImproved)
        return std::nullopt;

    env->results->setDualBound(newBound);
    env->solutionStatistics.hasExternalDualBoundBeenSet = true;
    env->output->outputInfo(
        fmt::format("        Updated dual bound from external provider (single-tree): {}", newBound));

    return newBound;
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