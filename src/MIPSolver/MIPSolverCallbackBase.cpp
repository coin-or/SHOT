/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCallbackBase.h"

namespace SHOT
{

bool MIPSolverCallbackBase::checkIterationLimit()
{
    auto currIter = env->results->getCurrentIteration();

    if(currIter->iterationNumber >= env->settings->getIntSetting("Relaxation.IterationLimit", "Dual")
            + env->settings->getIntSetting("IterationLimit", "Termination"))
    {
        return (true);
    }

    return (false);
}

bool MIPSolverCallbackBase::checkFixedNLPStrategy(SolutionPoint point)
{
    if(!env->settings->getBoolSetting("FixedInteger.Use", "Primal"))
    {
        return (false);
    }

    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    bool callNLPSolver = false;

    auto userSettingStrategy = env->settings->getIntSetting("FixedInteger.CallStrategy", "Primal");
    auto userSetting = env->settings->getIntSetting("FixedInteger.Source", "Primal");

    auto dualBound = env->results->getDualBound();

    if(abs(point.objectiveValue - dualBound) / ((1e-10) + abs(dualBound))
        < env->settings->getDoubleSetting("FixedInteger.DualPointGap.Relative", "Primal"))
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
            >= env->settings->getIntSetting("FixedInteger.Frequency.Iteration", "Primal"))
        {
            env->output->outputInfo(
                "     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
            callNLPSolver = true;
        }
        else if(env->timing->getElapsedTime("Total") - env->solutionStatistics.timeLastFixedNLPCall
            > env->settings->getDoubleSetting("FixedInteger.Frequency.Time", "Primal"))
        {
            env->output->outputInfo(
                "     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
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
        this->lastNumAddedHyperplanes, currIter->totNumHyperplanes, env->results->getDualBound(),
        env->results->getPrimalBound(), env->results->getAbsoluteObjectiveGap(),
        env->results->getRelativeObjectiveGap(), solution.objectiveValue, solution.maxDeviation.index,
        solution.maxDeviation.value, E_IterationLineType::DualCallback);

    this->lastNumAddedHyperplanes = 0;
}

MIPSolverCallbackBase::~MIPSolverCallbackBase() {}
} // namespace SHOT