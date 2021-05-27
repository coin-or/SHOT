/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddPrimalReductionCut.h"

#include "../DualSolver.h"
#include "../Enums.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"

#include "../Model/Problem.h"

namespace SHOT
{

TaskAddPrimalReductionCut::TaskAddPrimalReductionCut(
    EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue), taskIDIfFalse(taskIDFalse)
{
}

TaskAddPrimalReductionCut::~TaskAddPrimalReductionCut() = default;

void TaskAddPrimalReductionCut::run()
{
    if(env->tasks->isTerminated())
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    if(!env->results->hasPrimalSolution())
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    if(env->results->solutionIsGlobal
        && (env->results->isRelativeObjectiveGapToleranceMet() || env->results->isAbsoluteObjectiveGapToleranceMet()))
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    int maxIterations = env->settings->getSetting<int>("ReductionCut.MaxIterations", "Dual");

    if(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect >= maxIterations)
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    auto currIter = env->results->getCurrentIteration(); // The solved iteration

    if(currIter->forceObjectiveReductionCut) { }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible
        && !currIter->wasInfeasibilityRepairSuccessful)
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    double relativeGap = env->results->getRelativeCurrentObjectiveGap();

    // Different logic if gap is large
    if(relativeGap <= 1.0 && relativeGap > 0.1)
    {
        double factor
            = ((double)env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect) / (maxIterations + 1.0);

        env->dualSolver->cutOffToUse
            = factor * env->results->currentDualBound + (1 - factor) * env->results->currentPrimalBound;

        if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
        {
            env->results->currentDualBound = SHOT_DBL_MIN;
        }
        else
        {
            env->results->currentDualBound = SHOT_DBL_MAX;
        }
    }
    else
    {
        double reductionFactor = env->settings->getSetting<double>("ReductionCut.ReductionFactor", "Dual");

        if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
        {
            env->dualSolver->cutOffToUse
                = env->dualSolver->cutOffToUse - reductionFactor * std::abs(env->dualSolver->cutOffToUse);
            env->results->currentDualBound = SHOT_DBL_MIN;
        }
        else
        {
            env->dualSolver->cutOffToUse
                = env->dualSolver->cutOffToUse + reductionFactor * std::abs(env->dualSolver->cutOffToUse);
            env->results->currentDualBound = SHOT_DBL_MAX;
        }
    }

    std::stringstream tmpType;
    tmpType << "REDCUT-" << env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect + 1;

    env->report->outputIterationDetail(totalReductionCutUpdates + 1, tmpType.str(),
        env->timing->getElapsedTime("Total"), 0, 0, 0, env->dualSolver->cutOffToUse, 0, 0, 0, 0, currIter->maxDeviation,
        E_IterationLineType::DualReductionCut, true);

    env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect++;
    totalReductionCutUpdates++;
    env->solutionStatistics.numberOfPrimalReductionsPerformed++;
    env->solutionStatistics.hasReductionCutBeenAddedSincePrimalImprovement = true;

    env->tasks->setNextTask(taskIDIfTrue);
}

std::string TaskAddPrimalReductionCut::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT