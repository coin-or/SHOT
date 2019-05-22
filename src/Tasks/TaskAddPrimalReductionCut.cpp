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
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"

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

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints == 0)
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    auto currIter = env->results->getCurrentIteration(); // The solved iteration

    if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible && !currIter->wasInfeasibilityRepairSuccessful)
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

    if(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect
        >= env->settings->getSetting<int>("PrimalStagnation.MaxNumberOfPrimalCutReduction", "Termination"))
    {
        env->output->outputCritical("********No update since number of nonconvex objective cutoffs tried: "
            + std::to_string(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect));

        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    if(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect
        == env->settings->getSetting<int>("PrimalStagnation.MaxNumberOfPrimalCutReduction", "Termination") - 1)
    {
        env->output->outputCritical("********Final update since number of nonconvex objective cutoffs tried: "
            + std::to_string(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect));

        double reductionFactor = 0.01;

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

        env->output->outputCritical("        New cutoff: " + std::to_string(env->dualSolver->cutOffToUse));

        // Want to solve the following subproblems to optimality
        // env->dualSolver->MIPSolver->setSolutionLimit(2100000000);

        env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect++;
        env->tasks->setNextTask(taskIDIfTrue);
        return;
    }

    env->output->outputCritical("********Number of nonconvex objective cutoffs tried: "
        + std::to_string(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect));

    if(env->reformulatedProblem->objectiveFunction->properties.isMinimize && env->dualSolver->cutOffToUse > 0)
    {
        env->dualSolver->cutOffToUse = 0.999 * env->dualSolver->cutOffToUse;
        env->results->currentDualBound = SHOT_DBL_MIN;
    }
    else
    {
        env->dualSolver->cutOffToUse = 1.001 * env->dualSolver->cutOffToUse;
        env->results->currentDualBound = SHOT_DBL_MAX;
    }

    // menv->dualSolver->MIPSolver->setSolutionLimit(1 + env->dualSolver->MIPSolver->getSolutionLimit());
    env->output->outputCritical("        New cutoff: " + std::to_string(env->dualSolver->cutOffToUse));
    env->output->outputCritical(
        "        New solution limit: " + std::to_string(env->dualSolver->MIPSolver->getSolutionLimit()));

    env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect++;
    env->tasks->setNextTask(taskIDIfTrue);
}

std::string TaskAddPrimalReductionCut::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT