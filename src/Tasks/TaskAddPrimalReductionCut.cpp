/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddPrimalReductionCut.h"

namespace SHOT
{

TaskAddPrimalReductionCut::TaskAddPrimalReductionCut(
    EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue), taskIDIfFalse(taskIDFalse)
{
}

TaskAddPrimalReductionCut::~TaskAddPrimalReductionCut() {}

void TaskAddPrimalReductionCut::run()
{
    auto currIter = env->results->getCurrentIteration(); // The solved iteration

    if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible && !currIter->wasInfeasibilityRepairSuccessful)
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    /*
        if(currIter->solutionStatus != E_ProblemSolutionStatus::Optimal)
        {
            env->tasks->setNextTask(taskIDIfFalse);
            return;
        }*/

    if(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect
        >= env->settings->getIntSetting("PrimalStagnation.MaxNumberOfPrimalCutReduction", "Termination"))
    {
        env->output->outputAlways("********No update since number of nonconvex objective cutoffs tried: "
            + std::to_string(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect));

        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    if(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect
        == env->settings->getIntSetting("PrimalStagnation.MaxNumberOfPrimalCutReduction", "Termination") - 1)
    {
        env->output->outputAlways("********Final update since number of nonconvex objective cutoffs tried: "
            + std::to_string(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect));

        if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
        {
            env->dualSolver->cutOffToUse = env->results->getPrimalBound();
            // env->dualSolver->cutOffToUse = 0.99 * env->dualSolver->cutOffToUse;
            env->results->currentDualBound = SHOT_DBL_MIN;
        }
        else
        {
            env->dualSolver->cutOffToUse = env->results->getPrimalBound();
            // env->dualSolver->cutOffToUse = 1.01 * env->dualSolver->cutOffToUse;
            env->results->currentDualBound = SHOT_DBL_MAX;
        }

        env->output->outputAlways("        New cutoff: " + std::to_string(env->dualSolver->cutOffToUse));

        // Want to solve the following subproblems to optimality
        //env->dualSolver->MIPSolver->setSolutionLimit(2100000000);

        env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect++;
        env->tasks->setNextTask(taskIDIfTrue);
        return;
    }

    env->output->outputAlways("********Number of nonconvex objective cutoffs tried: "
        + std::to_string(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect));

    if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
    {
        env->dualSolver->cutOffToUse = 0.99 * env->dualSolver->cutOffToUse;
        env->results->currentDualBound = SHOT_DBL_MIN;
    }
    else
    {
        env->dualSolver->cutOffToUse = 1.01 * env->dualSolver->cutOffToUse;
        env->results->currentDualBound = SHOT_DBL_MAX;
    }

    // menv->dualSolver->MIPSolver->setSolutionLimit(1 + env->dualSolver->MIPSolver->getSolutionLimit());
    env->output->outputAlways("        New cutoff: " + std::to_string(env->dualSolver->cutOffToUse));
    env->output->outputAlways(
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