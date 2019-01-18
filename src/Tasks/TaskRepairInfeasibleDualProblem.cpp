/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskRepairInfeasibleDualProblem.h"

namespace SHOT
{

TaskRepairInfeasibleDualProblem::TaskRepairInfeasibleDualProblem(
    EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue), taskIDIfFalse(taskIDFalse)
{
}

TaskRepairInfeasibleDualProblem::~TaskRepairInfeasibleDualProblem() {}

void TaskRepairInfeasibleDualProblem::run()
{
    env->timing->startTimer("DualStrategy");

    auto currIter = env->results->getCurrentIteration();

    if(currIter->solutionStatus != E_ProblemSolutionStatus::Infeasible)
    {
        env->tasks->setNextTask(taskIDIfTrue);
        return;
    }

    currIter->hasInfeasibilityRepairBeenPerformed = true;

    if(env->dualSolver->MIPSolver->repairInfeasibility())
    {
        env->tasks->setNextTask(taskIDIfTrue);
        iterLastRepair = currIter->iterationNumber;

        env->results->setDualBound(env->dualSolver->MIPSolver->getDualObjectiveValue());

        currIter->wasInfeasibilityRepairSuccessful = true;

        // currIter->solutionStatus = env->dualSolver->MIPSolver->solveProblem();
        // currIter->solutionPoints = env->dualSolver->MIPSolver->getAllVariableSolutions();
    }
    else if(mainRepairTries < 1)
    {
        currIter->wasInfeasibilityRepairSuccessful = false;
        env->dualSolver->cutOffToUse = env->results->getPrimalBound();
        env->tasks->setNextTask(taskIDIfTrue);
        mainRepairTries++;
    }
    else
    {
        currIter->wasInfeasibilityRepairSuccessful = false;
        env->tasks->setNextTask(taskIDIfFalse);
        mainRepairTries++;
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskRepairInfeasibleDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT