/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckDualStagnation.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include "../Model/Problem.h"

namespace SHOT
{

TaskCheckDualStagnation::TaskCheckDualStagnation(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckDualStagnation::~TaskCheckDualStagnation() = default;

void TaskCheckDualStagnation::run()
{
    return;
    auto currIter = env->results->getCurrentIteration();

    if(env->problem->properties.isDiscrete && !currIter->isMIP())
    {
        return;
    }

    // To avoid unnecessary termination when there are many subsequent dual problems with the same objective value
    // but different nonlinear constraint errors
    if(env->results->getNumberOfIterations() > 1
        && std ::abs(currIter->maxDeviation - env->results->getPreviousIteration()->maxDeviation)
            > env->settings->getSetting<double>("DualStagnation.ConstraintTolerance", "Termination")
        && currIter->iterationNumber - env->solutionStatistics.iterationLastDualCutAdded < 5)
    {
        return;
    }

    if(!env->dualSolver->isSingleTree && !currIter->MIPSolutionLimitUpdated
        && currIter->iterationNumber - env->solutionStatistics.iterationLastDualCutAdded > 2
        && currIter->solutionStatus != E_ProblemSolutionStatus::SolutionLimit)
    {
        env->results->terminationReason = E_TerminationReason::NoDualCutsAdded;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since no additional dual cuts can be added.";
    }

    if(env->solutionStatistics.numberOfIterationsWithDualStagnation
        >= env->settings->getSetting<int>("DualStagnation.IterationLimit", "Termination"))
    {
        env->results->terminationReason = E_TerminationReason::ObjectiveStagnation;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since the dual bound has stagnated.";
    }

    env->solutionStatistics.numberOfIterationsWithDualStagnation++;
}

std::string TaskCheckDualStagnation::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT