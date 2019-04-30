/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckDualStagnation.h"

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
    if(env->solutionStatistics.numberOfIterationsWithDualStagnation == 0) // First MIP solution
    {
        return;
    }

    if(env->solutionStatistics.numberOfProblemsFeasibleMILP + env->solutionStatistics.numberOfProblemsOptimalMILP
        <= env->settings->getSetting<int>("DualObjectiveStagnation.IterationLimit", "Termination"))
    {
        return;
    }

    auto currIter = env->results->getCurrentIteration();

    if(env->problem->properties.isDiscrete && !currIter->isMIP())
    {
        return;
    }

    if(env->solutionStatistics.numberOfIterationsWithDualStagnation
        >= env->settings->getSetting<int>("DualObjectiveStagnation.IterationLimit", "Termination"))
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