/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckPrimalStagnation.h"

namespace SHOT
{

TaskCheckPrimalStagnation::TaskCheckPrimalStagnation(
    EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue), taskIDIfFalse(taskIDFalse)
{
}

TaskCheckPrimalStagnation::~TaskCheckPrimalStagnation() {}

void TaskCheckPrimalStagnation::run()
{
    /*
    if(env->solutionStatistics.numberOfIterationsWithPrimalStagnation == 0) // First MIP solution
    {
        env->tasks->setNextTask(taskIDIfFalse);
        env->solutionStatistics.numberOfIterationsWithPrimalStagnation++;
        return;
    }*/

    if(env->solutionStatistics.numberOfProblemsFeasibleMILP + env->solutionStatistics.numberOfProblemsOptimalMILP
        <= env->settings->getIntSetting("PrimalStagnation.IterationLimit", "Termination"))
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    auto currIter = env->results->getCurrentIteration();

    if(env->problem->properties.isDiscrete && !currIter->isMIP())
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    if(env->solutionStatistics.numberOfIterationsWithPrimalStagnation
        >= env->settings->getIntSetting("PrimalStagnation.IterationLimit", "Termination"))
    {
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReason = E_TerminationReason::ObjectiveStagnation;
        env->results->terminationReasonDescription = "Terminated since the primal bound has stagnated.";
        return;
    }

    env->solutionStatistics.numberOfIterationsWithPrimalStagnation++;
    env->tasks->setNextTask(taskIDIfFalse);
}

std::string TaskCheckPrimalStagnation::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT