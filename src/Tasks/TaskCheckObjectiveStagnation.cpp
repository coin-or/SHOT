/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckObjectiveStagnation.h"

namespace SHOT
{

TaskCheckObjectiveStagnation::TaskCheckObjectiveStagnation(EnvironmentPtr envPtr, std::string taskIDTrue) : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckObjectiveStagnation::~TaskCheckObjectiveStagnation()
{
}

void TaskCheckObjectiveStagnation::run()
{
    auto currIter = env->process->getCurrentIteration();

    if (!currIter->isMIP())
    {
        return;
    }

    if (env->solutionStatistics.numberOfProblemsFeasibleMILP + env->solutionStatistics.numberOfProblemsOptimalMILP <= env->settings->getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
    {
        return;
    }

    if (env->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate == 0) // First MIP solution
    {
        env->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate = currIter->iterationNumber;
        env->solutionStatistics.numberOfIterationsWithStagnationMIP = 0;
        return;
    }

    if (std::abs(
            (currIter->objectiveValue - env->process->iterations[env->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate - 1].objectiveValue)) > env->settings->getDoubleSetting("ObjectiveStagnation.Tolerance", "Termination"))
    {
        env->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate = currIter->iterationNumber;
        env->solutionStatistics.numberOfIterationsWithStagnationMIP = 0;

        return;
    }

    if (env->solutionStatistics.numberOfIterationsWithStagnationMIP >= env->settings->getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
    {
        env->process->terminationReason = E_TerminationReason::ObjectiveStagnation;
        env->tasks->setNextTask(taskIDIfTrue);
    }

    env->solutionStatistics.numberOfIterationsWithStagnationMIP++;
}

std::string TaskCheckObjectiveStagnation::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT