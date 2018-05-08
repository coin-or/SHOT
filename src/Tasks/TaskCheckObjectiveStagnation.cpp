/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckObjectiveStagnation.h"

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

    if (env->process->solutionStatistics.numberOfProblemsFeasibleMILP + env->process->solutionStatistics.numberOfProblemsOptimalMILP <= env->settings->getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
    {
        return;
    }

    if (env->process->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate == 0) // First MIP solution
    {
        env->process->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate = currIter->iterationNumber;
        env->process->solutionStatistics.numberOfIterationsWithStagnationMIP = 0;
        return;
    }

    if (std::abs(
            (currIter->objectiveValue - env->process->iterations[env->process->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate - 1].objectiveValue)) > env->settings->getDoubleSetting("ObjectiveStagnation.Tolerance", "Termination"))
    {
        env->process->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate = currIter->iterationNumber;
        env->process->solutionStatistics.numberOfIterationsWithStagnationMIP = 0;

        return;
    }

    if (env->process->solutionStatistics.numberOfIterationsWithStagnationMIP >= env->settings->getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
    {
        env->process->terminationReason = E_TerminationReason::ObjectiveStagnation;
        env->process->tasks->setNextTask(taskIDIfTrue);
    }

    env->process->solutionStatistics.numberOfIterationsWithStagnationMIP++;
}

std::string TaskCheckObjectiveStagnation::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
