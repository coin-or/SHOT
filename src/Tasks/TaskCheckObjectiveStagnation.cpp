/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckObjectiveStagnation.h"

TaskCheckObjectiveStagnation::TaskCheckObjectiveStagnation(std::string taskIDTrue)
{

    taskIDIfTrue = taskIDTrue;
}

TaskCheckObjectiveStagnation::~TaskCheckObjectiveStagnation()
{
}

void TaskCheckObjectiveStagnation::run()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (!currIter->isMIP())
    {
        return;
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFeasibleMILP + ProcessInfo::getInstance().solutionStatistics.numberOfProblemsOptimalMILP <= Settings::getInstance().getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
    {
        return;
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate == 0) // First MIP solution
    {
        ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate = currIter->iterationNumber;
        ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithStagnationMIP= 0;
        return;
    }

    if (std::abs(
            (currIter->objectiveValue - ProcessInfo::getInstance().iterations[ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate - 1].objectiveValue)) > Settings::getInstance().getDoubleSetting("ObjectiveStagnation.Tolerance", "Termination"))
    {
        ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate = currIter->iterationNumber;
        ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithStagnationMIP= 0;

        return;
    }

    if (ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithStagnationMIP>= Settings::getInstance().getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
    {
        ProcessInfo::getInstance().terminationReason = E_TerminationReason::ObjectiveStagnation;
        ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
    }

    ProcessInfo::getInstance().solutionStatistics.numberOfIterationsWithStagnationMIP++;
}

std::string TaskCheckObjectiveStagnation::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
