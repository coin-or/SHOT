/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckConstraintTolerance.h"

TaskCheckConstraintTolerance::TaskCheckConstraintTolerance(
    std::string taskIDTrue)
{
    taskIDIfTrue = taskIDTrue;
}

TaskCheckConstraintTolerance::~TaskCheckConstraintTolerance() {}

void TaskCheckConstraintTolerance::run()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (currIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination") &&
        currIter->solutionStatus == E_ProblemSolutionStatus::Optimal &&
        currIter->type == E_IterationProblemType::MIP)
    {
        ProcessInfo::getInstance().terminationReason = E_TerminationReason::ConstraintTolerance;
        ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
    }

    return;
}

std::string TaskCheckConstraintTolerance::getType()
{
    std::string type = typeid(this).name();
    return (type);
}