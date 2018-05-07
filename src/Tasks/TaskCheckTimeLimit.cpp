/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckTimeLimit.h"

TaskCheckTimeLimit::TaskCheckTimeLimit(std::string taskIDTrue)
{
    taskIDIfTrue = taskIDTrue;
}

TaskCheckTimeLimit::~TaskCheckTimeLimit()
{
}

void TaskCheckTimeLimit::run()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (ProcessInfo::getInstance().getElapsedTime("Total") >= Settings::getInstance().getDoubleSetting("TimeLimit", "Termination"))
    {
        ProcessInfo::getInstance().terminationReason = E_TerminationReason::TimeLimit;
        ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckTimeLimit::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
