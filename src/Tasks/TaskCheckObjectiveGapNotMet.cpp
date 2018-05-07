/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckObjectiveGapNotMet.h"

TaskCheckObjectiveGapNotMet::TaskCheckObjectiveGapNotMet(std::string taskIDTrue)
{
    taskIDIfTrue = taskIDTrue;
}

TaskCheckObjectiveGapNotMet::~TaskCheckObjectiveGapNotMet()
{
}

void TaskCheckObjectiveGapNotMet::run()
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (ProcessInfo::getInstance().primalSolutions.size() > 0 &&
        !ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet() &&
        !ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
    {
        ProcessInfo::getInstance().terminationReason = E_TerminationReason::ObjectiveGapNotReached;
        ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckObjectiveGapNotMet::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
