/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckObjectiveGapNotMet.h"

namespace SHOT
{

TaskCheckObjectiveGapNotMet::TaskCheckObjectiveGapNotMet(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckObjectiveGapNotMet::~TaskCheckObjectiveGapNotMet() {}

void TaskCheckObjectiveGapNotMet::run()
{
    auto currIter = env->results->getCurrentIteration();

    if(currIter->type != E_IterationProblemType::MIP)
        return;

    if(env->results->primalSolutions.size() > 0 && !env->results->isRelativeObjectiveGapToleranceMet()
        && !env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        env->results->terminationReason = E_TerminationReason::ObjectiveGapNotReached;
        env->tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckObjectiveGapNotMet::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT
