/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckMaxNumberOfPrimalReductionCuts.h"

#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

namespace SHOT
{

TaskCheckMaxNumberOfPrimalReductionCuts::TaskCheckMaxNumberOfPrimalReductionCuts(
    EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckMaxNumberOfPrimalReductionCuts::~TaskCheckMaxNumberOfPrimalReductionCuts() = default;

void TaskCheckMaxNumberOfPrimalReductionCuts::run()
{
    if(env->solutionStatistics.numberOfPrimalReductionCutsUpdatesWithoutEffect
        >= env->settings->getSetting<int>("ReductionCut.MaxIterations", "Dual"))
    {
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReason = E_TerminationReason::ObjectiveStagnation;
        env->results->terminationReasonDescription
            = "Terminated since the maximal number of objective cutoffs have been reached.";
        return;
    }
}

std::string TaskCheckMaxNumberOfPrimalReductionCuts::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT