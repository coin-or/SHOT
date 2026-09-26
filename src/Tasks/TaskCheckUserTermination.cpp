/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckUserTermination.h"

#include "../CallbackData.h"
#include "../EventHandler.h"
#include "../Output.h"
#include "../Results.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include <any>

namespace SHOT
{

TaskCheckUserTermination::TaskCheckUserTermination(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckUserTermination::~TaskCheckUserTermination() = default;

void TaskCheckUserTermination::run()
{
    // Check if user termination was requested by data provider callbacks
    if(!env->tasks->isTerminated() && env->events->hasDataProvider(E_EventType::UserTerminationCheck))
    {
        // Create termination callback data only when needed
        int iterationNumber = env->results->getNumberOfIterations();
        double timeElapsed = env->timing->getElapsedTime("Total");
        double currentDualBound = env->results->getCurrentDualBound();
        double currentPrimalBound = env->results->getPrimalBound();
        double relativeGap = env->results->getRelativeCurrentObjectiveGap();
        double absoluteGap = env->results->getAbsoluteCurrentObjectiveGap();

        TerminationCallbackData callbackData(iterationNumber, timeElapsed, currentDualBound, currentPrimalBound,
            relativeGap, absoluteGap, env->solutionStatistics);

        auto shouldTerminate = env->events->requestData<bool>(E_EventType::UserTerminationCheck, callbackData);

        if(shouldTerminate.has_value() && *shouldTerminate)
        {
            env->output->outputInfo("        User termination check requested termination");
            env->tasks->terminate();
        }
    }

    if(env->tasks->isTerminated())
    {
        env->results->terminationReason = E_TerminationReason::UserAbort;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated by user.";
    }
    else if(env->results->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Abort)
    {
        // The dual solver was interrupted, but not because SHOT asked it to: the preceding gap, iteration and time
        // limit checks have all been passed. Since the solver will keep returning the same interrupted status there
        // is nothing to be gained from continuing, but this is not a user abort and must not be reported as one.
        env->results->terminationReason = E_TerminationReason::Error;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since the dual solver was interrupted.";
    }
}

std::string TaskCheckUserTermination::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT