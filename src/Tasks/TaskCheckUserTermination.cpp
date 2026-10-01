/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckUserTermination.h"

#include "../Callback.h"
#include "../Output.h"
#include "../Results.h"
#include "../TaskHandler.h"
#include "../Timing.h"

namespace SHOT
{

TaskCheckUserTermination::TaskCheckUserTermination(EnvironmentPtr envPtr, std::string taskIDTrue, bool invokeCallbacks)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue), invokeCallbacks(invokeCallbacks)
{
}

TaskCheckUserTermination::~TaskCheckUserTermination() = default;

void TaskCheckUserTermination::run()
{
    // A termination requested by a callback is passed on to the task handler when the callbacks are invoked
    if(invokeCallbacks && !env->tasks->isTerminated() && env->callbacks->isActive(E_CallbackLocation::TerminationCheck))
    {
        auto context = std::make_shared<TerminationCheckContext>(env);
        env->callbacks->invoke(*context);
        context->invalidate();
    }

    if(env->tasks->isTerminated())
    {
        env->results->terminationReason = E_TerminationReason::UserAbort;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated by user.";
    }
    else if(invokeCallbacks && env->results->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Abort)
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