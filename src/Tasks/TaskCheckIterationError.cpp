/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckIterationError.h"

#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

namespace SHOT
{

TaskCheckIterationError::TaskCheckIterationError(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckIterationError::~TaskCheckIterationError() = default;

void TaskCheckIterationError::run()
{
    auto currIter = env->results->getCurrentIteration();

    if(currIter->solutionStatus == E_ProblemSolutionStatus::Error)
    {
        env->results->terminationReason = E_TerminationReason::Error;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since an error occured.";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible && env->results->hasPrimalSolution())
    {
        env->results->terminationReason = E_TerminationReason::ObjectiveGapNotReached;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription
            = "Terminated since the specified objective gap tolerance could not be met.";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        env->results->terminationReason = E_TerminationReason::InfeasibleProblem;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since the problem is infeasible.";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded)
    {
        env->results->terminationReason = E_TerminationReason::UnboundedProblem;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since the problem is unbounded.";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Numeric)
    {
        env->results->terminationReason = E_TerminationReason::NumericIssues;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated due to numerical issues.";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::None && env->results->hasPrimalSolution())
    {
        env->results->terminationReason = E_TerminationReason::ObjectiveGapNotReached;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription
            = "Terminated since the specified objective gap tolerance could not be met.";
    }
}

std::string TaskCheckIterationError::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT