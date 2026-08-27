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

TaskCheckIterationError::TaskCheckIterationError(
    EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDForcedReductionCut)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue), taskIDForcedReductionCut(taskIDForcedReductionCut)
{
}

TaskCheckIterationError::~TaskCheckIterationError() = default;

void TaskCheckIterationError::run()
{
    auto currIter = env->results->getCurrentIteration();

    // Always also check whether we actually got a solution in the current interation
    if(currIter->solutionStatus == E_ProblemSolutionStatus::Error /* && currIter->solutionPoints.size() == 0*/)
    {
        env->results->terminationReason = E_TerminationReason::Error;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since an error occured when solving the dual problem.";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible && currIter->solutionPoints.size() == 0)
    {
        env->results->terminationReason = E_TerminationReason::InfeasibleProblem;
        env->results->terminationReasonDescription = "Terminated since the dual problem is infeasible.";

        // The infeasibility repair loop (TaskRepairInfeasibleDualProblem) sets forceObjectiveReductionCut when it
        // detects it is looping without making progress, i.e. no more repairs can be done. In that situation the
        // problem being locally infeasible doesn't necessarily mean the algorithm is done, so give the reduction-cut
        // chain a chance instead of going straight to finalizing on a problem
        if(currIter->forceObjectiveReductionCut && !taskIDForcedReductionCut.empty())
            env->tasks->setNextTask(taskIDForcedReductionCut);
        else
            env->tasks->setNextTask(taskIDIfTrue);
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded && currIter->solutionPoints.size() == 0)
    {
        env->results->terminationReason = E_TerminationReason::UnboundedProblem;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since the dual problem is unbounded.";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Numeric && currIter->solutionPoints.size() == 0)
    {
        env->results->terminationReason = E_TerminationReason::NumericIssues;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription
            = "Terminated due to numerical issues when solving the dual problem.";
    }
}

std::string TaskCheckIterationError::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT