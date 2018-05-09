/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckIterationError.h"

TaskCheckIterationError::TaskCheckIterationError(EnvironmentPtr envPtr, std::string taskIDTrue) : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckIterationError::~TaskCheckIterationError()
{
}

void TaskCheckIterationError::run()
{
    auto currIter = env->process->getCurrentIteration();

    if (currIter->solutionStatus == E_ProblemSolutionStatus::Error)
    {
        env->process->terminationReason = E_TerminationReason::Error;
        env->tasks->setNextTask(taskIDIfTrue);
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        env->process->terminationReason = E_TerminationReason::InfeasibleProblem;
        env->tasks->setNextTask(taskIDIfTrue);
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::CutOff)
    {
        env->process->terminationReason = E_TerminationReason::InfeasibleProblem;
        env->tasks->setNextTask(taskIDIfTrue);
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded)
    {
        env->process->terminationReason = E_TerminationReason::UnboundedProblem;
        env->tasks->setNextTask(taskIDIfTrue);
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::Numeric)
    {
        env->process->terminationReason = E_TerminationReason::NumericIssues;
        env->tasks->setNextTask(taskIDIfTrue);
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::None &&
             env->process->primalSolutions.size() > 0)
    {
        env->process->terminationReason = E_TerminationReason::ObjectiveGapNotReached;
        env->tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckIterationError::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
