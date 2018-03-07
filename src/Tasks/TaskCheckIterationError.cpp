/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckIterationError.h"

TaskCheckIterationError::TaskCheckIterationError(std::string taskIDTrue)
{
	taskIDIfTrue = taskIDTrue;
}

TaskCheckIterationError::~TaskCheckIterationError()
{
}

void TaskCheckIterationError::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->solutionStatus == E_ProblemSolutionStatus::Error)
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::Error;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
	}
	else if (currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::InfeasibleProblem;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckIterationError::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
