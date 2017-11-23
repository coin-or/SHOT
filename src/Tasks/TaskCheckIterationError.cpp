/*
 * TaskCheckIterationError.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskCheckIterationError.h"

TaskCheckIterationError::TaskCheckIterationError(std::string taskIDTrue)
{

	taskIDIfTrue = taskIDTrue;
}

TaskCheckIterationError::~TaskCheckIterationError()
{
	// TODO Auto-generated destructor stub
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
