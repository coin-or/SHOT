/*
 * TaskCheckIterationError.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskCheckIterationError.h"

TaskCheckIterationError::TaskCheckIterationError(std::string taskIDTrue)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskIDIfTrue = taskIDTrue;
}

TaskCheckIterationError::~TaskCheckIterationError()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckIterationError::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->solutionStatus == E_ProblemSolutionStatus::Error)
	{
		processInfo->terminationReason = E_TerminationReason::Error;
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}
	else if (currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
	{
		processInfo->terminationReason = E_TerminationReason::InfeasibleProblem;
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckIterationError::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
