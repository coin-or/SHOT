/*
 * TaskCheckTimeLimit.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskCheckTimeLimit.h>

TaskCheckTimeLimit::TaskCheckTimeLimit(std::string taskIDTrue)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskIDIfTrue = taskIDTrue;
}

TaskCheckTimeLimit::~TaskCheckTimeLimit()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckTimeLimit::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (processInfo->getElapsedTime("Total") >= settings->getDoubleSetting("TimeLimit", "Algorithm"))
	{
		processInfo->terminationReason = E_TerminationReason::TimeLimit;
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckTimeLimit::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
