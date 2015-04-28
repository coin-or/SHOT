/*
 * TaskCheckIterationLimit.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskCheckIterationLimit.h>

TaskCheckIterationLimit::TaskCheckIterationLimit(std::string taskIDTrue)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskIDIfTrue = taskIDTrue;
}

TaskCheckIterationLimit::~TaskCheckIterationLimit()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckIterationLimit::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->iterationNumber
			>= settings->getIntSetting("IterLimitLP", "Algorithm")
					+ settings->getIntSetting("IterLimitMILP", "Algorithm"))
	{
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}
}
