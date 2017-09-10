/*
 * TaskCheckRelativeGap.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskCheckRelativeGap.h"

TaskCheckRelativeGap::TaskCheckRelativeGap(std::string taskIDTrue)
{

	taskIDIfTrue = taskIDTrue;
}
TaskCheckRelativeGap::~TaskCheckRelativeGap()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckRelativeGap::run()
{
	if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::RelativeGap;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckRelativeGap::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
