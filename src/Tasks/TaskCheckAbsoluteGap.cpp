/*
 * TaskCheckConstraintTolerance.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskCheckAbsoluteGap.h>

TaskCheckAbsoluteGap::TaskCheckAbsoluteGap(std::string taskIDTrue)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskIDIfTrue = taskIDTrue;
}

TaskCheckAbsoluteGap::~TaskCheckAbsoluteGap()
{
// TODO Auto-generated destructor stub
}

void TaskCheckAbsoluteGap::run()
{
	if (processInfo->isAbsoluteObjectiveGapToleranceMet())
	{
		processInfo->terminationReason = E_TerminationReason::AbsoluteGap;
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckAbsoluteGap::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
