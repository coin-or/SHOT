/*
 * TaskCheckRelativeGap.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskCheckRelativeGap.h>

TaskCheckRelativeGap::TaskCheckRelativeGap(std::string taskIDTrue)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskIDIfTrue = taskIDTrue;
}
TaskCheckRelativeGap::~TaskCheckRelativeGap()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckRelativeGap::run()
{
	auto currIter = processInfo->getCurrentIteration();

	double gap = processInfo->getRelativeObjectiveGap();

	if (currIter->isMILP() && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal
			&& gap <= settings->getDoubleSetting("GapTermTolRelative", "Algorithm"))
	{

		processInfo->terminationReason = E_TerminationReason::RelativeGap;
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckRelativeGap::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
