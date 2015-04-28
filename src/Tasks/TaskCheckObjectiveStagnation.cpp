/*
 * TaskCheckObjectiveStagnation.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskCheckObjectiveStagnation.h>

TaskCheckObjectiveStagnation::TaskCheckObjectiveStagnation(std::string taskIDTrue)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskIDIfTrue = taskIDTrue;
}

TaskCheckObjectiveStagnation::~TaskCheckObjectiveStagnation()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckObjectiveStagnation::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (!currIter->isMILP())
	{
		return;
	}

	if (processInfo->iterFeasMILP + processInfo->iterOptMILP
			<= settings->getIntSetting("ObjectiveStagnationIterationLimit", "Algorithm"))
	{
		return;
	}

	if (processInfo->iterSignificantObjectiveUpdate == 0) // First MILP solution
	{
		processInfo->iterSignificantObjectiveUpdate = currIter->iterationNumber;
		processInfo->itersWithStagnationMILP = 0;
		return;
	}

	if (std::abs(
			(currIter->objectiveValue
					- processInfo->iterations[processInfo->iterSignificantObjectiveUpdate - 1].objectiveValue))
			> settings->getDoubleSetting("ObjectiveStagnationTolerance", "Algorithm"))
	{
		processInfo->iterSignificantObjectiveUpdate = currIter->iterationNumber;
		processInfo->itersWithStagnationMILP = 0;
		processInfo->logger.message(4) << "Significant change in objective function value detected." << CoinMessageEol;
		return;
	}

	if (processInfo->itersWithStagnationMILP
			>= settings->getIntSetting("ObjectiveStagnationIterationLimit", "Algorithm"))
	{
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}

	processInfo->itersWithStagnationMILP++;
}
