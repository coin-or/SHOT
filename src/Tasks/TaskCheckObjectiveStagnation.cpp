/*
 * TaskCheckObjectiveStagnation.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskCheckObjectiveStagnation.h"

TaskCheckObjectiveStagnation::TaskCheckObjectiveStagnation(std::string taskIDTrue)
{

	taskIDIfTrue = taskIDTrue;
}

TaskCheckObjectiveStagnation::~TaskCheckObjectiveStagnation()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckObjectiveStagnation::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (!currIter->isMILP())
	{
		return;
	}

	if (ProcessInfo::getInstance().iterFeasMILP + ProcessInfo::getInstance().iterOptMILP
			<= Settings::getInstance().getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
	{
		return;
	}

	if (ProcessInfo::getInstance().iterSignificantObjectiveUpdate == 0) // First MILP solution
	{
		ProcessInfo::getInstance().iterSignificantObjectiveUpdate = currIter->iterationNumber;
		ProcessInfo::getInstance().itersWithStagnationMILP = 0;
		return;
	}

	if (std::abs(
			(currIter->objectiveValue
					- ProcessInfo::getInstance().iterations[ProcessInfo::getInstance().iterSignificantObjectiveUpdate
							- 1].objectiveValue))
			> Settings::getInstance().getDoubleSetting("ObjectiveStagnation.Tolerance", "Termination"))
	{
		ProcessInfo::getInstance().iterSignificantObjectiveUpdate = currIter->iterationNumber;
		ProcessInfo::getInstance().itersWithStagnationMILP = 0;

		return;
	}

	if (ProcessInfo::getInstance().itersWithStagnationMILP
			>= Settings::getInstance().getIntSetting("ObjectiveStagnation.IterationLimit", "Termination"))
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::ObjectiveStagnation;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);

	}

	ProcessInfo::getInstance().itersWithStagnationMILP++;
}

std::string TaskCheckObjectiveStagnation::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
