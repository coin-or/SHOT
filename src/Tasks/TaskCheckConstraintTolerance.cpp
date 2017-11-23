/*
 * TaskCheckConstraintTolerance.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskCheckConstraintTolerance.h"

TaskCheckConstraintTolerance::TaskCheckConstraintTolerance(std::string taskIDTrue)
{

	taskIDIfTrue = taskIDTrue;
}

TaskCheckConstraintTolerance::~TaskCheckConstraintTolerance()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckConstraintTolerance::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->maxDeviation < Settings::getInstance().getDoubleSetting("ConstrTermTolMILP", "Algorithm")
			&& currIter->solutionStatus == E_ProblemSolutionStatus::Optimal
			&& currIter->type == E_IterationProblemType::MIP)
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::ConstraintTolerance;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
	}

	return;
}

std::string TaskCheckConstraintTolerance::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
