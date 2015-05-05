/*
 * TaskCheckConstraintTolerance.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskCheckConstraintTolerance.h>

TaskCheckConstraintTolerance::TaskCheckConstraintTolerance(std::string taskIDTrue)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskIDIfTrue = taskIDTrue;
}

TaskCheckConstraintTolerance::~TaskCheckConstraintTolerance()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckConstraintTolerance::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm")
			&& currIter->solutionStatus == E_ProblemSolutionStatus::Optimal
			&& currIter->type == E_IterationProblemType::MIP)
	{
		processInfo->tasks->setNextTask(taskIDIfTrue);
	}

	if (currIter->iterationNumber == 1) return;

	auto prevIter = processInfo->getPreviousIteration();
	auto objChangeNorm = abs(currIter->objectiveValue - prevIter->objectiveValue);

	if (settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm")
			!= settings->getDoubleSetting("ObjectionFunctionTol", "Algorithm")
			&& (currIter->maxDeviation < settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm")
					&& currIter->solutionStatus == E_ProblemSolutionStatus::Optimal
					&& currIter->type == E_IterationProblemType::MIP
					&& objChangeNorm < settings->getDoubleSetting("ObjectionFunctionTol", "Algorithm")))
	{
		//processInfo->logger.message(2) << "Current objective change norm is " << objChangeNorm << CoinMessageEol;

		processInfo->tasks->setNextTask(taskIDIfTrue);
	}
}
