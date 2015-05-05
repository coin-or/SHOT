/*
 * TaskSolveIteration.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include <TaskSolveIteration.h>

TaskSolveIteration::TaskSolveIteration()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskSolveIteration::~TaskSolveIteration()
{
	// TODO Auto-generated destructor stub
}

void TaskSolveIteration::run()
{
	auto currIter = processInfo->getCurrentIteration();
	auto MILPSolver = processInfo->MILPSolver;

	// Sets the iteration time limit
	auto timeLim = settings->getDoubleSetting("TimeLimit", "Algorithm") - processInfo->getElapsedTime("Total");
	MILPSolver->setTimeLimit(timeLim);

	auto solStatus = MILPSolver->solveProblem();

	currIter->solutionStatus = solStatus;
	auto sols = MILPSolver->getAllVariableSolutions();
	currIter->solutionPoints = sols;

	currIter->objectiveValue = MILPSolver->getLastObjectiveValue();
	//processInfo->lastObjectiveValue = currIter->objectiveValue;

	auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(sols.at(0).point);

	currIter->maxDeviationConstraint = mostDevConstr.idx;
	currIter->maxDeviation = mostDevConstr.value;

	currIter->usedMILPSolutionLimit = MILPSolver->getSolutionLimit();

	//Check if new dual solution
	if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

		if ((isMinimization && currIter->objectiveValue > processInfo->currentObjectiveBounds.first)
				|| (!isMinimization && currIter->objectiveValue < processInfo->currentObjectiveBounds.first))
		{

			// New dual solution
			processInfo->currentObjectiveBounds.first = currIter->objectiveValue;

			processInfo->addDualSolution(sols.at(0), E_DualSolutionSource::MILPSolution);

			processInfo->logger.message(3) << "New dual bound: " << processInfo->currentObjectiveBounds.first
					<< CoinMessageEol;
		}
	}

	// Update solution stats
	if (currIter->type == E_IterationProblemType::MIP && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		processInfo->iterOptMILP = processInfo->iterOptMILP + 1;
	}
	else if (currIter->type == E_IterationProblemType::Relaxed)
	{
		processInfo->iterLP = processInfo->iterLP + 1;
	}
	else if (currIter->type == E_IterationProblemType::MIP
			&& (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
					|| currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit))
	{
		processInfo->iterFeasMILP = processInfo->iterFeasMILP + 1;
	}

}
