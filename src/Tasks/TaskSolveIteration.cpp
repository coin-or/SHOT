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

	if (solStatus == E_ProblemSolutionStatus::Error || solStatus == E_ProblemSolutionStatus::Infeasible
			|| solStatus == E_ProblemSolutionStatus::Unbounded)
	{

	}
	else
	{
		auto sols = MILPSolver->getAllVariableSolutions();
		currIter->solutionPoints = sols;

		currIter->objectiveValue = MILPSolver->getLastObjectiveValue();
		auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(sols.at(0).point);

		currIter->maxDeviationConstraint = mostDevConstr.idx;
		currIter->maxDeviation = mostDevConstr.value;

		//Check if new dual solution
		if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
		{
			bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

			if ((isMinimization && currIter->objectiveValue > processInfo->currentObjectiveBounds.first)
					|| (!isMinimization && currIter->objectiveValue < processInfo->currentObjectiveBounds.first))
			{
				// New dual solution
				processInfo->currentObjectiveBounds.first = currIter->objectiveValue;
				processInfo->iterLastDualBoundUpdate = currIter->iterationNumber;

				processInfo->addDualSolution(sols.at(0), E_DualSolutionSource::MILPSolution);

				processInfo->logger.message(3) << "New dual bound: " << processInfo->currentObjectiveBounds.first
						<< CoinMessageEol;
			}

			if (!currIter->isMILP()) processInfo->iterLastDualBoundUpdate = currIter->iterationNumber;
		}
	}

	currIter->usedMILPSolutionLimit = MILPSolver->getSolutionLimit();

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
std::string TaskSolveIteration::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

