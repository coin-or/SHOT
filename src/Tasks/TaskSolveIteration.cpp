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

	if (processInfo->getPrimalBound() < DBL_MAX) MILPSolver->setCutOff(processInfo->getPrimalBound());

	if (MILPSolver->getDiscreteVariableStatus() && processInfo->primalSolution.size() > 0)
	{
		MILPSolver->deleteMIPStarts();
		MILPSolver->addMIPStart(processInfo->primalSolution);
	}

	auto solStatus = MILPSolver->solveProblem();

	int fixIter = 0;
	double newCutOff = processInfo->getPrimalBound();

	while (solStatus == E_ProblemSolutionStatus::Infeasible && fixIter < 10)
	{
		MILPSolver->deleteMIPStarts();
		if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			if (newCutOff > 0) newCutOff = 1.01 * newCutOff;
			else newCutOff = 0.99 * newCutOff;
		}
		else
		{
			if (newCutOff > 0) newCutOff = 0.99 * newCutOff;
			else newCutOff = 1.01 * newCutOff;
		}

		//Remove cutoff
		//if (newCutOff < DBL_MAX) MILPSolver->setCutOff(newCutOff);

		processInfo->logger.message(1) << "Infeasible problem detected, setting new cutoff to " << newCutOff
				<< CoinMessageEol;

		solStatus = MILPSolver->solveProblem();
		fixIter++;

	}

	if (solStatus == E_ProblemSolutionStatus::Infeasible || solStatus == E_ProblemSolutionStatus::Error
			|| solStatus == E_ProblemSolutionStatus::Unbounded)
	{

	}
	else
	{
		currIter->solutionStatus = solStatus;

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

