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

	MILPSolver->setCutOff(processInfo->getPrimalBound());

	if (MILPSolver->getDiscreteVariableStatus() && processInfo->primalSolutions.size() > 0)
	{
		//MILPSolver->deleteMIPStarts();
		MILPSolver->addMIPStart(processInfo->primalSolution);
	}

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		stringstream ss;
		ss << settings->getStringSetting("DebugPath", "SHOTSolver");
		ss << "/lp";
		ss << currIter->iterationNumber - 1;
		ss << ".lp";
		processInfo->MILPSolver->writeProblemToFile(ss.str());
	}

	auto solStatus = MILPSolver->solveProblem();

	/*
	 int fixIter = 0;
	 double newCutOff = processInfo->getPrimalBound();

	 while ((solStatus == E_ProblemSolutionStatus::CutOff || solStatus == E_ProblemSolutionStatus::Infeasible)
	 && fixIter < 3)
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
	 if (newCutOff < DBL_MAX) MILPSolver->setCutOff(newCutOff);

	 processInfo->logger.message(1) << "Infeasible problem detected, setting new cutoff to " << newCutOff
	 << CoinMessageEol;

	 solStatus = MILPSolver->solveProblem();
	 fixIter++;

	 }
	 */
	if (solStatus == E_ProblemSolutionStatus::Infeasible || solStatus == E_ProblemSolutionStatus::Error
			|| solStatus == E_ProblemSolutionStatus::Unbounded)
	{
		currIter->solutionStatus = solStatus;
	}
	else
	{
		currIter->solutionStatus = solStatus;

		auto sols = MILPSolver->getAllVariableSolutions();
		currIter->solutionPoints = sols;

		if (sols.size() > 0)
		{
			currIter->objectiveValue = MILPSolver->getObjectiveValue();
			//std::cout << "OBJ: " << currIter->objectiveValue << std::endl;
			auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(sols.at(0).point);

			currIter->maxDeviationConstraint = mostDevConstr.idx;
			currIter->maxDeviation = mostDevConstr.value;

			bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

			if (currIter->isMILP() && currIter->solutionStatus != E_ProblemSolutionStatus::Optimal) // Do not have the point, only the objective bound
			{
				double tmpDualObjBound = MILPSolver->getDualObjectiveValue();

				DualSolution sol =
				{ sols.at(0).point, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound,
						currIter->iterationNumber };
				processInfo->addDualSolutionCandidate(sol);
			}

			if (currIter->isMILP() && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal) // Do not have the point, only the objective bound
			{
				DualSolution sol =
				{ sols.at(0).point, E_DualSolutionSource::MILPSolutionOptimal, currIter->objectiveValue,
						currIter->iterationNumber };
				processInfo->addDualSolutionCandidate(sol);
			}

			if (!currIter->isMILP()) // Have a dual solution
			{
				DualSolution sol =
				{ sols.at(0).point, E_DualSolutionSource::LPSolution, currIter->objectiveValue,
						currIter->iterationNumber };
				processInfo->addDualSolutionCandidate(sol);
			}
		}
	}

	currIter->usedMILPSolutionLimit = MILPSolver->getSolutionLimit();

	// Update solution stats
	if (currIter->type == E_IterationProblemType::MIP && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		if (processInfo->originalProblem->isConstraintQuadratic(-1))
		{
			processInfo->iterOptMIQP = processInfo->iterOptMIQP + 1;
		}
		else
		{
			processInfo->iterOptMILP = processInfo->iterOptMILP + 1;
		}
	}
	else if (currIter->type == E_IterationProblemType::Relaxed)
	{
		if (processInfo->originalProblem->isConstraintQuadratic(-1))
		{
			processInfo->iterQP = processInfo->iterQP + 1;
		}
		else
		{
			processInfo->iterLP = processInfo->iterLP + 1;
		}

	}
	else if (currIter->type == E_IterationProblemType::MIP
			&& (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
					|| currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit))
	{
		if (processInfo->originalProblem->isConstraintQuadratic(-1))
		{
			processInfo->iterFeasMIQP = processInfo->iterFeasMIQP + 1;
		}
		else
		{
			processInfo->iterFeasMILP = processInfo->iterFeasMILP + 1;
		}
	}
}

std::string TaskSolveIteration::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
