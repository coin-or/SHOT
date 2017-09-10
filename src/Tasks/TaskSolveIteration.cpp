/*
 * TaskSolveIteration.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include "TaskSolveIteration.h"

TaskSolveIteration::TaskSolveIteration(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;

}

TaskSolveIteration::~TaskSolveIteration()
{
	// TODO Auto-generated destructor stub
}

void TaskSolveIteration::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	// Sets the iteration time limit
	auto timeLim = Settings::getInstance().getDoubleSetting("TimeLimit", "Algorithm")
			- ProcessInfo::getInstance().getElapsedTime("Total");
	MILPSolver->setTimeLimit(timeLim);

	if (ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MILPSolver->setCutOff(ProcessInfo::getInstance().getPrimalBound());
	}

	if (Settings::getInstance().getBoolSetting("UpdateNonlinearObjectiveVariableBounds", "MILP")
			&& !currIter->MILPSolutionLimitUpdated)
	{
		MILPSolver->updateNonlinearObjectiveFromPrimalDualBounds();
	}

	if (MILPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		//MILPSolver->deleteMIPStarts();
		MILPSolver->addMIPStart(ProcessInfo::getInstance().primalSolution);
	}

	if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
	{
		stringstream ss;
		ss << Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver");
		ss << "/lp";
		ss << currIter->iterationNumber - 1;
		ss << ".lp";
		MILPSolver->writeProblemToFile(ss.str());
	}

	auto solStatus = MILPSolver->solveProblem();

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

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
					sols.at(0).point);

			currIter->maxDeviationConstraint = mostDevConstr.idx;
			currIter->maxDeviation = mostDevConstr.value;

			bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

			if (currIter->isMILP() && currIter->solutionStatus != E_ProblemSolutionStatus::Optimal) // Do not have the point, only the objective bound
			{
				double tmpDualObjBound = MILPSolver->getDualObjectiveValue();

				DualSolution sol =
				{ sols.at(0).point, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound,
						currIter->iterationNumber };
				ProcessInfo::getInstance().addDualSolutionCandidate(sol);
			}

			if (currIter->isMILP() && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal) // Do not have the point, only the objective bound
			{
				DualSolution sol =
				{ sols.at(0).point, E_DualSolutionSource::MILPSolutionOptimal, currIter->objectiveValue,
						currIter->iterationNumber };
				ProcessInfo::getInstance().addDualSolutionCandidate(sol);
			}

			if (!currIter->isMILP()) // Have a dual solution
			{
				DualSolution sol =
				{ sols.at(0).point, E_DualSolutionSource::LPSolution, currIter->objectiveValue,
						currIter->iterationNumber };
				ProcessInfo::getInstance().addDualSolutionCandidate(sol);
			}
		}
	}

	currIter->usedMILPSolutionLimit = MILPSolver->getSolutionLimit();

// Update solution stats
	if (currIter->type == E_IterationProblemType::MIP && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
	{
		if (ProcessInfo::getInstance().originalProblem->isConstraintQuadratic(-1))
		{
			ProcessInfo::getInstance().iterOptMIQP = ProcessInfo::getInstance().iterOptMIQP + 1;
		}
		else
		{
			ProcessInfo::getInstance().iterOptMILP = ProcessInfo::getInstance().iterOptMILP + 1;
		}
	}
	else if (currIter->type == E_IterationProblemType::Relaxed)
	{
		if (ProcessInfo::getInstance().originalProblem->isConstraintQuadratic(-1))
		{
			ProcessInfo::getInstance().iterQP = ProcessInfo::getInstance().iterQP + 1;
		}
		else
		{
			ProcessInfo::getInstance().iterLP = ProcessInfo::getInstance().iterLP + 1;
		}

	}
	else if (currIter->type == E_IterationProblemType::MIP
			&& (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
					|| currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit))
	{
		if (ProcessInfo::getInstance().originalProblem->isConstraintQuadratic(-1))
		{
			ProcessInfo::getInstance().iterFeasMIQP = ProcessInfo::getInstance().iterFeasMIQP + 1;
		}
		else
		{
			ProcessInfo::getInstance().iterFeasMILP = ProcessInfo::getInstance().iterFeasMILP + 1;
		}
	}
}

std::string TaskSolveIteration::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
