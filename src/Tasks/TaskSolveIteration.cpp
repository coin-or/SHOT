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

	bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

	// Sets the iteration time limit
	auto timeLim = Settings::getInstance().getDoubleSetting("TimeLimit", "Algorithm") - ProcessInfo::getInstance().getElapsedTime("Total");
	MILPSolver->setTimeLimit(timeLim);

	if (ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		if (isMinimization)
		{
			MILPSolver->setCutOff(
				ProcessInfo::getInstance().getPrimalBound() + Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual"));
		}
		else
		{
			MILPSolver->setCutOff(
				ProcessInfo::getInstance().getPrimalBound() - Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual"));
		}
	}

	if (Settings::getInstance().getBoolSetting("MIP.UpdateObjectiveBounds", "Dual") && !currIter->MILPSolutionLimitUpdated)
	{
		MILPSolver->updateNonlinearObjectiveFromPrimalDualBounds();
	}

	if (MILPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
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

	ProcessInfo::getInstance().outputInfo("Solving MIP problem.");
	auto solStatus = MILPSolver->solveProblem();
	ProcessInfo::getInstance().outputInfo("MIP problem solved.");

	// Must update the pointer to the current iteration if we use the lazy strategy since new iterations have been created when solving
	if (static_cast<ES_SolutionStrategy>(Settings::getInstance().getIntSetting("TreeStrategy ", "Dual")) == ES_SolutionStrategy::SingleTree)
	{
		currIter = ProcessInfo::getInstance().getCurrentIteration();
	}

	if (solStatus == E_ProblemSolutionStatus::Infeasible || solStatus == E_ProblemSolutionStatus::Error || solStatus == E_ProblemSolutionStatus::Unbounded)
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
			if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
			{
				stringstream ss;
				ss << Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver");
				ss << "/lpsolpt";
				ss << currIter->iterationNumber - 1;
				ss << ".txt";
				UtilityFunctions::saveVariablePointVectorToFile(sols.at(0).point,
																ProcessInfo::getInstance().originalProblem->getVariableNames(), ss.str());
			}

			currIter->objectiveValue = MILPSolver->getObjectiveValue();

			if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
			{
				std::vector<double> tmpObjValue;
				std::vector<std::string> tmpObjName;

				tmpObjValue.push_back(MILPSolver->getObjectiveValue());
				tmpObjName.push_back("objective");

				stringstream ss;
				ss << Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver");
				ss << "/lpobjsol";
				ss << currIter->iterationNumber - 1;
				ss << ".txt";
				UtilityFunctions::saveVariablePointVectorToFile(tmpObjValue, tmpObjName, ss.str());
			}

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
				sols.at(0).point);

			currIter->maxDeviationConstraint = mostDevConstr.idx;
			currIter->maxDeviation = mostDevConstr.value;

			if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
			{
				std::vector<double> tmpMostDevValue;
				std::vector<std::string> tmpConstrIndex;

				tmpMostDevValue.push_back(mostDevConstr.value);
				tmpConstrIndex.push_back(std::to_string(mostDevConstr.idx));

				stringstream ss;
				ss << Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver");
				ss << "/lpmostdevm";
				ss << currIter->iterationNumber - 1;
				ss << ".txt";
				UtilityFunctions::saveVariablePointVectorToFile(tmpMostDevValue, tmpConstrIndex, ss.str());
			}

			double tmpDualObjBound = MILPSolver->getDualObjectiveValue();
			if (currIter->isMILP())
			{
				DualSolution sol =
					{sols.at(0).point, E_DualSolutionSource::MILPSolutionFeasible, tmpDualObjBound,
					 currIter->iterationNumber};
				ProcessInfo::getInstance().addDualSolutionCandidate(sol);

				if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
				{
					DualSolution sol =
					{sols.at(0).point, E_DualSolutionSource::MILPSolutionOptimal, currIter->objectiveValue,
					 currIter->iterationNumber};
					ProcessInfo::getInstance().addDualSolutionCandidate(sol);
				}
			}
			else
			{
				DualSolution sol =
					{sols.at(0).point, E_DualSolutionSource::LPSolution, tmpDualObjBound,
					 currIter->iterationNumber};
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
	else if (currIter->type == E_IterationProblemType::MIP && (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit || currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit))
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
