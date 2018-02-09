/*
 * TaskSolveIteration.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include "TaskSolveIteration.h"

TaskSolveIteration::TaskSolveIteration(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;
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
	auto timeLim = Settings::getInstance().getDoubleSetting("TimeLimit", "Termination") - ProcessInfo::getInstance().getElapsedTime("Total");
	MIPSolver->setTimeLimit(timeLim);

	if (ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		if (isMinimization)
		{
			MIPSolver->setCutOff(
				ProcessInfo::getInstance().getPrimalBound() + Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual"));
		}
		else
		{
			MIPSolver->setCutOff(
				ProcessInfo::getInstance().getPrimalBound() - Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual"));
		}
	}

	if (Settings::getInstance().getBoolSetting("MIP.UpdateObjectiveBounds", "Dual") && !currIter->MIPSolutionLimitUpdated)
	{
		MIPSolver->updateNonlinearObjectiveFromPrimalDualBounds();
	}

	if (MIPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
	{
		MIPSolver->addMIPStart(ProcessInfo::getInstance().primalSolution);
	}

	if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
	{
		stringstream ss;
		ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
		ss << "/lp";
		ss << currIter->iterationNumber - 1;
		ss << ".lp";
		MIPSolver->writeProblemToFile(ss.str());
	}

	ProcessInfo::getInstance().outputInfo("Solving MIP problem.");
	auto solStatus = MIPSolver->solveProblem();
	ProcessInfo::getInstance().outputInfo("MIP problem solved.");

	// Must update the pointer to the current iteration if we use the lazy strategy since new iterations have been created when solving
	if (static_cast<ES_SolutionStrategy>(Settings::getInstance().getIntSetting("TreeStrategy", "Dual")) == ES_SolutionStrategy::SingleTree)
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

		auto sols = MIPSolver->getAllVariableSolutions();
		currIter->solutionPoints = sols;

		if (sols.size() > 0)
		{
			if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
			{
				stringstream ss;
				ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
				ss << "/lpsolpt";
				ss << currIter->iterationNumber - 1;
				ss << ".txt";
				UtilityFunctions::saveVariablePointVectorToFile(sols.at(0).point,
																ProcessInfo::getInstance().originalProblem->getVariableNames(), ss.str());
			}

			currIter->objectiveValue = MIPSolver->getObjectiveValue();

			if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
			{
				std::vector<double> tmpObjValue;
				std::vector<std::string> tmpObjName;

				tmpObjValue.push_back(MIPSolver->getObjectiveValue());
				tmpObjName.push_back("objective");

				stringstream ss;
				ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
				ss << "/lpobjsol";
				ss << currIter->iterationNumber - 1;
				ss << ".txt";
				UtilityFunctions::saveVariablePointVectorToFile(tmpObjValue, tmpObjName, ss.str());
			}

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
				sols.at(0).point);

			currIter->maxDeviationConstraint = mostDevConstr.idx;
			currIter->maxDeviation = mostDevConstr.value;

			if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
			{
				std::vector<double> tmpMostDevValue;
				std::vector<std::string> tmpConstrIndex;

				tmpMostDevValue.push_back(mostDevConstr.value);
				tmpConstrIndex.push_back(std::to_string(mostDevConstr.idx));

				stringstream ss;
				ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
				ss << "/lpmostdevm";
				ss << currIter->iterationNumber - 1;
				ss << ".txt";
				UtilityFunctions::saveVariablePointVectorToFile(tmpMostDevValue, tmpConstrIndex, ss.str());
			}

			double tmpDualObjBound = MIPSolver->getDualObjectiveValue();
			if (currIter->isMIP())
			{
				DualSolution sol =
					{sols.at(0).point, E_DualSolutionSource::MIPSolutionFeasible, tmpDualObjBound,
					 currIter->iterationNumber};
				ProcessInfo::getInstance().addDualSolutionCandidate(sol);

				if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
				{
					DualSolution sol =
					{sols.at(0).point, E_DualSolutionSource::MIPSolutionOptimal, currIter->objectiveValue,
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

	currIter->usedMIPSolutionLimit = MIPSolver->getSolutionLimit();

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
